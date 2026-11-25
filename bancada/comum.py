"""
Peças comuns da bancada de testes: conexão com a placa (Wi-Fi, USB ou placa
virtual), leitura/escrita dos logs, o simulador em C++ e as métricas das curvas.
"""
import base64
import csv
import json
import math
import os
import re
import socket
import struct
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from urllib.parse import urlparse

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)

NOMES_PARAM = ["pMax", "perda", "tAmb", "atraso", "kp", "ki", "iMax", "margem", "bandaI", "folga"]
# mesmos valores de PARAM_CONTROLE em src/main.cpp
PARAM_FABRICA = {"pMax": 3.2, "perda": 0.008, "tAmb": 25.0, "atraso": 5.0, "kp": 5.0,
                 "ki": 0.4, "iMax": 25.0, "margem": 1.0, "bandaI": 15.0, "folga": 4.0}
def params_do_codigo():
    """Lê os valores de PARAM_CONTROLE direto do src/main.cpp."""
    try:
        texto = open(os.path.join(RAIZ, "src", "main.cpp"), encoding="utf-8").read()
        bloco = re.search(r"const ParamControle PARAM_CONTROLE\s*=\s*\{(.*?)\};", texto, re.S).group(1)
        bloco = re.sub(r"//[^\n]*", "", bloco)
        valores = [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?(?:[eE]-?\d+)?", bloco)]
        return dict(zip(NOMES_PARAM, valores)) if len(valores) == len(NOMES_PARAM) else None
    except (OSError, AttributeError):
        return None


# algoritmo antigo (antes do freio preditivo), para comparação
PARAM_SEM_FREIO = dict(PARAM_FABRICA, margem=-1.0, bandaI=300.0)

COLUNAS = ["pc_s", "ms", "fase", "estado", "t_curva", "temp", "alvo", "duty",
           "ff", "p", "i", "teto", "uf", "vac", "vuse"]


# ======================================================================
# Curvas
# ======================================================================
def interp(pts, t):
    if t <= pts[0][0]:
        return pts[0][1]
    for a, b in zip(pts, pts[1:]):
        if t <= b[0]:
            return a[1] + (b[1] - a[1]) * (t - a[0]) / (b[0] - a[0])
    return pts[-1][1]


def inclinacao(pts, t):
    return interp(pts, t + 1.0) - interp(pts, t)


# ======================================================================
# Conexões. Todas têm: enviar(dict), receber(timeout) -> dict|None,
# agora() -> s, fechar().
# ======================================================================
class ConexaoWS:
    """Cliente WebSocket mínimo (só biblioteca padrão)."""

    def __init__(self, url, timeout=8.0):
        u = urlparse(url)
        self.host, self.porta = u.hostname, u.port or 80
        caminho = u.path or "/ws"
        self.sock = socket.create_connection((self.host, self.porta), timeout=timeout)
        chave = base64.b64encode(os.urandom(16)).decode()
        pedido = (f"GET {caminho} HTTP/1.1\r\nHost: {self.host}:{self.porta}\r\n"
                  "Upgrade: websocket\r\nConnection: Upgrade\r\n"
                  f"Sec-WebSocket-Key: {chave}\r\nSec-WebSocket-Version: 13\r\n\r\n")
        self.sock.sendall(pedido.encode())
        resp = b""
        while b"\r\n\r\n" not in resp:
            parte = self.sock.recv(1024)
            if not parte:
                raise ConnectionError("a placa fechou a conexão")
            resp += parte
        cab, self.buf = resp.split(b"\r\n\r\n", 1)
        if b" 101 " not in cab.split(b"\r\n")[0]:
            raise ConnectionError("resposta inesperada: " + cab.split(b"\r\n")[0].decode(errors="replace"))
        self.fila = []
        self.pedaco = b""

    def agora(self):
        return time.monotonic()

    def _quadro(self, opcode, dados):
        cab = bytes([0x80 | opcode])
        n = len(dados)
        if n < 126:
            cab += bytes([0x80 | n])
        elif n < 65536:
            cab += bytes([0x80 | 126]) + struct.pack(">H", n)
        else:
            cab += bytes([0x80 | 127]) + struct.pack(">Q", n)
        mascara = os.urandom(4)
        corpo = bytes(b ^ mascara[i % 4] for i, b in enumerate(dados))
        self.sock.sendall(cab + mascara + corpo)

    def enviar(self, obj):
        self._quadro(0x1, json.dumps(obj).encode())

    def _ler(self, n):
        while len(self.buf) < n:
            parte = self.sock.recv(4096)
            if not parte:
                raise ConnectionError("conexão perdida")
            self.buf += parte
        d, self.buf = self.buf[:n], self.buf[n:]
        return d

    def receber(self, timeout=1.0):
        fim = time.monotonic() + timeout
        while not self.fila:
            resta = fim - time.monotonic()
            if resta <= 0:
                return None
            self.sock.settimeout(resta)
            try:
                b0, b1 = self._ler(2)
            except socket.timeout:
                return None
            self.sock.settimeout(10)
            n = b1 & 0x7F
            if n == 126:
                n = struct.unpack(">H", self._ler(2))[0]
            elif n == 127:
                n = struct.unpack(">Q", self._ler(8))[0]
            mascara = self._ler(4) if b1 & 0x80 else None
            dados = self._ler(n)
            if mascara:
                dados = bytes(b ^ mascara[i % 4] for i, b in enumerate(dados))
            op = b0 & 0x0F
            if op == 0x9:
                self._quadro(0xA, dados)          # ping -> pong
            elif op == 0x8:
                raise ConnectionError("a placa fechou a conexão")
            elif op in (0x0, 0x1):
                self.pedaco += dados
                if b0 & 0x80:
                    try:
                        self.fila.append(json.loads(self.pedaco.decode("utf-8")))
                    except ValueError:
                        pass
                    self.pedaco = b""
        return self.fila.pop(0)

    def fechar(self):
        try:
            self._quadro(0x8, b"")
            self.sock.close()
        except OSError:
            pass


class ConexaoSerial:
    """USB. ATENÇÃO: o README pede para não ligar o USB ao PC com a parte de 220 V
    energizada (o termopar liga a placa ao GND). Prefira o Wi-Fi."""

    def __init__(self, porta):
        try:
            import serial
        except ImportError:
            raise SystemExit("Para usar a USB instale o pyserial:  pip install pyserial")
        self.s = serial.Serial(porta, 115200, timeout=0.1)
        self.buf = b""

    def agora(self):
        return time.monotonic()

    def enviar(self, obj):
        self.s.write((json.dumps(obj) + "\n").encode())

    def receber(self, timeout=1.0):
        fim = time.monotonic() + timeout
        while time.monotonic() < fim:
            while b"\n" in self.buf:
                linha, self.buf = self.buf.split(b"\n", 1)
                linha = linha.strip()
                if linha.startswith(b"{"):
                    try:
                        return json.loads(linha.decode("utf-8", errors="replace"))
                    except ValueError:
                        pass
            self.buf += self.s.read(512)
        return None

    def fechar(self):
        self.s.close()


def conectar(destino):
    if destino.startswith("ws://"):
        return ConexaoWS(destino)
    return ConexaoSerial(destino)


# ======================================================================
# Simulador (C++ com o código real de lib/Controle)
# ======================================================================
SIM_DIR = os.path.join(AQUI, "sim")
SIM_EXE = os.path.join(SIM_DIR, "simulador.exe" if os.name == "nt" else "simulador")
SIM_FONTES = [os.path.join(SIM_DIR, "simulador.cpp"),
              os.path.join(RAIZ, "lib", "Controle", "Controle.cpp"),
              os.path.join(RAIZ, "lib", "Controle", "Controle.h")]


def compilar_simulador():
    """Compila o simulador se ele não existir ou se o controle mudou."""
    if os.path.exists(SIM_EXE) and all(os.path.getmtime(SIM_EXE) >= os.path.getmtime(f) for f in SIM_FONTES):
        return SIM_EXE
    cmd = ["g++", "-O2", "-std=c++17", "-o", SIM_EXE, SIM_FONTES[0], SIM_FONTES[1]]
    try:
        subprocess.run(cmd, check=True, capture_output=True, text=True)
    except FileNotFoundError:
        raise SystemExit("Preciso do g++ no PATH para compilar o simulador (ex.: MSYS2).")
    except subprocess.CalledProcessError as e:
        raise SystemExit("Erro ao compilar o simulador:\n" + e.stderr)
    return SIM_EXE


def _lista(v):
    return ",".join(f"{x:.6g}" for x in v)


def linha_trabalho(tr):
    """tr: dict com id, modo ('curva'|'aberta'), ctl (dict), planta (dict), pts, ..."""
    ctl = tr.get("ctl", PARAM_FABRICA)
    pl = tr["planta"]
    partes = [f"id={tr.get('id', 'job')}", f"modo={tr.get('modo', 'curva')}",
              "ctl=" + _lista([ctl[n] for n in NOMES_PARAM]),
              "planta=" + _lista([pl["pMax"], pl["tau1"], pl["k1"], pl["k2"], pl["tau2"], pl["morto"], pl["Ta"]]),
              "pts=" + _lista([v for p in tr["pts"] for v in p[:2]])]
    for chave in ("tmax", "ruido", "seed", "vrede", "vctl", "amostra", "sem_monitor"):
        if chave in tr:
            partes.append(f"{chave}={tr[chave]}")
    if "estado" in tr:
        partes.append("estado=" + _lista(tr["estado"]))
    return " ".join(partes)


def simular(trabalhos, processos=None):
    """Roda vários trabalhos no simulador, em paralelo. Retorna a lista de resultados."""
    exe = compilar_simulador()
    if not trabalhos:
        return []
    processos = processos or max(1, min(len(trabalhos), (os.cpu_count() or 2)))
    grupos = [trabalhos[i::processos] for i in range(processos)]

    def roda(grupo):
        entrada = "\n".join(linha_trabalho(t) for t in grupo) + "\n"
        r = subprocess.run([exe], input=entrada, capture_output=True, text=True, check=True)
        return [json.loads(l) for l in r.stdout.splitlines() if l.strip()]

    with ThreadPoolExecutor(processos) as ex:
        partes = list(ex.map(roda, grupos))
    # devolve na ordem original
    saida = [None] * len(trabalhos)
    for g, res in enumerate(partes):
        for k, r in enumerate(res):
            saida[g + k * processos] = r
    return saida


# ======================================================================
# Logs
# ======================================================================
def salvar_cabecalho(f, meta):
    f.write("# " + json.dumps(meta, ensure_ascii=False) + "\n")
    f.write(",".join(COLUNAS) + "\n")


def linha_log(pc_s, fase, m):
    ctl = m.get("ctl") or [None] * 5
    duty = m.get("u", m.get("duty"))
    vals = [round(pc_s, 3), m.get("ms"), fase, m.get("state"), m.get("t"), m.get("temp"), m.get("target"),
            duty] + list(ctl) + [m.get("vac"), m.get("vuse")]
    return ",".join("" if v is None else str(v) for v in vals) + "\n"


def _num(v):
    if v is None or v == "":
        return None
    try:
        return float(v.replace(",", ".")) if isinstance(v, str) else float(v)
    except ValueError:
        return None


def ler_log(caminho):
    """Lê um log da bancada (ou um CSV baixado do site). Retorna (meta, linhas[dict])."""
    with open(caminho, encoding="utf-8-sig") as f:
        texto = f.read().splitlines()
    meta, i = {}, 0
    while i < len(texto) and texto[i].startswith("#"):
        corpo = texto[i][1:].strip()
        if corpo.startswith("{"):
            meta = json.loads(corpo)
        elif ";" in corpo:                         # CSV do site: "# Perfil;nome"
            k, v = corpo.split(";", 1)
            meta[k.strip().lower()] = v.strip()
        i += 1
    cab = texto[i]
    linhas = []
    if cab.startswith("tempo_s;"):                 # CSV do site
        meta.setdefault("origem", "site")
        for l in texto[i + 1:]:
            c = l.split(";")
            if len(c) < 5:
                continue
            linhas.append({"pc_s": _num(c[0]), "t_curva": _num(c[0]), "temp": _num(c[1]), "alvo": _num(c[3]),
                           "duty": _num(c[4]), "fase": "ciclo", "estado": "running"})
        return meta, linhas
    nomes = cab.split(",")
    for l in csv.reader(texto[i + 1:]):
        if not l:
            continue
        d = dict(zip(nomes, l))
        for k in nomes:
            if k not in ("fase", "estado"):
                d[k] = _num(d.get(k))
        linhas.append(d)
    return meta, linhas


# ======================================================================
# Métricas de uma curva (as mesmas para logs reais e simulados)
# ======================================================================
def _mediana5(v):
    out = []
    for k in range(len(v)):
        jan = sorted(x for x in v[max(0, k - 2):k + 3] if x is not None)
        out.append(jan[len(jan) // 2] if jan else None)
    return out


def metricas_curva(amostras, pts, liq=None, falha=0):
    """amostras: lista de (t_real, t_curva, temp, duty). pts: curva [[t, T], ...]."""
    am = [a for a in amostras if a[2] is not None and a[1] is not None]
    if len(am) < 5:
        return {"valido": False}
    treal = [a[0] for a in am]
    tc = [a[1] for a in am]
    temp = _mediana5([a[2] for a in am])
    fim = pts[-1][0]
    pico_curva = max(p[1] for p in pts)
    alvo = [interp(pts, x) for x in tc]
    incl = [inclinacao(pts, x) for x in tc]

    # Estar acima da curva só é overshoot depois que a placa a alcançou por baixo.
    # No começo (placa mais quente que o 1º ponto) e logo depois de uma descida (a
    # placa esfria sozinha, devagar) é limite de resfriamento, medido à parte.
    armado, sobe, desce = False, [], []
    for k in range(len(am)):
        if temp[k] <= alvo[k] + 0.5:
            armado = True
        elif incl[k] < -1e-6:
            armado = False
        if tc[k] >= fim:
            continue
        if incl[k] < -1e-6:
            desce.append(k)
        elif armado and tc[k] > 1:
            sobe.append(k)
    # trechos em que a curva está esperando a placa (relógio da curva lento): o atraso
    # ali é limite físico, já contado em tempo_extra; não entra no erro de seguimento
    vel = [1.0] * len(am)
    for k in range(len(am) - 1):
        dtr = treal[k + 1] - treal[k]
        if dtr > 0:
            vel[k] = (tc[k + 1] - tc[k]) / dtr
    erro = [temp[k] - alvo[k] for k in range(len(am))]
    # abaixo da curva com 100% de potência também é limite físico (degraus, rampas rápidas)
    duty = [a[3] or 0 for a in am]
    segue = [k for k in sobe if vel[k] > 0.9 and incl[k] < 5.0 and not (duty[k] >= 99 and erro[k] < 0)]

    tmax = max(temp)
    sobre = max([erro[k] for k in sobe], default=0.0)
    m = {"valido": True}
    m["pico"] = tmax
    m["pico_curva"] = pico_curva
    m["sobrepasso"] = max(0.0, sobre, tmax - pico_curva)            # overshoot (°C)
    m["deficit_pico"] = max(0.0, pico_curva - tmax)                 # não chegou ao pico (°C)
    m["erro_rms"] = math.sqrt(sum(erro[k] ** 2 for k in segue) / len(segue)) if segue else 0.0
    m["atraso_max"] = max([-erro[k] for k in segue], default=0.0)    # maior atraso na subida (°C)
    m["atraso_resfriamento"] = max([erro[k] for k in desce], default=0.0)  # placa acima da curva na descida
    m["tempo_extra"] = max(0.0, (treal[-1] - treal[0]) - fim)        # a curva esperou a placa (s)
    m["falha"] = falha

    # oscilação: patamares planos, depois de 60 s neles
    osc = 0.0
    for a, b in zip(pts, pts[1:]):
        if abs(b[1] - a[1]) < 0.01 and b[0] - a[0] > 90:
            v = [temp[k] for k in range(len(am)) if a[0] + 60 <= tc[k] <= b[0]]
            if len(v) > 4:
                osc = max(osc, max(v) - min(v))
    m["oscilacao"] = osc

    # taxas observadas (°C/s, janela de 5 s)
    taxa_sub, taxa_des = 0.0, 0.0
    j = 0
    for k in range(len(am)):
        while treal[k] - treal[j] > 5:
            j += 1
        if treal[k] - treal[j] >= 4:
            r = (temp[k] - temp[j]) / (treal[k] - treal[j])
            taxa_sub, taxa_des = max(taxa_sub, r), min(taxa_des, r)
    m["taxa_max_subida"] = taxa_sub
    m["taxa_max_descida"] = taxa_des

    if liq:
        dt = [(treal[k + 1] - treal[k]) for k in range(len(am) - 1)] + [0]
        m["tempo_acima_liq"] = sum(dt[k] for k in range(len(am)) if temp[k] >= liq)
        # o que a curva pede
        passo, t, ideal = 0.5, 0.0, 0.0
        while t < fim:
            ideal += passo if interp(pts, t) >= liq else 0
            t += passo
        m["tempo_acima_liq_curva"] = ideal
    return m


def custo_curva(m):
    """Quanto pior a curva, maior. O overshoot pesa mais que o resto."""
    if not m.get("valido"):
        return 1e4
    c = (4.0 * m["sobrepasso"] ** 2 + m["erro_rms"] ** 2 + 3.0 * m["deficit_pico"] ** 2
         + 0.3 * max(0.0, m["atraso_max"] - 6.0) ** 2 + 0.5 * m["oscilacao"] ** 2 + 0.02 * m["tempo_extra"])
    if m.get("falha"):
        c += 1e4
    if "tempo_acima_liq" in m:
        c += 0.02 * (m["tempo_acima_liq"] - m["tempo_acima_liq_curva"]) ** 2
    return c
