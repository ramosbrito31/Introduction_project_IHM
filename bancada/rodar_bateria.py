"""
Roda uma bateria de testes na placa de reflow e grava os logs.

    python rodar_bateria.py                              (Wi-Fi, ws://reflow.local/ws, bateria completa)
    python rodar_bateria.py --conexao ws://192.168.4.1/ws --grupos identificacao,limites
    python rodar_bateria.py --testes perfil_sac305 --params logs/<pasta>/analise/params_sugeridos.json
    python rodar_bateria.py --simular                    (placa virtual, sem hardware: testa o processo)
    python rodar_bateria.py --auto 2                     (bateria -> análise -> aplica parâmetros -> valida, 2 vezes)

Saída: logs/AAAA-MM-DD_HHMMSS/  com um CSV por teste, continuo.csv (tudo), resumo.json,
params_usados.json e bateria_usada.json. Depois:  python analisar.py logs/<pasta>

SEGURANÇA: fique perto da placa durante toda a bateria. O script manda "stop" se a
temperatura passar de temp_max_absoluta, se a placa entrar em erro ou se você
apertar Ctrl+C. Se a conexão cair, o firmware termina a curva sozinho (e as
proteções dele continuam valendo).
"""
import argparse
import datetime
import json
import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

from comum import (AQUI, PARAM_FABRICA, NOMES_PARAM, conectar, simular, salvar_cabecalho,
                   linha_log, metricas_curva)


# ======================================================================
# Placa virtual: fala o mesmo protocolo do firmware, com o simulador em C++
# ======================================================================
PLANTA_VIRTUAL = {   # de propósito diferente do modelo do firmware (mais lenta e com mais atraso)
    "pMax": 2.6, "tau1": 12.0, "k1": 0.0055, "k2": 1.6e-5, "tau2": 4.0, "morto": 1.5, "Ta": 24.0}


class PlacaVirtual:
    PASSO = 0.25

    def __init__(self, planta=None, vrede=220.0, ruido=0.5):
        self.pl = dict(planta or PLANTA_VIRTUAL)
        self.ctl = dict(PARAM_FABRICA)
        self.vrede, self.ruido = vrede, ruido
        self.t = 0.0
        self.T = self.Ts = self.pl["Ta"]
        self.x = 0.0
        self.estado = "idle"
        self.fila = []
        self.traj, self.k, self.info = None, 0, None
        self.semente = 1

    def agora(self):
        return self.t

    def fechar(self):
        pass

    def _estado(self, e, msg=None):
        self.estado = e
        m = {"type": "state", "state": e}
        if msg:
            m["msg"] = msg
        self.fila.append(m)

    def enviar(self, c):
        cmd = c.get("cmd")
        if cmd == "hello":
            self.fila.append({"type": "state", "state": self.estado})
        elif cmd == "params":
            if c.get("set") or c.get("reset") or c.get("save"):
                if self.estado == "running":
                    return self.fila.append({"type": "params", "event": "busy", "p": self.ctl})
                if c.get("reset"):
                    self.ctl = dict(PARAM_FABRICA)
                for k, v in (c.get("set") or {}).items():
                    if k not in NOMES_PARAM:
                        return self.fila.append({"type": "params", "event": "failed", "msg": "Parâmetro desconhecido.", "p": self.ctl})
                    self.ctl[k] = float(v)
                self.fila.append({"type": "params", "event": "saved" if c.get("save") else "applied", "p": dict(self.ctl)})
            else:
                self.fila.append({"type": "params", "p": dict(self.ctl)})
        elif cmd == "start":
            if self.estado == "running":
                return self.fila.append({"type": "reject", "msg": "Já existe um ciclo em andamento."})
            aberta = c.get("mode") == "open"
            tr = {"id": "v", "modo": "aberta" if aberta else "curva", "ctl": self.ctl, "planta": self.pl,
                  "pts": c["profile"]["pts"], "estado": [self.T, self.x, self.Ts], "ruido": self.ruido,
                  "seed": self.semente, "vrede": self.vrede, "vctl": self.vrede, "amostra": self.PASSO}
            if aberta:
                tr["tmax"] = c.get("tmax", 200)
            self.semente += 1
            r = simular([tr], processos=1)[0]
            if "erro" in r:
                return self.fila.append({"type": "reject", "msg": r["erro"]})
            self.traj, self.k, self.info, self.aberta = r["amostras"], 0, r, aberta
            self.fila.append({"type": "profile", "name": c["profile"].get("name"), "pts": c["profile"]["pts"],
                              "mode": c.get("mode")})
            self._estado("running")
        elif cmd == "stop":
            if self.estado == "running":
                a = self.traj[max(0, self.k - 1)]
                self.T = self.Ts = a[2]
                self.x = a[9] if a[9] is not None else 0.0
                self.traj = None
                self._estado("stopped")
            else:
                self.fila.append({"type": "state", "state": self.estado})

    def _resfriar(self, dt):
        p = self.pl
        for _ in range(5):
            h = dt / 5
            self.x += (0 - self.x) * (1 - math.exp(-h / p["tau1"]))
            d = self.T - p["Ta"]
            self.T += (p["pMax"] * self.x - p["k1"] * d - p["k2"] * d * abs(d)) * h
            self.Ts += (self.T - self.Ts) * (1 - math.exp(-h / p["tau2"]))

    def receber(self, timeout=1.0):
        if self.fila:
            return self.fila.pop(0)
        self.t += self.PASSO
        m = {"type": "telemetry", "ms": int(self.t * 1000), "vac": round(self.vrede), "vuse": round(self.vrede)}
        if self.estado == "running" and self.traj is not None:
            a = self.traj[self.k]
            self.k += 1
            m.update({"state": "running", "t": a[1], "temp": a[2], "target": a[3], "duty": round(a[4]), "u": a[4]})
            if not self.aberta:
                m["ctl"] = a[5:10]
            if self.k >= len(self.traj):
                self.T, self.x, self.Ts = self.info["fim"]
                self.traj = None
                if self.info["falha"]:
                    self._estado("error", f"falha {self.info['falha']} (simulada)")
                else:
                    self._estado("done")
            return m
        self._resfriar(self.PASSO)
        leitura = math.floor((self.Ts + random.uniform(-self.ruido, self.ruido)) / 0.25) * 0.25
        m.update({"state": self.estado, "t": None, "temp": leitura, "duty": 0, "u": 0})
        return m


# ======================================================================
# Execução da bateria
# ======================================================================
class Abortar(Exception):
    pass


def esperar(con, cond, tempo, ao_receber=None):
    """Lê mensagens até cond(msg) ser verdadeira ou acabar o tempo. Retorna a msg ou None."""
    fim = con.agora() + tempo
    while con.agora() < fim:
        m = con.receber(timeout=0.5)
        if m is None:
            continue
        if ao_receber:
            ao_receber(m)
        if cond(m):
            return m
    return None


class Bancada:
    def __init__(self, con, pasta, seg, log=print):
        self.con, self.pasta, self.seg, self.log = con, pasta, seg, log
        self.ultima = None             # última telemetria
        self.ultima_em = con.agora()
        self.continuo = open(os.path.join(pasta, "continuo.csv"), "w", encoding="utf-8")
        salvar_cabecalho(self.continuo, {"arquivo": "registro contínuo de toda a bateria"})
        self.arq = None
        self.fase = "espera"
        self.t0 = con.agora()

    def receber(self, timeout=0.5):
        m = self.con.receber(timeout=timeout)
        agora = self.con.agora()
        if m is None:
            if agora - self.ultima_em > self.seg["sem_telemetria_s"]:
                raise Abortar("sem telemetria da placa há %.0f s" % (agora - self.ultima_em))
            return None
        if m.get("type") == "telemetry":
            self.ultima, self.ultima_em = m, agora
            pc = agora - self.t0
            self.continuo.write(linha_log(pc, self.fase, m))
            if self.arq:
                self.arq.write(linha_log(pc, self.fase, m))
            temp = m.get("temp")
            if temp is not None and temp > self.seg["temp_max_absoluta"]:
                self.parar()
                raise Abortar(f"temperatura {temp:.1f} °C acima do limite de {self.seg['temp_max_absoluta']} °C")
        return m

    def parar(self):
        try:
            self.con.enviar({"cmd": "stop"})
        except OSError:
            pass

    def comando(self, cmd, cond, tempo=6.0):
        self.con.enviar(cmd)
        fim = self.con.agora() + tempo
        while self.con.agora() < fim:
            m = self.receber()
            if m is not None and cond(m):
                return m
        return None

    def esperar_esfriar(self, limite, max_s=3600):
        """Espera a placa ficar ociosa e abaixo do limite."""
        inicio, aviso = self.con.agora(), 0
        while True:
            self.receber()
            u = self.ultima or {}
            temp, est = u.get("temp"), u.get("state")
            if est in ("idle", "done", "stopped") and temp is not None and temp <= limite:
                return
            if est == "error":
                raise Abortar("a placa está em erro: " + str(u.get("msg")))
            agora = self.con.agora()
            if agora - aviso >= 30:
                aviso = agora
                self.log(f"   esfriando: {temp} °C (começa abaixo de {limite} °C)")
            if agora - inicio > max_s:
                raise Abortar("a placa não esfriou a tempo")

    def rodar_teste(self, idx, teste, params):
        nome = teste["nome"]
        caminho = os.path.join(self.pasta, f"{idx:02d}_{nome}.csv")
        self.log(f"\n[{idx:02d}] {nome}: {teste.get('descricao', '')}")
        limite = teste.get("temp_inicio_max", self.seg["temp_inicio_max"])
        self.fase = "espera"
        self.esperar_esfriar(limite)

        meta = {k: teste.get(k) for k in ("nome", "grupo", "tipo", "descricao", "pts", "tmax", "liq")}
        meta["params"] = params
        meta["inicio"] = datetime.datetime.now().isoformat(timespec="seconds")
        self.arq = open(caminho, "w", encoding="utf-8")
        salvar_cabecalho(self.arq, meta)

        # alguns segundos parada antes, para o log ter o ponto de partida
        self.fase = "pre"
        fim = self.con.agora() + 5
        while self.con.agora() < fim:
            self.receber()

        modo = {"aberta": "open", "estatico": "static"}.get(teste["tipo"], "curve")
        cmd = {"cmd": "start", "mode": modo, "profile": {"name": "bancada: " + nome, "pts": teste["pts"]}}
        if teste.get("liq"):
            cmd["profile"]["liq"] = teste["liq"]
        if modo == "open":
            cmd["tmax"] = teste["tmax"]
        self.fase = "ciclo"
        r = self.comando(cmd, lambda m: m.get("type") == "reject" or
                         (m.get("type") in ("state", "telemetry") and m.get("state") == "running"))
        if r is None or r.get("type") == "reject":
            self.arq.close()
            self.arq = None
            motivo = (r or {}).get("msg", "a placa não respondeu ao start")
            self.log("   RECUSADO: " + motivo)
            return {"nome": nome, "status": "recusado", "msg": motivo, "arquivo": os.path.basename(caminho)}

        # ciclo
        t_ini, ultimo_aviso, final = self.con.agora(), 0, None
        while True:
            m = self.receber()
            u = self.ultima or {}
            if m and m.get("type") in ("state", "telemetry") and m.get("state") not in (None, "running"):
                final = m.get("state")
                break
            agora = self.con.agora()
            if agora - ultimo_aviso >= 15 and u:
                ultimo_aviso = agora
                alvo = u.get("target")
                self.log(f"   t={u.get('t') or 0:6.0f} s  temp={u.get('temp')}  alvo={alvo if alvo is not None else '-'}  duty={u.get('duty')}%")
        dur = self.con.agora() - t_ini
        msg = (self.ultima or {}).get("msg")
        if final == "error":
            self.arq.close()
            self.arq = None
            raise Abortar(f"a placa entrou em erro no teste {nome}: {msg}")

        # pós-ciclo: registra o resfriamento
        self.fase = "pos"
        fim = self.con.agora() + teste.get("pos_s", self.seg["pos_ciclo_s"])
        while self.con.agora() < fim:
            self.receber()
        self.arq.close()
        self.arq = None
        self.log(f"   fim ({final}) em {dur:.0f} s")
        return {"nome": nome, "status": final, "duracao_s": round(dur, 1), "arquivo": os.path.basename(caminho)}


def resumo_rapido(pasta, teste, res):
    """Métricas principais logo depois do teste (a análise completa é no analisar.py)."""
    if teste["tipo"] == "aberta" or res.get("status") != "done":
        return
    from comum import ler_log
    _, linhas = ler_log(os.path.join(pasta, res["arquivo"]))
    am = [(l["pc_s"], l["t_curva"], l["temp"], l["duty"]) for l in linhas if l["fase"] == "ciclo"]
    m = metricas_curva(am, teste["pts"], teste.get("liq"))
    if m.get("valido"):
        res["metricas"] = {k: round(v, 2) for k, v in m.items() if isinstance(v, (int, float))}
        print(f"   overshoot {m['sobrepasso']:.1f} °C | erro rms {m['erro_rms']:.1f} °C | "
              f"pico {m['pico']:.1f}/{m['pico_curva']:.0f} °C | curva esperou {m['tempo_extra']:.0f} s")


def selecionar(bateria, grupos, testes):
    sel = bateria["testes"]
    if grupos:
        g = set(grupos.split(","))
        sel = [t for t in sel if t.get("grupo") in g]
    if testes:
        n = testes.split(",")
        desconhecidos = set(n) - {t["nome"] for t in bateria["testes"]}
        if desconhecidos:
            raise SystemExit("Testes desconhecidos: " + ", ".join(sorted(desconhecidos)))
        sel = [t for t in bateria["testes"] if t["nome"] in n]
    return sel


def rodar_bateria(con, bateria, sel, params_novos=None, gravar=False, saida=None, confirmar=True):
    seg = dict(bateria.get("seguranca", {}))
    seg.setdefault("temp_max_absoluta", 275)
    seg.setdefault("temp_inicio_max", 45)
    seg.setdefault("pos_ciclo_s", 150)
    seg.setdefault("sem_telemetria_s", 10)

    pasta = saida or os.path.join(AQUI, "logs", datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S"))
    os.makedirs(pasta, exist_ok=True)
    b = Bancada(con, pasta, seg)

    try:
        if not b.comando({"cmd": "hello"}, lambda m: m.get("type") == "telemetry", 10):
            raise Abortar("a placa não está mandando telemetria")
        if (b.ultima or {}).get("state") == "running":
            raise Abortar("já existe um ciclo em andamento na placa")

        if params_novos:
            cmd = {"cmd": "params", "set": params_novos}
            if gravar:
                cmd["save"] = True
            r = b.comando(cmd, lambda m: m.get("type") == "params" and "event" in m)
            if not r or r.get("event") not in ("applied", "saved"):
                raise Abortar("a placa não aceitou os parâmetros: " + str((r or {}).get("msg", "sem resposta")) +
                              " (o firmware gravado é a versão com o comando 'params'?)")
            print("Parâmetros aplicados" + (" e gravados na flash" if gravar else " (até reiniciar a placa)"))
        r = b.comando({"cmd": "params"}, lambda m: m.get("type") == "params" and "p" in m)
        if not r:
            raise Abortar("a placa não respondeu ao comando 'params': grave o firmware novo (pio run -t upload)")
        params = r["p"]
        with open(os.path.join(pasta, "params_usados.json"), "w", encoding="utf-8") as f:
            json.dump(params, f, indent=2)
        with open(os.path.join(pasta, "bateria_usada.json"), "w", encoding="utf-8") as f:
            json.dump(dict(bateria, testes=sel), f, indent=2, ensure_ascii=False)

        estimativa = sum(t["pts"][-1][0] * 1.2 + seg["pos_ciclo_s"] + 300 for t in sel) / 60
        print(f"\n{len(sel)} testes, cerca de {estimativa:.0f} min (depende de quanto a placa demora para esfriar).")
        print("Parâmetros em uso:", ", ".join(f"{k}={v:g}" for k, v in params.items()))
        print("Logs em:", pasta)
        if confirmar:
            print("\nFique perto da placa durante toda a bateria. Ctrl+C para tudo e desliga a resistência.")
            if input("Começar? [s/N] ").strip().lower() not in ("s", "sim", "y"):
                raise Abortar("cancelado")

        resultados = []
        status = "concluida"
        for i, t in enumerate(sel, 1):
            res = b.rodar_teste(i, t, params)
            resumo_rapido(pasta, t, res)
            resultados.append(res)
    except Abortar as e:
        b.parar()
        print("\nBATERIA INTERROMPIDA:", e)
        status, resultados = f"interrompida: {e}", locals().get("resultados", [])
    except KeyboardInterrupt:
        b.parar()
        print("\nInterrompido (Ctrl+C): mandei 'stop' para a placa.")
        status, resultados = "interrompida pelo usuário", locals().get("resultados", [])
    except OSError as e:
        print(f"\nCONEXÃO PERDIDA ({e}). Se havia um ciclo, o firmware termina a curva sozinho; "
              "confira a placa.")
        status, resultados = f"interrompida: conexão perdida ({e})", locals().get("resultados", [])
    finally:
        if b.arq:
            b.arq.close()
        b.continuo.close()

    with open(os.path.join(pasta, "resumo.json"), "w", encoding="utf-8") as f:
        json.dump({"status": status, "testes": resultados}, f, indent=2, ensure_ascii=False)
    print(f"\nBateria {status}. Logs em: {pasta}")
    return pasta, status


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conexao", default="ws://reflow.local/ws",
                    help="ws://reflow.local/ws (padrão), ws://<ip>/ws ou porta USB (COM5). Prefira o Wi-Fi.")
    ap.add_argument("--bateria", default=os.path.join(AQUI, "bateria.json"))
    ap.add_argument("--grupos", help="ex.: identificacao,limites,seguimento,perfis")
    ap.add_argument("--testes", help="nomes separados por vírgula")
    ap.add_argument("--params", help="JSON com parâmetros do controle para aplicar antes (ex.: params_sugeridos.json)")
    ap.add_argument("--gravar-params", action="store_true", help="grava os parâmetros na flash da placa")
    ap.add_argument("--saida", help="pasta dos logs (padrão: logs/<data>)")
    ap.add_argument("--simular", action="store_true", help="usa a placa virtual (sem hardware)")
    ap.add_argument("--sim-planta", help="JSON com a placa virtual (pMax, tau1, k1, k2, tau2, morto, Ta)")
    ap.add_argument("--sim-volts", type=float, default=220, help="tensão da rede da placa virtual")
    ap.add_argument("--auto", type=int, default=0,
                    help="depois da bateria, repete N vezes: analisa todos os logs, aplica os parâmetros sugeridos e valida")
    ap.add_argument("--grupos-validacao", default="seguimento,perfis")
    ap.add_argument("-y", "--sem-confirmar", action="store_true", help="não pergunta antes de começar")
    a = ap.parse_args()

    with open(a.bateria, encoding="utf-8") as f:
        bateria = json.load(f)
    sel = selecionar(bateria, a.grupos, a.testes)
    if not sel:
        raise SystemExit("Nenhum teste selecionado.")

    params = None
    if a.params:
        with open(a.params, encoding="utf-8") as f:
            params = json.load(f)
        params = {k: params[k] for k in NOMES_PARAM if k in params}

    if a.simular:
        planta = None
        if a.sim_planta:
            with open(a.sim_planta, encoding="utf-8") as f:
                planta = json.load(f)
        con = PlacaVirtual(planta, vrede=a.sim_volts)
        print("PLACA VIRTUAL (simulação): nenhum hardware é usado.")
    else:
        if not a.conexao.startswith("ws://"):
            print("AVISO: USB com a parte de 220 V ligada é perigoso nesta placa (ver README). Prefira o Wi-Fi.")
        try:
            con = conectar(a.conexao)
        except OSError as e:
            raise SystemExit(f"Não consegui conectar em {a.conexao}: {e}")

    confirmar = not (a.simular or a.sem_confirmar)
    try:
        pastas = []
        pasta, status = rodar_bateria(con, bateria, sel, params, a.gravar_params, a.saida, confirmar)
        pastas.append(pasta)
        for rodada in range(1, a.auto + 1):
            if not status.startswith("concluida"):
                break
            import analisar
            print(f"\n===== Otimização automática: rodada {rodada}/{a.auto} =====")
            sug = analisar.analisar(pastas, os.path.join(pasta, "analise"))
            if not sug:
                print("A análise não gerou parâmetros novos; parando.")
                break
            with open(sug, encoding="utf-8") as f:
                novos = json.load(f)
            novos = {k: novos[k] for k in NOMES_PARAM}
            sel_v = selecionar(bateria, a.grupos_validacao, None)
            pasta, status = rodar_bateria(con, bateria, sel_v, novos, False, None, False)
            pastas.append(pasta)
        if a.auto:
            import analisar
            analisar.analisar(pastas, os.path.join(pasta, "analise"))
    finally:
        con.fechar()


if __name__ == "__main__":
    main()
