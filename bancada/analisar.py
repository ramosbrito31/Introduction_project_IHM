"""
Analisa os logs da bancada e otimiza o controle.

    python analisar.py logs/2026-11-18_143000                 (uma bateria)
    python analisar.py logs/pastaA logs/pastaB                (junta várias)
    python analisar.py logs/pasta --iteracoes 400             (otimização mais longa)
    python analisar.py meu_log_do_site.csv --volts 127        (CSV baixado do site também serve)

O que faz:
 1. Identifica o modelo térmico da placa a partir de TODOS os logs (malha aberta e
    fechada): potência, atrasos, perda de calor (linear + quadrática).
 2. Calcula os limites físicos: velocidade máxima de aquecimento e de resfriamento
    em cada temperatura, temperatura máxima, calor acumulado depois de desligar.
    Aponta os trechos das curvas que a placa não consegue seguir.
 3. Mede cada teste real: overshoot, erro, pico, tempo acima do liquidus...
 4. Otimiza os parâmetros do controle no simulador (que roda o código real do
    firmware) contra o modelo identificado e variações dele (±10% de potência,
    ±25% de atraso), priorizando reduzir o overshoot.
 5. Escreve em <pasta>/analise/: relatorio.md (com gráficos), params_sugeridos.json,
    modelo_identificado.json e metricas.json.
"""
import argparse
import glob
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from comum import (AQUI, NOMES_PARAM, PARAM_FABRICA, PARAM_SEM_FREIO, ler_log, simular,
                   metricas_curva, custo_curva, interp)

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PASSO = 0.5          # s: grade da identificação
NOMES_MODELO = ["pMax", "tau1", "k1", "k2", "tau2", "morto", "Ta"]
LIM_MODELO = {"pMax": (0.3, 15), "tau1": (0.3, 150), "k1": (0.0, 0.05), "k2": (0.0, 5e-4),
              "tau2": (0.25, 60), "morto": (0.0, 15), "Ta": (5, 40)}


# ======================================================================
# Leitura
# ======================================================================
class Ensaio:
    def __init__(self, caminho, meta, linhas, volts_padrao):
        self.caminho, self.meta, self.linhas = caminho, meta, linhas
        self.nome = meta.get("nome") or os.path.splitext(os.path.basename(caminho))[0]
        self.tipo = meta.get("tipo") or "curva"
        self.pts = meta.get("pts")
        self.liq = meta.get("liq")
        self.params = meta.get("params")
        if self.pts is None and meta.get("origem") == "site":
            self.tipo = "site"
        bons = [l for l in linhas if l.get("temp") is not None and l.get("pc_s") is not None]
        self.t = np.array([l["pc_s"] for l in bons], float)
        self.temp = np.array([l["temp"] for l in bons], float)
        self.duty = np.array([l.get("duty") or 0.0 for l in bons], float)
        v = []
        for l in bons:
            x = l.get("vac") or l.get("vuse") or volts_padrao
            v.append(x)
        self.volts = np.array(v, float)
        self.fase = [l.get("fase") for l in bons]
        self.tc = [l.get("t_curva") for l in bons]

    def amostras_ciclo(self):
        return [(self.t[k], self.tc[k], self.temp[k], self.duty[k])
                for k in range(len(self.t)) if self.fase[k] == "ciclo" and self.tc[k] is not None]

    def grade(self):
        """Reamostra numa grade uniforme: temperatura interpolada, potência efetiva segurada."""
        if len(self.t) < 10:
            return None
        tg = np.arange(self.t[0], self.t[-1], PASSO)
        temp = np.interp(tg, self.t, self.temp)
        idx = np.clip(np.searchsorted(self.t, tg, side="right") - 1, 0, len(self.t) - 1)
        u = self.duty[idx] / 100.0 * (self.volts[idx] / 220.0) ** 2
        # buracos de telemetria (> 3 s) não entram no ajuste
        valido = np.ones(len(tg), bool)
        lacunas = np.where(np.diff(self.t) > 3.0)[0]
        for g in lacunas:
            valido[(tg > self.t[g]) & (tg < self.t[g + 1])] = False
        return {"t": tg, "temp": temp, "u": u, "valido": valido}


def carregar(entradas, volts):
    ensaios = []
    for e in entradas:
        arquivos = [e] if os.path.isfile(e) else sorted(glob.glob(os.path.join(e, "*.csv")))
        for a in arquivos:
            if os.path.basename(a) == "continuo.csv":
                continue
            try:
                meta, linhas = ler_log(a)
            except Exception as ex:
                print(f"  ignorando {a}: {ex}")
                continue
            en = Ensaio(a, meta, linhas, volts)
            if len(en.t) >= 20:
                ensaios.append(en)
    return ensaios


# ======================================================================
# 1. Identificação do modelo
# ======================================================================
def sim_modelo(th, g):
    pMax, tau1, k1, k2, tau2, morto, Ta = th
    u = g["u"]
    n = len(u)
    # tempo morto fracionário (interpolação linear) para o ajuste ser suave
    atraso = morto / PASSO
    i0 = int(math.floor(atraso))
    f = atraso - i0
    ud = np.zeros(n)
    if i0 < n:
        ud[i0:] = u[:n - i0]
    ud2 = np.zeros(n)
    if i0 + 1 < n:
        ud2[i0 + 1:] = u[:n - i0 - 1]
    ud = (1 - f) * ud + f * ud2
    a1 = 1 - math.exp(-PASSO / tau1)
    a2 = 1 - math.exp(-PASSO / tau2)
    ud = ud.tolist()
    T = Ts = float(g["temp"][0])
    x = 0.0
    out = [Ts]
    for k in range(1, n):
        x += (ud[k - 1] - x) * a1
        d = T - Ta
        T += (pMax * x - k1 * d - k2 * d * abs(d)) * PASSO
        Ts += (T - Ts) * a2
        out.append(Ts)
    return np.array(out)


def identificar(ensaios):
    from scipy.optimize import least_squares
    grades = [(e, e.grade()) for e in ensaios]
    grades = [(e, g) for e, g in grades if g is not None]
    if not grades:
        return None, {}

    # chute inicial a partir dos dados
    tmin = min(float(np.min(g["temp"])) for _, g in grades)
    taxa = 0.0
    for e, g in grades:
        cheio = g["u"] > 0.9
        if cheio.sum() > 20:
            d = np.gradient(g["temp"], PASSO)
            taxa = max(taxa, float(np.percentile(d[cheio], 95)))
    x0 = np.array([max(taxa * 1.2, 1.0), 8.0, 0.007, 1e-5, 2.0, 1.0, min(max(tmin - 1, 10), 35)])
    lo = np.array([LIM_MODELO[n][0] for n in NOMES_MODELO])
    hi = np.array([LIM_MODELO[n][1] for n in NOMES_MODELO])
    x0 = np.clip(x0, lo + 1e-9, hi - 1e-9)
    escala = np.array([1.0, 5.0, 0.005, 1e-5, 2.0, 1.0, 5.0])

    def residuos(th):
        r = []
        for _, g in grades:
            m = sim_modelo(th, g)
            r.append((m - g["temp"])[g["valido"]])
        return np.concatenate(r)

    melhor = None
    for chute in (x0, x0 * np.array([1, 2.5, 1, 1, 2, 2, 1])):   # dois pontos de partida
        chute = np.clip(chute, lo + 1e-9, hi - 1e-9)
        r = least_squares(residuos, chute, bounds=(lo, hi), x_scale=escala, loss="soft_l1", f_scale=2.0,
                          max_nfev=300)
        if melhor is None or r.cost < melhor.cost:
            melhor = r
    th = melhor.x
    modelo = dict(zip(NOMES_MODELO, [float(v) for v in th]))
    qualidade = {}
    for e, g in grades:
        m = sim_modelo(th, g)
        err = (m - g["temp"])[g["valido"]]
        qualidade[e.nome] = {"rmse": float(np.sqrt(np.mean(err ** 2))), "max": float(np.max(np.abs(err))),
                             "t": g["t"] - g["t"][0], "medido": g["temp"], "modelo": m, "u": g["u"]}
    return modelo, qualidade


# ======================================================================
# 2. Limites físicos
# ======================================================================
def perda_modelo(m, T):
    d = T - m["Ta"]
    return m["k1"] * d + m["k2"] * d * abs(d)


def limites(m, volts):
    s = (volts / 220.0) ** 2
    tabela = []
    for T in range(50, 301, 25):
        tabela.append({"T": T, "aquec_max": m["pMax"] * s - perda_modelo(m, T), "resf_max": perda_modelo(m, T)})
    # temperatura de equilíbrio com 100%
    lo, hi = m["Ta"], 1000.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if m["pMax"] * s > perda_modelo(m, mid):
            lo = mid
        else:
            hi = mid
    tmax = lo
    # inércia: 100% até 200 °C (ou até onde der), desliga e mede quanto ainda sobe
    corte = min(200.0, tmax - 20)
    r = simular([{"id": "inercia", "modo": "aberta", "planta": m, "pts": [[0, 100], [1500, 100]], "tmax": corte,
                  "ruido": 0, "vrede": volts, "amostra": 0.25, "sem_monitor": 1}], processos=1)[0]
    temps = [a[2] for a in r["amostras"]]
    k_corte = next((k for k, a in enumerate(r["amostras"]) if a[4] == 0 and k > 4), None)
    inercia = (max(temps) - corte) if k_corte else None
    t_pico = None
    if k_corte:
        kp = int(np.argmax(temps))
        t_pico = r["amostras"][kp][0] - r["amostras"][k_corte][0]
    return {"volts": volts, "tabela": tabela, "temp_max_100": tmax, "inercia_C": inercia,
            "inercia_corte_C": corte, "inercia_t_pico_s": t_pico}


def taxas_observadas(ensaios):
    """Maiores velocidades medidas, por faixa de 25 °C (aquecendo com ≥95%, esfriando com 0%)."""
    faixas = {}
    for e in ensaios:
        g = e.grade()
        if g is None:
            continue
        janela = int(4 / PASSO)
        temp, u = g["temp"], g["u"]
        for k in range(janela, len(temp)):
            taxa = (temp[k] - temp[k - janela]) / (janela * PASSO)
            f = int(temp[k] // 25 * 25)
            d = faixas.setdefault(f, {"aquec": 0.0, "resf": 0.0})
            uj = u[k - janela:k + 1]
            if np.all(uj > 0.95 * (uj.max() if uj.max() > 0 else 1)) and uj.min() > 0.9:
                d["aquec"] = max(d["aquec"], taxa)
            if np.all(uj == 0):
                d["resf"] = max(d["resf"], -taxa)
    return dict(sorted(faixas.items()))


def viabilidade(pts, m, volts, folga=0.85):
    """Trechos que pedem mais velocidade do que a placa tem."""
    s = (volts / 220.0) ** 2
    avisos = []
    for a, b in zip(pts, pts[1:]):
        r = (b[1] - a[1]) / (b[0] - a[0])
        if abs(r) < 1e-6:
            continue
        Ts = np.linspace(a[1], b[1], 12)
        if r > 0:
            cap = min(m["pMax"] * s - perda_modelo(m, T) for T in Ts)
            if r > folga * cap:
                avisos.append(f"{a[1]:.0f}→{b[1]:.0f} °C pede {r:.2f} °C/s de subida; a placa faz no máximo "
                              f"{cap:.2f} °C/s nessa faixa (a curva vai esperar a placa)")
        else:
            cap = min(perda_modelo(m, T) for T in Ts)
            if -r > cap:
                avisos.append(f"{a[1]:.0f}→{b[1]:.0f} °C pede {-r:.2f} °C/s de descida; a placa esfria sozinha só "
                              f"~{cap:.2f} °C/s no fim desse trecho (vai ficar acima da curva; use ventilação)")
    return avisos


# ======================================================================
# 4. Otimização
# ======================================================================
VAR_OTIM = ["pMax", "perda", "atraso", "kp", "ki", "iMax", "margem", "bandaI", "folga"]


def perda_linear(m, T=150.0):
    return perda_modelo(m, T) / (T - m["Ta"])


def faixas_otim(m):
    pl = perda_linear(m)
    return {"pMax": (0.6 * m["pMax"], 1.4 * m["pMax"]), "perda": (0.3 * pl, 2.0 * pl), "atraso": (0.5, 120),
            "kp": (0.3, 40), "ki": (0.005, 2.0), "iMax": (5, 60), "margem": (0.0, 8.0), "bandaI": (3, 80),
            "folga": (1.0, 10.0)}


def variantes(m):
    lento = dict(m, pMax=m["pMax"] * 0.9, tau1=m["tau1"] * 1.25, tau2=m["tau2"] * 1.25, morto=m["morto"] * 1.25)
    rapido = dict(m, pMax=m["pMax"] * 1.1, tau1=m["tau1"] * 0.8, tau2=m["tau2"] * 0.8, morto=m["morto"] * 0.8)
    return [("nominal", m), ("mais lenta", lento), ("mais rápida", rapido)]


def avaliar(params, m, curvas, volts, plantas=None, amostra=0.5, trajetorias=False):
    plantas = plantas or variantes(m)
    trabalhos = []
    for ip, (_, pl) in enumerate(plantas):
        for ic, c in enumerate(curvas):
            trabalhos.append({"id": f"{ip}_{ic}", "modo": "curva", "ctl": params, "planta": pl, "pts": c["pts"],
                              "estado": [35.0, 0.0, 35.0], "ruido": 0.5, "seed": 7 + ic, "vrede": volts,
                              "vctl": volts, "amostra": amostra})
    res = simular(trabalhos)
    saida = []
    for tr, r in zip(trabalhos, res):
        ip, ic = map(int, tr["id"].split("_"))
        c = curvas[ic]
        am = [(a[0], a[1], a[2], a[4]) for a in r["amostras"]]
        met = metricas_curva(am, c["pts"], c.get("liq"), r.get("falha", 0))
        item = {"planta": plantas[ip][0], "curva": c["nome"], "m": met, "custo": custo_curva(met)}
        if trajetorias:
            item["traj"] = r["amostras"]
        saida.append(item)
    return saida


def custo_total(resultados):
    cs = [r["custo"] for r in resultados]
    return float(np.mean(cs) + 0.5 * np.max(cs))


def otimizar(m, curvas, volts, inicio, iteracoes, log=print):
    from scipy.optimize import minimize
    fx = faixas_otim(m)
    loga = {k: k != "margem" for k in VAR_OTIM}

    def para_z(p):
        return np.array([math.log(p[k]) if loga[k] else p[k] / 2.0 for k in VAR_OTIM])

    def de_z(z):
        p = dict(inicio)
        for k, v in zip(VAR_OTIM, z):
            v = math.exp(v) if loga[k] else v * 2.0
            p[k] = float(min(max(v, fx[k][0]), fx[k][1]))
        return p

    cache = {}
    t0 = time.time()
    contagem = [0]

    def f(z):
        p = de_z(z)
        chave = tuple(round(p[k], 6) for k in VAR_OTIM)
        if chave not in cache:
            cache[chave] = custo_total(avaliar(p, m, curvas, volts))
            contagem[0] += 1
            if contagem[0] % 25 == 0:
                log(f"   {contagem[0]} avaliações, melhor custo até agora {min(cache.values()):.1f} "
                    f"({time.time() - t0:.0f} s)")
        return cache[chave]

    z0 = para_z({k: min(max(inicio[k], fx[k][0]), fx[k][1]) for k in VAR_OTIM})
    passo = np.array([0.25 if loga[k] else 0.75 for k in VAR_OTIM])
    simplex = [z0] + [z0 + np.eye(len(z0))[i] * passo[i] for i in range(len(z0))]
    r = minimize(f, z0, method="Nelder-Mead",
                 options={"maxfev": iteracoes, "initial_simplex": np.array(simplex), "xatol": 1e-3, "fatol": 0.05})
    # refinamento: reinicia o simplex em volta do melhor ponto (escapa de travadas do Nelder-Mead)
    if iteracoes > 120:
        z1 = r.x
        simplex = [z1] + [z1 + np.eye(len(z1))[i] * passo[i] * 0.4 for i in range(len(z1))]
        r = minimize(f, z1, method="Nelder-Mead",
                     options={"maxfev": iteracoes // 2, "initial_simplex": np.array(simplex), "xatol": 1e-3,
                              "fatol": 0.02})
    return de_z(r.x), float(r.fun), contagem[0]


# ======================================================================
# 5. Relatório
# ======================================================================
def fmt(v, casas=1):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{casas}f}".replace(".", ",")
    return str(v)


def graficos_identificacao(qualidade, pasta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    nomes = list(qualidade)
    if not nomes:
        return None
    col = 2
    lin = math.ceil(len(nomes) / col)
    fig, axs = plt.subplots(lin, col, figsize=(12, 2.8 * lin), squeeze=False)
    for ax, n in zip(axs.flat, nomes):
        q = qualidade[n]
        ax.plot(q["t"], q["medido"], color="#d9480f", lw=1.2, label="medido")
        ax.plot(q["t"], q["modelo"], color="#1971c2", lw=1.0, ls="--", label="modelo")
        ax2 = ax.twinx()
        ax2.fill_between(q["t"], q["u"] * 100, color="#868e96", alpha=0.15, step="post")
        ax2.set_ylim(0, 400)
        ax2.set_yticks([])
        ax.set_title(f"{n}  (erro rms {q['rmse']:.1f} °C)", fontsize=9)
        ax.set_xlabel("s", fontsize=8)
        ax.set_ylabel("°C", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.3)
    for ax in list(axs.flat)[len(nomes):]:
        ax.axis("off")
    axs.flat[0].legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    caminho = os.path.join(pasta, "identificacao.png")
    fig.savefig(caminho, dpi=90)
    plt.close(fig)
    return "identificacao.png"


def grafico_testes_reais(ensaios, pasta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fechados = [e for e in ensaios if e.tipo in ("curva", "estatico") and e.pts]
    if not fechados:
        return None
    col = 2
    lin = math.ceil(len(fechados) / col)
    fig, axs = plt.subplots(lin, col, figsize=(12, 2.8 * lin), squeeze=False)
    for ax, e in zip(axs.flat, fechados):
        am = e.amostras_ciclo()
        if not am:
            continue
        tr = [a[1] for a in am]      # tempo da curva (ela espera a placa nas subidas)
        ax.plot(tr, [interp(e.pts, a[1]) for a in am], color="#1971c2", lw=1, label="alvo (curva)")
        ax.plot(tr, [a[2] for a in am], color="#d9480f", lw=1.2, label="medido")
        ax2 = ax.twinx()
        ax2.plot(tr, [a[3] for a in am], color="#868e96", lw=0.6, alpha=0.6)
        ax2.set_ylim(0, 400)
        ax2.set_yticks([])
        if e.liq:
            ax.axhline(e.liq, color="#ae3ec9", lw=0.8, ls=":")
        ax.set_title(e.nome + " (eixo x = tempo da curva)", fontsize=9)
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.3)
    for ax in list(axs.flat)[len(fechados):]:
        ax.axis("off")
    axs.flat[0].legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(pasta, "testes_reais.png"), dpi=90)
    plt.close(fig)
    return "testes_reais.png"


def grafico_comparacao(m, curvas, volts, configs, pasta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    col = 2
    lin = math.ceil(len(curvas) / col)
    fig, axs = plt.subplots(lin, col, figsize=(12, 2.8 * lin), squeeze=False)
    cores = ["#868e96", "#f08c00", "#2f9e44"]
    res = {}
    for (nome, p), cor in zip(configs, cores):
        res[nome] = avaliar(p, m, curvas, volts, plantas=[("nominal", m)], amostra=0.5, trajetorias=True)
    for ic, (ax, c) in enumerate(zip(axs.flat, curvas)):
        tt = np.linspace(0, c["pts"][-1][0], 300)
        ax.plot(tt, [interp(c["pts"], x) for x in tt], color="#1971c2", lw=1, ls="--", label="curva")
        for (nome, _), cor in zip(configs, cores):
            tr = res[nome][ic]["traj"]
            ax.plot([a[1] for a in tr], [a[2] for a in tr], color=cor, lw=1.1,
                    label=f"{nome} (overshoot {res[nome][ic]['m'].get('sobrepasso', 0):.1f} °C)")
        ax.set_title(c["nome"] + " (simulado; eixo x = tempo da curva)", fontsize=9)
        ax.legend(fontsize=6.5, loc="lower center")
        ax.tick_params(labelsize=7)
        ax.grid(alpha=0.3)
    for ax in list(axs.flat)[len(curvas):]:
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(os.path.join(pasta, "otimizacao.png"), dpi=90)
    plt.close(fig)
    return "otimizacao.png"


def tabela_metricas(linhas):
    cab = "| Teste | Overshoot °C | Erro rms °C | Maior atraso °C | Pico / curva °C | Acima do liq. s (curva) | Curva esperou s | Oscilação °C |\n|---|---|---|---|---|---|---|---|\n"
    corpo = ""
    for nome, mt in linhas:
        if not mt.get("valido"):
            corpo += f"| {nome} | sem dados | | | | | | |\n"
            continue
        liq = "-"
        if "tempo_acima_liq" in mt:
            liq = f"{mt['tempo_acima_liq']:.0f} ({mt['tempo_acima_liq_curva']:.0f})"
        falha = " **FALHA**" if mt.get("falha") else ""
        corpo += (f"| {nome}{falha} | **{fmt(mt['sobrepasso'])}** | {fmt(mt['erro_rms'])} | {fmt(mt['atraso_max'])} | "
                  f"{fmt(mt['pico'])} / {mt['pico_curva']:.0f} | {liq} | {mt['tempo_extra']:.0f} | {fmt(mt['oscilacao'])} |\n")
    return cab + corpo


# ======================================================================
def curvas_para_otimizar(ensaios):
    vistos, curvas = set(), []
    for e in ensaios:
        if e.tipo in ("curva", "estatico") and e.pts and e.nome not in vistos:
            vistos.add(e.nome)
            curvas.append({"nome": e.nome, "pts": e.pts, "liq": e.liq})
    # completa com os testes de seguimento e perfis da bateria padrão
    with open(os.path.join(AQUI, "bateria.json"), encoding="utf-8") as f:
        bat = json.load(f)
    for t in bat["testes"]:
        if t["tipo"] in ("curva", "estatico") and t.get("grupo") in ("seguimento", "perfis", "limites") \
                and t["nome"] not in vistos:
            vistos.add(t["nome"])
            curvas.append({"nome": t["nome"], "pts": t["pts"], "liq": t.get("liq")})
    return curvas


def analisar(entradas, saida=None, iteracoes=250, volts=220.0):
    t_ini = time.time()
    if isinstance(entradas, str):
        entradas = [entradas]
    saida = saida or os.path.join(entradas[0] if os.path.isdir(entradas[0]) else os.path.dirname(entradas[0]),
                                  "analise")
    os.makedirs(saida, exist_ok=True)
    print(f"Lendo logs de: {', '.join(entradas)}")
    ensaios = carregar(entradas, volts)
    if not ensaios:
        print("Nenhum log encontrado.")
        return None
    print(f"  {len(ensaios)} logs: {', '.join(e.nome for e in ensaios)}")

    v_med = [float(np.median(e.volts)) for e in ensaios if len(e.volts)]
    volts_rede = float(np.median(v_med)) if v_med else volts

    params_usados = next((e.params for e in reversed(ensaios) if e.params), None) or dict(PARAM_FABRICA)
    params_usados = {k: float(params_usados.get(k, PARAM_FABRICA[k])) for k in NOMES_PARAM}

    # 1. modelo
    print("\n1) Identificando o modelo da placa...")
    m, qualidade = identificar(ensaios)
    if m is None:
        print("Dados insuficientes para identificar a placa.")
        return None
    rmse_geral = float(np.mean([q["rmse"] for q in qualidade.values()]))
    print("   " + ", ".join(f"{k}={v:.4g}" for k, v in m.items()) + f"   (erro rms médio {rmse_geral:.2f} °C)")

    # 2. limites
    print("2) Limites de aquecimento e resfriamento...")
    lim = limites(m, volts_rede)
    obs = taxas_observadas(ensaios)
    curvas = curvas_para_otimizar(ensaios)
    avisos_curvas = {c["nome"]: viabilidade(c["pts"], m, volts_rede) for c in curvas}

    # 3. testes reais
    print("3) Métricas dos testes reais...")
    reais = []
    for e in ensaios:
        if e.tipo in ("curva", "estatico") and e.pts:
            reais.append((e.nome, metricas_curva(e.amostras_ciclo(), e.pts, e.liq)))

    # validação do simulador: mesmos testes, mesmos parâmetros, no modelo
    sim_reais = []
    fechados = [e for e in ensaios if e.tipo in ("curva", "estatico") and e.pts]
    if fechados:
        cs = [{"nome": e.nome, "pts": e.pts, "liq": e.liq} for e in fechados]
        r = avaliar(params_usados, m, cs, volts_rede, plantas=[("nominal", m)])
        sim_reais = [(x["curva"], x["m"]) for x in r]

    # 4. otimização
    print(f"4) Otimizando o controle em {len(curvas)} curvas x 3 variações da placa...")
    inicio_ident = dict(params_usados, pMax=m["pMax"], perda=perda_linear(m), tAmb=m["Ta"],
                        atraso=m["tau1"] + m["tau2"] + m["morto"])
    candidatos = [("usados nos testes", params_usados), ("modelo identificado", inicio_ident),
                  ("identificado, mais cauteloso", dict(inicio_ident, kp=params_usados["kp"] * 0.5,
                                                         atraso=inicio_ident["atraso"] * 1.3, margem=1.0))]
    custos = []
    for nome, p in candidatos:
        c = custo_total(avaliar(p, m, curvas, volts_rede))
        custos.append(c)
        print(f"   ponto de partida '{nome}': custo {c:.1f}")
    inicio = candidatos[int(np.argmin(custos))][1]
    inicio = dict(inicio, tAmb=m["Ta"])
    melhor, custo, n_aval = otimizar(m, curvas, volts_rede, inicio, iteracoes)
    custo_usados = custos[0]
    if custo > min(custos):              # nunca sugere algo pior que um ponto de partida
        melhor, custo = candidatos[int(np.argmin(custos))][1], min(custos)
    melhor = {k: float(f"{melhor[k]:.4g}") for k in NOMES_PARAM}
    print(f"   custo: {custo_usados:.1f} (parâmetros usados) -> {custo:.1f} (sugeridos), {n_aval} simulações")

    antigo = dict(PARAM_SEM_FREIO, **{k: params_usados[k] for k in ("pMax", "perda", "tAmb", "atraso", "kp", "ki", "iMax")})
    antigo["margem"], antigo["bandaI"] = -1.0, 300.0
    comparar = [("usados", params_usados), ("sugeridos", melhor)]
    res_cmp = {n: avaliar(p, m, curvas, volts_rede) for n, p in comparar}
    res_antigo = avaliar(antigo, m, curvas, volts_rede)

    # ------------------------------------------------------------------ escrita
    with open(os.path.join(saida, "params_sugeridos.json"), "w", encoding="utf-8") as f:
        json.dump(melhor, f, indent=2)
    with open(os.path.join(saida, "modelo_identificado.json"), "w", encoding="utf-8") as f:
        json.dump(dict(m, volts_rede=volts_rede, erro_rms_medio=rmse_geral), f, indent=2)
    with open(os.path.join(saida, "metricas.json"), "w", encoding="utf-8") as f:
        json.dump({"reais": dict(reais), "sim_reais": dict(sim_reais),
                   "sim": {n: [{"planta": x["planta"], "curva": x["curva"], **x["m"]} for x in r]
                           for n, r in list(res_cmp.items()) + [("antigo_sem_freio", res_antigo)]},
                   "limites": lim, "observado": obs, "avisos_curvas": avisos_curvas,
                   "params_usados": params_usados, "params_sugeridos": melhor}, f, indent=2, ensure_ascii=False)

    print("5) Gráficos e relatório...")
    fig_id = graficos_identificacao(qualidade, saida)
    fig_re = grafico_testes_reais(ensaios, saida)
    fig_ot = grafico_comparacao(m, curvas, volts_rede,
                                [("sem freio (antigo)", antigo), ("usados", params_usados), ("sugeridos", melhor)], saida)

    def resumo_os(res, planta=None):
        v = [x["m"].get("sobrepasso", 0) for x in res if planta is None or x["planta"] == planta]
        return max(v), float(np.mean(v))

    r_usados, r_sug, r_ant = resumo_os(res_cmp["usados"]), resumo_os(res_cmp["sugeridos"]), resumo_os(res_antigo)
    L = []
    L.append("# Análise da bancada de testes\n")
    L.append(f"Logs: {', '.join(os.path.relpath(x, saida) for x in entradas)}  \nGerado em {time.strftime('%d/%m/%Y %H:%M')} "
             f"({time.time() - t_ini:.0f} s de análise, {n_aval} simulações)\n")
    L.append("## Resumo\n")
    L.append(f"- Overshoot simulado no modelo identificado (pior caso / média das curvas, nas 3 variações da placa):")
    L.append(f"  - algoritmo antigo, sem freio: **{fmt(r_ant[0])} / {fmt(r_ant[1])} °C**")
    L.append(f"  - parâmetros usados nos testes: **{fmt(r_usados[0])} / {fmt(r_usados[1])} °C**")
    L.append(f"  - parâmetros sugeridos: **{fmt(r_sug[0])} / {fmt(r_sug[1])} °C**")
    L.append(f"- Custo total (overshoot pesa 4x): {custo_usados:.1f} → {custo:.1f}")
    L.append(f"- Modelo da placa ajustado com erro rms médio de {fmt(rmse_geral, 2)} °C "
             f"({'bom' if rmse_geral < 2 else 'razoável' if rmse_geral < 4 else 'ruim: veja a seção 1'}).\n")
    L.append("### Parâmetros sugeridos\n")
    L.append("| Parâmetro | Usado | Sugerido |\n|---|---|---|")
    for k in NOMES_PARAM:
        L.append(f"| {k} | {params_usados[k]:.4g} | **{melhor[k]:.4g}** |")
    L.append("\nPara testar sem gravar o firmware (vale até reiniciar a placa):\n")
    L.append(f"```\npython rodar_bateria.py --grupos seguimento,perfis --params \"{os.path.join(saida, 'params_sugeridos.json')}\"\n```\n")
    L.append("Para deixar gravado na placa, acrescente `--gravar-params`. Para virar o padrão do firmware, "
             "troque `PARAM_CONTROLE` em `src/main.cpp` por:\n")
    L.append("```cpp\nconst ParamControle PARAM_CONTROLE = {\n" + "".join(
        f"  {melhor[k]:.4g}f,{' ' * max(1, 10 - len(f'{melhor[k]:.4g}'))}// {k}\n" for k in NOMES_PARAM) + "};\n```\n")

    L.append("## 1. Modelo identificado da placa\n")
    L.append("Potência → [tempo morto] → [atraso tau1: resistência] → placa → [atraso tau2: sensor]. "
             "Perda = k1·(T−Ta) + k2·(T−Ta)².\n")
    L.append("| pMax °C/s (220 V) | tau1 s | tau2 s | tempo morto s | k1 1/s | k2 1/(°C·s) | Ta °C |\n|---|---|---|---|---|---|---|")
    L.append(f"| {m['pMax']:.3f} | {m['tau1']:.2f} | {m['tau2']:.2f} | {m['morto']:.2f} | {m['k1']:.5f} | {m['k2']:.2e} | {m['Ta']:.1f} |\n")
    L.append(f"Atraso total ≈ {m['tau1'] + m['tau2'] + m['morto']:.1f} s (o controle de fábrica supõe "
             f"{PARAM_FABRICA['atraso']:.0f} s). Perda linearizada a 150 °C: {perda_linear(m):.4f} 1/s "
             f"(fábrica: {PARAM_FABRICA['perda']}).\n")
    L.append("| Log | Erro rms do modelo °C | Erro máx °C |\n|---|---|---|")
    for n, q in qualidade.items():
        L.append(f"| {n} | {fmt(q['rmse'], 2)} | {fmt(q['max'])} |")
    if fig_id:
        L.append(f"\n![ajuste do modelo]({fig_id})\n")

    L.append(f"## 2. Limites de aquecimento e resfriamento (rede de {volts_rede:.0f} V)\n")
    L.append("| Temperatura °C | Aquecimento máx °C/s (100%) | Resfriamento natural °C/s | Medido: aquecimento | Medido: resfriamento |\n|---|---|---|---|---|")
    for row in lim["tabela"]:
        o = obs.get(row["T"], {})
        L.append(f"| {row['T']} | {fmt(row['aquec_max'], 2)} | {fmt(row['resf_max'], 2)} | "
                 f"{fmt(o.get('aquec') or None, 2)} | {fmt(o.get('resf') or None, 2)} |")
    L.append(f"\n- Temperatura máxima com 100% de potência: **{lim['temp_max_100']:.0f} °C**.")
    if lim["inercia_C"] is not None:
        L.append(f"- Inércia: desligando a 100% em {lim['inercia_corte_C']:.0f} °C, a placa ainda sobe "
                 f"**{lim['inercia_C']:.1f} °C** (pico {lim['inercia_t_pico_s']:.0f} s depois). "
                 "É esse calor acumulado que o freio preditivo antecipa.")
    L.append("- \"Medido\" vem dos logs: a maior velocidade vista (janela de 4 s) entre a temperatura da linha e 25 °C acima.")
    L.append("- A placa não tem resfriamento ativo: descidas mais rápidas que a coluna de resfriamento natural "
             "não são possíveis sem ventilação.\n")
    algum = False
    for nome, av in avisos_curvas.items():
        if av:
            if not algum:
                L.append("### Trechos das curvas fora dos limites da placa\n")
                algum = True
            L.append(f"- **{nome}**: " + "; ".join(av))
    if not algum:
        L.append("Todas as curvas testadas estão dentro dos limites da placa.")
    L.append("")

    L.append("## 3. Testes reais\n")
    if reais:
        L.append(tabela_metricas(reais))
        if fig_re:
            L.append(f"\n![testes reais]({fig_re})\n")
        if sim_reais:
            L.append("### O simulador reproduz a placa?\n")
            L.append("Mesmos testes e parâmetros, no modelo identificado. Se os números forem parecidos com os de cima, "
                     "a otimização no simulador é confiável.\n")
            L.append(tabela_metricas(sim_reais))
    else:
        L.append("Nenhum teste em malha fechada nos logs (só identificação).\n")

    L.append("## 4. Otimização (simulação no modelo identificado)\n")
    L.append("Cada linha é uma curva na placa nominal; entre parênteses, o pior caso entre as 3 variações "
             "(nominal, 10% mais fraca e 25% mais lenta, 10% mais forte e 20% mais rápida).\n")
    L.append("| Curva | Overshoot sem freio | Overshoot usados | Overshoot sugeridos | Erro rms usados → sugeridos | Pico sugeridos / curva | Curva esperou s |\n|---|---|---|---|---|---|---|")
    for c in curvas:
        def pega(res, planta="nominal"):
            return next(x["m"] for x in res if x["curva"] == c["nome"] and x["planta"] == planta)

        def pior(res):
            return max(x["m"].get("sobrepasso", 0) for x in res if x["curva"] == c["nome"])
        a, u, s = pega(res_antigo), pega(res_cmp["usados"]), pega(res_cmp["sugeridos"])
        L.append(f"| {c['nome']} | {fmt(a.get('sobrepasso'))} ({fmt(pior(res_antigo))}) | {fmt(u.get('sobrepasso'))} "
                 f"({fmt(pior(res_cmp['usados']))}) | **{fmt(s.get('sobrepasso'))}** ({fmt(pior(res_cmp['sugeridos']))}) | "
                 f"{fmt(u.get('erro_rms'))} → {fmt(s.get('erro_rms'))} | {fmt(s.get('pico'))} / {s.get('pico_curva', 0):.0f} | "
                 f"{s.get('tempo_extra', 0):.0f} |")
    L.append(f"\n![comparação]({fig_ot})\n")
    L.append("## 5. Próximos passos\n")
    L.append("1. Rode a validação na placa real com os parâmetros sugeridos (comando acima).")
    L.append("2. Analise juntando as duas baterias, para o modelo ficar mais preciso: "
             f"`python analisar.py <pasta_da_identificacao> <pasta_da_validacao>`.")
    L.append("3. Se o overshoot real ficar maior que o simulado, o modelo está otimista: aumente `atraso` "
             "em 20–30% ou `margem` em 1 °C e repita.")
    L.append("4. Quando estiver bom, grave (`--gravar-params`) ou copie para `PARAM_CONTROLE`.")
    with open(os.path.join(saida, "relatorio.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")

    print(f"\nOvershoot simulado (pior/médio): sem freio {r_ant[0]:.1f}/{r_ant[1]:.1f} °C, "
          f"usados {r_usados[0]:.1f}/{r_usados[1]:.1f} °C, sugeridos {r_sug[0]:.1f}/{r_sug[1]:.1f} °C")
    print("Parâmetros sugeridos:", ", ".join(f"{k}={v:g}" for k, v in melhor.items()))
    print(f"Relatório: {os.path.join(saida, 'relatorio.md')}")
    return os.path.join(saida, "params_sugeridos.json")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("entradas", nargs="+", help="pastas de logs da bancada ou arquivos CSV")
    ap.add_argument("--saida", help="pasta do relatório (padrão: <primeira pasta>/analise)")
    ap.add_argument("--iteracoes", type=int, default=250, help="simulações da otimização (padrão 250)")
    ap.add_argument("--volts", type=float, default=220, help="tensão da rede para logs sem medição (CSV do site)")
    a = ap.parse_args()
    analisar(a.entradas, a.saida, a.iteracoes, a.volts)


if __name__ == "__main__":
    main()
