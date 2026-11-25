"""
Mostra os parâmetros do controle que a placa está usando agora e compara com os
do código (PARAM_CONTROLE em src/main.cpp).

    py ver_params.py                                 (rede Reflow-Setup: ws://192.168.4.1/ws)
    py ver_params.py --conexao ws://reflow.local/ws  (placa na rede de casa)
    py ver_params.py --reset                         (apaga os gravados na flash e volta aos do código)

Se algum valor for diferente do código, é porque foi gravado na flash pela bancada
(--gravar-params) ou aplicado com --params e a placa ainda não reiniciou.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from comum import NOMES_PARAM, conectar, params_do_codigo

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

UNIDADE = {"pMax": "°C/s em 220 V", "perda": "1/s", "tAmb": "°C", "atraso": "s", "kp": "%/°C",
           "ki": "%/(°C·s)", "iMax": "%", "margem": "°C", "bandaI": "°C", "folga": "°C"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conexao", default="ws://192.168.4.1/ws")
    ap.add_argument("--reset", action="store_true", help="volta aos valores do código e apaga os gravados na flash")
    a = ap.parse_args()

    try:
        con = conectar(a.conexao)
    except OSError as e:
        raise SystemExit(f"Não consegui conectar em {a.conexao}: {e}\n"
                         "O PC está na rede da placa (Reflow-Setup)? A placa está ligada?")
    cmd = {"cmd": "params", "reset": True} if a.reset else {"cmd": "params"}
    con.enviar(cmd)
    resposta, fim = None, time.monotonic() + 6
    while time.monotonic() < fim:
        m = con.receber(1)
        if m and m.get("type") == "params" and "p" in m:
            resposta = m
            break
    con.fechar()
    if not resposta:
        raise SystemExit("A placa não respondeu ao comando 'params': o firmware gravado é o da bancada?")

    if resposta.get("event") == "busy":
        print("A placa está no meio de um ciclo: o reset só funciona com ela parada.\n")
    elif a.reset:
        print("Parâmetros gravados na flash apagados: a placa voltou aos valores do código.\n")

    p, codigo = resposta["p"], params_do_codigo()
    print(f"{'parâmetro':10s} {'na placa':>12s} {'no código':>12s}")
    diferentes = []
    for n in NOMES_PARAM:
        v = p.get(n)
        c = codigo.get(n) if codigo else None
        marca = ""
        if v is not None and c is not None and abs(v - c) > 1e-6 * max(1.0, abs(c)):
            marca = "  <- diferente"
            diferentes.append(n)
        vs = "-" if v is None else f"{v:g}"
        cs = "-" if c is None else f"{c:g}"
        print(f"{n:10s} {vs:>12s} {cs:>12s}  {UNIDADE[n]}{marca}")
    if diferentes:
        print(f"\n{', '.join(diferentes)} diferente(s) do código: há valores gravados na flash (ou aplicados com "
              "--params). Para voltar aos do código:  py ver_params.py --reset")
    elif codigo:
        print("\nA placa está usando exatamente os valores do código.")


if __name__ == "__main__":
    main()
