"""
Converte uma música (MP3, WAV, FLAC ou OGG) para o formato que o ESP32 toca no
speaker ao fim do ciclo: WAV mono, 8 bits, 16 kHz, gravado em data/musica.wav.

    pip install miniaudio
    python ferramentas/converter_musica.py "C:/Users/.../Downloads/musica.mp3"
    pio run -t uploadfs

Opções:
    --inicio 12.5   começa a música em 12,5 s (para pular uma introdução)
    --max 20        corta em 20 s (padrão 30 s; o arquivo ocupa 16 kB por segundo)
    --taxa 16000    amostras por segundo (8000 a 32000)

O áudio é normalizado (o mais alto possível sem distorcer) e ganha uma entrada e
uma saída suaves, para não estalar no speaker.
"""
import argparse
import array
import os
import sys
import wave

try:
    import miniaudio
except ImportError:
    sys.exit("Falta a biblioteca miniaudio. Instale com:  pip install miniaudio")

PASTA = os.path.dirname(os.path.abspath(__file__))
SAIDA = os.path.normpath(os.path.join(PASTA, "..", "data", "musica.wav"))
FADE_S = 0.02        # entrada e saída suaves
LIMITE_KB = 700      # espaço que sobra no LittleFS (partição de 896 kB, com o site)


def main():
    ap = argparse.ArgumentParser(description="Converte uma música para data/musica.wav")
    ap.add_argument("arquivo")
    ap.add_argument("--inicio", type=float, default=0.0, help="segundo em que a música começa")
    ap.add_argument("--max", type=float, default=30.0, help="duração máxima, em segundos")
    ap.add_argument("--taxa", type=int, default=16000, help="amostras por segundo")
    ap.add_argument("--saida", default=SAIDA)
    a = ap.parse_args()
    if not 8000 <= a.taxa <= 32000:
        sys.exit("--taxa precisa ficar entre 8000 e 32000.")

    som = miniaudio.decode_file(a.arquivo, output_format=miniaudio.SampleFormat.SIGNED16,
                                nchannels=1, sample_rate=a.taxa)
    s = som.samples[int(a.inicio * a.taxa):][:int(a.max * a.taxa)]
    if not len(s):
        sys.exit("Não sobrou áudio: confira --inicio.")

    pico = max(max(s), -min(s)) or 1
    ganho = 127 * 0.98 / pico
    nf = min(int(FADE_S * a.taxa), len(s) // 2)
    out = array.array("B", bytes(len(s)))
    for i, v in enumerate(s):
        f = 1.0
        if i < nf:
            f = i / nf
        elif i >= len(s) - nf:
            f = (len(s) - 1 - i) / nf
        out[i] = max(0, min(255, 128 + round(v * ganho * f)))

    os.makedirs(os.path.dirname(a.saida), exist_ok=True)
    with wave.open(a.saida, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(1)       # 8 bits, sem sinal (padrão do WAV)
        w.setframerate(a.taxa)
        w.writeframes(out.tobytes())

    kb = os.path.getsize(a.saida) / 1024
    print(f"{a.saida}: {len(s) / a.taxa:.1f} s, {a.taxa} Hz, {kb:.0f} kB")
    if kb > LIMITE_KB:
        print(f"ATENÇÃO: passa de {LIMITE_KB} kB e pode não caber na memória da placa. Use --max menor.")
    print("Grave na placa com:  pio run -t uploadfs")


if __name__ == "__main__":
    main()
