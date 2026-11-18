# Bancada de testes do controle

Roda uma bateria de curvas na placa real, grava os logs e usa esses logs para
otimizar o controle de potência (duty), com prioridade para reduzir o overshoot.

```
bateria.json ──> rodar_bateria.py ──> logs/<data>/*.csv ──> analisar.py ──> analise/relatorio.md
     (curvas)        (pela placa)          (um por teste)      (modelo +        params_sugeridos.json
                                                                otimização)            │
                         rodar_bateria.py --params analise/params_sugeridos.json  <────┘  (validação)
```

## Antes de começar

1. Grave o firmware novo: `pio run -t upload`. Ele traz o comando `params`, a malha aberta e a telemetria detalhada.
2. Ligue a placa na tomada e conecte o PC na mesma rede Wi-Fi da placa (ou na rede `Reflow-Setup`).
   **Não use o cabo USB com a parte de 220 V ligada** (ver README principal).
3. Python com numpy, scipy e matplotlib (o seu `py` já tem) e o `g++` do MSYS2 no PATH (já está).
   Os comandos abaixo rodam dentro da pasta `bancada/`.
4. Placa **vazia**, termostato de segurança instalado e você por perto durante toda a bateria.

## 1. Rodar a bateria

```powershell
py rodar_bateria.py                                    # tudo (~2 a 3 h, quase tudo esperando esfriar)
py rodar_bateria.py --grupos identificacao,limites     # só o necessário para o modelo e os limites (~1 h)
py rodar_bateria.py --conexao ws://192.168.4.1/ws      # na rede própria da placa
```

Ele pergunta antes de começar, espera a placa esfriar abaixo de 45 °C entre os testes,
mostra o andamento a cada 15 s e grava em `logs/AAAA-MM-DD_HHMMSS/`:

| Arquivo | Conteúdo |
|---|---|
| `NN_<teste>.csv` | Um por teste: 5 s antes, o ciclo e 150 s de resfriamento depois. Colunas: tempo, estado, posição na curva, temperatura, alvo, duty e as partes do controle (feed-forward, P, I, teto do freio). |
| `continuo.csv` | Tudo o que chegou, inclusive as esperas (bom para medir o resfriamento). |
| `params_usados.json`, `bateria_usada.json`, `resumo.json` | O que foi testado, com quais parâmetros e o resultado rápido de cada teste. |

**Segurança:** o script manda `stop` se passar de 275 °C, se a placa entrar em erro, se a telemetria
sumir por 10 s ou se você apertar Ctrl+C. Se o Wi-Fi cair, o firmware termina a curva sozinho e
as proteções dele continuam valendo.

### Os testes (`bateria.json`)

| Grupo | Teste | Para quê |
|---|---|---|
| identificacao | `id_degrau_100` | 100% até 170 °C e desliga: velocidade máxima, atraso e quanto ainda sobe depois de desligar |
| | `id_degrau_40` | 40% por 6 min: linearidade e perda de calor em regime |
| | `id_pulsos` | 3 pulsos de 15 s: separa o atraso da resistência do atraso do sensor |
| limites | `lim_aquecimento_maximo` | 100% até 235 °C e resfriamento livre: limites de aquecer e esfriar em toda a faixa |
| | `lim_rampa_impossivel` | Pede 5 °C/s: a curva precisa esperar a placa sem passar do ponto |
| | `lim_descidas` | Descidas de 0,5 e 1,5 °C/s: até onde o resfriamento natural acompanha |
| seguimento | `seg_degrau_120`, `seg_rampa_1`, `seg_rampa_2`, `seg_escada` | Overshoot ao chegar nos patamares, em várias temperaturas e velocidades |
| perfis | `perfil_sac305`, `perfil_sn63pb37`, `perfil_sn42bi58` | Os perfis do site |

Os testes `id_*` e `lim_aquecimento_maximo` são em **malha aberta**: a potência é programada e
cortada de vez ao atingir `tmax`. Para criar os seus, copie um bloco do JSON: `tipo` é `aberta`
(pontos `[s, %]`), `curva` ou `estatico` (pontos `[s, °C]`); `liq` é opcional.

## 2. Analisar

```powershell
py analisar.py logs\2026-11-18_143000
py analisar.py logs\pastaA logs\pastaB      # junta várias baterias (o modelo fica melhor)
```

Leva de 2 a 4 minutos e escreve em `<pasta>\analise\`:

- `relatorio.md`: abra no VS Code (Ctrl+Shift+V) e veja
  1. o **modelo identificado** da placa (potência, atrasos, perdas) e o quanto ele bate com cada log;
  2. os **limites**: aquecimento máximo e resfriamento natural em cada temperatura, temperatura máxima,
     quanto a placa ainda sobe depois de desligar, e os trechos de cada curva que a placa não consegue seguir;
  3. as métricas de cada teste real (overshoot, erro, pico, tempo acima do liquidus, oscilação);
  4. a comparação no simulador: algoritmo antigo (sem freio) × parâmetros usados × sugeridos.
- `params_sugeridos.json`: os parâmetros otimizados.
- `modelo_identificado.json`, `metricas.json`: os números, para comparar baterias.

A otimização roda o **mesmo código de controle do firmware** (`lib/Controle`), compilado em
`sim/simulador.cpp`, contra o modelo identificado e duas variações dele (placa 10% mais fraca e
mais lenta, e 10% mais forte e mais rápida), para os parâmetros não ficarem ajustados demais para um caso só.

## 3. Validar na placa e gravar

```powershell
py rodar_bateria.py --grupos seguimento,perfis --params logs\<pasta>\analise\params_sugeridos.json
py analisar.py logs\<pasta_da_identificacao> logs\<pasta_da_validacao>
```

`--params` aplica na RAM (vale até reiniciar a placa). Quando estiver bom:

- `--gravar-params` grava na flash da placa, ou
- copie o bloco `PARAM_CONTROLE` do relatório para `src/main.cpp`, para virar o padrão do firmware.

Para voltar ao padrão de fábrica: `{"cmd":"params","reset":true}` (ou regrave o firmware e use `reset`).

## Tudo automático

```powershell
py rodar_bateria.py --auto 2
```

Roda a bateria, analisa, aplica os parâmetros sugeridos, roda os grupos `seguimento` e `perfis`
para validar e repete (2 vezes no exemplo), juntando todos os logs a cada análise. Ele não grava nada na
flash. Ao final, confira o último `relatorio.md` e grave os parâmetros que preferir. Mesmo no automático,
**fique perto da placa**.

## Testar sem a placa

```powershell
py rodar_bateria.py --simular                  # placa virtual: mesma bateria em ~2 s
py rodar_bateria.py --simular --auto 1         # o processo inteiro, ~6 min
py analisar.py logs\<pasta>
```

A placa virtual é de propósito diferente do modelo de fábrica: mais fraca (2,6 °C/s), com atraso de
~18 s e perda não linear. Use `--sim-planta arquivo.json` para outra placa
(`{"pMax":2.6,"tau1":12,"k1":0.0055,"k2":1.6e-5,"tau2":4,"morto":1.5,"Ta":24}`).

## Parâmetros do controle

| Parâmetro | Unidade | O que faz |
|---|---|---|
| `pMax` | °C/s | Velocidade de subida a 100% em 220 V (modelo do feed-forward e do freio) |
| `perda` | 1/s | Perda de calor por °C acima do ambiente |
| `tAmb` | °C | Ambiente |
| `atraso` | s | Atraso térmico do modelo. **O mais importante contra overshoot**: maior = freia antes |
| `kp`, `ki`, `iMax` | %/°C, %/(°C·s), % | PI |
| `margem` | °C | Quanto o freio deixa passar da curva (negativo desliga o freio) |
| `bandaI` | °C | A integral só acumula com o erro menor que isto |
| `folga` | °C | Atraso tolerado antes de a curva esperar a placa (perto do pico cai para 1 °C) |

## Limitações

- O modelo é de uma placa vazia. Com uma placa de circuito em cima, a massa aumenta e o atraso também:
  rode `seguimento` e `perfis` com uma placa de sucata para confirmar.
- Descer mais rápido que o resfriamento natural é impossível sem ventilação: o relatório mostra
  a velocidade disponível em cada temperatura. Nos perfis do site, o trecho de resfriamento quase sempre fica acima da curva.
- Se o overshoot real ficar maior que o simulado, o modelo está otimista: junte mais logs na análise
  ou aumente `atraso` em 20 a 30%.
