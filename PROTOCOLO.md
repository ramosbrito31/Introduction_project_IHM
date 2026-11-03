# Protocolo de comunicação: ESP32 ↔ Interface Web (placa de reflow)

Transporte: **WebSocket** em `ws://<ip-da-placa>/ws` (ex.: `ws://192.168.4.1/ws` no modo Access Point).
Todas as mensagens são **JSON em texto**, uma por frame. A página reconecta sozinha se a conexão cair.

## Página → ESP32 (comandos)

| Comando | Exemplo | O que o ESP32 deve fazer |
|---|---|---|
| `hello` | `{"cmd":"hello"}` | Enviado ao conectar. Responder com `state`, e, se houver ciclo em andamento, `profile` + `history`. |
| `start` | ver abaixo | Iniciar o ciclo com a curva recebida. Responder com `profile` (eco) e `state: "running"`. |
| `stop` | `{"cmd":"stop"}` | Desligar a resistência imediatamente. Responder `state: "stopped"`. |

```json
{
  "cmd": "start",
  "mode": "curve",
  "profile": {
    "name": "SAC305 (sem chumbo)",
    "liq": 217,
    "pts": [[0,25],[90,150,"Pré-aquec."],[180,180,"Imersão"],[210,217,"Rampa"],
            [240,245,"Refusão"],[270,217,"Refusão"],[360,50,"Resfriamento"]]
  }
}
```

- `pts`: lista de pontos `[tempo_s, temperatura_C, "nome da fase"]`. O setpoint entre dois pontos é **interpolação linear**. O nome da fase vale para o trecho que **termina** naquele ponto (opcional).
- `mode`: `"curve"` (perfil de reflow) ou `"static"` (modo estático: rampa até T e patamar por N minutos, já convertido em pontos pela página).
- Se a página não receber `state: "running"` em 4 s, mostra "A placa não respondeu".

## ESP32 → Página

### `telemetry` (enviar a cada 250–500 ms, **sempre**, inclusive ocioso)
```json
{"type":"telemetry","state":"running","t":123.5,"temp":171.8,"target":172.5,"duty":46,"phase":"Imersão","rate":0.31}
```
| Campo | Obrig. | Descrição |
|---|---|---|
| `temp` | sim | Temperatura medida (°C). |
| `state` | sim | `idle` · `running` · `done` · `stopped` · `error` |
| `t` | durante ciclo | Segundos desde o início do ciclo. Enviar `null` fora do ciclo. |
| `duty` | sim | Esforço da resistência, 0–100 (%). |
| `target` | opcional | Setpoint atual. Se ausente, a página calcula pela curva. |
| `phase` | opcional | Nome da fase. Se ausente, a página deduz pela curva. |
| `rate` | opcional | °C/s. Se ausente, a página calcula (janela de ~3 s). |

A página só plota pontos quando `state == "running"` e `t` cresce. Se `t` voltar para trás, entende como novo ciclo.
Sem telemetria por 4 s → indicador "Sem dados".

### `state` (mudança de estado)
```json
{"type":"state","state":"done"}
{"type":"state","state":"error","msg":"Termopar desconectado"}
```
- `done` → aviso visual + sonoro de término (com opção de mudo).
- `error` → aviso vermelho fixo com `msg`.

### `profile` (curva que a placa está executando)
```json
{"type":"profile","name":"SAC305 (sem chumbo)","liq":217,"mode":"curve","pts":[[0,25],[90,150,"Pré-aquec."]]}
```
Prevalece sobre a curva local: garante que a linha "planejada" do gráfico é exatamente a que o controle está seguindo.

### `history` (opcional: página aberta/recarregada no meio do ciclo)
```json
{"type":"history","samples":[[0,25.1,25,0],[0.5,25.3,25.4,80]]}
```
Cada amostra: `[t, temp, alvo, duty]` (alvo e duty opcionais).

## Regras do lado da página
- Superfície quente: badge pulsante quando `temp ≥ 50 °C` (constante `HOT_C`).
- Log: a página guarda as amostras do ciclo e exporta CSV (`;` e vírgula decimal, compatível com Excel pt-BR).
- Sem bibliotecas externas: tudo funciona no modo AP, sem internet.
