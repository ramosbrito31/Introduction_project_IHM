# Introduction_project_IHM
Interface e controle de uma placa de reflow com ESP32.

O site escolhe a curva de temperatura, o ESP32 lê o termopar e aciona a resistência por um relé de estado sólido (SSR), e o site mostra o resultado em tempo real. As mensagens entre os dois estão em [PROTOCOLO.md](PROTOCOLO.md).

| Onde | O quê |
|---|---|
| `src/main.cpp` | Wi-Fi, comandos, leitura do sensor e acionamento do SSR |
| `lib/Controle/` | Lógica do controle e das proteções, sem dependência do Arduino |
| `data/index.html` | O site (vai para a memória da placa) |
| `test/test_controle/` | Testes do controle no PC, com uma placa simulada |

## Materiais

| Item | Função |
|---|---|
| ESP32 LOLIN32 | Controle e site |
| Módulo MAX6675 + termopar tipo K | Temperatura da placa |
| SSR Fotek SSR-40DA (entrada 3–32 V DC) | Liga e desliga a placa |
| Módulo ZMPT101B | Confere se há tensão na placa |
| Fonte HLK-PM12 (12 V, 3 W) | Alimenta a parte de baixa tensão |
| Fusível de vidro 5x20 de 6 A + porta-fusível | Protege contra curto-circuito |
| Placa de aquecimento LJXH 10x15 cm, 220 V | Aquecimento |

**Também é preciso, além da lista original:**

| Item | Por quê |
|---|---|
| Conversor buck de 12 V para 5 V (ex.: MP1584 ou LM2596) | O ESP32 não pode receber 12 V |
| Termostato bimetálico NF ou fusível térmico, para a corrente da placa | Único corte que funciona se o SSR entrar em curto |
| Transistor NPN (ex.: BC337) e resistores de 1 kΩ e 10 kΩ | Aciona o SSR com 12 V, em vez dos 3,3 V do limite mínimo dele |
| Dissipador para o SSR | O SSR esquenta com a corrente da placa |
| Fusível de 1 A para a HLK-PM12 (recomendado) | O de 6 A é grande demais para proteger uma fonte de 3 W |

## Ligações

**Antes de energizar, confirme a tensão da sua tomada:** com uma placa de 220 V em 127 V, a potência cai para cerca de um terço.

### Rede elétrica

```
Fase ──[fusível 6 A]──[termostato/fusível térmico]──[SSR 1 → 2]──┬── placa ──┬── Neutro
                                                                 └── ZMPT ───┘
Fase ──[fusível 1 A]── HLK-PM12 ── Neutro
Terra ── carcaça metálica da placa
```

Em 220 V fase-fase, cada fio é uma fase: interrompa o que passa pelo fusível e pelo SSR da mesma forma, e lembre que a placa continua ligada à rede pelo outro fio mesmo com o SSR desligado.

### Baixa tensão

| De | Para |
|---|---|
| HLK-PM12 +12 V | Entrada do buck e terminal 3 (+) do SSR |
| HLK-PM12 −, buck −, ESP32 GND | Todos juntos (GND comum) |
| Buck 5 V (ajuste **antes** de ligar ao ESP32) | Pino 5V do ESP32 |
| Terminal 4 (−) do SSR | Coletor do transistor |
| Emissor do transistor | GND |
| GPIO 26 | Base do transistor, por um resistor de 1 kΩ |
| Base do transistor | GND, por um resistor de 10 kΩ |
| MAX6675 VCC / GND | 3V3 / GND |
| MAX6675 SCK / SO / CS | GPIO 18 / GPIO 19 / GPIO 23 |
| ZMPT101B VCC / GND | 3V3 / GND |
| ZMPT101B OUT | GPIO 34 |

O resistor de 10 kΩ mantém o SSR desligado enquanto o ESP32 reinicia. Os pinos ficam no início de `src/main.cpp`.

### Cuidados

- **Conexão USB:** não ligue o cabo USB ao PC com a parte de 220 V energizada. Para gravar, desligue a placa da tomada.
- **Aterramento:** o termopar conecta a placa metálica ao GND do ESP32. Sem aterramento, um defeito de isolação na resistência pode energizar todo o circuito.
- **Fiação:** use fios adequados à corrente, com tudo isolado em uma caixa, e passe os fios do termopar longe da rede elétrica.
- **Calor:** a placa queima isolação comum, então use fios e terminais resistentes a alta temperatura na ligação dela.
- **Termostato:** escolha uma temperatura de abertura acima do pico das suas curvas e abaixo do limite da placa, e mantenha-o bem encostado nela.
- **Autenticidade do SSR:** SSRs "Fotek" falsificados são comuns. Trate as especificações como não garantidas e mantenha o corte térmico independente.

### Confirme a potência da placa

Não encontrei a ficha da LJXH, então leia a potência na etiqueta ou meça a resistência a frio com um multímetro: `potência ≈ 220² ÷ resistência`.

- **Fusível:** 6 A em 220 V suportam até cerca de 1320 W. Use um valor em torno de 1,25 vez a corrente da placa (`corrente = potência ÷ 220`) e troque o fusível se a placa passar de uns 1000 W.
- **Temperatura máxima:** confira se a placa aguenta o pico das curvas. Se ela for PTC, ela limita a própria temperatura, e a curva pode não ser alcançada.
- **Segurança:** o fusível só protege contra curto-circuito. Não desliga a placa em caso de superaquecimento nem de SSR em curto.

### Ajuste do ZMPT101B

Alimentar com 3,3 V mantém a saída dentro do limite do ADC do ESP32. Com a placa conectada:

1. Rode um ciclo curto e abra o monitor serial. O campo `vpp` da telemetria mostra a amplitude medida.
2. Ajuste o trimpot até `vpp` ficar entre 1500 e 2500 com a placa energizada, sem deformar a onda.
3. Com o SSR desligado, `vpp` precisa ficar bem abaixo de 400 (`LIMIAR` em `lib/Controle/Controle.h`).

**Se o módulo não funcionar com 3,3 V:** alguns não funcionam. Alimente com 5 V e reduza a saída com um divisor resistivo antes do GPIO 34, porque o ESP32 não aceita mais de 3,3 V.

**Se ainda não instalou o módulo:** use `USAR_SENSOR_TENSAO = false` em `src/main.cpp`. Não energize com a placa desconectada: a fuga do SSR desligado pode parecer tensão para o ZMPT e gerar um alarme falso de curto.

### Segurança

O firmware desliga a resistência quando detecta um defeito, mas se o próprio SSR entrar em curto, a falha mais comum dele, o firmware só consegue avisar. O fusível de 6 A também não abre nesse caso, porque a corrente continua normal. **O termostato ou fusível térmico é obrigatório.** Não deixe a placa ligada sem supervisão.

## Como gravar

```bash
pio run -t upload
```
```bash
pio run -t uploadfs
```

O primeiro comando grava o firmware e o segundo grava o site; sem ele, a placa não entrega a página.

## Proteções

Todas desligam a resistência e mostram o motivo no site.

| Situação | Quando dispara |
|---|---|
| Termopar solto ou com defeito | 1 s de leituras inválidas, com ou sem ciclo |
| Temperatura excessiva | 315 °C, com ou sem ciclo |
| Temperatura não sobe | Potência ≥ 60%, mais de 5 °C abaixo do alvo e menos de 5 °C de subida em 60 s |
| Tensão na placa com o SSR desligado | Cerca de 0,5 s (ZMPT101B). Só sai reiniciando a placa |
| Esquenta com a resistência desligada | Sobe 20 °C acima do mínimo, depois de 30 s desligada. Vale mesmo sem o ZMPT101B. Só sai reiniciando a placa |
| SSR ligado, mas sem tensão na placa | Cerca de 0,5 s em potência alta (fusível queimado, SSR aberto ou falta de rede) |
| Firmware travado | 5 s sem o loop rodar: o ESP32 reinicia e o SSR desliga |

Depois de um erro, a placa aceita um novo ciclo assim que o problema for resolvido. Queda de Wi-Fi não interrompe o ciclo: o botão **Parar** depende da conexão, então mantenha acesso físico à tomada.

## Como o controle funciona

A cada 250 ms o firmware lê o termopar e calcula a potência:

- **Feed-forward:** usa o modelo da placa para estimar a potência que a curva exige, antecipando o atraso térmico.
- **PI:** corrige o erro que sobrar.
- **Janela do SSR:** a potência vira a fração ligada de cada janela de 1 s.
- **Curva que espera:** numa subida, se a placa ficar mais de 4 °C para trás, o relógio da curva desacelera. Assim o pico é atingido mesmo em uma placa lenta, e o ciclo pode durar mais que a curva.

## Calibração

Os valores de fábrica em `src/main.cpp` (`PARAM_CONTROLE`) são estimativas, porque ainda não foram medidos em uma placa real. **Calibre antes de soldar qualquer coisa**, começando com a placa vazia e vigiada:

1. **`pMax` e `atraso`:** no modo Estático, peça 300 °C e clique em **Parar** ao chegar a uns 150 °C. No CSV baixado, olhe o começo da subida: `pMax = subida em °C/s ÷ (duty ÷ 100)`, e `atraso` é o tempo entre a potência subir e a temperatura começar a reagir.
2. **`perda`:** continue registrando o resfriamento em uma curva terminada em descida. Entre dois instantes, `perda = (queda em °C/s) / (temperatura − ambiente)`.
3. **`kp` e `ki`:** só mexa se precisar. Se oscilar, reduza `kp` pela metade. Se ficar sempre um pouco abaixo ou acima do alvo, aumente `ki`.

Se o seu sensor não for o MAX6675, troque `lerMax6675()` em `src/main.cpp` e `decodificarMax6675()` em `lib/Controle/`.

## Testes sem a placa

```bash
pio test -e native
```

Precisa de um `g++` no PATH (no Windows, por exemplo, o do MSYS2). Roda o controle real contra um modelo térmico da placa, com ruído no sensor, e cobre placas mais fracas e mais fortes que o modelo e as proteções.

Para testar só o site, abra `data/index.html` direto no navegador: ele entra em modo de simulação.
