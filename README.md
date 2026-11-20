# Introduction_project_IHM
Interface e controle de uma placa de reflow com ESP32.

O site escolhe a curva de temperatura, o ESP32 lê o termopar e aciona a resistência por um relé de estado sólido (SSR), e o site mostra o resultado em tempo real. As mensagens entre os dois estão em [PROTOCOLO.md](PROTOCOLO.md).

| Onde | O quê |
|---|---|
| `src/main.cpp` | Wi-Fi, comandos, leitura do sensor e acionamento do SSR |
| `lib/Controle/` | Lógica do controle e das proteções, sem dependência do Arduino |
| `data/index.html` | O site (vai para a memória da placa) |
| `test/test_controle/` | Testes do controle no PC, com uma placa simulada |
| `bancada/` | Bateria de testes automática na placa real, análise dos logs e otimização do controle ([guia](bancada/README.md)) |

## Materiais

| Item | Função |
|---|---|
| ESP32 LOLIN32 | Controle e site |
| Módulo MAX6675 + termopar tipo K | Temperatura da placa |
| SSR Fotek SSR-40DA (entrada 3–32 V DC) | Liga e desliga a placa |
| Módulo ZMPT101B | Mede a tensão da rede, para o controle se ajustar a ela |
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

O firmware se ajusta à tensão da rede, medida pelo ZMPT101B. Isso fica no botão de engrenagem do site (**Configurações avançadas**):

- **Automática (sensor):** usa a tensão medida, depois de calibrar o sensor (ver [Ajuste do ZMPT101B](#ajuste-do-zmpt101b)). É o padrão.
- **Manual:** 127 V ou 220 V (padrão). Vale também no modo automático enquanto o sensor não tiver leitura válida (sem calibração, solto ou fora de 90 a 260 V).

Nada disso muda a ligação elétrica. A tensão em uso corrige o `pMax` do controle pelo quadrado da tensão, a cada segundo, inclusive durante o ciclo.

### Rede elétrica

```
Fase ──[fusível 6 A]──[termostato/fusível térmico]──[SSR 1 → 2]── placa ── Neutro
Fase ──[fusível 1 A]──┬── HLK-PM12 ──┬── Neutro
                      └──── ZMPT ────┘
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
| LED 1 (vermelho, superfície quente) | GPIO 32 → resistor de 220–330 Ω → LED → GND |
| LED 2 (status) | GPIO 33 → resistor de 220–330 Ω → LED → GND |
| Speaker | GPIO 25 → resistor de 470 Ω → base do BC337 (10 kΩ da base ao GND); speaker entre os 5 V do buck e o coletor; emissor no GND |

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

O módulo fica ligado na rede, antes do SSR (no mesmo ramal da HLK-PM12), e mede a tensão da tomada. Alimentar com 3,3 V mantém a saída dentro do limite do ADC do ESP32.

1. Ligue tudo na tomada e abra **Configurações avançadas** no site. A linha **Sensor** mostra a leitura ao vivo.
2. Se aparecer **onda cortada**, gire o trimpot para baixar até o aviso sumir. Se a sua rede pode ser 127 ou 220 V, ajuste na maior.
3. Meça a tomada com um multímetro em AC, digite o valor em **Calibrar o sensor** e clique em **Calibrar**. O fator fica gravado no ESP32.
4. Se mexer no trimpot depois, calibre de novo.

**Se o módulo não funcionar com 3,3 V:** alguns não funcionam. Alimente com 5 V e reduza a saída com um divisor resistivo antes do GPIO 34, porque o ESP32 não aceita mais de 3,3 V.

**Se ainda não instalou o módulo:** não precisa mudar nada. Sem leitura, o firmware usa a tensão manual. Para esconder o sensor no site, use `USAR_SENSOR_TENSAO = false` em `src/main.cpp`.

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

## LEDs e som

| Aviso | Quando |
|---|---|
| LED 2 piscando | Desde que liga, até um aparelho abrir o site (ou a bancada) pelo Wi-Fi |
| LED 2 aceso | Há um aparelho conectado |
| LED 1 (vermelho) aceso | Superfície a 50 °C ou mais (apaga abaixo de 45 °C), ou termopar com defeito |
| Três notas subindo | A placa ligou |
| Três apitos rápidos | O ciclo terminou (chegou ao fim da curva) |
| Apito a cada 3 s | Enquanto a superfície estiver quente, inclusive durante o ciclo |

Os limites ficam no início de `src/main.cpp` (`TEMP_QUENTE`, `INTERVALO_APITO_MS`).

## Proteções

Todas desligam a resistência e mostram o motivo no site.

| Situação | Quando dispara |
|---|---|
| Termopar solto ou com defeito | 1 s de leituras inválidas, com ou sem ciclo |
| Temperatura excessiva | 315 °C, com ou sem ciclo |
| Temperatura não sobe | Potência ≥ 60%, mais de 5 °C abaixo do alvo e menos de 5 °C de subida em 60 s |
| Esquenta com a resistência desligada (SSR em curto) | Sobe 20 °C acima do mínimo, depois de 30 s desligada. Só sai reiniciando a placa |
| Firmware travado | 5 s sem o loop rodar: o ESP32 reinicia e o SSR desliga |

Depois de um erro, a placa aceita um novo ciclo assim que o problema for resolvido. Queda de Wi-Fi não interrompe o ciclo: o botão **Parar** depende da conexão, então mantenha acesso físico à tomada.

## Como o controle funciona

A cada 250 ms o firmware lê o termopar e calcula a potência:

- **Feed-forward:** usa o modelo da placa para estimar a potência que a curva exige, antecipando o atraso térmico.
- **PI:** corrige o erro que sobrar. A integral só acumula perto da curva (`bandaI`), para não "carregar" nas rampas e passar do ponto depois.
- **Freio preditivo (contra overshoot):** a placa não resfria sozinha rápido, e o calor que já está na resistência continua chegando depois que o SSR desliga. A cada leitura o controle simula o modelo da placa à frente e limita a potência para que, mesmo desligando logo em seguida, a temperatura prevista não passe do maior alvo da curva nos próximos segundos + `margem`.
- **Janela do SSR:** a potência vira a fração ligada de cada janela de 1 s.
- **Curva que espera:** numa subida, se a placa ficar mais de `folga` °C para trás (4 °C de fábrica), o relógio da curva desacelera. Perto do topo de cada subida a folga cai para 1 °C, para a placa chegar ao pico antes de a descida começar. Assim o pico é atingido mesmo em uma placa lenta, e o ciclo pode durar mais que a curva.

## Calibração

Os valores de fábrica em `src/main.cpp` (`PARAM_CONTROLE`) são estimativas, porque ainda não foram medidos em uma placa real. **Calibre antes de soldar qualquer coisa**, começando com a placa vazia e vigiada.

**Automática (recomendada):** a [bancada de testes](bancada/README.md) roda uma bateria de curvas pelo Wi-Fi, identifica o modelo da placa pelos logs e otimiza todos os parâmetros num simulador que usa o mesmo código do firmware.

**Manual:**

1. **`pMax` e `atraso`:** no modo Estático, peça 300 °C e clique em **Parar** ao chegar a uns 150 °C. No CSV baixado, olhe o começo da subida: `pMax = subida em °C/s ÷ (duty ÷ 100)`, e `atraso` é o tempo entre a potência subir e a temperatura começar a reagir. O `pMax` em `src/main.cpp` é o valor **em 220 V**: se calibrar em 127 V, multiplique o que medir por 3.
2. **`perda`:** continue registrando o resfriamento em uma curva terminada em descida. Entre dois instantes, `perda = (queda em °C/s) / (temperatura − ambiente)`.
3. **`kp` e `ki`:** só mexa se precisar. Se oscilar, reduza `kp` pela metade. Se ficar sempre um pouco abaixo ou acima do alvo, aumente `ki`.

Se o seu sensor não for o MAX6675, troque `lerMax6675()` em `src/main.cpp` e `decodificarMax6675()` em `lib/Controle/`.

## Testes sem a placa

```bash
pio test -e native
```

Precisa de um `g++` no PATH (no Windows, por exemplo, o do MSYS2). Roda o controle real contra um modelo térmico da placa, com ruído no sensor, e cobre placas mais fracas e mais fortes que o modelo e as proteções.

Para testar só o site, abra `data/index.html` direto no navegador: ele entra em modo de simulação.
