#include <Arduino.h>

/*
  Placa de Reflow: firmware do ESP32
  ---------------------------------------------------------------
  - Controle: termopar (MAX6675) -> PI + feed-forward -> SSR por janela de tempo.
    A lógica fica em lib/Controle (testada no PC); aqui só o que toca o hardware.
  - Wi-Fi: servidor web serve o index.html (LittleFS) + WebSocket em /ws
  - USB:   as MESMAS mensagens JSON pela Serial, uma por linha
  - Wi-Fi HÍBRIDO com fallback:
      1. Ao ligar, tenta entrar na rede salva (modo Station) por 15 s.
      2. Se não houver rede salva ou não conectar, cria a própria rede
         "Reflow-Setup" (modo Access Point) em 192.168.4.1.
      3. Se a rede de casa cair por mais de 30 s, a rede própria volta
         sozinha (e continua tentando reconectar na de casa).
      A rede de casa é configurada pelo próprio site (botão de Wi-Fi no topo).
      Em qualquer rede, também dá para acessar por http://reflow.local
  O ciclo de aquecimento NÃO depende do Wi-Fi: se a conexão cair, ele continua.
  Ligações e calibração: ver README.md

  Bibliotecas: ESPAsyncWebServer + AsyncTCP, ArduinoJson (v7), LittleFS
  (Preferences e ESPmDNS já vêm com o pacote do ESP32)
*/
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include "Controle.h"

// ---- ligações ----
const int PINO_SSR = 26;   // entrada + do SSR (nível alto liga)
const int PINO_SCK = 18;   // MAX6675 SCK
const int PINO_SO  = 19;   // MAX6675 SO
const int PINO_CS  = 23;   // MAX6675 CS
const int PINO_ZMPT = 34;  // ZMPT101B OUT (precisa ser ADC1: o ADC2 não funciona com Wi-Fi)
const bool USAR_SENSOR_TENSAO = true;  // false enquanto o ZMPT101B não estiver ligado

// ---- modelo da placa: CALIBRE com a sua placa (ver README.md) ----
const ParamControle PARAM_CONTROLE = {
  3.2f,    // pMax   °C/s com 100% de potência
  0.008f,  // perda  1/s
  25.0f,   // tAmb   °C
  5.0f,    // atraso s
  5.0f,    // kp     %/°C
  0.4f,    // ki     %/(°C·s)
  25.0f,   // iMax   %
};
const uint32_t PERIODO_CONTROLE_MS = 250;  // o MAX6675 leva até 220 ms por conversão
const uint32_t WATCHDOG_S = 5;             // loop travado por esse tempo => reinicia (SSR desliga)

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences prefs;

// ---- configuração da rede própria ----
const char* AP_SSID  = "Reflow-Setup";
const char* AP_SENHA = "reflowpoli";     // TROQUE (mínimo 8 caracteres)
const char* HOSTNAME = "reflow";         // => http://reflow.local

String redeSalva, senhaSalva;            // rede de casa (gravada na flash)

// ------------------------------------------------------------------
// Estado do sistema (é isto que vai para o site)
// ------------------------------------------------------------------
Ponto    curva[MAX_PONTOS];
int      nPontos = 0;
char     nomePerfil[48] = "";
float    liquidus = 0;
bool     modoEstatico = false;

enum Estado { EST_IDLE, EST_RUNNING, EST_DONE, EST_STOPPED, EST_ERR };
Estado   estado = EST_IDLE;
char     msgErro[96] = "";

float    tCurva = 0;        // s: relógio da curva (pode andar devagar se a placa atrasar)
float    tempAtual = 0;     // °C medida
float    alvo = 0;          // setpoint atual
float    duty = 0;          // 0..100 %

Controlador      controlador(PARAM_CONTROLE);
JanelaSSR        janelaSSR;
LeituraSensor    sensor;
MonitorSeguranca monitor;
MonitorTensao    monitorTensao;
Falha            falhaAtual = FALHA_NENHUMA;
bool             ssrPreso = false;   // só sai reiniciando a placa

const char* nomeEstado() {
  switch (estado) {
    case EST_RUNNING: return "running";
    case EST_DONE:    return "done";
    case EST_STOPPED: return "stopped";
    case EST_ERR:     return "error";
    default:      return "idle";
  }
}

// ------------------------------------------------------------------
// ENVIO: ESP32 -> site (Wi-Fi e USB ao mesmo tempo)
// ------------------------------------------------------------------
void enviar(JsonDocument& doc) {
  String s;
  serializeJson(doc, s);
  ws.textAll(s);       // todos os navegadores conectados por Wi-Fi
  Serial.println(s);   // site conectado por USB (uma mensagem por linha)
}

void enviarTelemetria() {
  JsonDocument doc;
  doc["type"]  = "telemetry";
  doc["state"] = nomeEstado();
  if (sensor.ok()) doc["temp"] = tempAtual;
  else             doc["temp"] = nullptr;
  doc["duty"]  = roundf(duty);
  if (USAR_SENSOR_TENSAO) doc["vpp"] = monitorTensao.amplitude();   // para ajustar o ZMPT101B
  if (estado == EST_RUNNING) {
    doc["t"]      = tCurva;
    doc["target"] = alvo;
  } else {
    doc["t"] = nullptr;
  }
  if (estado == EST_ERR) doc["msg"] = msgErro;
  enviar(doc);
}

void enviarEstado() {
  JsonDocument doc;
  doc["type"]  = "state";
  doc["state"] = nomeEstado();
  if (estado == EST_ERR) doc["msg"] = msgErro;
  enviar(doc);
}

void enviarPerfil() {   // eco da curva que o controle está seguindo
  JsonDocument doc;
  doc["type"] = "profile";
  doc["name"] = nomePerfil;
  doc["mode"] = modoEstatico ? "static" : "curve";
  if (liquidus > 0) doc["liq"] = liquidus;
  JsonArray pts = doc["pts"].to<JsonArray>();
  for (int i = 0; i < nPontos; i++) {
    JsonArray p = pts.add<JsonArray>();
    p.add(curva[i].t);
    p.add(curva[i].T);
    if (curva[i].fase[0]) p.add(curva[i].fase);
  }
  enviar(doc);
}

void mudarEstado(Estado novo) {
  estado = novo;
  enviarEstado();
}

void rejeitar(const char* msg) {   // comando recusado; o estado não muda
  JsonDocument doc;
  doc["type"] = "reject";
  doc["msg"]  = msg;
  enviar(doc);
}

void desligarSSR() {
  duty = 0;
  digitalWrite(PINO_SSR, LOW);
}

void entrarEmFalha(Falha f) {
  desligarSSR();
  if (f == FALHA_SSR_PRESO) ssrPreso = true;   // inclui o caso detectado pelo sensor de tensão
  if (f == falhaAtual) return;     // o aviso já foi dado
  falhaAtual = f;
  strlcpy(msgErro, textoFalha(f), sizeof(msgErro));
  mudarEstado(EST_ERR);
}

// ------------------------------------------------------------------
// WI-FI HÍBRIDO
// ------------------------------------------------------------------
bool apLigado()  { return WiFi.getMode() & WIFI_MODE_AP; }
bool staLigado() { return WiFi.status() == WL_CONNECTED; }

void ligarAP() {
  if (apLigado()) return;
  WiFi.mode(WIFI_AP_STA);                // AP + STA: permite procurar/testar redes
  WiFi.softAP(AP_SSID, AP_SENHA);
  Serial.printf("# Rede propria ativa: %s  ip %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void iniciarWiFi() {
  prefs.begin("wifi", true);
  redeSalva  = prefs.getString("ssid", "");
  senhaSalva = prefs.getString("pass", "");
  prefs.end();

  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true);

  if (redeSalva.length()) {               // 1. tenta a rede de casa
    WiFi.mode(WIFI_STA);
    WiFi.begin(redeSalva.c_str(), senhaSalva.c_str());
    uint32_t t0 = millis();
    while (!staLigado() && millis() - t0 < 15000) delay(200);
  }
  if (staLigado()) Serial.printf("# Conectado a %s  ip %s\n", redeSalva.c_str(), WiFi.localIP().toString().c_str());
  else             ligarAP();             // 2. fallback: rede própria

  MDNS.begin(HOSTNAME);
  MDNS.addService("http", "tcp", 80);
}

void enviarWifi(const char* evento = nullptr, const char* msg = nullptr) {
  JsonDocument doc;
  doc["type"] = "wifi";
  if (evento) doc["event"] = evento;
  if (msg)    doc["msg"]   = msg;
  doc["host"]  = String(HOSTNAME) + ".local";
  doc["saved"] = redeSalva;
  if (apLigado()) {
    doc["ap"]["ssid"] = AP_SSID;
    doc["ap"]["ip"]   = WiFi.softAPIP().toString();
  }
  if (staLigado()) {
    doc["sta"]["ssid"] = WiFi.SSID();
    doc["sta"]["ip"]   = WiFi.localIP().toString();
    doc["sta"]["rssi"] = WiFi.RSSI();
  }
  enviar(doc);
}

// Pedidos vindos do site. São tratados no loop(), porque escanear e
// conectar demoram e não podem travar o servidor.
volatile bool pedidoScan = false, pedidoConectar = false, pedidoEsquecer = false;
String novaRede, novaSenha;
uint32_t conectandoDesde = 0, reiniciarEm = 0, quedaDesde = 0;

void tratarWiFi() {
  // procurar redes
  if (pedidoScan) {
    pedidoScan = false;
    ligarAP();                            // scan exige o modo STA ativo junto
    WiFi.scanNetworks(true);              // assíncrono
  }
  int n = WiFi.scanComplete();
  if (n >= 0) {
    JsonDocument doc;
    doc["type"] = "wifiScan";
    JsonArray nets = doc["nets"].to<JsonArray>();
    for (int i = 0; i < n && i < 20; i++) {
      JsonObject r = nets.add<JsonObject>();
      r["ssid"]   = WiFi.SSID(i);
      r["rssi"]   = WiFi.RSSI(i);
      r["secure"] = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    }
    WiFi.scanDelete();
    enviar(doc);
  }

  // conectar numa rede nova (mantendo a rede própria no ar enquanto testa)
  if (pedidoConectar) {
    pedidoConectar = false;
    ligarAP();
    WiFi.disconnect(false);
    WiFi.begin(novaRede.c_str(), novaSenha.c_str());
    conectandoDesde = millis();
    enviarWifi("connecting");
  }
  if (conectandoDesde) {
    if (staLigado()) {
      conectandoDesde = 0;
      prefs.begin("wifi", false);
      prefs.putString("ssid", novaRede);
      prefs.putString("pass", novaSenha);
      prefs.end();
      redeSalva = novaRede; senhaSalva = novaSenha;
      enviarWifi("connected");            // o site mostra o novo IP
      reiniciarEm = millis() + 10000;     // dá tempo de anotar o endereço
    } else if (millis() - conectandoDesde > 15000) {
      conectandoDesde = 0;
      WiFi.disconnect(false);
      if (redeSalva.length()) WiFi.begin(redeSalva.c_str(), senhaSalva.c_str());
      enviarWifi("failed", "Não conectou. Confira a senha e o sinal.");
    }
  }

  if (pedidoEsquecer) {
    pedidoEsquecer = false;
    prefs.begin("wifi", false); prefs.clear(); prefs.end();
    redeSalva = ""; senhaSalva = "";
    enviarWifi("forgotten");
    reiniciarEm = millis() + 3000;
  }

  // reinicia para aplicar (nunca no meio de um ciclo)
  if (reiniciarEm && millis() > reiniciarEm && estado != EST_RUNNING) ESP.restart();

  // 3. fallback em funcionamento: rede de casa caiu => liga a rede própria
  if (!apLigado() && !staLigado()) {
    if (!quedaDesde) quedaDesde = millis();
    else if (millis() - quedaDesde > 30000) { ligarAP(); quedaDesde = 0; }
  } else {
    quedaDesde = 0;
  }
}

// ------------------------------------------------------------------
// RECEBIMENTO: site -> ESP32 (mesma função para Wi-Fi e USB)
// ------------------------------------------------------------------
void iniciarCiclo(JsonDocument& doc) {
  if (estado == EST_RUNNING) return rejeitar("Já existe um ciclo em andamento.");
  if (ssrPreso)              return rejeitar(textoFalha(FALHA_SSR_PRESO));
  if (!sensor.ok())          return rejeitar(textoFalha(FALHA_SENSOR));
  if (tempAtual >= MonitorSeguranca::TEMP_CORTE) return rejeitar(textoFalha(FALHA_SOBRETEMP));

  JsonObject prof = doc["profile"];
  JsonArray  pts  = prof["pts"];
  int n = pts.size();
  if (n > MAX_PONTOS) n = MAX_PONTOS + 1;     // validarCurva recusa sem olhar os pontos

  static Ponto nova[MAX_PONTOS];
  if (n <= MAX_PONTOS) {
    int i = 0;
    for (JsonArray p : pts) {
      nova[i].t = p[0] | NAN;
      nova[i].T = p[1] | NAN;
      strlcpy(nova[i].fase, p[2] | "", sizeof(nova[i].fase));
      i++;
    }
  }
  const char* problema = validarCurva(nova, n);
  if (problema) return rejeitar(problema);

  memcpy(curva, nova, sizeof(Ponto) * n);
  nPontos = n;
  strlcpy(nomePerfil, prof["name"] | "Perfil", sizeof(nomePerfil));
  liquidus     = prof["liq"] | 0.0f;
  modoEstatico = strcmp(doc["mode"] | "curve", "static") == 0;

  tCurva = 0;
  alvo = interpolar(curva, nPontos, 0);
  controlador.reiniciar();
  monitor.reiniciar();
  monitorTensao.reiniciar();
  falhaAtual = FALHA_NENHUMA;
  enviarPerfil();
  mudarEstado(EST_RUNNING);     // a página espera isto em até 4 s
}

void processarComando(JsonDocument& doc) {
  const char* cmd = doc["cmd"] | "";

  if (strcmp(cmd, "hello") == 0) {
    enviarEstado();
    enviarWifi();
    if (estado == EST_RUNNING) enviarPerfil();
  }
  else if (strcmp(cmd, "wifiStatus") == 0) {
    enviarWifi();
  }
  else if (strcmp(cmd, "wifiScan") == 0) {
    pedidoScan = true;
  }
  else if (strcmp(cmd, "wifiSet") == 0) {
    if (estado == EST_RUNNING) { enviarWifi("busy", "Espere o ciclo terminar para trocar de rede."); return; }
    novaRede  = doc["ssid"] | "";
    novaSenha = doc["pass"] | "";
    if (novaRede.length()) pedidoConectar = true;
  }
  else if (strcmp(cmd, "wifiForget") == 0) {
    if (estado == EST_RUNNING) { enviarWifi("busy", "Espere o ciclo terminar para trocar de rede."); return; }
    pedidoEsquecer = true;
  }
  else if (strcmp(cmd, "start") == 0) {
    iniciarCiclo(doc);
  }
  else if (strcmp(cmd, "stop") == 0) {
    desligarSSR();
    if (estado == EST_RUNNING) mudarEstado(EST_STOPPED);
    else enviarEstado();        // não apaga um erro nem um "concluído"
  }
}

// Wi-Fi: as mensagens chegam em outra tarefa (a do servidor). Só vão para a fila;
// quem processa é o loop(), para o estado do controle ter um único dono.
QueueHandle_t filaComandos;

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type != WS_EVT_DATA) return;
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    char* copia = (char*)malloc(len + 1);
    if (!copia) return;
    memcpy(copia, data, len);
    copia[len] = 0;
    if (xQueueSend(filaComandos, &copia, 0) != pdTRUE) free(copia);
  }
}

void lerFilaWs() {
  char* msg;
  while (xQueueReceive(filaComandos, &msg, 0) == pdTRUE) {
    JsonDocument doc;
    if (deserializeJson(doc, (const char*)msg) == DeserializationError::Ok) processarComando(doc);
    free(msg);
  }
}

// USB: lê a Serial linha a linha
void lerSerial() {
  static String linha;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      JsonDocument doc;
      if (deserializeJson(doc, linha) == DeserializationError::Ok) processarComando(doc);
      linha = "";
    } else if (c != '\r' && linha.length() < 1500) {
      linha += c;
    }
  }
}

// ------------------------------------------------------------------
// CONTROLE
// ------------------------------------------------------------------
uint16_t lerMax6675() {
  uint16_t v = 0;
  digitalWrite(PINO_CS, LOW);
  delayMicroseconds(1);
  for (int i = 0; i < 16; i++) {
    digitalWrite(PINO_SCK, HIGH);
    delayMicroseconds(1);
    v = (v << 1) | digitalRead(PINO_SO);
    digitalWrite(PINO_SCK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(PINO_CS, HIGH);   // inicia a próxima conversão
  return v;
}

void controlar(float dt) {
  float lido = 0;
  bool valida = decodificarMax6675(lerMax6675(), lido);
  sensor.atualizar(valida, lido);
  tempAtual = sensor.temperatura();

  duty = 0;
  if (estado == EST_RUNNING && sensor.ok()) {
    tCurva += dt * velocidadeCurva(curva, nPontos, tCurva, tempAtual);
    alvo = interpolar(curva, nPontos, tCurva);
    if (tCurva >= curva[nPontos-1].t) mudarEstado(EST_DONE);
    else duty = controlador.calcular(curva, nPontos, tCurva, tempAtual, dt);
  }

  // roda em qualquer estado: sensor solto ou SSR em curto também importam fora do ciclo
  Falha f = monitor.verificar(millis(), sensor.ok(), tempAtual, duty, estado == EST_RUNNING ? alvo : 0);
  if (f == FALHA_NENHUMA && USAR_SENSOR_TENSAO) f = monitorTensao.falha();
  if (f != FALHA_NENHUMA) entrarEmFalha(f);
}

void iniciarWatchdog() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WATCHDOG_S * 1000;
  cfg.trigger_panic = true;
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(WATCHDOG_S, true);
#endif
  esp_task_wdt_add(NULL);   // vigia o loop()
}

// ------------------------------------------------------------------
void setup() {
  // antes de tudo: resistência desligada
  digitalWrite(PINO_SSR, LOW);
  pinMode(PINO_SSR, OUTPUT);
  digitalWrite(PINO_CS, HIGH);
  pinMode(PINO_CS, OUTPUT);
  pinMode(PINO_SCK, OUTPUT);
  pinMode(PINO_SO, INPUT);

  Serial.setRxBufferSize(2048);   // o comando "start" com a curva pode passar de 256 bytes
  Serial.begin(115200);
  LittleFS.begin(true);

  iniciarWiFi();   // rede de casa se configurada; senão rede própria "Reflow-Setup"

  filaComandos = xQueueCreate(8, sizeof(char*));
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html"); // serve index.html.gz automaticamente
  server.begin();
  iniciarWatchdog();
}

void loop() {
  static uint32_t ultimoControle = 0, ultimaTelemetria = 0;
  esp_task_wdt_reset();

  lerSerial();
  lerFilaWs();
  tratarWiFi();

  uint32_t agora = millis();
  if (agora - ultimoControle >= PERIODO_CONTROLE_MS) {
    controlar((agora - ultimoControle) / 1000.0f);
    ultimoControle = agora;
  }
  // chamado a toda volta: é o que forma a janela de tempo do SSR
  bool ssrLigado = janelaSSR.atualizar(millis(), estado == EST_RUNNING ? duty : 0);
  digitalWrite(PINO_SSR, ssrLigado ? HIGH : LOW);

  static uint32_t ultimaAmostra = 0;
  if (USAR_SENSOR_TENSAO && millis() != ultimaAmostra) {
    ultimaAmostra = millis();
    monitorTensao.amostra(ultimaAmostra, analogRead(PINO_ZMPT), ssrLigado);
  }

  if (millis() - ultimaTelemetria >= 250) { ultimaTelemetria = millis(); enviarTelemetria(); }

  ws.cleanupClients();
}
