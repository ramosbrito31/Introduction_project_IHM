#include <Arduino.h>

/*
  Placa de Reflow: esqueleto da COMUNICAÇÃO do ESP32 com o site
  ---------------------------------------------------------------
  - Wi-Fi: servidor web serve o index.html (LittleFS) + WebSocket em /ws
  - USB:   as MESMAS mensagens JSON pela Serial, uma por linha
  O controle térmico (leitura do sensor, PID, SSR) está marcado como
  "SUA LÓGICA AQUI": só a parte de comunicação está completa.

  Bibliotecas: ESPAsyncWebServer + AsyncTCP, ArduinoJson (v7), LittleFS
*/
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ------------------------------------------------------------------
// Estado do sistema (é isto que vai para o site)
// ------------------------------------------------------------------
struct Ponto { float t; float T; char fase[20]; };
Ponto    curva[20];
int      nPontos = 0;
char     nomePerfil[48] = "";
float    liquidus = 0;
bool     modoEstatico = false;

enum Estado { IDLE, RUNNING, DONE, STOPPED, ERR };
Estado   estado = IDLE;
char     msgErro[48] = "";

uint32_t inicioMs = 0;
float    tempAtual = 25.0;  // °C medida
float    alvo = 0;          // setpoint atual
float    duty = 0;          // 0..100 %

const char* nomeEstado() {
  switch (estado) {
    case RUNNING: return "running";
    case DONE:    return "done";
    case STOPPED: return "stopped";
    case ERR:     return "error";
    default:      return "idle";
  }
}
float tempoCiclo() { return (millis() - inicioMs) / 1000.0f; }

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
  doc["temp"]  = tempAtual;
  doc["duty"]  = duty;
  if (estado == RUNNING) {
    doc["t"]      = tempoCiclo();
    doc["target"] = alvo;
  } else {
    doc["t"] = nullptr;
  }
  enviar(doc);
}

void enviarEstado() {
  JsonDocument doc;
  doc["type"]  = "state";
  doc["state"] = nomeEstado();
  if (estado == ERR) doc["msg"] = msgErro;
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

// ------------------------------------------------------------------
// RECEBIMENTO: site -> ESP32 (mesma função para Wi-Fi e USB)
// ------------------------------------------------------------------
void processarComando(JsonDocument& doc) {
  const char* cmd = doc["cmd"] | "";

  if (strcmp(cmd, "hello") == 0) {
    enviarEstado();
    if (estado == RUNNING) enviarPerfil();
  }
  else if (strcmp(cmd, "start") == 0) {
    JsonObject prof = doc["profile"];
    JsonArray  pts  = prof["pts"];
    if (pts.isNull() || pts.size() < 2) return;

    strlcpy(nomePerfil, prof["name"] | "Perfil", sizeof(nomePerfil));
    liquidus     = prof["liq"] | 0.0f;
    modoEstatico = strcmp(doc["mode"] | "curve", "static") == 0;
    nPontos = 0;
    for (JsonArray p : pts) {
      if (nPontos >= 20) break;
      curva[nPontos].t = p[0];
      curva[nPontos].T = p[1];
      strlcpy(curva[nPontos].fase, p[2] | "", sizeof(curva[nPontos].fase));
      nPontos++;
    }
    inicioMs = millis();
    enviarPerfil();
    mudarEstado(RUNNING);   // a página espera isto em até 4 s
  }
  else if (strcmp(cmd, "stop") == 0) {
    duty = 0;
    // SUA LÓGICA AQUI: desligar a resistência IMEDIATAMENTE
    mudarEstado(STOPPED);
  }
}

// Wi-Fi: chega uma mensagem pelo WebSocket
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type != WS_EVT_DATA) return;
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    JsonDocument doc;
    if (deserializeJson(doc, data, len) == DeserializationError::Ok) processarComando(doc);
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
// CONTROLE (placeholder)
// ------------------------------------------------------------------
float interpolar(float t) {
  if (t <= curva[0].t) return curva[0].T;
  for (int i = 1; i < nPontos; i++)
    if (t <= curva[i].t)
      return curva[i-1].T + (curva[i].T - curva[i-1].T) * (t - curva[i-1].t) / (curva[i].t - curva[i-1].t);
  return curva[nPontos-1].T;
}

void controlar() {
  // SUA LÓGICA AQUI: tempAtual = lerTermopar();
  if (estado == RUNNING) {
    float t = tempoCiclo();
    alvo = interpolar(t);
    // SUA LÓGICA AQUI: duty = pid(alvo, tempAtual); acionar SSR/MOSFET
    if (t >= curva[nPontos-1].t) {
      duty = 0;
      mudarEstado(DONE);
    }
  } else {
    duty = 0;
  }
}

// ------------------------------------------------------------------
void setup() {
  Serial.setRxBufferSize(2048);   // o comando "start" com a curva pode passar de 256 bytes
  Serial.begin(115200);
  LittleFS.begin(true);

  // Simplificado: só Access Point. O modo híbrido (Station + fallback AP) entra aqui.
  WiFi.softAP("Reflow-Setup");

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html"); // serve index.html.gz automaticamente
  server.begin();
}

void loop() {
  static uint32_t ultimoControle = 0, ultimaTelemetria = 0;
  lerSerial();

  if (millis() - ultimoControle >= 100) { ultimoControle = millis(); controlar(); }
  if (millis() - ultimaTelemetria >= 250) { ultimaTelemetria = millis(); enviarTelemetria(); }

  ws.cleanupClients();
}
