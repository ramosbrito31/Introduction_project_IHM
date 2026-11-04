#include <Arduino.h>

/*
  Placa de Reflow: esqueleto da COMUNICAÇÃO do ESP32 com o site
  ---------------------------------------------------------------
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
  O controle térmico (leitura do sensor, PID, SSR) está marcado como
  "SUA LÓGICA AQUI": só a parte de comunicação está completa.
  O ciclo de aquecimento NÃO depende do Wi-Fi: se a conexão cair, ele continua.

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
struct Ponto { float t; float T; char fase[20]; };
Ponto    curva[20];
int      nPontos = 0;
char     nomePerfil[48] = "";
float    liquidus = 0;
bool     modoEstatico = false;

enum Estado { EST_IDLE, EST_RUNNING, EST_DONE, EST_STOPPED, EST_ERR };
Estado   estado = EST_IDLE;
char     msgErro[48] = "";

uint32_t inicioMs = 0;
float    tempAtual = 25.0;  // °C medida
float    alvo = 0;          // setpoint atual
float    duty = 0;          // 0..100 %

const char* nomeEstado() {
  switch (estado) {
    case EST_RUNNING: return "running";
    case EST_DONE:    return "done";
    case EST_STOPPED: return "stopped";
    case EST_ERR:     return "error";
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
  if (estado == EST_RUNNING) {
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
    mudarEstado(EST_RUNNING);   // a página espera isto em até 4 s
  }
  else if (strcmp(cmd, "stop") == 0) {
    duty = 0;
    // SUA LÓGICA AQUI: desligar a resistência IMEDIATAMENTE
    mudarEstado(EST_STOPPED);
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
  if (estado == EST_RUNNING) {
    float t = tempoCiclo();
    alvo = interpolar(t);
    // SUA LÓGICA AQUI: duty = pid(alvo, tempAtual); acionar SSR/MOSFET
    if (t >= curva[nPontos-1].t) {
      duty = 0;
      mudarEstado(EST_DONE);
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

  iniciarWiFi();   // rede de casa se configurada; senão rede própria "Reflow-Setup"

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html"); // serve index.html.gz automaticamente
  server.begin();
}

void loop() {
  static uint32_t ultimoControle = 0, ultimaTelemetria = 0;
  lerSerial();
  tratarWiFi();

  if (millis() - ultimoControle >= 100) { ultimoControle = millis(); controlar(); }
  if (millis() - ultimaTelemetria >= 250) { ultimaTelemetria = millis(); enviarTelemetria(); }

  ws.cleanupClients();
}

