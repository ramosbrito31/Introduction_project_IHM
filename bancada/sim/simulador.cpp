/*
  Simulador da placa de reflow para a bancada de testes.

  Roda o MESMO código de controle do firmware (lib/Controle) contra um modelo
  térmico da placa mais completo que o usado pelo controle:

      potência ──[tempo morto]──[atraso tau1: resistência]──> placa ──[atraso tau2: sensor]──> leitura
      dT/dt = pMax·x − k1·(T−Ta) − k2·(T−Ta)·|T−Ta|

  É compilado e chamado pelos scripts Python (analisar.py e a placa virtual do
  rodar_bateria.py). Não precisa compilar à mão.

  Entrada (stdin): um trabalho por linha, campos chave=valor separados por espaço
      id=nome modo=curva|aberta
      ctl=pMax,perda,tAmb,atraso,kp,ki,iMax,margem,bandaI,folga  (parâmetros do firmware)
      planta=pMax,tau1,k1,k2,tau2,morto,Ta                   (placa "verdadeira")
      pts=t0,T0,t1,T1,...       (curva; em malha aberta T é a potência em %)
      tmax=250                  (malha aberta: corte)
      estado=T,x,Ts             (estado inicial da placa; padrão: parada em Ta)
      ruido=0.5 seed=1          (ruído do sensor, ±°C)
      vrede=220 vctl=220        (tensão real da rede e a que o controle considera)
      amostra=0.25              (período das amostras de saída, s)
      sem_monitor=1             (desliga as proteções; padrão: ligadas, como no firmware)
  Saída (stdout): uma linha JSON por trabalho
      {"id":..,"falha":0,"fim":[T,x,Ts],"amostras":[[t_real,t_curva,temp,alvo,duty,ff,p,i,teto,uf],...]}
*/
#include "../../lib/Controle/Controle.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>

struct Planta {
  float pMax = 3.2f, tau1 = 5, k1 = 0.008f, k2 = 0, tau2 = 0.5f, morto = 0, Ta = 25;
  float T = 25, x = 0, Ts = 25;
  std::vector<float> fila;   // tempo morto
  size_t pos = 0;
  void iniciar(float dt) {
    size_t n = (size_t)(morto / dt + 0.5f);
    fila.assign(n, 0.0f);
    pos = 0;
  }
  void passo(float u, float dt) {
    float ud = u;
    if (!fila.empty()) { ud = fila[pos]; fila[pos] = u; pos = (pos + 1) % fila.size(); }
    x  += (ud - x) * (1.0f - expf(-dt / (tau1 > 1e-3f ? tau1 : 1e-3f)));
    float d = T - Ta;
    T  += (pMax * x - k1 * d - k2 * d * fabsf(d)) * dt;
    Ts += (T - Ts) * (1.0f - expf(-dt / (tau2 > 1e-3f ? tau2 : 1e-3f)));
  }
};

static std::vector<float> lista(const std::string& s) {
  std::vector<float> v;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) if (!item.empty()) v.push_back((float)atof(item.c_str()));
  return v;
}

static uint32_t rng = 1;
static float aleatorio() {   // 0..1, determinístico pela semente
  rng = rng * 1664525u + 1013904223u;
  return (rng >> 8) / 16777216.0f;
}

static void num(std::string& o, float v) {
  char b[32];
  if (isnan(v) || isinf(v)) { o += "null"; return; }
  snprintf(b, sizeof b, "%.4g", v);
  o += b;
}

static void rodar(const std::string& linha) {
  std::string id = "job", modo = "curva";
  ParamControle ctl = {3.2f, 0.008f, 25.0f, 5.0f, 5.0f, 0.4f, 25.0f, 1.0f, 15.0f, 4.0f};
  Planta pl;
  std::vector<float> pts, estado;
  float tmax = 250, ruido = 0.5f, vrede = 220, vctl = 220, amostra = 0.25f;
  bool comMonitor = true;
  rng = 1;

  std::stringstream ss(linha);
  std::string tok;
  while (ss >> tok) {
    size_t eq = tok.find('=');
    if (eq == std::string::npos) continue;
    std::string k = tok.substr(0, eq), v = tok.substr(eq + 1);
    if (k == "id") id = v;
    else if (k == "modo") modo = v;
    else if (k == "ctl") {
      std::vector<float> a = lista(v);
      for (int i = 0; i < N_PARAM && i < (int)a.size(); i++) *campoParam(ctl, i) = a[i];
    }
    else if (k == "planta") {
      std::vector<float> a = lista(v);
      float* c[] = {&pl.pMax, &pl.tau1, &pl.k1, &pl.k2, &pl.tau2, &pl.morto, &pl.Ta};
      for (size_t i = 0; i < 7 && i < a.size(); i++) *c[i] = a[i];
    }
    else if (k == "pts") pts = lista(v);
    else if (k == "estado") estado = lista(v);
    else if (k == "tmax") tmax = (float)atof(v.c_str());
    else if (k == "ruido") ruido = (float)atof(v.c_str());
    else if (k == "seed") rng = (uint32_t)atoi(v.c_str()) * 2654435761u + 1;
    else if (k == "vrede") vrede = (float)atof(v.c_str());
    else if (k == "vctl") vctl = (float)atof(v.c_str());
    else if (k == "amostra") amostra = (float)atof(v.c_str());
    else if (k == "sem_monitor") comMonitor = atoi(v.c_str()) == 0;
  }

  int n = (int)pts.size() / 2;
  if (n > MAX_PONTOS) n = MAX_PONTOS;
  Ponto curva[MAX_PONTOS];
  for (int i = 0; i < n; i++) { curva[i].t = pts[2*i]; curva[i].T = pts[2*i+1]; curva[i].fase[0] = 0; }
  bool aberta = modo == "aberta";
  const char* erro = n < 2 ? "curva vazia" : (aberta ? validarMalhaAberta(curva, n, tmax) : validarCurva(curva, n));
  if (erro) { printf("{\"id\":\"%s\",\"erro\":\"curva invalida\"}\n", id.c_str()); fflush(stdout); return; }

  const float dt = 0.01f;
  pl.T = pl.Ts = pl.Ta; pl.x = 0;
  if (estado.size() >= 1) pl.T = pl.Ts = estado[0];
  if (estado.size() >= 2) pl.x = estado[1];
  if (estado.size() >= 3) pl.Ts = estado[2];
  pl.iniciar(dt);
  float escala = (vrede / TENSAO_NOMINAL_PLACA) * (vrede / TENSAO_NOMINAL_PLACA);

  Controlador controlador(paramParaTensao(ctl, vctl));
  controlador.reiniciar();
  JanelaSSR ssr;
  LeituraSensor sensor;
  MonitorSeguranca monitor;
  bool cortado = false;

  float fim = curva[n-1].t;
  float limiteReal = aberta ? fim + 1 : fim * 3 + 600;   // a curva pode esperar a placa
  float tCurva = 0, duty = 0, alvo = 0;
  int falha = 0;
  std::string o;
  o.reserve(1 << 16);
  o += "{\"id\":\""; o += id; o += "\",\"amostras\":[";
  bool primeira = true;
  int passosCtl = 25;                                    // 250 ms
  int passosAmostra = (int)(amostra / dt + 0.5f);
  if (passosAmostra < 1) passosAmostra = 1;

  for (long k = 0; ; k++) {
    float tReal = k * dt;
    uint32_t ms = (uint32_t)(k * 10);
    if (k % passosCtl == 0) {
      float ruidoAgora = (aleatorio() - 0.5f) * 2.0f * ruido;
      float lido = floorf((pl.Ts + ruidoAgora) / 0.25f) * 0.25f;
      sensor.atualizar(true, lido);
      float temp = sensor.temperatura();
      if (k > 0) {
        if (aberta) {
          tCurva += 0.25f;
          alvo = tmax;
          duty = dutyMalhaAberta(curva, n, tCurva, temp, tmax, cortado);
        } else {
          tCurva += 0.25f * controlador.velocidade(curva, n, tCurva, temp);
          alvo = interpolar(curva, n, tCurva);
          duty = controlador.calcular(curva, n, tCurva, temp, 0.25f);
        }
      } else {
        alvo = aberta ? tmax : interpolar(curva, n, 0);
      }
      if (comMonitor) {
        Falha f = monitor.verificar(ms, sensor.ok(), temp, duty, alvo);
        if (f != FALHA_NENHUMA) { falha = (int)f; duty = 0; }
      }
      if (k % passosAmostra == 0 || falha || tCurva >= fim) {
        const DiagControle& d = controlador.diag();
        if (!primeira) o += ",";
        primeira = false;
        o += "["; num(o, tReal); o += ","; num(o, tCurva); o += ","; num(o, temp); o += ",";
        if (aberta) o += "null"; else num(o, alvo);
        o += ","; num(o, duty);
        if (aberta) o += ",null,null,null,null,null";
        else { o += ","; num(o, d.ff); o += ","; num(o, d.p); o += ","; num(o, d.i); o += ","; num(o, d.teto); o += ","; num(o, d.uf); }
        o += "]";
      }
      if (falha || tCurva >= fim || tReal >= limiteReal) break;
    }
    bool ligado = ssr.atualizar(ms, duty);
    pl.passo(ligado ? escala : 0.0f, dt);
  }
  o += "],\"falha\":"; o += std::to_string(falha);
  o += ",\"fim\":["; num(o, pl.T); o += ","; num(o, pl.x); o += ","; num(o, pl.Ts); o += "]}";
  fputs(o.c_str(), stdout);
  fputc('\n', stdout);
  fflush(stdout);
}

int main() {
  std::string linha;
  while (std::getline(std::cin, linha)) {
    if (linha.empty() || linha[0] == '#') continue;
    rodar(linha);
  }
  return 0;
}
