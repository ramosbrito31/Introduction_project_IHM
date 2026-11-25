#include "Controle.h"
#include <math.h>

static float limitar(float v, float a, float b) { return v < a ? a : (v > b ? b : v); }

// ------------------------------------------------------------------
// Curva
// ------------------------------------------------------------------
float interpolar(const Ponto* c, int n, float t) {
  if (t <= c[0].t) return c[0].T;
  for (int i = 1; i < n; i++)
    if (t <= c[i].t)
      return c[i-1].T + (c[i].T - c[i-1].T) * (t - c[i-1].t) / (c[i].t - c[i-1].t);
  return c[n-1].T;
}

const char* validarCurva(const Ponto* c, int n) {
  if (n < 2)          return "A curva precisa de pelo menos 2 pontos.";
  if (n > MAX_PONTOS) return "A curva tem pontos demais (máximo 20).";
  for (int i = 0; i < n; i++) {
    // escrito como !(dentro) para também recusar NaN
    if (!(c[i].T >= 0 && c[i].T <= TEMP_MAX_CURVA)) return "Temperatura fora do limite (0 a 300 °C).";
    if (!(c[i].t >= 0 && c[i].t <= TEMPO_MAX_CURVA)) return "Tempo fora do limite (0 a 7200 s).";
    if (i > 0 && !(c[i].t > c[i-1].t)) return "O tempo precisa sempre aumentar ao longo da curva.";
  }
  return nullptr;
}

float velocidadeCurva(const Ponto* c, int n, float t, float tempC, float folga) {
  // Atraso de regime = folga + parte da FAIXA. Perto do topo de cada subida (o pico
  // do perfil, ou a entrada de um patamar) a folga diminui até FOLGA_TOPO: senão a
  // descida começaria com a placa ainda abaixo do pico e ela nunca chegaria lá.
  const float FAIXA = 4.0f, MINIMA = 0.05f;     // °C, fração
  const float APROX = 15.0f, FOLGA_TOPO = 1.0f; // °C
  float alvo = interpolar(c, n, t);
  int i = 0;
  while (i < n - 1 && c[i].t <= t) i++;                     // c[i]: fim do trecho atual
  // Só espera nas subidas. Olha o trecho atual (e não t + 1 s): senão, no último
  // segundo antes do pico a descida já aparecia e a curva passava do pico sem a placa.
  bool subindo = i > 0 && c[i-1].t <= t && t < c[i].t && c[i].T > c[i-1].T;
  if (!subindo) return 1.0f;
  while (i < n - 1 && c[i+1].T > c[i].T) i++;               // fim desta subida
  float f = folga * limitar((c[i].T - alvo) / APROX, 0.0f, 1.0f);
  if (f < FOLGA_TOPO) f = folga < FOLGA_TOPO ? folga : FOLGA_TOPO;
  return limitar(1.0f - (alvo - tempC - f) / FAIXA, MINIMA, 1.0f);
}

// ------------------------------------------------------------------
// Sensor
// ------------------------------------------------------------------
bool decodificarMax6675(uint16_t bruto, float& tempC) {
  if (bruto == 0x0000 || bruto == 0xFFFF) return false;  // linha presa: módulo solto
  if (bruto & 0x8000) return false;                      // bit 15 é sempre 0
  if (bruto & 0x0002) return false;                      // bit 1 (ID) é sempre 0
  if (bruto & 0x0004) return false;                      // bit 2: termopar aberto
  tempC = (bruto >> 3) * 0.25f;
  return true;
}

void LeituraSensor::atualizar(bool valida, float tempC) {
  // depois de declarar falha, a referência antiga não vale mais: a placa pode ter
  // esfriado enquanto o termopar estava solto
  bool semReferencia = !temValor_ || falhas_ >= FALHAS_MAX;
  bool aceita = valida && (semReferencia || (tempC - temp_ < SALTO_MAX && temp_ - tempC < SALTO_MAX));
  if (aceita) {
    temp_ = tempC;
    temValor_ = true;
    falhas_ = 0;
  } else if (falhas_ < FALHAS_MAX) {
    falhas_++;
  }
}

// ------------------------------------------------------------------
// Controlador
// ------------------------------------------------------------------
bool tensaoValida(int v) { return v == 127 || v == 220; }

ParamControle paramParaTensao(ParamControle p, float volts) {
  if (volts >= TENSAO_MIN && volts <= TENSAO_MAX) {
    float r = volts / TENSAO_NOMINAL_PLACA;
    p.pMax *= r * r;
  }
  return p;
}

const char* const NOMES_PARAM[N_PARAM] = {
  "pMax", "perda", "tAmb", "atraso", "kp", "ki", "iMax", "margem", "bandaI", "folga" };

float* campoParam(ParamControle& p, int i) {
  switch (i) {
    case 0: return &p.pMax;   case 1: return &p.perda;  case 2: return &p.tAmb;
    case 3: return &p.atraso; case 4: return &p.kp;     case 5: return &p.ki;
    case 6: return &p.iMax;   case 7: return &p.margem; case 8: return &p.bandaI;
    case 9: return &p.folga;
    default: return nullptr;
  }
}

const char* validarParam(int i, float v) {
  static const float MIN[N_PARAM] = { 0.1f, 0.0f,  0.0f,   0.1f, 0.0f,   0.0f,  0.0f,  -1.0f,   0.5f,  0.0f };
  static const float MAX[N_PARAM] = { 20.0f, 0.1f, 50.0f, 120.0f, 100.0f, 10.0f, 100.0f, 50.0f, 300.0f, 30.0f };
  if (i < 0 || i >= N_PARAM) return "Parâmetro desconhecido.";
  if (!(v >= MIN[i] && v <= MAX[i])) return "Parâmetro fora da faixa permitida.";
  return nullptr;
}

// Maior duty (%) que pode ser aplicado agora (por uma janela do SSR) sem que a
// temperatura prevista passe da curva + margem, mesmo com a resistência
// desligada logo depois. O modelo é linear, então a previsão é
//     T(τ) = resposta livre(τ) + duty × resposta a 100%(τ)
// e o limite sai direto, sem tentativa e erro.
// A previsão olha a curva em segundos REAIS à frente, mesmo com a curva esperando a
// placa: ela volta a andar assim que a placa alcança. (Com o tempo da curva, uma curva
// parada fazia o freio segurar a placa abaixo do alvo e a espera nunca acabava.)
float Controlador::tetoFreio(const Ponto* c, int n, float t, float T) const {
  const float DT = 0.25f;      // s: passo da previsão
  const float SEGURA = 1.0f;   // s: o duty de agora vale por uma janela do SSR
  float tau = p_.atraso > 0.05f ? p_.atraso : 0.05f;
  float a = 1.0f - expf(-DT / tau);
  float H = limitar(6.0f * tau, 10.0f, 120.0f);
  int passos = (int)(H / DT);

  float Tl = T, xl = uf_;      // resposta livre: desliga agora
  float Ts = 0, xs = 0;        // resposta a 100% por SEGURA s, partindo do zero
  float teto = 1e9f;
  float maxCurva = interpolar(c, n, t);   // maior alvo daqui até τ: depois do pico a
                                           // curva desce, mas chegar ao pico é permitido
  for (int k = 1; k <= passos; k++) {
    float tk = k * DT;
    float us = (tk - DT < SEGURA) ? 1.0f : 0.0f;
    xl += (0.0f - xl) * a;
    xs += (us - xs) * a;
    Tl += (p_.pMax * xl - p_.perda * (Tl - p_.tAmb)) * DT;
    Ts += (p_.pMax * xs - p_.perda * Ts) * DT;
    // não exige baixar (isso é papel do PI): só não deixa subir além da curva
    float a2 = interpolar(c, n, t + tk);
    if (a2 > maxCurva) maxCurva = a2;
    float lim = maxCurva + p_.margem;
    if (lim < T) lim = T;
    if (Ts > 1e-4f) {
      float u = (lim - Tl) / Ts;
      if (u < teto) teto = u;
    }
    if (tk > SEGURA && xl < 0.003f && xs < 0.003f) break;   // nada mais chegando
  }
  return limitar(teto * 100.0f, 0.0f, 100.0f);
}

float Controlador::calcular(const Ponto* c, int n, float t, float tempC, float dt) {
  // modelo interno: a potência aplicada chega à placa com o atraso
  float tau = p_.atraso > 0.05f ? p_.atraso : 0.05f;
  uf_ += (dutyAnt_ / 100.0f - uf_) * (1.0f - expf(-dt / tau));

  // Olha "atraso" segundos à frente: o calor de agora só aparece no sensor depois.
  // São segundos reais: com a curva esperando a placa, isso é menos tempo de curva
  // (senão, perto do pico, enxergaria a descida e cortaria a potência antes da hora).
  float vel     = velocidade(c, n, t, tempC);
  float tAdiante = t + p_.atraso * vel;
  float adiante = interpolar(c, n, tAdiante);
  float subida  = (interpolar(c, n, tAdiante + 1.0f) - adiante) * vel;   // °C/s reais
  float ff = (p_.perda * (adiante - p_.tAmb) + subida) / p_.pMax * 100.0f;

  float erro = interpolar(c, n, t) - tempC;
  float p    = p_.kp * erro;
  float semI = ff + p;
  float teto = p_.margem >= 0 ? tetoFreio(c, n, t, tempC) : 100.0f;

  // anti-windup: não acumula integral com a saída saturada (ou freada) no mesmo
  // sentido do erro, nem longe da curva (rampas e degraus)
  bool saturadoAlto  = semI + integral_ >= teto && erro > 0;
  bool saturadoBaixo = semI + integral_ <= 0.0f && erro < 0;
  bool naBanda       = erro < p_.bandaI && -erro < p_.bandaI;
  if (!saturadoAlto && !saturadoBaixo && naBanda)
    integral_ = limitar(integral_ + p_.ki * erro * dt, -p_.iMax, p_.iMax);

  float u = limitar(semI + integral_, 0.0f, teto);
  dutyAnt_ = u;
  diag_ = DiagControle{ff, p, integral_, teto, uf_};
  return u;
}

// ------------------------------------------------------------------
// Malha aberta
// ------------------------------------------------------------------
const char* validarMalhaAberta(const Ponto* c, int n, float tMax) {
  const char* e = validarCurva(c, n);
  if (e) return e;
  for (int i = 0; i < n; i++)
    if (!(c[i].T <= 100.0f)) return "Malha aberta: potência acima de 100%.";
  if (!(tMax > 0 && tMax <= TEMP_MAX_MALHA_ABERTA)) return "Malha aberta: tmax fora do limite (até 280 °C).";
  return nullptr;
}

float dutyMalhaAberta(const Ponto* c, int n, float t, float tempC, float tMax, bool& cortado) {
  if (tempC >= tMax) cortado = true;
  if (cortado) return 0.0f;
  return limitar(interpolar(c, n, t), 0.0f, 100.0f);
}

// ------------------------------------------------------------------
// SSR
// ------------------------------------------------------------------
bool JanelaSSR::atualizar(uint32_t agoraMs, float duty) {
  if (!(duty > 0)) { iniciada_ = false; return false; }   // desligar é imediato, sem esperar a janela

  if (!iniciada_ || agoraMs - inicio_ >= JANELA_MS) {
    inicio_ = agoraMs;
    iniciada_ = true;
    uint32_t ms = (uint32_t)(limitar(duty, 0.0f, 100.0f) * JANELA_MS / 100.0f);
    if (ms < PULSO_MIN_MS) ms = 0;
    if (JANELA_MS - ms < PULSO_MIN_MS) ms = JANELA_MS;
    ligadoMs_ = ms;
  }
  return agoraMs - inicio_ < ligadoMs_;
}

// ------------------------------------------------------------------
// Proteções
// ------------------------------------------------------------------
const char* textoFalha(Falha f) {
  switch (f) {
    case FALHA_SENSOR:       return "Termopar desconectado ou com defeito.";
    case FALHA_SOBRETEMP:    return "Temperatura acima do limite de segurança.";
    case FALHA_SEM_RESPOSTA: return "A temperatura não sobe: confira resistência, SSR e termopar.";
    case FALHA_SSR_PRESO:    return "Resistência energizada com o SSR desligado: SSR em curto? TIRE DA TOMADA.";
    default:                 return "";
  }
}

void MedidorRede::fecharBloco() {
  if (n_ < AMOSTRAS_MIN) return;
  float media = (float)soma_ / n_;
  float var = (float)somaQ_ / n_ - media * media;
  float rms = var > 0 ? sqrtf(var) : 0;
  saturado_ = min_ <= 5 || max_ >= 4090;     // ADC de 12 bits: 0 a 4095
  rms_ = valida_ ? rms_ + (rms - rms_) * SUAVIZA : rms;
  valida_ = true;
}

void MedidorRede::amostra(uint32_t agora, int adc) {
  if (!iniciado_) { iniciado_ = true; inicio_ = agora; n_ = 0; }
  if (agora - inicio_ >= BLOCO_MS) {
    fecharBloco();
    inicio_ = agora;
    n_ = 0;
  }
  if (n_ == 0) { min_ = max_ = adc; soma_ = somaQ_ = 0; }
  if (adc < min_) min_ = adc;
  if (adc > max_) max_ = adc;
  soma_  += adc;
  somaQ_ += (int64_t)adc * adc;
  n_++;
}

void MonitorSeguranca::reiniciar() {
  subindo_ = false;
  desligada_ = false;
  temMinimo_ = false;
}

Falha MonitorSeguranca::verificar(uint32_t agora, bool sensorOk, float T, float duty, float alvo) {
  if (!sensorOk) return FALHA_SENSOR;
  if (T >= TEMP_CORTE) return FALHA_SOBRETEMP;

  // potência alta, longe do alvo: precisa estar subindo
  if (duty >= DUTY_ALTO && alvo - T > ERRO_MIN) {
    if (!subindo_) { subindo_ = true; subIni_ = agora; subT0_ = T; }
    else if (agora - subIni_ >= JANELA_SUB_MS) {
      if (T - subT0_ < SUBIDA_MIN) return FALHA_SEM_RESPOSTA;
      subIni_ = agora; subT0_ = T;
    }
  } else {
    subindo_ = false;
  }

  // resistência desligada: não pode esquentar
  if (duty < DUTY_BAIXO) {
    if (!desligada_) { desligada_ = true; desIni_ = agora; temMinimo_ = false; }
    else if (agora - desIni_ >= INERCIA_MS) {
      if (!temMinimo_ || T < minimo_) { minimo_ = T; temMinimo_ = true; }
      if (T - minimo_ >= SUBIDA_MAX) return FALHA_SSR_PRESO;
    }
  } else {
    desligada_ = false;
  }
  return FALHA_NENHUMA;
}
