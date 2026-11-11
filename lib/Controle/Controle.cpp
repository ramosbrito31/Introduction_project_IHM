#include "Controle.h"

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

float velocidadeCurva(const Ponto* c, int n, float t, float tempC) {
  // Atraso de regime = FOLGA + parte da FAIXA. Precisa ser pequeno, porque é o
  // quanto a placa fica abaixo do pico quando o trecho de subida termina.
  const float FOLGA = 4.0f, FAIXA = 4.0f, MINIMA = 0.05f;   // °C, °C, fração
  float alvo = interpolar(c, n, t);
  if (interpolar(c, n, t + 1.0f) <= alvo) return 1.0f;      // só espera nas subidas
  return limitar(1.0f - (alvo - tempC - FOLGA) / FAIXA, MINIMA, 1.0f);
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
float Controlador::calcular(const Ponto* c, int n, float t, float tempC, float dt) {
  // Olha "atraso" segundos à frente: o calor de agora só aparece no sensor depois.
  // São segundos reais: com a curva esperando a placa, isso é menos tempo de curva
  // (senão, perto do pico, enxergaria a descida e cortaria a potência antes da hora).
  float vel     = velocidadeCurva(c, n, t, tempC);
  float tAdiante = t + p_.atraso * vel;
  float adiante = interpolar(c, n, tAdiante);
  float subida  = (interpolar(c, n, tAdiante + 1.0f) - adiante) * vel;   // °C/s reais
  float ff = (p_.perda * (adiante - p_.tAmb) + subida) / p_.pMax * 100.0f;

  float erro = interpolar(c, n, t) - tempC;
  float semI = ff + p_.kp * erro;

  // anti-windup: não acumula integral com a saída saturada no mesmo sentido do erro
  bool saturadoAlto  = semI + integral_ >= 100.0f && erro > 0;
  bool saturadoBaixo = semI + integral_ <= 0.0f   && erro < 0;
  if (!saturadoAlto && !saturadoBaixo)
    integral_ = limitar(integral_ + p_.ki * erro * dt, -p_.iMax, p_.iMax);

  return limitar(semI + integral_, 0.0f, 100.0f);
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
    case FALHA_SEM_TENSAO:   return "SSR ligado, mas sem tensão na placa: confira fusível, SSR e rede.";
    default:                 return "";
  }
}

void MonitorTensao::reiniciar() {
  comTensao_ = semTensao_ = 0;
}

void MonitorTensao::fecharBloco() {
  if (n_ < AMOSTRAS_MIN) return;
  amplitude_ = max_ - min_;
  if (descartar_ > 0) { descartar_--; return; }

  bool temTensao = amplitude_ >= LIMIAR;
  if (ultimoSsr_) {
    semTensao_ = temTensao ? 0 : semTensao_ + 1;
  } else {
    comTensao_ = temTensao ? comTensao_ + 1 : 0;
  }
}

void MonitorTensao::amostra(uint32_t agora, int adc, bool ssrLigado) {
  if (!iniciado_) {
    iniciado_ = true;
    ultimoSsr_ = ssrLigado;
    inicio_ = agora;
    n_ = 0;
  }
  if (ssrLigado != ultimoSsr_) {
    // Este bloco ficou misturado. O seguinte também é ignorado, porque o SSR
    // só muda na próxima passagem por zero da rede.
    ultimoSsr_ = ssrLigado;
    descartar_ = 2;
  }
  if (agora - inicio_ >= BLOCO_MS) {
    fecharBloco();
    inicio_ = agora;
    n_ = 0;
  }
  if (n_ == 0) { min_ = max_ = adc; }
  if (adc < min_) min_ = adc;
  if (adc > max_) max_ = adc;
  n_++;
}

Falha MonitorTensao::falha() const {
  if (comTensao_ >= BLOCOS_FALHA) return FALHA_SSR_PRESO;
  if (semTensao_ >= BLOCOS_FALHA) return FALHA_SEM_TENSAO;
  return FALHA_NENHUMA;
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
