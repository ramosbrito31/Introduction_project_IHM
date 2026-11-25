/*
  Testes da biblioteca Controle no PC, sem a placa:
      pio test -e native -v

  A "placa" aqui é um modelo térmico (mesmo modelo da simulação do site):
  subida proporcional à potência com atraso, e perda proporcional à diferença
  para o ambiente. O sensor lê a cada 250 ms, com passos de 0,25 °C e ruído.
*/
#include <unity.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Controle.h"

// Mesmos valores padrão do src/main.cpp
static const ParamControle PARAM = { 3.2f, 0.008f, 25.0f, 5.0f, 5.0f, 0.4f, 25.0f, 1.0f, 15.0f, 4.0f };

static const Ponto SAC305[] = {
  {0,25,""},{90,150,""},{180,180,""},{210,217,""},{240,245,""},{270,217,""},{360,50,""}};
static const Ponto SN42[] = {
  {0,25,""},{60,90,""},{150,120,""},{170,138,""},{195,165,""},{225,138,""},{300,50,""}};
static const Ponto ESTATICO[] = {{0,25,""},{104,150,""},{704,150,""}};
#define N(c) ((int)(sizeof(c)/sizeof(c[0])))

struct Planta {
  float pMax, perda, tau;
  float T = 25, uf = 0;
  float extraCs = 0;     // °C/s vindos de fora (SSR em curto)
  void passo(bool ligada, float dt) {
    uf += ((ligada ? 1.0f : 0.0f) - uf) * dt / tau;
    T  += (pMax * uf - perda * (T - 25.0f) + extraCs) * dt;
  }
};

struct Resultado { float erroMaxSubida; float pico; float tAcimaLiq; float erroPatamar; Falha falha; float tFalha; float duracao; };

static float sensorLe(float T) {
  float ruido = (rand() % 100 - 50) / 100.0f;      // ±0,5 °C
  return floor((T + ruido) / 0.25f) * 0.25f;
}

// Roda um ciclo inteiro em passos de 10 ms, com o código real do firmware.
static Resultado ciclo(const Ponto* c, int n, Planta pl, bool ssrMorto = false, float liq = 0,
                        const ParamControle& par = PARAM) {
  Controlador ctl(par);
  JanelaSSR ssr;
  LeituraSensor sensor;
  MonitorSeguranca mon;
  Resultado r = {0, 0, 0, 0, FALHA_NENHUMA, -1, 0};
  float duty = 0;
  const float dt = 0.01f;
  const float fim = c[n-1].t;
  float t = 0;                                     // relógio da curva, como no firmware
  for (int k = 0; t < fim; k++) {
    uint32_t ms = (uint32_t)(k * 10);
    if (k % 25 == 0) {                             // a cada 250 ms, como no firmware
      sensor.atualizar(true, sensorLe(pl.T));
      t += 0.25f * ctl.velocidade(c, n, t, sensor.temperatura());
      r.duracao = k * dt;
      float alvo = interpolar(c, n, t);
      duty = ctl.calcular(c, n, t, sensor.temperatura(), 0.25f);
      Falha f = mon.verificar(ms, sensor.ok(), sensor.temperatura(), duty, alvo);
      if (f != FALHA_NENHUMA) { r.falha = f; r.tFalha = t; return r; }
    }
    pl.passo(ssr.atualizar(ms, duty) && !ssrMorto, dt);
    float e = interpolar(c, n, t) - pl.T;
    float subida = interpolar(c, n, t + 1) - interpolar(c, n, t);
    if (subida > 0 && t > 15 && fabs(e) > r.erroMaxSubida) r.erroMaxSubida = fabs(e);
    if (subida == 0 && t > c[1].t + 60 && t < fim - 1 && fabs(e) > r.erroPatamar) r.erroPatamar = fabs(e);
    if (pl.T > r.pico) r.pico = pl.T;
    if (liq > 0 && pl.T >= liq) r.tAcimaLiq += dt;
  }
  return r;
}

static void relata(const char* nome, const Resultado& r) {
  char m[160];
  snprintf(m, sizeof m, "%s: erro max na subida %.1f C, pico %.1f C, acima do liquidus %.0f s, erro patamar %.1f C, duracao %.0f s",
           nome, r.erroMaxSubida, r.pico, r.tAcimaLiq, r.erroPatamar, r.duracao);
  TEST_MESSAGE(m);
}

void setUp() { srand(1); }
void tearDown() {}

// ---------- seguimento da curva ----------
void test_placa_conforme_o_modelo() {
  Planta pl{3.2f, 0.008f, 5.0f};
  Resultado r = ciclo(SAC305, N(SAC305), pl, false, 217);
  relata("nominal", r);
  TEST_ASSERT_EQUAL(FALHA_NENHUMA, r.falha);
  TEST_ASSERT_LESS_THAN_FLOAT(6.0f, r.erroMaxSubida);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 245.0f, r.pico);
  TEST_ASSERT_GREATER_THAN_FLOAT(40.0f, r.tAcimaLiq);    // tempo acima do liquidus da liga
}

// Os parâmetros nunca serão exatos antes da calibração.
// Esta placa não consegue subir na velocidade da curva perto do pico (precisaria
// de 3,2 °C/s e só tem 2,4): a curva precisa esperá-la e o pico precisa ser atingido.
void test_placa_mais_fraca_que_o_modelo() {
  Planta pl{2.4f, 0.0105f, 8.0f};
  Resultado r = ciclo(SAC305, N(SAC305), pl, false, 217);
  relata("fraca", r);
  TEST_ASSERT_EQUAL(FALHA_NENHUMA, r.falha);
  TEST_ASSERT_LESS_THAN_FLOAT(20.0f, r.erroMaxSubida);
  TEST_ASSERT_FLOAT_WITHIN(10.0f, 245.0f, r.pico);
  TEST_ASSERT_GREATER_THAN_FLOAT(40.0f, r.tAcimaLiq);
  TEST_ASSERT_GREATER_THAN_FLOAT(360.0f, r.duracao);     // demorou mais que a curva
}

// Esta nunca passa de 170 °C: o ciclo precisa parar com erro, não ficar esperando
void test_placa_que_nao_alcanca_o_pico_gera_falha() {
  Planta pl{1.2f, 0.008f, 8.0f};
  Resultado r = ciclo(SAC305, N(SAC305), pl);
  relata("nao alcanca", r);
  TEST_ASSERT_EQUAL(FALHA_SEM_RESPOSTA, r.falha);
}

void test_curva_nao_espera_na_descida_nem_sem_atraso() {
  TEST_ASSERT_EQUAL_FLOAT(1.0f, velocidadeCurva(SAC305, N(SAC305), 300, 100));  // descida
  TEST_ASSERT_EQUAL_FLOAT(1.0f, velocidadeCurva(SAC305, N(SAC305), 45, 85));    // subida em dia
  TEST_ASSERT_EQUAL_FLOAT(0.05f, velocidadeCurva(SAC305, N(SAC305), 45, 40));   // bem atrasada
}

void test_placa_mais_forte_que_o_modelo() {
  Planta pl{4.2f, 0.006f, 3.0f};
  Resultado r = ciclo(SAC305, N(SAC305), pl, false, 217);
  relata("forte", r);
  TEST_ASSERT_EQUAL(FALHA_NENHUMA, r.falha);
  TEST_ASSERT_LESS_THAN_FLOAT(15.0f, r.erroMaxSubida);
  TEST_ASSERT_LESS_THAN_FLOAT(255.0f, r.pico);           // não pode passar muito do pico
}

void test_curva_de_baixa_temperatura() {
  Planta pl{3.2f, 0.008f, 5.0f};
  Resultado r = ciclo(SN42, N(SN42), pl, false, 138);
  relata("Sn42Bi58", r);
  TEST_ASSERT_EQUAL(FALHA_NENHUMA, r.falha);
  TEST_ASSERT_FLOAT_WITHIN(6.0f, 165.0f, r.pico);
}

void test_patamar_estatico_estabiliza() {
  Planta pl{2.4f, 0.0105f, 8.0f};
  Resultado r = ciclo(ESTATICO, N(ESTATICO), pl);
  relata("estatico", r);
  TEST_ASSERT_EQUAL(FALHA_NENHUMA, r.falha);
  TEST_ASSERT_LESS_THAN_FLOAT(2.0f, r.erroPatamar);
}

// ---------- proteções ----------
void test_resistencia_que_nao_esquenta_gera_falha() {
  Planta pl{3.2f, 0.008f, 5.0f};
  Resultado r = ciclo(SAC305, N(SAC305), pl, true);
  TEST_ASSERT_EQUAL(FALHA_SEM_RESPOSTA, r.falha);
  TEST_ASSERT_LESS_THAN_FLOAT(90.0f, r.tFalha);
}

void test_termopar_solto_gera_falha_em_um_segundo() {
  LeituraSensor s; MonitorSeguranca m;
  s.atualizar(true, 100);
  TEST_ASSERT_TRUE(s.ok());
  for (int i = 0; i < 3; i++) { s.atualizar(false, 0); TEST_ASSERT_TRUE(s.ok()); }  // falha breve é tolerada
  s.atualizar(false, 0);
  TEST_ASSERT_FALSE(s.ok());
  TEST_ASSERT_EQUAL(FALHA_SENSOR, m.verificar(0, s.ok(), s.temperatura(), 50, 100));
}

void test_leitura_isolada_absurda_e_ignorada() {
  LeituraSensor s;
  s.atualizar(true, 100);
  s.atualizar(true, 900);
  TEST_ASSERT_TRUE(s.ok());
  TEST_ASSERT_EQUAL_FLOAT(100, s.temperatura());
  s.atualizar(true, 101);
  TEST_ASSERT_EQUAL_FLOAT(101, s.temperatura());
  for (int i = 0; i < 4; i++) s.atualizar(true, 900);   // persistente: é falha
  TEST_ASSERT_FALSE(s.ok());
}

void test_sensor_volta_mesmo_com_a_placa_mais_fria() {
  LeituraSensor s;
  s.atualizar(true, 200);
  for (int i = 0; i < 4; i++) s.atualizar(false, 0);
  TEST_ASSERT_FALSE(s.ok());
  s.atualizar(true, 30);                                // reconectou, placa já fria
  TEST_ASSERT_TRUE(s.ok());
  TEST_ASSERT_EQUAL_FLOAT(30, s.temperatura());
}

void test_sensor_sem_nenhuma_leitura_nao_esta_ok() {
  LeituraSensor s;
  TEST_ASSERT_FALSE(s.ok());
}

void test_sobretemperatura_corta_mesmo_ocioso() {
  MonitorSeguranca m;
  TEST_ASSERT_EQUAL(FALHA_NENHUMA, m.verificar(0, true, 314, 0, 0));
  TEST_ASSERT_EQUAL(FALHA_SOBRETEMP, m.verificar(250, true, 315, 0, 0));
}

void test_ssr_em_curto_gera_falha_ocioso() {
  Planta pl{3.2f, 0.008f, 5.0f};
  pl.extraCs = 1.0f;
  MonitorSeguranca m;
  Falha f = FALHA_NENHUMA; uint32_t ms = 0;
  for (; ms < 120000 && f == FALHA_NENHUMA; ms += 250) {
    for (int i = 0; i < 25; i++) pl.passo(false, 0.01f);
    f = m.verificar(ms, true, pl.T, 0, 0);
  }
  TEST_ASSERT_EQUAL(FALHA_SSR_PRESO, f);
}

// Depois de desligar em plena potência, a inércia e o resfriamento não são defeito
void test_resfriamento_normal_nao_gera_alarme_falso() {
  Planta pl{3.2f, 0.008f, 5.0f};
  pl.T = 245; pl.uf = 1;
  MonitorSeguranca m;
  for (uint32_t ms = 0; ms < 600000; ms += 250) {
    for (int i = 0; i < 25; i++) pl.passo(false, 0.01f);
    TEST_ASSERT_EQUAL(FALHA_NENHUMA, m.verificar(ms, true, pl.T, 0, 0));
  }
}

// ---------- sensor de tensão (ZMPT101B na rede) ----------
// Alimenta o medidor por 'segundos' com uma senoide de 'pico' contagens em volta
// de 'centro', com ruído do ADC, a 1 amostra por ms.
static MedidorRede simulaRede(float pico, float segundos, float hz = 60, int centro = 1900) {
  MedidorRede m;
  for (uint32_t ms = 0; ms < (uint32_t)(segundos * 1000); ms++) {
    float onda = sinf(2 * (float)M_PI * hz * ms / 1000.0f);
    int adc = (int)(centro + pico * onda + rand() % 21 - 10);
    m.amostra(ms, adc < 0 ? 0 : (adc > 4095 ? 4095 : adc));
  }
  return m;
}
void test_rede_mede_o_valor_eficaz() {
  // senoide: eficaz = pico / raiz(2)
  MedidorRede m60 = simulaRede(1000, 3, 60);
  TEST_ASSERT_TRUE(m60.temLeitura());
  TEST_ASSERT_FLOAT_WITHIN(15, 707, m60.rmsContagens());
  MedidorRede m50 = simulaRede(1000, 3, 50);
  TEST_ASSERT_FLOAT_WITHIN(15, 707, m50.rmsContagens());
  TEST_ASSERT_FALSE(m60.saturado());
}
void test_rede_127_e_220_ficam_na_proporcao_certa() {
  // com o mesmo ajuste do trimpot, a leitura é proporcional à tensão
  MedidorRede a = simulaRede(1200, 3), b = simulaRede(1200 * 127.0f / 220.0f, 3);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 127.0f / 220.0f, b.rmsContagens() / a.rmsContagens());
}
void test_rede_sem_onda_nao_tem_leitura() {
  TEST_ASSERT_FALSE(simulaRede(0, 3).temLeitura());     // só ruído: sensor solto ou sem rede
  MedidorRede nada;
  TEST_ASSERT_FALSE(nada.temLeitura());                 // nenhuma amostra ainda
}
void test_rede_avisa_saturacao() {
  TEST_ASSERT_TRUE(simulaRede(2500, 3).saturado());     // trimpot alto demais
}

// ---------- SSR ----------
static float fracaoLigada(float duty, uint32_t inicioMs = 0) {
  JanelaSSR j; int on = 0;
  for (uint32_t k = 0; k < 5000; k++) on += j.atualizar(inicioMs + k, duty);
  return on / 5000.0f * 100.0f;
}
void test_janela_entrega_a_potencia_pedida() {
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 37.0f, fracaoLigada(37));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, fracaoLigada(100));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 100.0f, fracaoLigada(99));   // sobra menor que o pulso mínimo
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, fracaoLigada(0));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, fracaoLigada(1));     // pulso curto demais
}
void test_janela_funciona_quando_millis_da_a_volta() {
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, fracaoLigada(50, 0xFFFFFFFFu - 2500));
}
void test_potencia_zero_desliga_no_meio_da_janela() {
  JanelaSSR j;
  TEST_ASSERT_TRUE(j.atualizar(0, 80));
  TEST_ASSERT_FALSE(j.atualizar(1, 0));
}

// ---------- entrada ----------
void test_validar_curva() {
  TEST_ASSERT_NULL(validarCurva(SAC305, N(SAC305)));
  Ponto um[] = {{0,25,""}};
  TEST_ASSERT_NOT_NULL(validarCurva(um, 1));
  Ponto quente[] = {{0,25,""},{60,301,""}};
  TEST_ASSERT_NOT_NULL(validarCurva(quente, 2));
  Ponto repetido[] = {{0,25,""},{60,100,""},{60,150,""}};
  TEST_ASSERT_NOT_NULL(validarCurva(repetido, 3));
  Ponto voltando[] = {{0,25,""},{60,100,""},{30,150,""}};
  TEST_ASSERT_NOT_NULL(validarCurva(voltando, 3));
  Ponto longo[] = {{0,25,""},{7201,100,""}};
  TEST_ASSERT_NOT_NULL(validarCurva(longo, 2));
  Ponto nan[] = {{0,25,""},{60,NAN,""}};
  TEST_ASSERT_NOT_NULL(validarCurva(nan, 2));
}
void test_interpolar() {
  TEST_ASSERT_EQUAL_FLOAT(25, interpolar(SAC305, N(SAC305), -1));
  TEST_ASSERT_EQUAL_FLOAT(87.5f, interpolar(SAC305, N(SAC305), 45));
  TEST_ASSERT_EQUAL_FLOAT(50, interpolar(SAC305, N(SAC305), 999));
}
void test_decodificar_max6675() {
  float T = -1;
  TEST_ASSERT_TRUE(decodificarMax6675(100 << 3, T));       // 100 × 0,25
  TEST_ASSERT_EQUAL_FLOAT(25.0f, T);
  TEST_ASSERT_FALSE(decodificarMax6675((100 << 3) | 0x04, T));  // termopar aberto
  TEST_ASSERT_FALSE(decodificarMax6675(0x0000, T));
  TEST_ASSERT_FALSE(decodificarMax6675(0xFFFF, T));
}

void test_tensao_da_rede_ajusta_a_potencia() {
  TEST_ASSERT_TRUE(tensaoValida(127));
  TEST_ASSERT_TRUE(tensaoValida(220));
  TEST_ASSERT_FALSE(tensaoValida(110));
  TEST_ASSERT_FALSE(tensaoValida(0));
  TEST_ASSERT_EQUAL_FLOAT(PARAM.pMax, paramParaTensao(PARAM, 220).pMax);
  // 220 V em 127 V: (127/220)² ≈ 0,333 da potência
  TEST_ASSERT_FLOAT_WITHIN(0.01f, PARAM.pMax * 0.333f, paramParaTensao(PARAM, 127).pMax);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, PARAM.pMax * 0.9f, paramParaTensao(PARAM, 208.7f).pMax);  // medida
  TEST_ASSERT_EQUAL_FLOAT(PARAM.pMax, paramParaTensao(PARAM, 999).pMax);   // inválida: não mexe
  TEST_ASSERT_EQUAL_FLOAT(PARAM.pMax, paramParaTensao(PARAM, 0).pMax);
  TEST_ASSERT_EQUAL_FLOAT(PARAM.perda, paramParaTensao(PARAM, 127).perda); // o resto não muda
}
void test_placa_220v_em_127v_segue_curva_baixa() {
  // Planta com 1/3 da potência e o controle informado da tensão
  Planta fraca = { PARAM.pMax * 0.333f, PARAM.perda, PARAM.atraso };
  Ponto baixa[] = {{0,25,""},{200,100,""},{500,100,""}};
  Controlador ctl(paramParaTensao(PARAM, 127));
  JanelaSSR ssr;
  float duty = 0, t = 0, erroMax = 0;
  for (int k = 0; t < 500; k++) {
    if (k % 25 == 0) {
      t += 0.25f * velocidadeCurva(baixa, 3, t, fraca.T);
      duty = ctl.calcular(baixa, 3, t, fraca.T, 0.25f);
      if (t > 250) erroMax = fmaxf(erroMax, fabsf(interpolar(baixa, 3, t) - fraca.T));
    }
    fraca.passo(ssr.atualizar((uint32_t)(k * 10), duty), 0.01f);
  }
  TEST_ASSERT_LESS_THAN_FLOAT(3.0f, erroMax);   // patamar a 100 °C, dentro de 3 °C
}

// ---------- freio preditivo ----------
// Placa com 3x mais atraso do que o modelo de fábrica supõe, mas com o atraso
// informado ao controle: o freio precisa segurar o calor acumulado.
void test_freio_reduz_overshoot_em_placa_lenta() {
  Planta pl{3.2f, 0.008f, 15.0f};
  ParamControle com = PARAM; com.atraso = 15.0f;
  ParamControle sem = com;   sem.margem = -1.0f; sem.bandaI = 1000.0f;   // algoritmo antigo
  Resultado rCom = ciclo(ESTATICO, N(ESTATICO), pl, false, 0, com);
  Resultado rSem = ciclo(ESTATICO, N(ESTATICO), pl, false, 0, sem);
  relata("lenta com freio", rCom);
  relata("lenta sem freio", rSem);
  TEST_ASSERT_EQUAL(FALHA_NENHUMA, rCom.falha);
  TEST_ASSERT_LESS_THAN_FLOAT(rSem.pico - 2.0f, rCom.pico);   // pelo menos 2 °C a menos
  TEST_ASSERT_LESS_THAN_FLOAT(153.0f, rCom.pico);
}

void test_freio_deixa_chegar_ao_pico() {
  Planta pl{3.2f, 0.008f, 15.0f};
  ParamControle com = PARAM; com.atraso = 15.0f;
  Resultado r = ciclo(SAC305, N(SAC305), pl, false, 217, com);
  relata("SAC305 lenta com freio", r);
  TEST_ASSERT_EQUAL(FALHA_NENHUMA, r.falha);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 245.0f, r.pico);
}

// O modelo diz que a placa é o dobro do que ela é, com atraso grande (o freio prevê
// muito calor acumulado). Antes, o freio olhava a curva no tempo dela: com a curva
// esperando a placa, segurava a placa abaixo do alvo e a espera não acabava
// (1680 s e pico de 218 °C numa curva de 360 s).
void test_modelo_otimista_nao_trava_a_curva() {
  Planta pl{2.2f, 0.008f, 10.0f};                            // chega a ~300 °C com 100%
  ParamControle otimista = PARAM; otimista.pMax = 4.4f; otimista.atraso = 25.0f;   // o dobro
  Resultado r = ciclo(SAC305, N(SAC305), pl, false, 217, otimista);
  relata("modelo otimista", r);
  TEST_ASSERT_EQUAL(FALHA_NENHUMA, r.falha);
  TEST_ASSERT_GREATER_THAN_FLOAT(228.0f, r.pico);
  TEST_ASSERT_LESS_THAN_FLOAT(900.0f, r.duracao);
}

void test_curva_nao_passa_do_pico_sem_a_placa() {
  // t = 239,6 s: alvo 244,6 °C, a 0,4 s do pico (245 °C em 240 s); placa 4,6 °C atrás
  TEST_ASSERT_LESS_THAN_FLOAT(0.2f, velocidadeCurva(SAC305, N(SAC305), 239.6f, 240.0f));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, velocidadeCurva(SAC305, N(SAC305), 240.5f, 240.0f));   // já na descida
}

void test_curva_espera_mais_perto_do_pico() {
  // 4,5 °C atrás: no meio da subida quase não espera; a 2 °C do pico, espera bastante
  TEST_ASSERT_GREATER_THAN_FLOAT(0.8f, velocidadeCurva(SAC305, N(SAC305), 45, 83.0f));    // alvo 87,5
  TEST_ASSERT_LESS_THAN_FLOAT(0.3f, velocidadeCurva(SAC305, N(SAC305), 238, 238.63f));    // alvo 243,13
}

// ---------- malha aberta e parâmetros ----------
void test_malha_aberta_corta_e_nao_volta() {
  Ponto p[] = {{0,100,""},{100,100,""}};
  bool cortado = false;
  TEST_ASSERT_EQUAL_FLOAT(100.0f, dutyMalhaAberta(p, 2, 10, 100, 150, cortado));
  TEST_ASSERT_EQUAL_FLOAT(0.0f,   dutyMalhaAberta(p, 2, 20, 150, 150, cortado));
  TEST_ASSERT_EQUAL_FLOAT(0.0f,   dutyMalhaAberta(p, 2, 30, 120, 150, cortado));   // esfriou: continua desligada
  TEST_ASSERT_NULL(validarMalhaAberta(p, 2, 200));
  TEST_ASSERT_NOT_NULL(validarMalhaAberta(p, 2, 300));      // acima de 280 °C
  Ponto demais[] = {{0,100,""},{10,120,""}};
  TEST_ASSERT_NOT_NULL(validarMalhaAberta(demais, 2, 200)); // mais de 100%
}

void test_parametros_por_nome() {
  ParamControle p = PARAM;
  for (int i = 0; i < N_PARAM; i++) TEST_ASSERT_NOT_NULL(campoParam(p, i));
  TEST_ASSERT_NULL(campoParam(p, N_PARAM));
  *campoParam(p, 4) = 7.0f;
  TEST_ASSERT_EQUAL_FLOAT(7.0f, p.kp);
  TEST_ASSERT_EQUAL_STRING("kp", NOMES_PARAM[4]);
  TEST_ASSERT_NULL(validarParam(4, 3.0f));
  TEST_ASSERT_NOT_NULL(validarParam(4, -1.0f));
  TEST_ASSERT_NOT_NULL(validarParam(0, NAN));
  for (int i = 0; i < N_PARAM; i++) TEST_ASSERT_NULL(validarParam(i, *campoParam(p, i)));   // o padrão é válido
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_freio_reduz_overshoot_em_placa_lenta);
  RUN_TEST(test_freio_deixa_chegar_ao_pico);
  RUN_TEST(test_modelo_otimista_nao_trava_a_curva);
  RUN_TEST(test_curva_nao_passa_do_pico_sem_a_placa);
  RUN_TEST(test_curva_espera_mais_perto_do_pico);
  RUN_TEST(test_malha_aberta_corta_e_nao_volta);
  RUN_TEST(test_parametros_por_nome);
  RUN_TEST(test_tensao_da_rede_ajusta_a_potencia);
  RUN_TEST(test_placa_220v_em_127v_segue_curva_baixa);
  RUN_TEST(test_placa_conforme_o_modelo);
  RUN_TEST(test_placa_mais_fraca_que_o_modelo);
  RUN_TEST(test_placa_que_nao_alcanca_o_pico_gera_falha);
  RUN_TEST(test_curva_nao_espera_na_descida_nem_sem_atraso);
  RUN_TEST(test_placa_mais_forte_que_o_modelo);
  RUN_TEST(test_curva_de_baixa_temperatura);
  RUN_TEST(test_patamar_estatico_estabiliza);
  RUN_TEST(test_resistencia_que_nao_esquenta_gera_falha);
  RUN_TEST(test_termopar_solto_gera_falha_em_um_segundo);
  RUN_TEST(test_leitura_isolada_absurda_e_ignorada);
  RUN_TEST(test_sensor_volta_mesmo_com_a_placa_mais_fria);
  RUN_TEST(test_sensor_sem_nenhuma_leitura_nao_esta_ok);
  RUN_TEST(test_sobretemperatura_corta_mesmo_ocioso);
  RUN_TEST(test_ssr_em_curto_gera_falha_ocioso);
  RUN_TEST(test_resfriamento_normal_nao_gera_alarme_falso);
  RUN_TEST(test_rede_mede_o_valor_eficaz);
  RUN_TEST(test_rede_127_e_220_ficam_na_proporcao_certa);
  RUN_TEST(test_rede_sem_onda_nao_tem_leitura);
  RUN_TEST(test_rede_avisa_saturacao);
  RUN_TEST(test_janela_entrega_a_potencia_pedida);
  RUN_TEST(test_janela_funciona_quando_millis_da_a_volta);
  RUN_TEST(test_potencia_zero_desliga_no_meio_da_janela);
  RUN_TEST(test_validar_curva);
  RUN_TEST(test_interpolar);
  RUN_TEST(test_decodificar_max6675);
  return UNITY_END();
}
