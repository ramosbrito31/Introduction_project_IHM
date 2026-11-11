#pragma once
/*
  Lógica do controle de aquecimento, sem nada específico do Arduino:
  o mesmo código roda no ESP32 e nos testes no PC (test/test_controle).
  Quem lê o pino do sensor e aciona o SSR é o src/main.cpp.
*/
#include <stdint.h>

// ------------------------------------------------------------------
// Curva de temperatura
// ------------------------------------------------------------------
const int   MAX_PONTOS      = 20;
const float TEMP_MAX_CURVA  = 300.0f;   // °C (mesmo limite do site)
const float TEMPO_MAX_CURVA = 7200.0f;  // s  (mesmo limite do site)

struct Ponto { float t; float T; char fase[20]; };

float interpolar(const Ponto* curva, int n, float t);

// nullptr = curva válida; senão, o texto do problema
const char* validarCurva(const Ponto* curva, int n);

// Velocidade do relógio da curva (0,05 a 1). Numa subida, se a placa ficar para
// trás, a curva a espera: senão o tempo acabaria antes de a placa chegar ao pico
// e a solda sairia ruim sem nenhum aviso.
float velocidadeCurva(const Ponto* curva, int n, float t, float tempC);

// ------------------------------------------------------------------
// Sensor: termopar tipo K com MAX6675
// ------------------------------------------------------------------
// Converte os 16 bits lidos do MAX6675. Retorna false se o termopar estiver
// aberto ou se o módulo não estiver respondendo (fio solto).
bool decodificarMax6675(uint16_t bruto, float& tempC);

// Descarta leituras isoladas absurdas (ruído do chaveamento) e só declara
// falha depois de várias leituras ruins seguidas.
class LeituraSensor {
public:
  void  atualizar(bool valida, float tempC);
  bool  ok() const { return temValor_ && falhas_ < FALHAS_MAX; }
  float temperatura() const { return temp_; }   // última leitura aceita
private:
  static const int FALHAS_MAX = 4;              // 1 s lendo a cada 250 ms
  static constexpr float SALTO_MAX = 25.0f;     // °C entre duas leituras
  bool  temValor_ = false;
  int   falhas_ = 0;
  float temp_ = 0;
};

// ------------------------------------------------------------------
// Controlador: PI + feed-forward
// ------------------------------------------------------------------
// A potência vem de duas partes:
//  - feed-forward: o que o modelo da placa diz ser necessário para seguir a
//    curva (é conhecida de antemão, então dá para antecipar o atraso térmico);
//  - PI: corrige o que sobrar de erro.
struct ParamControle {
  float pMax;    // °C/s: velocidade de subida com 100% de potência
  float perda;   // 1/s:  perda de calor (°C/s por °C acima do ambiente)
  float tAmb;    // °C:   temperatura ambiente
  float atraso;  // s:    atraso entre ligar a resistência e o sensor perceber
  float kp;      // %/°C
  float ki;      // %/(°C·s)
  float iMax;    // %:    limite da parte integral
};

class Controlador {
public:
  explicit Controlador(const ParamControle& p) : p_(p) {}
  void  reiniciar() { integral_ = 0; }
  // Retorna a potência (duty) de 0 a 100 %. dt = segundos desde a última chamada.
  float calcular(const Ponto* curva, int n, float t, float tempC, float dt);
private:
  ParamControle p_;
  float integral_ = 0;
};

// ------------------------------------------------------------------
// SSR: potência por janela de tempo
// ------------------------------------------------------------------
// O SSR comuta só na passagem por zero da rede, então PWM rápido não funciona.
// A potência vira a fração ligada de cada janela de 1 s.
class JanelaSSR {
public:
  // Chamar o tempo todo. Retorna como o pino deve ficar agora.
  bool atualizar(uint32_t agoraMs, float duty);
private:
  static const uint32_t JANELA_MS  = 1000;
  static const uint32_t PULSO_MIN_MS = 20;      // menos que isso o SSR nem conduz
  uint32_t inicio_ = 0;
  uint32_t ligadoMs_ = 0;
  bool     iniciada_ = false;
};

// ------------------------------------------------------------------
// Proteções
// ------------------------------------------------------------------
enum Falha { FALHA_NENHUMA, FALHA_SENSOR, FALHA_SOBRETEMP, FALHA_SEM_RESPOSTA, FALHA_SSR_PRESO, FALHA_SEM_TENSAO };

const char* textoFalha(Falha f);

// Sensor de tensão (ZMPT101B) ligado nos terminais da placa, depois do SSR.
// Compara o que foi mandado ao SSR com o que chega de fato à placa.
class MonitorTensao {
public:
  // Chamar a cada 1 ms, aproximadamente, com a leitura do ADC e o estado do pino do SSR.
  void  amostra(uint32_t agoraMs, int adc, bool ssrLigado);
  Falha falha() const;
  void  reiniciar();
  int   amplitude() const { return amplitude_; }   // última, de pico a pico, em contagens do ADC
private:
  static const uint32_t BLOCO_MS = 40;          // mais de 2 ciclos da rede, em 50 ou 60 Hz
  static const int AMOSTRAS_MIN = 15;           // bloco com menos que isso é descartado
  static const int LIMIAR = 400;                // contagens: abaixo disso, considera sem tensão
  static const int BLOCOS_FALHA = 10;           // blocos seguidos até declarar falha

  void fecharBloco();

  bool     iniciado_ = false;
  bool     ultimoSsr_ = false;
  uint32_t inicio_ = 0;
  int      n_ = 0, min_ = 0, max_ = 0;
  int      descartar_ = 0;     // blocos a ignorar depois de o SSR mudar de estado
  int      amplitude_ = 0;
  int      comTensao_ = 0;     // blocos seguidos com tensão e SSR desligado
  int      semTensao_ = 0;     // blocos seguidos sem tensão e SSR ligado
};

class MonitorSeguranca {
public:
  // Chamar a cada leitura do sensor, com ou sem ciclo em andamento.
  // alvo só importa durante o ciclo (fora dele, duty = 0).
  Falha verificar(uint32_t agoraMs, bool sensorOk, float tempC, float duty, float alvo);
  void  reiniciar();

  static constexpr float TEMP_CORTE = 315.0f;   // °C: desliga, seja qual for o estado
private:
  // Potência alta e abaixo do alvo: a temperatura precisa estar subindo.
  // Se não sobe, o SSR/resistência falhou ou o sensor saiu de cima da placa.
  static constexpr float    DUTY_ALTO     = 60.0f;  // %
  static constexpr float    ERRO_MIN      = 5.0f;   // °C abaixo do alvo
  static constexpr float    SUBIDA_MIN    = 5.0f;   // °C
  static const uint32_t     JANELA_SUB_MS = 60000;
  // Resistência desligada: depois da inércia térmica, a temperatura não pode subir.
  // Se sobe, o SSR está em curto (a falha típica de SSR).
  static constexpr float    DUTY_BAIXO    = 5.0f;   // %
  static const uint32_t     INERCIA_MS    = 30000;
  static constexpr float    SUBIDA_MAX    = 20.0f;  // °C

  bool     subindo_ = false;
  uint32_t subIni_ = 0;
  float    subT0_ = 0;
  bool     desligada_ = false;
  uint32_t desIni_ = 0;
  bool     temMinimo_ = false;
  float    minimo_ = 0;
};
