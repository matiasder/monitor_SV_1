#include <Arduino.h>
#include <EasyNextionLibrary.h>
#include <AFE4950.h>

extern "C" {
  #include "biosignals.h"
}

// =========================================================================
// Hardware
// =========================================================================
// AFE4950 (SPI + control)
#define SCLK     18
#define MISO     19
#define MOSI     23
#define CS        5
#define AFE_RST   4
#define FIFO_RDY 16

// Nextion en UART2 con pines personalizados. Serial (USB) queda libre
// para depuracion y el comando CAL.
#define NEXTION_TX   25   // ESP32 TX -> RX de la Nextion
#define NEXTION_RX   33   // ESP32 RX <- TX de la Nextion
#define BAUD_NEXTION 115200
#define ID_WAVE_ECG  8
#define ID_WAVE_PPG  7
#define CANAL        0
#define ALTURA_WAVE  200

// =========================================================================
// Formato de paquete del AFE4950 (segun readme.md del ejemplo)
// Palabra de 4 bytes: [LSB][MID][MSB][TAG]  (valor de 24 bits little-endian)
// Ciclo de muestras del FIFO: ECG, ECG, PPG, ECG, ECG, PPG, ...
// =========================================================================
#define TAG_HEADER   0x02
#define TAG_PDIFF    0x03
#define TAG_SAMPLE   0x05
#define TAG_STATUS   0x06   // byte0 = ST (0 sin senal, 1 estabilizando, 2 valida), byte1 = SEG
#define TAG_AC_IR    0x07
#define TAG_DC_IR    0x08
#define TAG_AC_RED   0x09
#define TAG_DC_RED   0x0A   // ultima palabra del bloque SpO2

// Cada ciclo (ECG,ECG,PPG) se convierte en UNA muestra a 500 Hz:
// ecg = promedio de las 2 muestras ECG, ppg = la muestra PPG.
#define FS_SISTEMA 500.0f
#define PPG_SIGN   (+1.0f)      // poner -1.0f si el pulso sale invertido
#define ALPHA_HP   0.0063f      // HPF de 1 polo ~0.5 Hz @ 500 Hz

// SpO2 = A - B*R  (curva del sistema indicada en el readme)
#define SPO2_A     110.0f
#define SPO2_B     25.0f
#define NSPO2      5            // promedio de los ultimos 5 valores (~5 s)
#define DEBUG_SPO2 1

#define USAR_PIR   1
#define N_BUF      2048         // ~4 s de historia
#define MS_TO_SAMP(x) ((uint32_t)((x) * FS_SISTEMA / 1000.0f))
#define B_BUF(buf, i) buf[(i) % N_BUF]

// =========================================================================
// Estructuras
// =========================================================================
struct MuestraTriada {
  float    ecg;        // ECG sin baseline
  float    ppg;        // PPG AC (sin DC, con signo corregido)
  int32_t  rawECG;
  int32_t  rawPPG;     // PPG crudo (incluye DC) -> usado para PIR
  uint32_t sampleIdx;
};

struct DatosSpO2 {
  uint8_t  estado;     // 0 / 1 / 2
  uint8_t  seg;
  uint32_t acIr, dcIr, acRed, dcRed;
};

struct SignosVitales {
  int fc, fp, spo2, sbp, dbp;
  bool calibrado;
};

// =========================================================================
// Globales
// =========================================================================
AFE4950 afe;

int32_t ecgBuf[N_BUF], ppgBuf[N_BUF], dcBuf[N_BUF];
volatile uint32_t totalMuestras = 0;

PanTompkins pt;
PPGDetector ppgDet;
uint32_t Rprev = 0;

const int NPROM = 8;
float hPtt[NPROM], hPir[NPROM];
int nProm = 0, iProm = 0;
float pttProm = 0, pirProm = 0;

volatile bool calibrando = false, calibrado = false;
float SBPO = 120, DBPO = 80, PTTO = 0, PIRO = 0;
float sumPtt = 0, sumPir = 0;
int nCal = 0;
const int N_CAL = 30;

// SpO2
float hSpo2[NSPO2];
int nSpo2 = 0, iSpo2 = 0;
float spo2Prom = 0;

QueueHandle_t colaMuestras;   // Adq -> Proc
QueueHandle_t colaGrafica;    // Adq -> Display (250 Hz)
QueueHandle_t colaSpo2;       // Adq -> Proc (1 Hz)
QueueHandle_t buzonSignos;    // Proc -> Display (mailbox)

EasyNex myNextion(Serial2);

void procesarLatido(uint32_t R, uint32_t Rsig);
void revisarYPromediar(float ptt, float pir);
void calcularPA(float ptt, float pir, SignosVitales &sv);
void actualizarSpO2(const DatosSpO2 &d);

// =========================================================================
// Utilidades
// =========================================================================
static inline int32_t valor24Signed(const uint8_t *w) {
  int32_t v = ((int32_t)w[2] << 16) | ((int32_t)w[1] << 8) | w[0];
  if (v & 0x800000) v |= 0xFF000000;   // complemento a 2
  return v;
}

static inline uint32_t valor24Unsigned(const uint8_t *w) {
  return ((uint32_t)w[2] << 16) | ((uint32_t)w[1] << 8) | w[0];
}

void agregarPuntoGrafica(int idWave, int canal, int valor) {
  Serial2.print("add ");
  Serial2.print(idWave);
  Serial2.print(",");
  Serial2.print(canal);
  Serial2.print(",");
  Serial2.print(valor);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
}

// Autoescala lenta para la pantalla
struct AutoEscala { float lo = 0, hi = 1; };

uint8_t escalar(AutoEscala &s, float v) {
  if (v < s.lo) s.lo = v; else s.lo += (v - s.lo) * 0.001f;
  if (v > s.hi) s.hi = v; else s.hi -= (s.hi - v) * 0.001f;
  float span = s.hi - s.lo;
  if (span < 1.0f) span = 1.0f;
  int y = (int)((v - s.lo) / span * (ALTURA_WAVE - 1));
  return (uint8_t)constrain(y, 0, ALTURA_WAVE - 1);
}

// Un ciclo completo (ECG,ECG,PPG) -> una muestra a 500 Hz
static void emitirMuestra(float ecgProm, int32_t ppgRaw) {
  static uint8_t divisorGrafica = 0;
  static float ecgBase = 0, ppgBase = 0;
  static bool baseInit = false;

  MuestraTriada m;
  m.rawECG = (int32_t)ecgProm;
  m.rawPPG = ppgRaw;

  if (!baseInit) {
    ecgBase = ecgProm;
    ppgBase = (float)ppgRaw;
    baseInit = true;
  }
  ecgBase += ALPHA_HP * (ecgProm - ecgBase);
  ppgBase += ALPHA_HP * ((float)ppgRaw - ppgBase);

  m.ecg = ecgProm - ecgBase;
  m.ppg = ((float)ppgRaw - ppgBase) * PPG_SIGN;
  m.sampleIdx = totalMuestras;

  B_BUF(ecgBuf, m.sampleIdx) = (int32_t)m.ecg;
  B_BUF(ppgBuf, m.sampleIdx) = (int32_t)m.ppg;   // AC -> PTT
  B_BUF(dcBuf,  m.sampleIdx) = ppgRaw;           // DC -> PIR
  totalMuestras++;

  xQueueSendToBack(colaMuestras, &m, 0);

  if (++divisorGrafica >= 2) {                   // 250 Hz para la pantalla
    divisorGrafica = 0;
    xQueueSendToBack(colaGrafica, &m, 0);
  }
}

// =========================================================================
// Tarea 1: Adquisicion y parseo de paquetes del AFE4950 (Core 1, prio 5)
// =========================================================================
void tareaAdquisicion(void *pvParameters) {
  uint8_t fase = 0;          // 0,1 = ECG ; 2 = PPG
  float ecgAcum = 0;
  DatosSpO2 spo2 = {};

  for (;;) {
    if (!afe.ready()) {
      vTaskDelay(1);
      continue;
    }

    const uint8_t *data = afe.getData();
    size_t len = afe.getBytesToSend() & ~(size_t)3;   // solo palabras completas

    for (size_t o = 0; o < len; o += 4) {
      const uint8_t *w = data + o;

      switch (w[3]) {
        case TAG_HEADER:            // inicio de paquete: el ciclo vuelve a ECG
          fase = 0;
          ecgAcum = 0;
          break;

        case TAG_PDIFF:             // w[0] + 1 = numero de muestras (no se necesita)
          break;

        case TAG_SAMPLE: {
          int32_t v = valor24Signed(w);
          if (fase < 2) {
            ecgAcum += (float)v;
            fase++;
          } else {                  // muestra PPG: cierra el ciclo
            emitirMuestra(ecgAcum * 0.5f, v);
            fase = 0;
            ecgAcum = 0;
          }
          break;
        }

        case TAG_STATUS:
          spo2.estado = w[0];
          spo2.seg    = w[1];
          break;
        case TAG_AC_IR:   spo2.acIr  = valor24Unsigned(w); break;
        case TAG_DC_IR:   spo2.dcIr  = valor24Unsigned(w); break;
        case TAG_AC_RED:  spo2.acRed = valor24Unsigned(w); break;
        case TAG_DC_RED:
          spo2.dcRed = valor24Unsigned(w);
          xQueueSendToBack(colaSpo2, &spo2, 0);   // bloque SpO2 completo
          break;

        default:
          break;
      }
    }
  }
}

// =========================================================================
// SpO2: relacion de ratios con los AC/DC entregados por el AFE
// =========================================================================
void actualizarSpO2(const DatosSpO2 &d) {
  // Solo es valida con ESTADO = 2; en los demas estados AC/DC llegan en 0
  if (d.estado != 2 || d.dcIr == 0 || d.dcRed == 0 || d.acIr == 0) {
    nSpo2 = 0; iSpo2 = 0; spo2Prom = 0;
#if DEBUG_SPO2
    Serial.printf("SpO2: estado=%u (%us)\n", d.estado, d.seg);
#endif
    return;
  }

  float R = ((float)d.acRed / (float)d.dcRed) / ((float)d.acIr / (float)d.dcIr);
  if (R < 0.2f || R > 3.0f) return;           // fuera de rango fisiologico razonable

  float s = SPO2_A - SPO2_B * R;
  s = constrain(s, 0.0f, 100.0f);

  hSpo2[iSpo2] = s;
  iSpo2 = (iSpo2 + 1) % NSPO2;
  if (nSpo2 < NSPO2) nSpo2++;

  float suma = 0;
  for (int j = 0; j < nSpo2; j++) suma += hSpo2[j];
  spo2Prom = suma / nSpo2;

#if DEBUG_SPO2
  Serial.printf("SpO2: R=%.3f -> %.1f (prom %.1f)\n", R, s, spo2Prom);
#endif
}

// =========================================================================
// Tarea 2: Procesamiento y estimacion de PA (Core 1, prio 4)
// =========================================================================
void tareaProcesamiento(void *pvParameters) {
  pt_init(&pt, FS_SISTEMA);
  ppg_init(&ppgDet, FS_SISTEMA);

  MuestraTriada m;
  DatosSpO2 dsp;
  SignosVitales sv = {0, 0, 0, 0, 0, false};
  TickType_t ultActualizacionUI = xTaskGetTickCount();

  for (;;) {
    if (xQueueReceive(colaMuestras, &m, pdMS_TO_TICKS(50)) == pdTRUE) {
      PT_Result r = pt_process(&pt, m.ecg);
      if (r.beat) {
        uint32_t idxNotif = m.sampleIdx;
        if (idxNotif >= MS_TO_SAMP(500)) {
          uint32_t ini = idxNotif - MS_TO_SAMP(400);
          if (Rprev != 0 && ini < Rprev + MS_TO_SAMP(250)) {
            ini = Rprev + MS_TO_SAMP(250);
          }
          uint32_t R = ini;
          for (uint32_t i = ini; i <= idxNotif; i++) {
            if (B_BUF(ecgBuf, i) > B_BUF(ecgBuf, R)) R = i;
          }
          if (Rprev != 0) procesarLatido(Rprev, R);
          Rprev = R;
        }
      }

      // Timestamp derivado del indice de muestra (los paquetes llegan en rafaga)
      uint32_t tUs = (uint32_t)(m.sampleIdx * (uint32_t)(1000000.0f / FS_SISTEMA));
      ppg_process_t(&ppgDet, m.ppg, tUs);
    }

    // Parametros de SpO2 (1 Hz)
    while (xQueueReceive(colaSpo2, &dsp, 0) == pdTRUE) {
      actualizarSpO2(dsp);
    }

    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      float s, d;
      if (sscanf(cmd.c_str(), "CAL %f %f", &s, &d) == 2) {
        SBPO = s; DBPO = d;
        sumPtt = 0; sumPir = 0; nCal = 0;
        calibrado = false; calibrando = true;
        Serial.println("Calibrando PA: mantengase quieto...");
      }
    }

    if ((xTaskGetTickCount() - ultActualizacionUI) >= pdMS_TO_TICKS(1000)) {
      ultActualizacionUI = xTaskGetTickCount();

      float bpmECG = pt_bpm(&pt);
      float bpmPPG = ppg_bpm(&ppgDet);

      sv.fc   = (bpmECG > 0.0f) ? (int)(bpmECG + 0.5f) : 0;
      sv.fp   = (bpmPPG > 0.0f) ? (int)(bpmPPG + 0.5f) : 0;
      sv.spo2 = (nSpo2 > 0) ? (int)(spo2Prom + 0.5f) : 0;   // 0 = sin medida valida

      calcularPA(pttProm, pirProm, sv);
      xQueueOverwrite(buzonSignos, &sv);
    }
  }
}

// =========================================================================
// PTT / PIR y modelo de presion
// =========================================================================
void procesarLatido(uint32_t R, uint32_t Rsig) {
  float rr = (Rsig - R) * 1000.0f / FS_SISTEMA;
  if (rr < 333 || rr > 2000) return;

  uint32_t fin = R + MS_TO_SAMP(500);
  if (fin > Rsig) fin = Rsig;

  uint32_t k = R + MS_TO_SAMP(100);
  int32_t mejor = INT32_MIN / 2;

  for (uint32_t i = R + MS_TO_SAMP(100); i < fin; i++) {
    int32_t pend = B_BUF(ppgBuf, i + 5) - B_BUF(ppgBuf, i - 5);
    if (pend > mejor) { mejor = pend; k = i; }
  }
  if (mejor <= 0) return;

  float ptt = (k - R) * 1000.0f / FS_SISTEMA;
  float pir = 1.0f;

#if USAR_PIR
  int32_t IH = INT32_MIN, IL = INT32_MAX;
  for (uint32_t i = R; i <= k; i++) {
    if (B_BUF(dcBuf, i) > IH) IH = B_BUF(dcBuf, i);
  }

  uint32_t finL = k + MS_TO_SAMP(300);
  if (finL > Rsig) finL = Rsig;
  for (uint32_t i = k; i <= finL; i++) {
    if (B_BUF(dcBuf, i) < IL) IL = B_BUF(dcBuf, i);
  }

  if (IL <= 0 || IH <= 0) return;
  pir = (float)IH / (float)IL;
#endif

  revisarYPromediar(ptt, pir);
}

void revisarYPromediar(float ptt, float pir) {
  if (ptt < 150 || ptt > 450) return;
  if (USAR_PIR && (pir <= 1.0f || pir > 1.3f)) return;   // *** revisar rango con el AFE ***

  if (nProm == NPROM && fabsf(ptt - pttProm) > 0.2f * pttProm) return;

  hPtt[iProm] = ptt;
  hPir[iProm] = pir;
  iProm = (iProm + 1) % NPROM;
  if (nProm < NPROM) nProm++;

  float sP = 0, sI = 0;
  for (int j = 0; j < nProm; j++) { sP += hPtt[j]; sI += hPir[j]; }
  pttProm = sP / nProm;
  pirProm = sI / nProm;

  if (calibrando) {
    sumPtt += ptt; sumPir += pir; nCal++;
    if (nCal >= N_CAL) {
      PTTO = sumPtt / nCal;
      PIRO = sumPir / nCal;
      calibrando = false; calibrado = true;
      Serial.printf("Calibracion Lista: PTT0=%.1f ms, PIR0=%.4f\n", PTTO, PIRO);
    }
  }
}

void calcularPA(float ptt, float pir, SignosVitales &sv) {
  sv.calibrado = calibrado;
  if (!calibrado || ptt == 0) { sv.sbp = 0; sv.dbp = 0; return; }

  float PPO  = SBPO - DBPO;
  float MBPO = DBPO + PPO / 3.0f;
  float r    = PTTO / ptt;

  float mbp = MBPO * (PIRO / pir);
  float pp  = PPO * (pir / PIRO) * r * r;

  sv.sbp = (int)(mbp + 2.0f * pp / 3.0f + 0.5f);
  sv.dbp = (int)(mbp - pp / 3.0f + 0.5f);
}

// =========================================================================
// Tarea 3: Nextion (Core 0, prio 2)
// =========================================================================
void tareaDisplay(void *pvParameters) {
  MuestraTriada m;
  SignosVitales sv;
  AutoEscala escEcg, escPpg;

  for (;;) {
    myNextion.NextionListen();

    while (xQueueReceive(colaGrafica, &m, 0) == pdTRUE) {
      agregarPuntoGrafica(ID_WAVE_ECG, CANAL, escalar(escEcg, m.ecg));
      agregarPuntoGrafica(ID_WAVE_PPG, CANAL, escalar(escPpg, m.ppg));
    }

    if (xQueueReceive(buzonSignos, &sv, 0) == pdTRUE) {
      myNextion.writeNum("n0.val", sv.fc);
      myNextion.writeNum("n1.val", sv.spo2);
      myNextion.writeNum("n2.val", sv.fp);
      myNextion.writeNum("nSBP.val", sv.sbp);
      myNextion.writeNum("nDBP.val", sv.dbp);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =========================================================================
// Setup & Loop
// =========================================================================
void setup() {
  Serial.begin(115200);       // USB: solo depuracion / comando CAL
  Serial.setTimeout(10);
  myNextion.begin(BAUD_NEXTION);
  // EasyNex abre Serial2 con los pines por defecto (16/17): se reabre con los nuestros
  Serial2.end();
  Serial2.begin(BAUD_NEXTION, SERIAL_8N1, NEXTION_RX, NEXTION_TX);

  while (!afe.configure(SCLK, MISO, MOSI, CS, AFE_RST, FIFO_RDY)) {
    Serial.println("Error con la configuracion del AFE4950");
    delay(500);
  }
  afe.enableSpO2(true);       // agrega ESTADO + AC/DC IR + AC/DC RED cada segundo

  colaMuestras = xQueueCreate(128, sizeof(MuestraTriada));
  colaGrafica  = xQueueCreate(64,  sizeof(MuestraTriada));
  colaSpo2     = xQueueCreate(4,   sizeof(DatosSpO2));
  buzonSignos  = xQueueCreate(1,   sizeof(SignosVitales));
  assert(colaMuestras && colaGrafica && colaSpo2 && buzonSignos);

  xTaskCreatePinnedToCore(tareaAdquisicion,   "Adquisicion",   4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(tareaProcesamiento, "Procesamiento", 8192, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(tareaDisplay,       "NextionUI",     4096, NULL, 2, NULL, 0);

  afe.enableCapture();        // inicia la captura

  Serial.println("Escribe CAL <SBP> <DBP> en la consola para calibrar (ej. CAL 120 80)");
}

void loop() {
  vTaskDelete(NULL);
}

