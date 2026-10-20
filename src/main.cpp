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
#define FIFO_RDY 25   // Movido desde GPIO16: Serial2 (RX2) lo usa ahora la Nextion

// Nextion en UART2 (RX2=GPIO16, TX2=GPIO17). Serial (USB) queda libre
// para depuracion y el comando CAL.
#define BAUD_NEXTION 115200
#define ID_WAVE_ECG  8
#define ID_WAVE_PPG  7
#define CANAL        0
#define ALTURA_WAVE  200

// =========================================================================
// Formato de datos del AFE  (*** CONFIRMAR CON AFE4950.h ***)
// Se asume, por muestra: [ECG 3 bytes][PPG 3 bytes], big-endian, signed 24 bit
// =========================================================================
#define AFE_HEADER_BYTES   0   // bytes de cabecera al inicio de getData(), si los hay
#define AFE_BYTES_PER_CH   3
#define AFE_BYTES_PER_SAMP (2 * AFE_BYTES_PER_CH)

#define PPG_SIGN  (+1.0f)      // poner -1.0f si el pulso sale invertido
#define ALPHA_HP  0.0063f      // HPF de 1 polo ~0.5 Hz @ 500 Hz (quita DC / deriva)

#define FS_SISTEMA 500.0f
#define USAR_PIR   1
#define N_BUF      2048        // ~4 s de historia
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

struct SignosVitales {
  int fc, fp, spo2, sbp, dbp;
  bool calibrado;
};

// =========================================================================
// Globales
// =========================================================================
AFE4950 afe;

int32_t ecgBuf[N_BUF], ppgBuf[N_BUF], dcBuf[N_BUF];   // int32: el AFE entrega 22-24 bit
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

QueueHandle_t colaMuestras;   // Adq -> Proc
QueueHandle_t colaGrafica;    // Adq -> Display (250 Hz)
QueueHandle_t buzonSignos;    // Proc -> Display (mailbox)

EasyNex myNextion(Serial2);

void procesarLatido(uint32_t R, uint32_t Rsig);
void revisarYPromediar(float ptt, float pir);
void calcularPA(float ptt, float pir, SignosVitales &sv);

// =========================================================================
// Utilidades
// =========================================================================
static inline int32_t leerInt24(const uint8_t *p) {
  int32_t v = ((int32_t)p[0] << 16) | ((int32_t)p[1] << 8) | p[2];
  if (v & 0x800000) v |= 0xFF000000;   // extension de signo
  return v;
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

// Autoescala lenta para la pantalla (el AFE no tiene rango fijo 0-4095)
struct AutoEscala { float lo = 0, hi = 1; };

uint8_t escalar(AutoEscala &s, float v) {
  if (v < s.lo) s.lo = v; else s.lo += (v - s.lo) * 0.001f;
  if (v > s.hi) s.hi = v; else s.hi -= (s.hi - v) * 0.001f;
  float span = s.hi - s.lo;
  if (span < 1.0f) span = 1.0f;
  int y = (int)((v - s.lo) / span * (ALTURA_WAVE - 1));
  return (uint8_t)constrain(y, 0, ALTURA_WAVE - 1);
}

// =========================================================================
// Tarea 1: Adquisicion desde la FIFO del AFE4950 (Core 1, prio 5)
// =========================================================================
void tareaAdquisicion(void *pvParameters) {
  static uint8_t divisorGrafica = 0;
  static float ecgBase = 0, ppgBase = 0;
  static bool baseInit = false;

  for (;;) {
    if (!afe.ready()) {          // watermark de 3 muestras -> ~cada 6 ms
      vTaskDelay(1);
      continue;
    }

    const uint8_t *data = afe.getData();
    size_t len = afe.getBytesToSend();
    if (len <= AFE_HEADER_BYTES) continue;

    const uint8_t *p = data + AFE_HEADER_BYTES;
    size_t nMuestras = (len - AFE_HEADER_BYTES) / AFE_BYTES_PER_SAMP;

    for (size_t s = 0; s < nMuestras; s++, p += AFE_BYTES_PER_SAMP) {
      MuestraTriada m;
      m.rawECG = leerInt24(p);
      m.rawPPG = leerInt24(p + AFE_BYTES_PER_CH);

      if (!baseInit) {
        ecgBase = m.rawECG;
        ppgBase = m.rawPPG;
        baseInit = true;
      }
      ecgBase += ALPHA_HP * (m.rawECG - ecgBase);
      ppgBase += ALPHA_HP * (m.rawPPG - ppgBase);

      m.ecg = (float)m.rawECG - ecgBase;
      m.ppg = ((float)m.rawPPG - ppgBase) * PPG_SIGN;
      m.sampleIdx = totalMuestras;

      B_BUF(ecgBuf, m.sampleIdx) = (int32_t)m.ecg;
      B_BUF(ppgBuf, m.sampleIdx) = (int32_t)m.ppg;   // AC  -> PTT
      B_BUF(dcBuf,  m.sampleIdx) = m.rawPPG;         // DC  -> PIR
      totalMuestras++;

      xQueueSendToBack(colaMuestras, &m, 0);

      if (++divisorGrafica >= 2) {                   // 250 Hz para la pantalla
        divisorGrafica = 0;
        xQueueSendToBack(colaGrafica, &m, 0);
      }
    }
  }
}

// =========================================================================
// Tarea 2: Procesamiento y estimacion de PA (Core 1, prio 4)
// =========================================================================
void tareaProcesamiento(void *pvParameters) {
  pt_init(&pt, FS_SISTEMA);
  ppg_init(&ppgDet, FS_SISTEMA);

  MuestraTriada m;
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

      // Timestamp derivado del indice de muestra (las muestras llegan en
      // rafagas de 3, micros() no refleja el instante real de muestreo)
      uint32_t tUs = (uint32_t)(m.sampleIdx * (uint32_t)(1000000.0f / FS_SISTEMA));
      ppg_process_t(&ppgDet, m.ppg, tUs);
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
      sv.spo2 = 98;   // marcador: un solo canal PPG no permite SpO2

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
      myNextion.writeNum("n3.val", sv.sbp);
      myNextion.writeNum("n4.val", sv.dbp);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =========================================================================
// Setup & Loop
// =========================================================================
void setup() {
  Serial.begin(115200);       // USB: solo depuracion / comando CAL
  Serial.setTimeout(10);      // readStringUntil no debe bloquear al DSP
  myNextion.begin(BAUD_NEXTION);

  while (!afe.configure(SCLK, MISO, MOSI, CS, AFE_RST, FIFO_RDY)) {
    Serial.println("Error con la configuracion del AFE4950");
    delay(500);
  }

  colaMuestras = xQueueCreate(128, sizeof(MuestraTriada));
  colaGrafica  = xQueueCreate(64,  sizeof(MuestraTriada));
  buzonSignos  = xQueueCreate(1,   sizeof(SignosVitales));
  assert(colaMuestras && colaGrafica && buzonSignos);

  xTaskCreatePinnedToCore(tareaAdquisicion,   "Adquisicion",   4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(tareaProcesamiento, "Procesamiento", 8192, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(tareaDisplay,       "NextionUI",     4096, NULL, 2, NULL, 0);

  afe.enableCapture();        // inicia la captura ECG+PPG a 500 Hz

  Serial.println("Escribe CAL <SBP> <DBP> en la consola para calibrar (ej. CAL 120 80)");
}

void loop() {
  vTaskDelete(NULL);
}
