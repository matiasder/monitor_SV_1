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
#define MISO     19     // OJO: el codigo del companero usa 26. Dejar el pin al que esta cableado.
#define MOSI     23
#define CS        5
#define AFE_RST   4
#define FIFO_RDY 16

// Nextion en Serial (UART0 / USB)
#define BAUD_NEXTION 115200
#define ID_WAVE_ECG  8
#define ID_WAVE_PPG  7
#define CANAL        0
#define ALTURA_WAVE  200

// =========================================================================
// Formato de paquete del AFE4950
// Palabra de 4 bytes: [LSB][MID][MSB][TAG]
// =========================================================================
#define TAG_HEADER   0x02
#define TAG_PDIFF    0x03
#define TAG_SAMPLE   0x05
#define TAG_STATUS   0x06   // byte0 = ST, byte1 = SEG
#define TAG_AC_IR    0x07
#define TAG_DC_IR    0x08
#define TAG_AC_RED   0x09
#define TAG_DC_RED   0x0A

// =========================================================================
// Muestreo y filtros
// FS_SISTEMA = ciclos (ECG,ECG,PPG) por segundo.
// Con el diagnostico: ~1251 muestras por slot en 5 s -> ~250 ciclos/s
// (ECG a 500 Hz, PPG a 250 Hz). CONFIRMAR con la linea "Ciclos por s" de
// AFE_Diagnostico.ino; si dice ~500, volver a los valores de la derecha.
// =========================================================================
#define FS_SISTEMA 250.0f     // (500.0f)
#define ALPHA_HP   0.0126f    // HPF ~0.5 Hz @ 250 Hz        (0.0063f @ 500 Hz)
#define ALPHA_LPF  0.5f       // LPF ~28 Hz  @ 250 Hz        (0.3f    @ 500 Hz)
#define DIV_GRAFICA 1         // 1 = dibuja cada ciclo (250 Hz) (2 @ 500 Hz)
#define PPG_SIGN   (+1.0f)    // +1.0f o -1.0f segun la polaridad del sensor

// SpO2 = A - B*R
#define SPO2_A       110.0f
#define SPO2_B       25.0f
#define SPO2_MIN_PCT 80.0f    // por debajo se considera senal mala (igual que el companero)
#define NSPO2        5

#define USAR_PIR   1
#define N_BUF      2048       // ~8 s de historia a 250 Hz
#define MS_TO_SAMP(x) ((uint32_t)((x) * FS_SISTEMA / 1000.0f))
#define B_BUF(buf, i) buf[(i) % N_BUF]
// Mitad de la ventana (en muestras) para la pendiente del PPG: ~10 ms
#define PEND_N ((uint32_t)(5.0f * FS_SISTEMA / 500.0f + 0.5f))

// =========================================================================
// Estructuras
// =========================================================================
struct MuestraTriada {
  float    ecg;        // ECG filtrado y sin baseline
  float    ppg;        // PPG AC filtrado
  int32_t  rawECG;
  int32_t  rawPPG;     // PPG crudo (incluye DC) -> para PIR
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
QueueHandle_t colaGrafica;    // Adq -> Display
QueueHandle_t colaSpo2;       // Adq -> Proc (1 Hz)
QueueHandle_t buzonSignos;    // Proc -> Display (mailbox)

EasyNex myNextion(Serial);

void procesarLatido(uint32_t R, uint32_t Rsig);
void revisarYPromediar(float ptt, float pir);
void calcularPA(float ptt, float pir, SignosVitales &sv);
void actualizarSpO2(const DatosSpO2 &d);

// =========================================================================
// Utilidades de muestreo y filtrado
// =========================================================================
static inline int32_t valor24Signed(const uint8_t *w) {
  int32_t v = ((int32_t)w[2] << 16) | ((int32_t)w[1] << 8) | w[0];
  if (v & 0x800000) v |= 0xFF000000;   // complemento a 2
  return v;
}

static inline uint32_t valor24Unsigned(const uint8_t *w) {
  return ((uint32_t)w[2] << 16) | ((uint32_t)w[1] << 8) | w[0];
}

// Filtro paso bajo IIR de 1 polo (ver ALPHA_LPF)
static inline float filtrarLPF(float in, float &outPrev) {
  outPrev += ALPHA_LPF * (in - outPrev);
  return outPrev;
}

void agregarPuntoGrafica(int idWave, int canal, int valor) {
  Serial.print("add ");
  Serial.print(idWave);
  Serial.print(",");
  Serial.print(canal);
  Serial.print(",");
  Serial.print(valor);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

// Autoescala adaptativa para la pantalla Nextion
struct AutoEscala { float lo = -100.0f, hi = 100.0f; };

uint8_t escalar(AutoEscala &s, float v) {
  if (v < s.lo) s.lo = v; else s.lo += (v - s.lo) * 0.001f;
  if (v > s.hi) s.hi = v; else s.hi -= (s.hi - v) * 0.001f;
  float span = s.hi - s.lo;
  if (span < 1.0f) span = 1.0f;

  // Inversion de coordenadas Y para Nextion (0 arriba, 199 abajo)
  int y = (ALTURA_WAVE - 1) - (int)((v - s.lo) / span * (ALTURA_WAVE - 1));
  return (uint8_t)constrain(y, 0, ALTURA_WAVE - 1);
}

static void emitirMuestra(float ecgProm, int32_t ppgRaw) {
  static uint8_t divisorGrafica = 0;
  static float ecgBase = 0, ppgBase = 0;
  static float lpfEcg = 0, lpfPpg = 0;
  static bool baseInit = false;

  if (!baseInit) {
    ecgBase = ecgProm;
    ppgBase = (float)ppgRaw;
    lpfEcg = 0;
    lpfPpg = 0;
    baseInit = true;
  }

  // 1. Quitar offset DC / baseline
  ecgBase += ALPHA_HP * (ecgProm - ecgBase);
  ppgBase += ALPHA_HP * ((float)ppgRaw - ppgBase);

  float ecgSinBase = ecgProm - ecgBase;
  float ppgSinBase = ((float)ppgRaw - ppgBase) * PPG_SIGN;

  // 2. Filtrar ruido de alta frecuencia
  MuestraTriada m;
  m.ecg = filtrarLPF(ecgSinBase, lpfEcg);
  m.ppg = filtrarLPF(ppgSinBase, lpfPpg);
  m.rawECG = (int32_t)ecgProm;
  m.rawPPG = ppgRaw;
  m.sampleIdx = totalMuestras;

  B_BUF(ecgBuf, m.sampleIdx) = (int32_t)m.ecg;
  B_BUF(ppgBuf, m.sampleIdx) = (int32_t)m.ppg;   // AC -> PTT
  B_BUF(dcBuf,  m.sampleIdx) = ppgRaw;           // DC -> PIR
  totalMuestras++;

  xQueueSendToBack(colaMuestras, &m, 0);

  if (++divisorGrafica >= DIV_GRAFICA) {
    divisorGrafica = 0;
    xQueueSendToBack(colaGrafica, &m, 0);
  }
}

// =========================================================================
// Tarea 1: Adquisicion y decodificacion del AFE4950 (Core 1, prio 5)
//
// Cambios respecto a la version anterior (tomados del codigo que si funciona):
//  - La fase del ciclo ECG,ECG,PPG CONTINUA entre paquetes: la cabecera ya no
//    la reinicia (un paquete puede traer un numero de muestras que no es
//    multiplo de 3).
//  - Se descartan palabras con TAG invalido y se reinicia la secuencia.
//  - El bloque de SpO2 (0x06..0x0A) solo se acepta si llega completo y en orden.
// =========================================================================
void tareaAdquisicion(void *pvParameters) {
  uint8_t fase = 0;            // 0 = ECG1, 1 = ECG2, 2 = PPG (continua entre paquetes)
  float ecg1 = 0, ecg2 = 0;
  uint8_t spoNext = 0;         // proximo TAG esperado del bloque SpO2 (0 = ninguno)
  DatosSpO2 spo2 = {};

  for (;;) {
    if (!afe.ready()) {
      vTaskDelay(1);
      continue;
    }

    const uint8_t *data = afe.getData();
    size_t len = afe.getBytesToSend();

    for (size_t o = 0; o + 4 <= len; o += 4) {
      const uint8_t *w = data + o;
      uint8_t tag = w[3];

      bool tagValido = (tag == TAG_HEADER || tag == TAG_PDIFF ||
                        (tag >= TAG_SAMPLE && tag <= TAG_DC_RED));
      if (!tagValido) {          // palabra corrupta: reinicia secuencias
        fase = 0;
        spoNext = 0;
        continue;
      }

      // ---- Bloque de oximetria ----
      if (tag == TAG_STATUS) {
        spo2 = DatosSpO2();      // todo a 0
        spo2.estado = w[0];
        spo2.seg    = w[1];
        spoNext = TAG_AC_IR;
        continue;
      }
      if (tag > TAG_STATUS) {    // 0x07..0x0A
        if (tag != spoNext) {    // fuera de secuencia: se descarta el bloque
          spoNext = 0;
          continue;
        }
        switch (tag) {
          case TAG_AC_IR:  spo2.acIr  = valor24Unsigned(w); break;
          case TAG_DC_IR:  spo2.dcIr  = valor24Unsigned(w); break;
          case TAG_AC_RED: spo2.acRed = valor24Unsigned(w); break;
          case TAG_DC_RED: spo2.dcRed = valor24Unsigned(w); break;
        }
        if (tag == TAG_DC_RED) {
          spoNext = 0;
          xQueueSendToBack(colaSpo2, &spo2, 0);   // bloque completo
        } else {
          spoNext++;
        }
        continue;
      }
      spoNext = 0;               // cualquier otra palabra interrumpe el bloque

      // ---- Muestras ECG, ECG, PPG ----
      if (tag != TAG_SAMPLE) continue;           // cabecera / POINTER_DIFF

      float v = (float)valor24Signed(w);
      if (fase == 0) {
        ecg1 = v;
        fase = 1;
      } else if (fase == 1) {
        ecg2 = v;
        fase = 2;
      } else {                   // 3.a muestra: PPG, cierra el ciclo
        emitirMuestra((ecg1 + ecg2) * 0.5f, (int32_t)v);
        fase = 0;
      }
    }
  }
}

// =========================================================================
// SpO2
// =========================================================================
void actualizarSpO2(const DatosSpO2 &d) {
  if (d.estado != 2 || d.acIr == 0 || d.dcIr == 0 || d.acRed == 0 || d.dcRed == 0) {
    nSpo2 = 0; iSpo2 = 0; spo2Prom = 0;
    return;
  }

  float R = ((float)d.acRed / (float)d.dcRed) / ((float)d.acIr / (float)d.dcIr);
  if (R < 0.2f || R > 3.0f) return;

  float s = SPO2_A - SPO2_B * R;
  if (s < SPO2_MIN_PCT) {        // senal mala: no se muestra
    nSpo2 = 0; iSpo2 = 0; spo2Prom = 0;
    return;
  }
  if (s > 100.0f) s = 100.0f;

  hSpo2[iSpo2] = s;
  iSpo2 = (iSpo2 + 1) % NSPO2;
  if (nSpo2 < NSPO2) nSpo2++;

  float suma = 0;
  for (int j = 0; j < nSpo2; j++) suma += hSpo2[j];
  spo2Prom = suma / nSpo2;
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

      uint32_t tUs = (uint32_t)(m.sampleIdx * (uint32_t)(1000000.0f / FS_SISTEMA));
      ppg_process_t(&ppgDet, m.ppg, tUs);
    }

    while (xQueueReceive(colaSpo2, &dsp, 0) == pdTRUE) {
      actualizarSpO2(dsp);
    }

    if ((xTaskGetTickCount() - ultActualizacionUI) >= pdMS_TO_TICKS(1000)) {
      ultActualizacionUI = xTaskGetTickCount();

      float bpmECG = pt_bpm(&pt);
      float bpmPPG = ppg_bpm(&ppgDet);

      sv.fc   = (bpmECG > 0.0f) ? (int)(bpmECG + 0.5f) : 0;
      sv.fp   = (bpmPPG > 0.0f) ? (int)(bpmPPG + 0.5f) : 0;
      sv.spo2 = (nSpo2 > 0) ? (int)(spo2Prom + 0.5f) : 0;

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
    int32_t pend = B_BUF(ppgBuf, i + PEND_N) - B_BUF(ppgBuf, i - PEND_N);
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
  if (USAR_PIR && (pir <= 1.0f || pir > 1.3f)) return;

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
// Tarea 3: Nextion UI (Core 0, prio 2)
// =========================================================================
void tareaDisplay(void *pvParameters) {
  MuestraTriada m;
  SignosVitales sv;

  AutoEscala escEcg;
  AutoEscala escPpg;

  for (;;) {
    myNextion.NextionListen();

    while (xQueueReceive(colaGrafica, &m, 0) == pdTRUE) {
      uint8_t valEcg = escalar(escEcg, m.ecg);
      uint8_t valPpg = escalar(escPpg, m.ppg);

      agregarPuntoGrafica(ID_WAVE_ECG, CANAL, valEcg);
      agregarPuntoGrafica(ID_WAVE_PPG, CANAL, valPpg);
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
  Serial.begin(BAUD_NEXTION);
  myNextion.begin(BAUD_NEXTION);

  while (!afe.configure(SCLK, MISO, MOSI, CS, AFE_RST, FIFO_RDY)) {
    delay(500);
  }
  afe.enableSpO2(true);

  colaMuestras = xQueueCreate(128, sizeof(MuestraTriada));
  colaGrafica  = xQueueCreate(64,  sizeof(MuestraTriada));
  colaSpo2     = xQueueCreate(4,   sizeof(DatosSpO2));
  buzonSignos  = xQueueCreate(1,   sizeof(SignosVitales));
  assert(colaMuestras && colaGrafica && colaSpo2 && buzonSignos);

  xTaskCreatePinnedToCore(tareaAdquisicion,   "Adquisicion",   4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(tareaProcesamiento, "Procesamiento", 8192, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(tareaDisplay,       "NextionUI",     4096, NULL, 2, NULL, 0);

  afe.enableCapture();
}

void loop() {
  vTaskDelete(NULL);
}

