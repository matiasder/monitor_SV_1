#include <Arduino.h>
#include <EasyNextionLibrary.h>

extern "C" {
  #include "biosignals.h"
}

// =========================================================================
// Configuración de Hardware y Nextion
// =========================================================================
#define BAUD_NEXTION 115200
#define ID_WAVE_ECG  8
#define ID_WAVE_PPG  7
#define CANAL        0
#define ALTURA_WAVE  200

const int PIN_ECG    = 34; // Salida AD8232
const int PIN_PPG    = 35; // PPG AC (pulso)
const int PIN_PPG_DC = 32; // PPG DC (antes del filtro pasa-altos)

#define FS_SISTEMA 500.0f  // Frecuencia de muestreo requerida para PTT
#define USAR_PIR   1
#define N_BUF      2048    // Buffer circular para ~4 s de historia
#define MS_TO_SAMP(x) ((uint32_t)((x) * FS_SISTEMA / 1000.0f))
#define B_BUF(buf, i) buf[(i) % N_BUF]

// =========================================================================
// Estructuras de Datos
// =========================================================================
struct MuestraTriada {
  float ecg;
  float ppg;
  uint16_t dcMv;
  uint16_t crudoECG;
  uint16_t crudoPPG;
  uint32_t sampleIdx;
};

struct SignosVitales {
  int fc;
  int fp;
  int spo2;
  int sbp;
  int dbp;
  bool calibrado;
};

// =========================================================================
// Variables Globales para PTT / PIR & DSP
// =========================================================================
uint16_t ecgBuf[N_BUF], ppgBuf[N_BUF], dcBuf[N_BUF];
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

// =========================================================================
// Handles de FreeRTOS
// =========================================================================
TaskHandle_t hTareaAdq = NULL;
QueueHandle_t colaMuestras; // Tarea Adq -> Tarea Proc
QueueHandle_t colaGrafica;  // Tarea Adq -> Tarea Display (submuestreado)
QueueHandle_t buzonSignos;  // Tarea Proc -> Tarea Display (Mailbox)

hw_timer_t *timerMuestreo = NULL;
EasyNex myNextion(Serial);

// Prototipos
void procesarLatido(uint32_t R, uint32_t Rsig);
void revisarYPromediar(float ptt, float pir);
void calcularPA(float ptt, float pir, SignosVitales &sv);

// =========================================================================
// ISR Timer (500 Hz / 2000 us)
// =========================================================================
void IRAM_ATTR onTimerMuestreo() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(hTareaAdq, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
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

// =========================================================================
// Tarea 1: Adquisición de Datos (Core 1 - Prioridad 5)
// =========================================================================
void tareaAdquisicion(void *pvParameters) {
  static uint8_t divisorGrafica = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    MuestraTriada m;
    m.sampleIdx = totalMuestras;
    m.crudoECG = analogRead(PIN_ECG);
    m.crudoPPG = analogRead(PIN_PPG);
    m.dcMv     = analogReadMilliVolts(PIN_PPG_DC);

    m.ecg = (float)m.crudoECG - 2048.0f;
    m.ppg = (float)m.crudoPPG - 2048.0f;

    // Almacenar directamente en buffers circulares para cálculo de PTT/PIR
    B_BUF(ecgBuf, m.sampleIdx) = m.crudoECG;
    B_BUF(ppgBuf, m.sampleIdx) = m.crudoPPG;
    B_BUF(dcBuf, m.sampleIdx)  = m.dcMv;
    totalMuestras++;

    // Enviar ráfaga completa a la tarea DSP
    xQueueSendToBack(colaMuestras, &m, 0);

    // Submuestreo a 250 Hz para la pantalla (evita saturación del puerto serie)
    divisorGrafica++;
    if (divisorGrafica >= 2) {
      divisorGrafica = 0;
      xQueueSendToBack(colaGrafica, &m, 0);
    }
  }
}

// =========================================================================
// Tarea 2: Procesamiento y Estimación de PA (Core 1 - Prioridad 4)
// =========================================================================
void tareaProcesamiento(void *pvParameters) {
  pt_init(&pt, FS_SISTEMA);
  ppg_init(&ppgDet, FS_SISTEMA);

  MuestraTriada m;
  SignosVitales sv = {0, 0, 0, 0, 0, false};
  TickType_t ultActualizacionUI = xTaskGetTickCount();

  for (;;) {
    if (xQueueReceive(colaMuestras, &m, portMAX_DELAY) == pdTRUE) {
      // 1. Detección por Pan-Tompkins
      PT_Result r = pt_process(&pt, (float)m.crudoECG);
      if (r.beat) {
        // Ubicación exacta del pico R
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

      // 2. Detección PPG alternativa para Frecuencia de Pulso (FP)
      ppg_process_t(&ppgDet, m.ppg, micros());
    }

    // 3. Revisar comandos de calibración por Serial (e.g. CAL 120 80)
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

    // 4. Actualizar mailbox cada 1 segundo
    if ((xTaskGetTickCount() - ultActualizacionUI) >= pdMS_TO_TICKS(1000)) {
      ultActualizacionUI = xTaskGetTickCount();

      float bpmECG = pt_bpm(&pt);
      float bpmPPG = ppg_bpm(&ppgDet);

      sv.fc   = (bpmECG > 0.0f) ? (int)(bpmECG + 0.5f) : 0;
      sv.fp   = (bpmPPG > 0.0f) ? (int)(bpmPPG + 0.5f) : 0;
      sv.spo2 = 98; // Opcional / Marcador

      calcularPA(pttProm, pirProm, sv);
      xQueueOverwrite(buzonSignos, &sv);
    }
  }
}

// =========================================================================
// Algoritmo PTT / PIR & Modelo de Presión (Ding et al., 2017)
// =========================================================================
void procesarLatido(uint32_t R, uint32_t Rsig) {
  float rr = (Rsig - R) * 1000.0f / FS_SISTEMA;
  if (rr < 333 || rr > 2000) return;

  // Búsqueda del punto de máxima pendiente del PPG (Llegada del pulso)
  uint32_t fin = R + MS_TO_SAMP(500);
  if (fin > Rsig) fin = Rsig;

  uint32_t k = R + MS_TO_SAMP(100);
  int mejor = -10000;

  for (uint32_t i = R + MS_TO_SAMP(100); i < fin; i++) {
    int pend = (int)B_BUF(ppgBuf, i + 5) - (int)B_BUF(ppgBuf, i - 5);
    if (pend > mejor) {
      mejor = pend;
      k = i;
    }
  }

  if (mejor <= 0) return;

  float ptt = (k - R) * 1000.0f / FS_SISTEMA;
  float pir = 1.0f;

#if USAR_PIR
  uint16_t IH = 0, IL = 65535;
  for (uint32_t i = R; i <= k; i++) {
    if (B_BUF(dcBuf, i) > IH) IH = B_BUF(dcBuf, i);
  }

  uint32_t finL = k + MS_TO_SAMP(300);
  if (finL > Rsig) finL = Rsig;

  for (uint32_t i = k; i <= finL; i++) {
    if (B_BUF(dcBuf, i) < IL) IL = B_BUF(dcBuf, i);
  }

  if (IL == 0) return;
  pir = (float)IH / IL;
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
      Serial.printf("Calibracion Lista: PTT0=%.1f ms, PIR0=%.4f\n", PTTO, PIRO);
    }
  }
}

void calcularPA(float ptt, float pir, SignosVitales &sv) {
  sv.calibrado = calibrado;
  if (!calibrado || ptt == 0) {
    sv.sbp = 0;
    sv.dbp = 0;
    return;
  }

  float PPO  = SBPO - DBPO;
  float MBPO = DBPO + PPO / 3.0f;
  float r    = PTTO / ptt;

  float mbp = MBPO * (PIRO / pir);
  float pp  = PPO * (pir / PIRO) * r * r;

  sv.sbp = (int)(mbp + 2.0f * pp / 3.0f + 0.5f);
  sv.dbp = (int)(mbp - pp / 3.0f + 0.5f);
}

// =========================================================================
// Tarea 3: UI & Pantalla Nextion (Core 0 - Prioridad 2)
// =========================================================================
void tareaDisplay(void *pvParameters) {
  MuestraTriada m;
  SignosVitales sv;

  for (;;) {
    myNextion.NextionListen();

    // 1. Dibujar formas de onda submuestreadas
    while (xQueueReceive(colaGrafica, &m, 0) == pdTRUE) {
      uint8_t valorECG = map(m.crudoECG, 0, 4095, 0, ALTURA_WAVE - 1);
      uint8_t valorPPG = map(m.crudoPPG, 0, 4095, 0, ALTURA_WAVE - 1);

      agregarPuntoGrafica(ID_WAVE_ECG, CANAL, valorECG);
      agregarPuntoGrafica(ID_WAVE_PPG, CANAL, valorPPG);
    }

    // 2. Transmitir Signos Vitales y Presión Arterial a la pantalla Nextion
    if (xQueueReceive(buzonSignos, &sv, 0) == pdTRUE) {
      myNextion.writeNum("n0.val", sv.fc);
      myNextion.writeNum("n1.val", sv.spo2);
      myNextion.writeNum("n2.val", sv.fp);

      myNextion.writeNum("nSBP.val", sv.sbp);
      myNextion.writeNum("nDBP.val", sv.dbp);
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // ~50 Hz ciclo de refresco UI
  }
}

// =========================================================================
// Setup & Loop
// =========================================================================
void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  myNextion.begin(BAUD_NEXTION);

  // Crear buffers de comunicación inter-tarea
  colaMuestras = xQueueCreate(128, sizeof(MuestraTriada));
  colaGrafica  = xQueueCreate(64, sizeof(MuestraTriada));
  buzonSignos  = xQueueCreate(1, sizeof(SignosVitales));

  assert(colaMuestras && colaGrafica && buzonSignos);

  // Creación de tareas asignando núcleos y prioridades
  xTaskCreatePinnedToCore(tareaAdquisicion,  "Adquisicion",  4096, NULL, 5, &hTareaAdq, 1);
  xTaskCreatePinnedToCore(tareaProcesamiento, "Procesamiento", 8192, NULL, 4, NULL,       1);
  xTaskCreatePinnedToCore(tareaDisplay,       "NextionUI",     4096, NULL, 2, NULL,       0);

  // Configuración del Timer Hardware a 500 Hz (2000 us)
  timerMuestreo = timerBegin(0, 80, true);
  timerAttachInterrupt(timerMuestreo, &onTimerMuestreo, true);
  timerAlarmWrite(timerMuestreo, 2000, true); // 1000000 us / 500 Hz = 2000 us
  timerAlarmEnable(timerMuestreo);

  Serial.println("Escribe CAL <SBP> <DBP> en la consola para calibrar (ej. CAL 120 80)");
}

void loop() {
  vTaskDelete(NULL);
}