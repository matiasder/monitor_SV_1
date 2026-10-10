#include <Arduino.h>
#include <EasyNextionLibrary.h>

extern "C" {
  #include "biosignals.h"
}

// ---------- Configuración de Hardware y Nextion ----------
#define BAUD_NEXTION 115200
#define ID_WAVE_ECG  8
#define ID_WAVE_PPG  7
#define CANAL        0
#define ALTURA_WAVE  200

const int PIN_ECG = 34;
const int PIN_PPG = 35;
#define FS_DETECTOR 250.0f

// ---------- Estructuras de Datos para Colas ----------
struct MuestraDual {
  float ecg;
  float ppg;
  uint32_t timestamp_us;
  uint16_t crudoECG;
  uint16_t crudoPPG;
};

struct SignosVitales {
  int fc;
  int fp;
  int spo2;
};

// ---------- Handles de FreeRTOS ----------
TaskHandle_t hTareaAdq = NULL;
QueueHandle_t colaMuestras; // Tarea Adq -> Tarea Proc
QueueHandle_t colaGrafica;  // Tarea Adq -> Tarea Display
QueueHandle_t buzonSignos;  // Tarea Proc -> Tarea Display (Mailbox de tamaño 1)

hw_timer_t *timerMuestreo = NULL;
EasyNex myNextion(Serial);

// ---------- ISR del Timer de Hardware ----------
void IRAM_ATTR onTimerMuestreo() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Notifica a la tarea de adquisición de prioridad alta para que despierte de inmediato
  vTaskNotifyGiveFromISR(hTareaAdq, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// Auxiliar para enviar comandos puros a Nextion
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
  for (;;) {
    // Espera la notificación del Timer de Hardware (exactamente a 250 Hz)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    MuestraDual m;
    m.timestamp_us = micros();
    m.crudoECG = analogRead(PIN_ECG);
    m.crudoPPG = analogRead(PIN_PPG);
    m.ecg = (float)m.crudoECG - 2048.0f;
    m.ppg = (float)m.crudoPPG - 2048.0f;

    // Enviar muestra a la tarea de procesamiento
    xQueueSendToBack(colaMuestras, &m, 0);

    // Enviar a la cola gráfica descartando muestras antiguas si se llena
    xQueueSendToBack(colaGrafica, &m, 0);
  }
}

// =========================================================================
// Tarea 2: Procesamiento de Señales / Algoritmos (Core 1 - Prioridad 4)
// =========================================================================
void tareaProcesamiento(void *pvParameters) {
  PanTompkins pt;
  PPGDetector ppg;
  
  pt_init(&pt, FS_DETECTOR);
  ppg_init(&ppg, FS_DETECTOR);

  MuestraDual m;
  TickType_t ultActualizacionUI = xTaskGetTickCount();

  for (;;) {
    // Bloqueado hasta recibir una nueva muestra desde la cola
    if (xQueueReceive(colaMuestras, &m, portMAX_DELAY) == pdTRUE) {
      pt_process_t(&pt, m.ecg, m.timestamp_us);
      ppg_process_t(&ppg, m.ppg, m.timestamp_us);
    }

    // Actualizar el Mailbox de signos vitales cada 1 segundo (1000 ms)
    if ((xTaskGetTickCount() - ultActualizacionUI) >= pdMS_TO_TICKS(1000)) {
      ultActualizacionUI = xTaskGetTickCount();

      float bpmECG = pt_bpm(&pt);
      float bpmPPG = ppg_bpm(&ppg);

      SignosVitales sv;
      sv.fc = (bpmECG > 0.0f) ? (int)(bpmECG + 0.5f) : 0;
      sv.fp = (bpmPPG > 0.0f) ? (int)(bpmPPG + 0.5f) : 0;
      sv.spo2 = 0; 

      // Sobrescribir siempre con el último dato calculado (Mailbox)
      xQueueOverwrite(buzonSignos, &sv);
    }
  }
}

// =========================================================================
// Tarea 3: Gestión de Pantalla y UI Nextion (Core 0 - Prioridad 2)
// =========================================================================
void tareaDisplay(void *pvParameters) {
  MuestraDual m;
  SignosVitales sv;

  for (;;) {
    // Escuchar eventos de la pantalla
    myNextion.NextionListen();

    // 1. Dibujar gráficos en la Nextion si hay datos en la cola
    while (xQueueReceive(colaGrafica, &m, 0) == pdTRUE) {
      uint8_t valorECG = map(m.crudoECG, 0, 4095, 0, ALTURA_WAVE - 1);
      uint8_t valorPPG = map(m.crudoPPG, 0, 4095, 0, ALTURA_WAVE - 1);

      agregarPuntoGrafica(ID_WAVE_ECG, CANAL, valorECG);
      agregarPuntoGrafica(ID_WAVE_PPG, CANAL, valorPPG);
    }

    // 2. Leer los signos vitales actualizados si existen en el buzón
    if (xQueueReceive(buzonSignos, &sv, 0) == pdTRUE) {
      myNextion.writeNum("n0.val", sv.fc);
      myNextion.writeNum("n1.val", sv.spo2);
      myNextion.writeNum("n2.val", sv.fp);
    }

    // Liberar la CPU del Core 0 un instante (20 ms ≈ refresco de 50 Hz para la pantalla)
    vTaskDelay(pdMS_TO_TICKS(20));
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

  // Crear colas y buzones
  colaMuestras = xQueueCreate(64, sizeof(MuestraDual));  // Buffer para ráfagas de procesamiento
  colaGrafica  = xQueueCreate(32, sizeof(MuestraDual));  // Buffer para la gráfica visual
  buzonSignos  = xQueueCreate(1, sizeof(SignosVitales)); // Mailbox para mantener el último valor

  assert(colaMuestras && colaGrafica && buzonSignos);

  // Crear tareas asignando prioridades y núcleos según la arquitectura propuesta
  xTaskCreatePinnedToCore(tareaAdquisicion,  "Adquisicion",   4096, NULL, 5, &hTareaAdq, 1); // Core 1 (Tiempo Real)
  xTaskCreatePinnedToCore(tareaProcesamiento, "Procesamiento", 8192, NULL, 4, NULL,       1); // Core 1 (DSP / Algoritmos)
  xTaskCreatePinnedToCore(tareaDisplay,       "NextionUI",     4096, NULL, 2, NULL,       0); // Core 0 (Comunicaciones y UI)

// Timer 0, prescaler 80 (80MHz/80 = 1MHz -> 1 tick = 1 us)
  timerMuestreo = timerBegin(0, 80, true);
  timerAttachInterrupt(timerMuestreo, &onTimerMuestreo, true);
  
  // 1000000 us / 250 Hz = 4000 ticks de 1 us
  timerAlarmWrite(timerMuestreo, 4000, true); 
  timerAlarmEnable(timerMuestreo);
}

void loop() {
  // El loop principal no realiza ningún trabajo; FreeRTOS administra el sistema.
  vTaskDelete(NULL);
}