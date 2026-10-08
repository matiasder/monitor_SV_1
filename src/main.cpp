#include <Arduino.h>
#include <EasyNextionLibrary.h>
extern "C" {
  #include "biosignals.h"
}

// ---------- Nextion ----------
#define BAUD_NEXTION 115200
#define ID_WAVE_ECG 8   // ID del Waveform de ECG en Nextion
#define ID_WAVE_PPG 7   // ID del Waveform de PPG en Nextion
#define CANAL 0
#define ALTURA_WAVE 200 // Rango vertical del Waveform (0 a 199 px)
#define RX1_PIN 16
#define TX1_PIN 17

// ---------- ADC ----------
const int PIN_ECG = 34; // Salida analógica del AD8232 (o similar)
const int PIN_PPG = 35; // Sensor PPG

// ---------- Temporización ----------
const unsigned long PERIODO_GRAFICA = 20;       // 20 ms = 50 Hz, refresco visual
const unsigned long PERIODO_INDICADORES = 1000; // 1 Hz, campos numéricos

// Frecuencia de muestreo REAL para ambos detectores (ECG y PPG comparten timer)
#define FS_DETECTOR 250.0f

EasyNex myNextion(Serial);
PanTompkins pt;   // detector de QRS (ECG)
PPGDetector ppg;  // detector de pulso (PPG)

unsigned long tgrafica = 0;
unsigned long tindicadores = 0;

// ---- Muestreo por timer: la ISR solo marca bandera + timestamp preciso ----
// (NO se hace analogRead dentro de la ISR: no es seguro en ESP32)
hw_timer_t *timerMuestreo = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool banderaMuestra = false;
volatile uint32_t tsMuestra_us = 0;

void IRAM_ATTR onTimerMuestreo() {
  portENTER_CRITICAL_ISR(&timerMux);
  tsMuestra_us = micros(); // instante exacto de la interrupción (preciso)
  banderaMuestra = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void agregarPuntoGrafica(int idWave, int canal, int valor) {
  Serial1.print("add ");
  Serial1.print(idWave);
  Serial1.print(",");
  Serial1.print(canal);
  Serial1.print(",");
  Serial1.print(valor);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
  Serial1.write(0xFF);
}

void setup() {
  Serial.begin(115200); // debug, libre (Nextion usa Serial1)

  myNextion.begin(BAUD_NEXTION);
  // Serial1.begin(BAUD_NEXTION, SERIAL_8N1, RX1_PIN, TX1_PIN); // reconfigura DESPUÉS de myNextion.begin()

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pt_init(&pt, FS_DETECTOR);
  ppg_init(&ppg, FS_DETECTOR);

  // Un solo timer a FS_DETECTOR Hz, compartido por ambas señales.
  // Prescaler 80 sobre clock de 80MHz -> 1 tick = 1us
  timerMuestreo = timerBegin(0, 80, true);
  timerAttachInterrupt(timerMuestreo, &onTimerMuestreo, true);
  timerAlarmWrite(timerMuestreo, (uint64_t)(1000000.0f / FS_DETECTOR), true);
  timerAlarmEnable(timerMuestreo);

  delay(500);
}

void loop() {
  myNextion.NextionListen();
  unsigned long ahora = millis();

  // --- Procesar una muestra de CADA señal cuando el timer lo solicita ---
  if (banderaMuestra) {
    uint32_t t_us;
    portENTER_CRITICAL(&timerMux);
    t_us = tsMuestra_us;
    banderaMuestra = false;
    portEXIT_CRITICAL(&timerMux);

    int crudoECG = analogRead(PIN_ECG); // lecturas fuera de la ISR, seguras
    int crudoPPG = analogRead(PIN_PPG);

    float muestraECG = (float)crudoECG - 2048.0f; // centra respecto a la línea base
    float muestraPPG = (float)crudoPPG - 2048.0f;

    pt_process_t(&pt, muestraECG, t_us);
    ppg_process_t(&ppg, muestraPPG, t_us);
  }

  // --- Graficar ambas señales en Nextion a 25 Hz (visual, datos reales del ADC) ---
  if (ahora - tgrafica >= PERIODO_GRAFICA) {
    tgrafica = ahora;

    int crudoECG = analogRead(PIN_ECG);
    int crudoPPG = analogRead(PIN_PPG);
    uint8_t valorECG = map(crudoECG, 0, 4095, 0, ALTURA_WAVE - 1);
    uint8_t valorPPG = map(crudoPPG, 0, 4095, 0, ALTURA_WAVE - 1);

    agregarPuntoGrafica(ID_WAVE_ECG, CANAL, valorECG);
    agregarPuntoGrafica(ID_WAVE_PPG, CANAL, valorPPG);
  }

  // --- Actualizar campos numéricos con FC (ECG) y FP (PPG) reales ---
  if (ahora - tindicadores >= PERIODO_INDICADORES) {
    tindicadores = ahora;

    float bpmECG = pt_bpm(&pt);
    float bpmPPG = ppg_bpm(&ppg);

    int fc = (bpmECG > 0.0f) ? (int)(bpmECG + 0.5f) : 0;
    int fp = (bpmPPG > 0.0f) ? (int)(bpmPPG + 0.5f) : 0;
    int spo2 = 00; // pendiente: algoritmo real de SpO2 (razón roja/IR)

    myNextion.writeNum("n0.val", fc);
    myNextion.writeNum("n1.val", spo2);
    myNextion.writeNum("n2.val", fp);
  }
}