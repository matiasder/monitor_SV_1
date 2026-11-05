# Monitor de Signos Vitales (ESP32 + AFE4950 + Nextion)

Monitor de signos vitales de bajo costo basado en un ESP32. Adquiere **ECG** y **PPG** con el front-end analógico **AFE4950** de Texas Instruments, mide **temperatura** con un termistor y muestra todo en una pantalla táctil **Nextion**.

Variables que muestra en pantalla:

| Variable | Fuente | Método |
|---|---|---|
| Frecuencia cardiaca (FC) | ECG | Pan-Tompkins + validación de intervalos RR |
| Frecuencia de pulso (FP) | PPG | Detector de pulsos propio (umbral adaptativo) |
| SpO2 | PPG rojo / infrarrojo | Relación de relaciones `R`, `SpO2 = 110 − 25·R` |
| Presión arterial (PANI) | ECG + PPG | Sin manguito: PTT + PIR con calibración ([ver sección](#presión-arterial-sin-manguito-ptt--pir)) |
| PTT | ECG + PPG | Tiempo del pico R a la máxima pendiente del pulso |
| Temperatura | NTC ST1307 | Puente + AD623 + Steinhart-Hart |

Además dibuja en tiempo real las ondas de ECG y PPG y tiene **alarmas de FC** con buzzer, LEDs y un margen que parpadea en la pantalla.

> ⚠️ Es un proyecto académico. **No es un dispositivo médico** y no debe usarse para diagnóstico ni para tomar decisiones clínicas.

---

## Hardware

- ESP32 (placa AZ-Delivery DevKit v4)
- AFE4950 (ECG + PPG + bloque de SpO2 en hardware)
- Pantalla Nextion (UART)
- Sensor de temperatura ST1307 (NTC YSI 400) con puente, REF3025, MCP6001 y AD623
- Buzzer activo y LEDs verde, ámbar y rojo

### Conexiones

| Función | Pin ESP32 | Notas |
|---|---|---|
| AFE4950 SCLK | 18 | SPI |
| AFE4950 MISO | 19 | |
| AFE4950 MOSI | 23 | |
| AFE4950 CS | 5 | |
| AFE4950 RESET | 4 | |
| AFE4950 FIFO_RDY | 16 | |
| Nextion RX (← TX de la pantalla) | 25 | UART2, 115200 baudios |
| Nextion TX (→ RX de la pantalla) | 33 | |
| Temperatura (salida del AD623) | 13 | ADC2: no se puede usar junto con WiFi |
| Buzzer | 12 | Pin de arranque: no debe quedar en HIGH al encender |
| LED verde (encendido) | 14 | |
| LED ámbar (aviso) | 27 | |
| LED rojo (peligro) | 26 | |

El USB (`Serial`) queda libre para depurar.

---

## Compilar y cargar

El proyecto usa [PlatformIO](https://platformio.org/).

1. Instala la librería del AFE4950. En [`platformio.ini`](platformio.ini) aparece como una ruta local (`file://...`); cámbiala por la ubicación de la librería en tu PC.
2. La librería [EasyNextionLibrary](https://github.com/Seithan/EasyNextionLibrary) se descarga sola.
3. Compila y carga:

```bash
pio run -t upload
pio device monitor        # 115200 baudios
```

4. Carga en la Nextion el `.tft` del proyecto HMI con el Nextion Editor. El archivo HMI no está en este repositorio.

---

## Arquitectura

El firmware corre sobre **FreeRTOS** con cinco tareas:

| Tarea | Núcleo | Prioridad | Qué hace |
|---|---|---|---|
| `tareaAdquisicion` | 1 | 5 | Lee el FIFO del AFE4950, decodifica las palabras (ECG, ECG, PPG y bloque de SpO2) y filtra las señales |
| `tareaProcesamiento` | 1 | 4 | FC, FP, SpO2, PTT/PIR, calibración y presión. Publica los resultados cada segundo |
| `tareaAlarmas` | 0 | 3 | Niveles de alarma de FC, buzzer, LEDs y margen de la pantalla (tick de 10 ms) |
| `tareaDisplay` | 0 | 2 | Logo de inicio, ondas, valores numéricos y lectura de los comandos de la Nextion |
| `tareaTemperatura` | 0 | 1 | Lectura del termistor una vez por segundo |

Las tareas se comunican con colas (`colaMuestras`, `colaGrafica`, `colaSpo2`, `colaCal`, `buzonSignos`). El puerto de la Nextion se protege con un mutex.

### Estructura del código

```
src/
├── main.cpp        Tareas, filtros, PTT/PA, alarmas, temperatura y comunicación con la Nextion
├── biosignals.c    Biquads (pasa-bajos, pasa-altos, pasa-banda, notch), Pan-Tompkins y detector PPG
└── biosignals.h
```

---

## Procesamiento de señales

Frecuencia de muestreo efectiva: **250 Hz**. El AFE entrega ciclos ECG, ECG, PPG y las dos muestras de ECG de cada ciclo se promedian.

Cadena de filtrado, igual para ECG y PPG:

1. **Pasa-altos ~0,5 Hz**: quita el offset y la deriva de la línea base.
2. **Notch de red**: 2 etapas a 60 Hz más una a 120 Hz (`ECG_NOTCH_Q`, `ECG_NOTCH_ETAPAS`, `ECG_NOTCH_120`, `PPG_NOTCH`). Es necesario cuando el equipo se alimenta de la red eléctrica y no por USB.
3. **Pasa-bajos ~28 Hz** (IIR de un polo).

Con la señal filtrada:

- **FC**: Pan-Tompkins (pasa-banda 5–15 Hz). Cada latido se valida por su intervalo RR (30–200 lpm) y la FC es la mediana de los últimos 8 RR. Si no hay latidos en 3 s, la FC vuelve a 0.
- **FP**: detector propio sobre la PPG, con amplitud mínima y periodo refractario de 300 ms. La FP es la mediana de los últimos 5 intervalos.
- **SpO2**: usa los valores AC/DC rojo e infrarrojo que calcula el AFE. Promedia 5 lecturas y no muestra nada si el valor baja de 80 %.
- **Sin dedo**: si se pierde la señal, FP, PTT y presión pasan a 0.

### Temperatura

- La salida del AD623 llega al GPIO13. Cada segundo se promedian 40 lecturas tomadas cada 5 ms. Son 200 ms, o sea 12 ciclos de 60 Hz, así que el ruido de la red se cancela.
- La lectura pasa por la inversión del puente y del AD623 y luego por Steinhart-Hart (curva YSI 44004).
- Si el sensor está abierto, en corto o fuera de 25–45 °C, se muestra 0.
- Las constantes del circuito (`T_VREF`, `T_VL`, `T_RS`, ...) y la corrección `T_CAL_GAIN` / `T_CAL_OFFSET` deben ajustarse con valores medidos.

---

## Presión arterial sin manguito (PTT + PIR)

El equipo estima la presión arterial latido a latido **sin manguito**, combinando el ECG y la PPG del dedo. Usa un modelo basado en el de Ding et al. (2016), que combina dos medidas:

- **PTT (Pulse Transit Time)**: el tiempo que tarda la onda de pulso en viajar del corazón al dedo. Cuando sube la presión, las arterias se ponen más rígidas, la onda viaja más rápido y el PTT baja. Este dato sigue sobre todo la **presión de pulso** (sistólica − diastólica).
- **PIR (PPG Intensity Ratio)**: la relación entre la luz máxima y la mínima que recibe el sensor en cada latido. Depende de cuánto se dilata la arteria y sigue sobre todo la **presión media** y el tono vascular.

Ninguna de las dos da mmHg por sí sola. Por eso el modelo necesita una **calibración** inicial con un tensiómetro convencional, y después sigue los cambios relativos respecto a ese punto.

### Medición en cada latido

Cada pico R que acepta la detección de FC dispara este cálculo en `procesarLatido()`:

1. **PTT**: se busca la máxima pendiente de subida de la PPG entre 100 y 500 ms después del R. La pendiente se calcula sobre una ventana de ±10 ms. El PTT es el tiempo entre el R y ese punto.
2. **PIR**: sobre la PPG cruda, que incluye la componente DC:
   - `IH` = intensidad máxima entre el R y el punto de máxima pendiente (pie del pulso),
   - `IL` = intensidad mínima en los 300 ms siguientes (pico del pulso),
   - `PIR = IH / IL`.
3. **Validación** en `revisarYPromediar()`:
   - el AFE debe reportar señal válida,
   - el PTT debe estar entre 150 y 450 ms y el PIR entre 1,0 y 1,3,
   - si el latido se desvía más del 20 % del promedio, se descarta; con 5 descartes seguidos (un cambio real, no ruido) el promedio se reinicia.
4. Los latidos válidos entran a un **promedio móvil de 8 latidos** de PTT y PIR.

### Modelo

Con los valores de calibración `SBP0`, `DBP0`, `PTT0` y `PIR0`:

```
PP0  = SBP0 − DBP0                      presión de pulso de referencia
MBP0 = DBP0 + PP0/3                     presión media de referencia

MBP  = MBP0 · (PIR0 / PIR)
PP   = PP0  · (PIR / PIR0) · (PTT0 / PTT)²

SBP  = MBP + 2·PP/3
DBP  = MBP −   PP/3
```

La pantalla se actualiza cada segundo con el PTT y el PIR promediados (función `calcularPA()`).

### Cómo calibrar

1. Coloca los electrodos del ECG y el dedo en el sensor de PPG. Espera a que la FC y la FP sean estables y a que el PTT (`n36`) muestre un valor.
2. Mide la presión con un tensiómetro de referencia, en la misma postura y con el brazo a la altura del corazón.
3. Ingresa la sistólica y la diastólica en la pantalla. La Nextion envía `BP:<sys>/<dia>\n`, por ejemplo `BP:120/80`.
4. El texto de estado (`t1`) muestra **"Calibrando: n/30"** y la barra `j0` avanza mientras se acumulan **30 latidos válidos**. Quédate quieto durante ese tiempo.
5. Al terminar, `PTT0` y `PIR0` toman el promedio de esos 30 latidos y el estado cambia a **"Calibrado"**. Desde ahí se muestran la sistólica (`n3`) y la diastólica (`n4`).

Mensajes posibles al calibrar:

| Mensaje | Causa |
|---|---|
| `Formato invalido` | La trama no tiene dos números |
| `Valores fuera de rango` | Sistólica fuera de 60–250, diastólica fuera de 30–150, o sistólica ≤ diastólica |
| `Coloque el dedo y espere` | No hay dedo en el sensor de PPG |

### Comportamiento

- **Sin calibrar**, o con FP = 0 (por ejemplo, sin dedo), la presión se muestra **0/0**.
- **Si se retira el dedo**, la calibración vigente **se conserva** y la medición sigue al volver a ponerlo. Una calibración en curso se cancela y el historial de PTT/PIR se rehace con la nueva colocación.
- **Una calibración nueva** reemplaza a la anterior solo cuando termina; mientras tanto se sigue usando la vigente.
- La calibración se guarda en RAM, así que **se pierde al apagar el equipo**.

### Limitaciones

- La estimación vale cerca del punto de calibración. Conviene recalibrar al cambiar de persona, de postura, de colocación del sensor o después de unos minutos de uso.
- El PTT medido desde el pico R incluye el periodo de pre-eyección del corazón, que no depende solo de la presión.
- El movimiento, la mano fría o una mala colocación de los electrodos alteran el PTT y el PIR. Por eso hay tantos filtros de validación.
- No está validado frente a un estándar clínico (por ejemplo, ISO 81060-2).

---

## Alarmas de FC

| Nivel | Límites predeterminados | Buzzer | Indicación visual |
|---|---|---|---|
| Normal | 50–120 lpm | — | LED verde |
| Aviso | < 50 o > 120 lpm | 3 pulsos lentos cada 4 s | LED y margen ámbar (~0,6 Hz) |
| Peligro | < 40 o > 140 lpm | 10 pulsos rápidos cada 2 s | LED y margen rojo (2 Hz) |

- Hace falta estar 3 s fuera de rango para subir de nivel. Bajar de nivel es inmediato, con una histéresis de 2 lpm.
- Con FC = 0 (sin ECG) no hay alarma.
- El botón de silencio (`bt1`) solo apaga el buzzer; el margen y los LEDs siguen parpadeando.
- Los límites se ajustan desde la página de configuración (`page2`, sliders `h0`–`h3`). El firmware revisa el rango y el orden de los valores. El botón *Restaurar* vuelve a los valores predeterminados.

---

## Interfaz Nextion

| Componente | Contenido |
|---|---|
| Página `logo` | Logo de inicio, se muestra 2,5 s al encender |
| Página `page0` | Pantalla principal |
| Waveforms id 8 / id 7 | Ondas de ECG / PPG (`ID_WAVE_ECG`, `ID_WAVE_PPG`) |
| `n0`, `n1`, `n2` | FC, SpO2, FP |
| `n3`, `n4` | Presión sistólica y diastólica |
| `n5` | Temperatura (°C, entero) |
| `n36` | PTT (ms) |
| `t1` | Estado de la calibración (`txt_maxl` ≥ 80) |
| `j0` | Progreso de la calibración |
| `m0`, `t20`–`t23` | Margen de alarma |
| `page2`: `h0`–`h3`, `n20`–`n23` | Límites de las alarmas |

Mensajes de la Nextion al ESP32:

- `BP:<sys>/<dia>\n`: inicia la calibración.
- Un byte `0x00` / `0x01`: estado del silencio (`bt1`).
- `0x55` seguido de 4 bytes: nuevos límites de alarma (aviso bajo, aviso alto, peligro bajo, peligro alto).

Los nombres de los componentes están en `#define` al inicio de [`src/main.cpp`](src/main.cpp) (`OBJ_ESTADO`, `OBJ_PROGRESO`, `OBJ_TEMP`, `PAGINA_PRINCIPAL`, ...).

---

## Parámetros principales

Todos están en [`src/main.cpp`](src/main.cpp):

| Parámetro | Para qué sirve |
|---|---|
| `FS_SISTEMA` | Frecuencia de muestreo efectiva (250 Hz) |
| `ECG_NOTCH_Q`, `ECG_NOTCH_ETAPAS` | Ancho y profundidad del notch de 60 Hz |
| `PPG_SIGN` | Polaridad de la PPG |
| `SPO2_A`, `SPO2_B` | Curva de calibración de la SpO2 |
| `PPG_MIN_AMP` | Amplitud mínima para aceptar pulsos |
| `FC_AVISO_*`, `FC_PELIGRO_*` | Límites predeterminados de las alarmas |
| `T_*` | Constantes del circuito de temperatura |
| `DIAG_CAL`, `DIAG_FC`, `DEBUG_TEMP` | Mensajes de diagnóstico |

---

## Autores

- Matías Berrío Toro
- Simón Andrés Duarte Martinez
- Maria Isabel Rios Jaramillo
- Santiago Soto Betancur
