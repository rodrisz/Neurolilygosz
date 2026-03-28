/*
 * ============================================================================
 *  TWatch_NeuroSky_MindWave.ino
 * ============================================================================
 *  Programa para LilyGO T-Watch 2020 V2
 *  Conexión Bluetooth Clásico (SPP) con NeuroSky MindWave Mobile
 *  Parseo completo del protocolo ThinkGear Serial Stream
 *  
 *  Autor: Rodrigo - Proyecto Neuro-Crawler
 *  Plataforma: ESP32 (LilyGO T-Watch 2020 V2)
 *  Librería: TTGO_TWatch_Library (esp core 2.0.14 recomendado)
 *  
 *  PROTOCOLO THINKGEAR - Resumen:
 *  ─────────────────────────────────────────────────────────
 *  Paquete: [SYNC][SYNC][PLENGTH][PAYLOAD...][CHKSUM]
 *    SYNC     = 0xAA (dos bytes consecutivos)
 *    PLENGTH  = longitud del payload (0-169)
 *    CHKSUM   = complemento a 1 de la suma de bytes del payload
 *  
 *  Códigos Single-Byte (CODE < 0x80, VALUE = 1 byte):
 *    0x02 = POOR_SIGNAL Quality (0-255, 0=buena, 200=sin contacto)
 *    0x04 = ATTENTION eSense  (0-100)
 *    0x05 = MEDITATION eSense (0-100)
 *    0x16 = Blink Strength    (0-255)
 *  
 *  Códigos Multi-Byte (CODE >= 0x80, VLENGTH + VALUE):
 *    0x80 = RAW Wave Value    (2 bytes big-endian signed)
 *    0x83 = ASIC_EEG_POWER   (24 bytes: 8 bandas × 3 bytes)
 *           delta, theta, lowAlpha, highAlpha,
 *           lowBeta, highBeta, lowGamma, midGamma
 *  
 *  NOTAS BLUETOOTH:
 *  ─────────────────────────────────────────────────────────
 *  - MindWave Mobile usa Bluetooth Clásico SPP (Serial Port Profile)
 *  - Baud rate interno del MindWave: 57600 (modo normal+raw)
 *  - En SPP/RFCOMM sobre ESP32, el baud rate es virtual/transparente:
 *    los bytes llegan por el canal RFCOMM sin configurar baud rate
 *  - PIN por defecto: "0000"
 *  - MAC del MindWave: A4:DA:32:6F:EE:BA
 *  - Para poner en discoverable: mantener switch ON arriba 4-5s
 *    hasta que LED parpadee doble
 * ============================================================================
 */

// ─── MODELO DE T-WATCH ─────────────────────────────────────────────────────
// Descomentar SOLO la línea de tu modelo
// #define LILYGO_WATCH_2020_V1
#define LILYGO_WATCH_2020_V2
// #define LILYGO_WATCH_2020_V3

#include <Arduino.h>
#include <math.h>
//hola prueba git
#include <LilyGoWatch.h>
#include "BluetoothSerial.h"
#include <SD.h>
#include <FS.h>

// ─── VERIFICACIÓN BT CLÁSICO ───────────────────────────────────────────────
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error "Bluetooth no habilitado. Ejecuta 'make menuconfig' y habilitalo."
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
  #error "SPP no habilitado. Requerido para MindWave."
#endif

// ─── OBJETOS T-WATCH ───────────────────────────────────────────────────────
TTGOClass    *watch = nullptr;
TFT_eSPI     *tft   = nullptr;
AXP20X_Class *power = nullptr;

// ─── BLUETOOTH ─────────────────────────────────────────────────────────────
BluetoothSerial SerialBT;

// Dirección MAC del MindWave Mobile
uint8_t neuroskyMAC[6] = { 0xA4, 0xDA, 0x32, 0x6F, 0xEE, 0xBA };

// ─── BAUDRATE REFERENCIA ───────────────────────────────────────────────────
// El MindWave transmite a 57600 baud internamente.
// En SPP/RFCOMM del ESP32 esto es transparente (no se configura),
// pero lo definimos como referencia para el monitor serial de debug.
#define MINDWAVE_BAUDRATE 57600

// ─── CÓDIGOS THINKGEAR ─────────────────────────────────────────────────────
#define TG_SYNC           0xAA
#define TG_CODE_POOR_SIG  0x02
#define TG_CODE_ATTENTION 0x04
#define TG_CODE_MEDITATE  0x05
#define TG_CODE_BLINK     0x16
#define TG_CODE_RAW       0x80
#define TG_CODE_EEG_POWER 0x83

// ─── DATOS PARSEADOS ───────────────────────────────────────────────────────
volatile uint8_t  poorSignal    = 200;
volatile uint8_t  attention     = 0;
volatile uint8_t  meditation    = 0;
volatile uint8_t  blinkStrength = 0;
volatile int16_t  rawWave       = 0;

// Bandas EEG (ASIC_EEG_POWER)
volatile uint32_t eegDelta     = 0;
volatile uint32_t eegTheta     = 0;
volatile uint32_t eegLowAlpha  = 0;
volatile uint32_t eegHighAlpha = 0;
volatile uint32_t eegLowBeta   = 0;
volatile uint32_t eegHighBeta  = 0;
volatile uint32_t eegLowGamma  = 0;
volatile uint32_t eegMidGamma  = 0;

// ─── FÓRMULAS NEURO-CRAWLER ────────────────────────────────────────────────
float indiceFatiga = 0.0;   // IFN = (Theta + Alpha) / Beta
float ratioCarga   = 0.0;   // RC  = Beta / Alpha

// ─── FREERTOS DUAL-CORE (Core 0: BT+SD) ──────────────────────────────────
#include "freertos/semphr.h"

struct NeuroData {
  uint8_t  poorSignal;
  uint8_t  attention;
  uint8_t  meditation;
  uint8_t  blinkStrength;
  int16_t  rawWave;
  uint32_t eegDelta, eegTheta, eegLowAlpha, eegHighAlpha;
  uint32_t eegLowBeta, eegHighBeta, eegLowGamma, eegMidGamma;
  float    indiceFatiga;
  float    ratioCarga;
  bool     dataReceived;
  uint32_t totalPackets;
  uint32_t badChecksums;
  unsigned long lastPacketTime;
};

NeuroData          neuroShared;              // datos compartidos protegidos por mutex
SemaphoreHandle_t  neuroMutex   = NULL;      // mutex para neuroShared
SemaphoreHandle_t  sdMutex      = NULL;      // mutex para archivoSesion/sdLogging
TaskHandle_t       btTaskHandle = NULL;      // handle de la tarea Core 0
volatile bool      btTaskRunning = false;    // flag para detener la tarea

// Timestamp cacheado del RTC (escrito por Core 1, leido por Core 0)
char cachedFecha[12] = "0000-00-00";         // "YYYY-MM-DD"
char cachedHora[10]  = "00:00:00";           // "HH:MM:SS"
SemaphoreHandle_t  rtcCacheMutex = NULL;     // mutex para timestamp cacheado
unsigned long lastRtcCacheUpdate = 0;

// Forward declarations Core 0
void btSdTask(void *param);
void readNeuroData();
void escribirLineaSD_Core0();
void actualizarCacheRTC();

// ─── PARSER STATE MACHINE ──────────────────────────────────────────────────
enum ParserState {
  WAIT_SYNC1,
  WAIT_SYNC2,
  WAIT_PLENGTH,
  READ_PAYLOAD,
  WAIT_CHECKSUM
};

ParserState  parserState    = WAIT_SYNC1;
uint8_t      payloadBuf[256];
uint8_t      pLength        = 0;
uint8_t      payloadIdx     = 0;
uint8_t      checksumAccum  = 0;

// ─── MOTOR VIBRACIÓN (DRV2605L via I2C) ──────────────────────────────────
Adafruit_DRV2605 *drv = nullptr;
bool motorOn = false;
unsigned long vibStartTime = 0;
unsigned long vibLastToggle = 0;
bool vibActiva = false;          // vibración intermitente med=100 activa
bool vibEstado = false;          // estado actual del motor en intermitente

// Forward declarations
void drawInicioScreen();
void drawMenuScreen();

// Función helper para disparar un efecto del DRV2605
void drvPlayEffect(uint8_t effect) {
  if (!drv) return;
  drv->setWaveform(0, effect);
  drv->setWaveform(1, 0);
  drv->go();
}

void drvStop() {
  if (!drv) return;
  drv->stop();
}

// ─── MENÚ / UMBRAL ──────────────────────────────────────────────────────
bool menuActivo = false;
int umbralVibracion = 0;         // 0=desactivado, 70, 80 o 90
unsigned long lastTouchTime = 0; // debounce touch

// ─── PANTALLA INICIO ────────────────────────────────────────────────────
bool pantallaInicio = false;     // true = mostrar pantalla inicio, false = ya se pulsó Conectar
bool btIniciado = false;         // true = Bluetooth ya fue inicializado

// ─── ESTADO CONEXIÓN ───────────────────────────────────────────────────────
bool          btConnected       = false;
bool          dataReceived      = false;
unsigned long lastPacketTime    = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long connectAttemptTime= 0;
int           connectAttempts   = 0;
uint32_t      totalPackets      = 0;
uint32_t      badChecksums      = 0;

// ─── SD CARD & LOGGING ───────────────────────────────────────────────────
bool sdReady          = false;     // SD card inicializada correctamente
bool sdLogging        = false;     // sesion activa grabando datos
uint16_t sesionNumero = 0;         // numero de sesion actual
File archivoSesion;                // archivo abierto de la sesion actual
unsigned long lastSDWrite = 0;     // debounce escritura SD
#define SD_WRITE_INTERVAL 1000     // escribir cada 1 segundo

// ─── SELECCION DE USUARIO ────────────────────────────────────────────────
int  usuarioActual     = -1;       // -1=no seleccionado, 0=Rodrigo, 1=Julieth, 2=Ambos
bool pantallaUsuario   = true;     // true = mostrar pantalla seleccion usuario
const char *usuarioNombres[3]  = { "Rodrigo", "Julieth", "Ambos" };
const char *usuarioCarpetas[3] = { "/rodrigo", "/julieth", "/ambos" };

// ─── SELECCION DE EJERCICIO GAPAXION ─────────────────────────────────────
int  gapaxionActual      = -1;     // -1=no seleccionado, 0..9 = G1..G10
bool pantallaGapaxion    = false;  // true = mostrar pantalla seleccion ejercicio
const char *gapaxionNombres[10] = { "G1","G2","G3","G4","G5","G6","G7","G8","G9","G10" };

// ─── CONFIGURACION RELOJ (FECHA/HORA) ───────────────────────────────────
bool pantallaReloj       = false;  // true = mostrar pantalla config reloj
int  relojCampos[6]      = {0,0,0,0,0,0};  // año,mes,dia,hora,min,seg
int  relojCampoSel       = 0;     // campo seleccionado (0-5)
const char *relojLabels[6] = { "Ano", "Mes", "Dia", "Hora", "Min", "Seg" };

// Forward declarations
void drawUserSelectScreen();
void drawGapaxionSelectScreen();
void drawRelojScreen();

// ─── WALKING MEDITATION (Mindful Steps) ──────────────────────────────────
bool walkingMedActivo    = false;   // modo walking meditation activo
bool walkingMedSeleccion = false;   // pantalla seleccion nivel
bool walkingMedEjercicio = false;   // ejercicio en curso
bool walkingMedResultado = false;   // pantalla resultado
int  walkNivel           = 1;       // 1, 2, 3
int  walkPasosObjetivo   = 20;
int  walkPasosContados   = 0;
uint32_t walkStepCountPrev = 0;     // step count anterior del BMA423
unsigned long walkUltimoPaso   = 0; // millis del ultimo paso detectado
unsigned long walkInicioTime   = 0; // millis inicio del ejercicio
float walkTiempoObjetivo = 2500.0;  // ms entre pasos segun nivel
float walkTolerancia     = 1000.0;  // ms tolerancia segun nivel
int   walkUmbralSuavidad = 800;     // mg umbral suavidad segun nivel
float walkSumaRitmo      = 0;      // suma de puntajes de ritmo
float walkSumaSuavidad   = 0;      // suma de puntajes de suavidad
int   walkPasosEvaluados = 0;      // pasos con puntaje calculado
float walkPuntajeFinal   = 0;      // puntaje total al terminar
float walkRitmoFinal     = 0;      // puntaje ritmo final
float walkSuavidadFinal  = 0;      // puntaje suavidad final
float walkMejorPaso      = 0;      // mejor puntaje individual
float walkPeorPaso       = 100;    // peor puntaje individual
float walkUltimoTiming   = 0;      // ultimo tiempo entre pasos (ms)
float walkUltimoPuntaje  = 0;      // puntaje del ultimo paso
bool  walkGuiaActiva     = false;  // guia haptica activa (nivel 1)

// Deteccion de pasos: 0=AUTO (pico acelerometro), 1=TAP (toque pantalla)
int   walkDeteccionMode  = 0;

// Peak detection para modo AUTO
float walkPrevMag        = 0;
bool  walkPeakAbove      = false;
unsigned long walkLastPeakTime = 0;
#define WALK_PEAK_THRESHOLD 1150.0f  // mg — umbral de impacto de paso
#define WALK_PEAK_DEBOUNCE  1000     // ms — minimo entre pasos detectados

// Buffer acelerometro para suavidad
#define WALK_ACCEL_BUF_SIZE 50
float walkAccelSamples[WALK_ACCEL_BUF_SIZE];
int   walkAccelIdx   = 0;
int   walkAccelCount = 0;
unsigned long walkLastAccelRead  = 0;
unsigned long walkLastScreenUpdate = 0;
unsigned long walkNextGuiaTime   = 0;

// Configuracion por nivel
struct WalkLevel {
  const char *nombre;
  float tiempoObj;    // ms entre pasos
  float tolerancia;   // ms
  int   umbralSuav;   // mg
  int   pasosObj;
  bool  guiaHaptica;
};

const WalkLevel walkLevels[3] = {
  { "Paseo Consciente", 2500.0, 1000.0, 800, 20, true  },  // Nivel 1
  { "Camino Interior",  4500.0,  800.0, 500, 30, true  },  // Nivel 2
  { "Maestro Zen",      8000.0, 1500.0, 300, 40, true  },  // Nivel 3
};

// Forward declarations Walking Med
void drawWalkingSelScreen();
void drawWalkingScreen();
void drawWalkingResultScreen();
void iniciarWalkingExercise();
void finalizarWalkingMed();
void registrarPaso(unsigned long now);

// Estado del callback BT para mostrar en pantalla
String btStatus = "Inicializando...";

// ─── PANTALLA 240×240 ──────────────────────────────────────────────────────
#define SCREEN_W  240
#define SCREEN_H  240

#define COLOR_BG         TFT_BLACK
#define COLOR_HEADER     0x1A3A
#define COLOR_ATTENTION  0xFD20
#define COLOR_MEDITATE   0x07E0
#define COLOR_SIGNAL_OK  0x07E0
#define COLOR_SIGNAL_BAD 0xF800
#define COLOR_BAR_BG     0x2104
#define COLOR_TEXT        TFT_WHITE
#define COLOR_TEXT_DIM    0x7BEF
#define COLOR_FATIGA      0xFFE0
#define COLOR_CARGA       0x07FF

#define DISPLAY_INTERVAL_MS 500

// ════════════════════════════════════════════════════════════════════════════
//  CALLBACK BLUETOOTH - Debug de eventos SPP
// ════════════════════════════════════════════════════════════════════════════

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  switch (event) {
    case ESP_SPP_INIT_EVT:
      Serial.println("[BT-CB] SPP Inicializado");
      btStatus = "SPP Init OK";
      break;

    case ESP_SPP_DISCOVERY_COMP_EVT:
      Serial.printf("[BT-CB] Discovery completo. Status:%d Canales:%d\n",
                    param->disc_comp.status, param->disc_comp.scn_num);
      if (param->disc_comp.status == ESP_SPP_SUCCESS) {
        btStatus = "Dispositivo encontrado";
        for (int i = 0; i < param->disc_comp.scn_num; i++) {
          Serial.printf("[BT-CB]   Canal[%d]: %d\n", i, param->disc_comp.scn[i]);
        }
      } else {
        btStatus = "Discovery fallido";
      }
      break;

    case ESP_SPP_OPEN_EVT:
      Serial.println("[BT-CB] Conexion SPP abierta!");
      btStatus = "SPP Conectado!";
      btConnected = true;
      break;

    case ESP_SPP_CLOSE_EVT:
      Serial.println("[BT-CB] Conexion SPP cerrada");
      btStatus = "Desconectado";
      btConnected = false;
      break;

    case ESP_SPP_DATA_IND_EVT:
      // Datos recibidos - se procesan en el loop vía SerialBT.read()
      break;

    case ESP_SPP_CL_INIT_EVT:
      Serial.printf("[BT-CB] Cliente SPP iniciando. Status:%d\n",
                    param->cl_init.status);
      btStatus = "Conectando SPP...";
      break;

    case ESP_SPP_SRV_OPEN_EVT:
      Serial.println("[BT-CB] Servidor SPP: cliente conectado");
      break;

    default:
      Serial.printf("[BT-CB] Evento: %d\n", event);
      break;
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  PARSEO THINKGEAR
// ════════════════════════════════════════════════════════════════════════════

void parsePayload(uint8_t *payload, uint8_t length) {
  uint8_t i = 0;

  while (i < length) {
    // Saltar EXCODE (0x55)
    while (i < length && payload[i] == 0x55) { i++; }
    if (i >= length) break;

    uint8_t code = payload[i++];

    if (code < 0x80) {
      // ── Single-Byte Value ──
      if (i >= length) break;
      uint8_t value = payload[i++];

      switch (code) {
        case TG_CODE_POOR_SIG:
          poorSignal = value;
          break;
        case TG_CODE_ATTENTION:
          attention = value;
          dataReceived = true;
          break;
        case TG_CODE_MEDITATE:
          meditation = value;
          dataReceived = true;
          break;
        case TG_CODE_BLINK:
          blinkStrength = value;
          break;
      }
    } else {
      // ── Multi-Byte Value ──
      if (i >= length) break;
      uint8_t vLength = payload[i++];
      if (i + vLength > length) break;

      switch (code) {
        case TG_CODE_RAW:
          if (vLength == 2) {
            rawWave = (int16_t)((payload[i] << 8) | payload[i + 1]);
          }
          break;

        case TG_CODE_EEG_POWER:
          if (vLength == 24) {
            uint8_t j = i;
            eegDelta     = ((uint32_t)payload[j]<<16) | ((uint32_t)payload[j+1]<<8) | payload[j+2]; j+=3;
            eegTheta     = ((uint32_t)payload[j]<<16) | ((uint32_t)payload[j+1]<<8) | payload[j+2]; j+=3;
            eegLowAlpha  = ((uint32_t)payload[j]<<16) | ((uint32_t)payload[j+1]<<8) | payload[j+2]; j+=3;
            eegHighAlpha = ((uint32_t)payload[j]<<16) | ((uint32_t)payload[j+1]<<8) | payload[j+2]; j+=3;
            eegLowBeta   = ((uint32_t)payload[j]<<16) | ((uint32_t)payload[j+1]<<8) | payload[j+2]; j+=3;
            eegHighBeta  = ((uint32_t)payload[j]<<16) | ((uint32_t)payload[j+1]<<8) | payload[j+2]; j+=3;
            eegLowGamma  = ((uint32_t)payload[j]<<16) | ((uint32_t)payload[j+1]<<8) | payload[j+2]; j+=3;
            eegMidGamma  = ((uint32_t)payload[j]<<16) | ((uint32_t)payload[j+1]<<8) | payload[j+2];

            // Fórmulas Neuro-Crawler
            float totalAlpha = (float)(eegLowAlpha + eegHighAlpha);
            float totalBeta  = (float)(eegLowBeta  + eegHighBeta);

            if (totalBeta > 0)  indiceFatiga = ((float)eegTheta + totalAlpha) / totalBeta;
            else                indiceFatiga = 0.0;

            if (totalAlpha > 0) ratioCarga = totalBeta / totalAlpha;
            else                ratioCarga = 0.0;
          }
          break;
      }
      i += vLength;
    }
  }
}

/**
 * Máquina de estados del parser ThinkGear.
 * Alimentar byte por byte desde SerialBT.read().
 */
void parseByte(uint8_t b) {
  switch (parserState) {

    case WAIT_SYNC1:
      if (b == TG_SYNC) parserState = WAIT_SYNC2;
      break;

    case WAIT_SYNC2:
      if (b == TG_SYNC)  parserState = WAIT_PLENGTH;
      else               parserState = WAIT_SYNC1;
      break;

    case WAIT_PLENGTH:
      if (b == TG_SYNC) break;   // SYNCs extra, seguir esperando
      if (b > 169) { parserState = WAIT_SYNC1; break; }
      pLength      = b;
      payloadIdx   = 0;
      checksumAccum= 0;
      parserState  = (pLength == 0) ? WAIT_CHECKSUM : READ_PAYLOAD;
      break;

    case READ_PAYLOAD:
      payloadBuf[payloadIdx++] = b;
      checksumAccum += b;
      if (payloadIdx >= pLength) parserState = WAIT_CHECKSUM;
      break;

    case WAIT_CHECKSUM:
      {
        uint8_t expected = ~checksumAccum & 0xFF;
        if (b == expected) {
          lastPacketTime = millis();
          totalPackets++;
          parsePayload(payloadBuf, pLength);

          // Copiar datos parseados al struct compartido (Core 0 → Core 1)
          if (neuroMutex && xSemaphoreTake(neuroMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            neuroShared.poorSignal     = poorSignal;
            neuroShared.attention      = attention;
            neuroShared.meditation     = meditation;
            neuroShared.blinkStrength  = blinkStrength;
            neuroShared.rawWave        = rawWave;
            neuroShared.eegDelta       = eegDelta;
            neuroShared.eegTheta       = eegTheta;
            neuroShared.eegLowAlpha    = eegLowAlpha;
            neuroShared.eegHighAlpha   = eegHighAlpha;
            neuroShared.eegLowBeta     = eegLowBeta;
            neuroShared.eegHighBeta    = eegHighBeta;
            neuroShared.eegLowGamma    = eegLowGamma;
            neuroShared.eegMidGamma    = eegMidGamma;
            neuroShared.indiceFatiga   = indiceFatiga;
            neuroShared.ratioCarga     = ratioCarga;
            neuroShared.dataReceived   = dataReceived;
            neuroShared.totalPackets   = totalPackets;
            neuroShared.badChecksums   = badChecksums;
            neuroShared.lastPacketTime = lastPacketTime;
            xSemaphoreGive(neuroMutex);
          }
        } else {
          badChecksums++;
        }
        parserState = WAIT_SYNC1;
      }
      break;
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  CORE 0: TAREA BT + SD  (corre independiente del loop principal)
// ════════════════════════════════════════════════════════════════════════════

/**
 * readNeuroData() — Core 1 llama esto para copiar datos frescos desde
 * neuroShared (escrito por Core 0) a las variables globales que usa la UI.
 */
void readNeuroData() {
  if (neuroMutex && xSemaphoreTake(neuroMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    poorSignal     = neuroShared.poorSignal;
    attention      = neuroShared.attention;
    meditation     = neuroShared.meditation;
    blinkStrength  = neuroShared.blinkStrength;
    rawWave        = neuroShared.rawWave;
    eegDelta       = neuroShared.eegDelta;
    eegTheta       = neuroShared.eegTheta;
    eegLowAlpha    = neuroShared.eegLowAlpha;
    eegHighAlpha   = neuroShared.eegHighAlpha;
    eegLowBeta     = neuroShared.eegLowBeta;
    eegHighBeta    = neuroShared.eegHighBeta;
    eegLowGamma    = neuroShared.eegLowGamma;
    eegMidGamma    = neuroShared.eegMidGamma;
    indiceFatiga   = neuroShared.indiceFatiga;
    ratioCarga     = neuroShared.ratioCarga;
    dataReceived   = neuroShared.dataReceived;
    totalPackets   = neuroShared.totalPackets;
    badChecksums   = neuroShared.badChecksums;
    lastPacketTime = neuroShared.lastPacketTime;
    xSemaphoreGive(neuroMutex);
  }
}

/**
 * actualizarCacheRTC() — Core 1 llama cada ~1s para cachear el timestamp
 * del RTC (I2C). Core 0 lee este cache para escribir en SD sin tocar I2C.
 */
void actualizarCacheRTC() {
  RTC_Date dt = watch->rtc->getDateTime();
  if (rtcCacheMutex && xSemaphoreTake(rtcCacheMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    snprintf(cachedFecha, sizeof(cachedFecha), "%04d-%02d-%02d", dt.year, dt.month, dt.day);
    snprintf(cachedHora,  sizeof(cachedHora),  "%02d:%02d:%02d", dt.hour, dt.minute, dt.second);
    xSemaphoreGive(rtcCacheMutex);
  }
}

/**
 * escribirLineaSD_Core0() — Escribe una linea CSV desde Core 0.
 * Usa timestamp cacheado (no accede I2C) y datos del parser (ya en Core 0).
 */
void escribirLineaSD_Core0() {
  if (!sdLogging || !archivoSesion) return;

  // Leer timestamp cacheado
  char fecha[12], hora[10];
  if (rtcCacheMutex && xSemaphoreTake(rtcCacheMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    memcpy(fecha, cachedFecha, sizeof(fecha));
    memcpy(hora,  cachedHora,  sizeof(hora));
    xSemaphoreGive(rtcCacheMutex);
  } else {
    strncpy(fecha, "----.--.--", sizeof(fecha));
    strncpy(hora,  "--:--:--",   sizeof(hora));
  }

  char linea[200];
  snprintf(linea, sizeof(linea),
    "%d,%s,%s,%d,%d,%d,%.2f,%.2f,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
    sesionNumero,
    fecha, hora,
    attention, meditation, poorSignal,
    indiceFatiga, ratioCarga,
    eegDelta, eegTheta, eegLowAlpha, eegHighAlpha,
    eegLowBeta, eegHighBeta, eegLowGamma, eegMidGamma
  );

  if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    archivoSesion.println(linea);
    archivoSesion.flush();
    xSemaphoreGive(sdMutex);
  }
}

/**
 * btSdTask() — Tarea FreeRTOS en Core 0.
 * Lee bytes BT, parsea ThinkGear, escribe SD cada 1s.
 * Corre mientras btTaskRunning == true.
 */
void btSdTask(void *param) {
  Serial.println("[CORE0] Tarea BT+SD iniciada");
  unsigned long taskLastSDWrite = millis();

  while (btTaskRunning) {
    // ── Leer y parsear bytes BT ──
    if (btConnected && SerialBT.connected()) {
      int avail = SerialBT.available();
      while (avail > 0) {
        uint8_t b = SerialBT.read();
        parseByte(b);
        avail--;
      }
    } else if (btConnected && !SerialBT.connected()) {
      // Conexion perdida — señalar a Core 1
      btConnected = false;
      Serial.println("[CORE0] Conexion BT perdida");
    }

    // ── Escribir SD cada SD_WRITE_INTERVAL ──
    unsigned long now = millis();
    if (sdLogging && (now - taskLastSDWrite >= SD_WRITE_INTERVAL)) {
      taskLastSDWrite = now;
      escribirLineaSD_Core0();
    }

    vTaskDelay(pdMS_TO_TICKS(1));  // ceder a WiFi/BT stack en Core 0
  }

  Serial.println("[CORE0] Tarea BT+SD finalizada");
  btTaskHandle = NULL;
  vTaskDelete(NULL);
}

// ════════════════════════════════════════════════════════════════════════════
//  INTERFAZ GRÁFICA
// ════════════════════════════════════════════════════════════════════════════

void drawBar(int x, int y, int w, int h, int value, uint16_t color) {
  tft->fillRoundRect(x, y, w, h, 3, COLOR_BAR_BG);
  int fillW = map(constrain(value, 0, 100), 0, 100, 0, w);
  if (fillW > 0) tft->fillRoundRect(x, y, fillW, h, 3, color);
  tft->drawRoundRect(x, y, w, h, 3, COLOR_TEXT_DIM);
}

const char* eSenseLabel(uint8_t v) {
  if (v == 0)  return "---";
  if (v <= 20) return "Muy Bajo";
  if (v <= 40) return "Bajo";
  if (v <= 60) return "Neutro";
  if (v <= 80) return "Alto";
  return "Muy Alto";
}

uint16_t eSenseColor(uint8_t v) {
  if (v == 0)  return COLOR_TEXT_DIM;
  if (v <= 25) return TFT_RED;
  if (v <= 50) return TFT_YELLOW;
  if (v <= 75) return TFT_GREEN;
  return TFT_CYAN;
}

// ════════════════════════════════════════════════════════════════════════════
//  PANTALLA SELECCION DE USUARIO
// ════════════════════════════════════════════════════════════════════════════

void drawUserSelectScreen() {
  tft->fillScreen(COLOR_BG);

  // Titulo
  tft->setTextDatum(MC_DATUM);
  tft->setTextColor(COLOR_ATTENTION, COLOR_BG);
  tft->drawString("GAPAXIONSZ", SCREEN_W / 2, 30, 4);

  // Subtitulo
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString("Selecciona Usuario", SCREEN_W / 2, 65, 2);

  // Boton Rodrigo (y:90, h:40)
  tft->fillRoundRect(30, 90, 180, 40, 8, TFT_BLUE);
  tft->drawRoundRect(30, 90, 180, 40, 8, TFT_CYAN);
  tft->setTextColor(COLOR_TEXT, TFT_BLUE);
  tft->drawString("Rodrigo", SCREEN_W / 2, 110, 4);

  // Boton Julieth (y:140, h:40)
  tft->fillRoundRect(30, 140, 180, 40, 8, 0x600C);
  tft->drawRoundRect(30, 140, 180, 40, 8, 0xF81F);
  tft->setTextColor(COLOR_TEXT, 0x600C);
  tft->drawString("Julieth", SCREEN_W / 2, 160, 4);

  // Boton Ambos (y:190, h:40)
  tft->fillRoundRect(30, 190, 180, 40, 8, 0x0320);
  tft->drawRoundRect(30, 190, 180, 40, 8, TFT_GREEN);
  tft->setTextColor(COLOR_TEXT, 0x0320);
  tft->drawString("Ambos", SCREEN_W / 2, 210, 4);
}

// ════════════════════════════════════════════════════════════════════════════
//  PANTALLA SELECCION DE EJERCICIO GAPAXION
// ════════════════════════════════════════════════════════════════════════════

void drawGapaxionSelectScreen() {
  tft->fillScreen(COLOR_BG);

  // Titulo
  tft->setTextDatum(MC_DATUM);
  tft->setTextColor(COLOR_ATTENTION, COLOR_BG);
  tft->drawString("GAPAXIONSZ", SCREEN_W / 2, 14, 2);

  // Subtitulo con nombre de usuario
  tft->setTextColor(TFT_CYAN, COLOR_BG);
  if (usuarioActual >= 0) {
    tft->drawString(usuarioNombres[usuarioActual], SCREEN_W / 2, 32, 2);
  }
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString("Selecciona Ejercicio", SCREEN_W / 2, 50, 2);

  // 10 botones en grid 5x2: G1-G5 (fila 1), G6-G10 (fila 2)
  // Cada boton: 42x32, gap 4px
  int btnW = 42, btnH = 32, gap = 4;
  int startX = 7;  // (240 - 5*42 - 4*4) / 2 = 7
  int y1 = 68, y2 = 108;

  for (int i = 0; i < 10; i++) {
    int col = i % 5;
    int row = i / 5;
    int bx = startX + col * (btnW + gap);
    int by = (row == 0) ? y1 : y2;

    uint16_t bgColor = (i == gapaxionActual) ? TFT_GREEN : TFT_BLUE;
    uint16_t borderColor = (i == gapaxionActual) ? TFT_WHITE : TFT_CYAN;

    tft->fillRoundRect(bx, by, btnW, btnH, 6, bgColor);
    tft->drawRoundRect(bx, by, btnW, btnH, 6, borderColor);
    tft->setTextColor(COLOR_TEXT, bgColor);
    tft->drawString(gapaxionNombres[i], bx + btnW / 2, by + btnH / 2, 2);
  }

  // Info del gapaxion seleccionado
  if (gapaxionActual >= 0) {
    tft->setTextColor(TFT_GREEN, COLOR_BG);
    char sel[20];
    snprintf(sel, sizeof(sel), "Ejercicio: %s", gapaxionNombres[gapaxionActual]);
    tft->drawString(sel, SCREEN_W / 2, 152, 2);
  }

  // Boton CONTINUAR (solo visible si hay gapaxion seleccionado)
  if (gapaxionActual >= 0) {
    tft->fillRoundRect(40, 170, 160, 36, 8, TFT_GREEN);
    tft->drawRoundRect(40, 170, 160, 36, 8, TFT_WHITE);
    tft->setTextColor(COLOR_BG, TFT_GREEN);
    tft->drawString("CONTINUAR", SCREEN_W / 2, 188, 4);
  }

  // Boton VOLVER (esquina inferior)
  tft->fillRoundRect(70, 216, 100, 22, 4, TFT_RED);
  tft->setTextColor(COLOR_TEXT, TFT_RED);
  tft->drawString("VOLVER", SCREEN_W / 2, 227, 2);
}

// ════════════════════════════════════════════════════════════════════════════
//  PANTALLA CONFIGURACION RELOJ (FECHA/HORA)
// ════════════════════════════════════════════════════════════════════════════

void drawRelojScreen() {
  tft->fillScreen(COLOR_BG);

  // Header
  tft->fillRoundRect(0, 0, SCREEN_W, 28, 0, COLOR_HEADER);
  tft->setTextColor(TFT_YELLOW, COLOR_HEADER);
  tft->setTextDatum(MC_DATUM);
  tft->drawString("AJUSTAR RELOJ", SCREEN_W / 2, 14, 2);

  // Fecha/hora actual del RTC
  RTC_Date dt = watch->rtc->getDateTime();
  char actualBuf[24];
  snprintf(actualBuf, sizeof(actualBuf), "%04d-%02d-%02d %02d:%02d:%02d",
           dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString(actualBuf, SCREEN_W / 2, 40, 1);

  // 6 columnas: Ano Mes Dia Hora Min Seg
  // Cada columna: label + valor + boton[+] + boton[-]
  int colW = 38, gap = 2;
  int startX = (SCREEN_W - 6 * colW - 5 * gap) / 2;  // centrado

  for (int i = 0; i < 6; i++) {
    int cx = startX + i * (colW + gap) + colW / 2;
    int bx = startX + i * (colW + gap);

    // Label
    uint16_t labelColor = (i == relojCampoSel) ? TFT_YELLOW : COLOR_TEXT_DIM;
    tft->setTextColor(labelColor, COLOR_BG);
    tft->drawString(relojLabels[i], cx, 58, 1);

    // Boton [+] (y=68, h=28)
    uint16_t btnColor = (i == relojCampoSel) ? TFT_CYAN : COLOR_BAR_BG;
    tft->fillRoundRect(bx, 68, colW, 28, 4, btnColor);
    tft->setTextColor(COLOR_TEXT, btnColor);
    tft->drawString("+", cx, 82, 2);

    // Valor (y=102)
    char valBuf[6];
    if (i == 0) snprintf(valBuf, sizeof(valBuf), "%04d", relojCampos[i]);
    else        snprintf(valBuf, sizeof(valBuf), "%02d", relojCampos[i]);

    uint16_t valColor = (i == relojCampoSel) ? TFT_GREEN : COLOR_TEXT;
    tft->setTextColor(valColor, COLOR_BG);
    tft->drawString(valBuf, cx, 112, 2);

    // Boton [-] (y=128, h=28)
    tft->fillRoundRect(bx, 128, colW, 28, 4, btnColor);
    tft->setTextColor(COLOR_TEXT, btnColor);
    tft->drawString("-", cx, 142, 2);
  }

  // Separador
  tft->drawFastHLine(10, 164, SCREEN_W - 20, COLOR_TEXT_DIM);

  // Preview de la fecha/hora configurada
  char prevBuf[24];
  snprintf(prevBuf, sizeof(prevBuf), "%04d-%02d-%02d %02d:%02d:%02d",
           relojCampos[0], relojCampos[1], relojCampos[2],
           relojCampos[3], relojCampos[4], relojCampos[5]);
  tft->setTextColor(TFT_GREEN, COLOR_BG);
  tft->drawString(prevBuf, SCREEN_W / 2, 176, 2);

  // Boton GUARDAR (y=192, h=28)
  tft->fillRoundRect(20, 192, 200, 28, 6, TFT_GREEN);
  tft->drawRoundRect(20, 192, 200, 28, 6, TFT_WHITE);
  tft->setTextColor(COLOR_BG, TFT_GREEN);
  tft->drawString("GUARDAR", SCREEN_W / 2, 206, 2);

  // Boton VOLVER (y=226, h=12)
  tft->fillRoundRect(70, 224, 100, 14, 4, TFT_RED);
  tft->setTextColor(COLOR_TEXT, TFT_RED);
  tft->drawString("VOLVER", SCREEN_W / 2, 231, 1);
}

void drawInicioScreen() {
  tft->fillScreen(COLOR_BG);

  // Titulo
  tft->setTextDatum(MC_DATUM);
  tft->setTextColor(COLOR_ATTENTION, COLOR_BG);
  tft->drawString("GAPAXIONSZ", SCREEN_W / 2, 50, 4);

  // Subtitulo — nombre del usuario y ejercicio seleccionado
  tft->setTextColor(TFT_CYAN, COLOR_BG);
  if (usuarioActual >= 0 && gapaxionActual >= 0) {
    char subtitulo[30];
    snprintf(subtitulo, sizeof(subtitulo), "%s - %s", usuarioNombres[usuarioActual], gapaxionNombres[gapaxionActual]);
    tft->drawString(subtitulo, SCREEN_W / 2, 85, 2);
  } else if (usuarioActual >= 0) {
    tft->drawString(usuarioNombres[usuarioActual], SCREEN_W / 2, 85, 2);
  }
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString("Neurofeedback Entrenamiento", SCREEN_W / 2, 105, 2);

  // Boton CONECTAR
  tft->fillRoundRect(40, 140, 160, 50, 8, TFT_BLUE);
  tft->drawRoundRect(40, 140, 160, 50, 8, TFT_CYAN);
  tft->setTextColor(COLOR_TEXT, TFT_BLUE);
  tft->drawString("CONECTAR", SCREEN_W / 2, 165, 4);

  // ── BOTÓN USUARIO (esquina superior izquierda) ──
  tft->fillRoundRect(1, 1, 58, 26, 4, 0x0320);
  tft->setTextColor(COLOR_TEXT, 0x0320);
  tft->setTextDatum(MC_DATUM);
  tft->drawString("User", 30, 14, 2);

  // ── BOTÓN MENU (esquina superior derecha) ──
  tft->fillRoundRect(180, 1, 58, 26, 4, TFT_BLUE);
  tft->setTextColor(COLOR_TEXT, TFT_BLUE);
  tft->setTextDatum(MC_DATUM);
  tft->drawString("Menu", 209, 14, 2);

  // Instruccion
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString("Pulsa CONECTAR", SCREEN_W / 2, 210, 1);
  tft->drawString("y Enciende MindWave", SCREEN_W / 2, 224, 1);
}

// ════════════════════════════════════════════════════════════════════════════
//  SD CARD - Funciones de logging
// ════════════════════════════════════════════════════════════════════════════

// Lee el numero de sesion guardado en /{usuario}/{gapaxion}/sesion.txt, lo incrementa y lo guarda
uint16_t leerYActualizarSesion() {
  if (usuarioActual < 0 || gapaxionActual < 0) return 0;
  const char *carpetaUsr = usuarioCarpetas[usuarioActual];
  const char *carpetaGap = gapaxionNombres[gapaxionActual];

  // Crear carpeta del usuario si no existe
  if (!SD.exists(carpetaUsr)) {
    SD.mkdir(carpetaUsr);
    Serial.printf("[SD] Carpeta usuario creada: %s\n", carpetaUsr);
  }

  // Crear subcarpeta del gapaxion si no existe
  char carpetaFull[40];
  snprintf(carpetaFull, sizeof(carpetaFull), "%s/%s", carpetaUsr, carpetaGap);
  if (!SD.exists(carpetaFull)) {
    SD.mkdir(carpetaFull);
    Serial.printf("[SD] Carpeta gapaxion creada: %s\n", carpetaFull);
  }

  char ruta[60];
  snprintf(ruta, sizeof(ruta), "%s/%s/sesion.txt", carpetaUsr, carpetaGap);

  uint16_t num = 0;
  if (SD.exists(ruta)) {
    File f = SD.open(ruta, FILE_READ);
    if (f) {
      String s = f.readStringUntil('\n');
      num = (uint16_t)s.toInt();
      f.close();
    }
  }
  num++;
  File f = SD.open(ruta, FILE_WRITE);
  if (f) {
    f.println(num);
    f.close();
  }
  return num;
}

// Inicia una nueva sesion: crea archivo CSV en /{usuario}/{gapaxion}/
void iniciarLogSD() {
  if (!sdReady || usuarioActual < 0 || gapaxionActual < 0) return;

  sesionNumero = leerYActualizarSesion();

  char filename[80];
  snprintf(filename, sizeof(filename), "%s/%s/sesion_%03d.csv",
           usuarioCarpetas[usuarioActual], gapaxionNombres[gapaxionActual], sesionNumero);

  archivoSesion = SD.open(filename, FILE_WRITE);
  if (!archivoSesion) {
    Serial.printf("[SD] Error abriendo %s\n", filename);
    sdLogging = false;
    return;
  }

  // Escribir encabezado
  archivoSesion.println("Sesion,Fecha,Hora,Attention,Meditation,PoorSignal,IFN,RC,Delta,Theta,LowAlpha,HighAlpha,LowBeta,HighBeta,LowGamma,MidGamma");
  archivoSesion.flush();

  sdLogging = true;
  lastSDWrite = millis();
  Serial.printf("[SD] Sesion #%d [%s/%s] iniciada -> %s\n",
                sesionNumero, usuarioNombres[usuarioActual], gapaxionNombres[gapaxionActual], filename);
}

// Escribe una linea de datos en el CSV
void escribirLineaSD() {
  if (!sdLogging || !archivoSesion) return;

  RTC_Date dt = watch->rtc->getDateTime();
  char linea[200];
  snprintf(linea, sizeof(linea),
    "%d,%04d-%02d-%02d,%02d:%02d:%02d,%d,%d,%d,%.2f,%.2f,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
    sesionNumero,
    dt.year, dt.month, dt.day,
    dt.hour, dt.minute, dt.second,
    attention, meditation, poorSignal,
    indiceFatiga, ratioCarga,
    eegDelta, eegTheta, eegLowAlpha, eegHighAlpha,
    eegLowBeta, eegHighBeta, eegLowGamma, eegMidGamma
  );
  archivoSesion.println(linea);
  archivoSesion.flush();
}

// Cierra el archivo de sesion
void cerrarLogSD() {
  if (sdLogging && archivoSesion) {
    archivoSesion.close();
    Serial.printf("[SD] Sesion #%d cerrada\n", sesionNumero);
  }
  sdLogging = false;
}

void finalizarSesion() {
  // Detener tarea Core 0 primero (antes de cerrar SD y BT)
  if (btTaskRunning) {
    btTaskRunning = false;
    vTaskDelay(pdMS_TO_TICKS(50));  // esperar a que la tarea termine su ciclo
    Serial.println("[CORE0] Tarea detenida");
  }

  // Cerrar SD bajo mutex (la tarea ya no escribe)
  if (sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    cerrarLogSD();
    xSemaphoreGive(sdMutex);
  } else {
    cerrarLogSD();
  }

  drvStop();
  if (btIniciado) {
    SerialBT.disconnect();
    SerialBT.end();
  }
  btConnected = false;
  btIniciado = false;
  dataReceived = false;
  connectAttempts = 0;
  totalPackets = 0;
  badChecksums = 0;
  menuActivo = false;
  pantallaInicio = false;
  pantallaGapaxion = false;
  pantallaReloj = false;
  pantallaUsuario = true;
  usuarioActual = -1;
  gapaxionActual = -1;
  drawUserSelectScreen();
}

// ════════════════════════════════════════════════════════════════════════════
//  WALKING MEDITATION - Funciones
// ════════════════════════════════════════════════════════════════════════════

void finalizarWalkingMed() {
  walkingMedActivo    = false;
  walkingMedSeleccion = false;
  walkingMedEjercicio = false;
  walkingMedResultado = false;
  drvStop();
  menuActivo = true;
  drawMenuScreen();
}

void iniciarWalkingExercise() {
  const WalkLevel &lv = walkLevels[walkNivel - 1];
  walkPasosObjetivo  = lv.pasosObj;
  walkTiempoObjetivo = lv.tiempoObj;
  walkTolerancia     = lv.tolerancia;
  walkUmbralSuavidad = lv.umbralSuav;
  walkGuiaActiva     = lv.guiaHaptica;

  walkPasosContados  = 0;
  walkPasosEvaluados = 0;
  walkSumaRitmo      = 0;
  walkSumaSuavidad   = 0;
  walkMejorPaso      = 0;
  walkPeorPaso       = 100;
  walkUltimoTiming   = 0;
  walkUltimoPuntaje  = 0;
  walkAccelIdx       = 0;
  walkAccelCount     = 0;
  walkPrevMag        = 0;
  walkPeakAbove      = false;
  walkLastPeakTime   = 0;

  // Reset step counter (usado solo en modo AUTO como referencia)
  BMA *bma = watch->bma;
  bma->resetStepCounter();
  walkStepCountPrev = 0;

  walkInicioTime       = millis();
  walkUltimoPaso       = millis();
  walkLastAccelRead    = millis();
  walkLastScreenUpdate = millis();
  walkNextGuiaTime     = millis() + (unsigned long)walkTiempoObjetivo;

  walkingMedSeleccion = false;
  walkingMedEjercicio = true;
  walkingMedResultado = false;

  // Vibracion de inicio
  drvPlayEffect(7);  // Soft Bump — señal de inicio

  drawWalkingScreen();
  Serial.printf("[WALK] Ejercicio iniciado: Nivel %d, %d pasos, %.1fs/paso\n",
                walkNivel, walkPasosObjetivo, walkTiempoObjetivo / 1000.0);
}

// Calcular desviacion estandar del buffer de acelerometro
float calcAccelStdDev() {
  if (walkAccelCount < 2) return 0;
  int n = min(walkAccelCount, WALK_ACCEL_BUF_SIZE);
  float sum = 0, sumSq = 0;
  for (int i = 0; i < n; i++) {
    sum   += walkAccelSamples[i];
    sumSq += walkAccelSamples[i] * walkAccelSamples[i];
  }
  float mean = sum / n;
  float variance = (sumSq / n) - (mean * mean);
  return (variance > 0) ? sqrt(variance) : 0;
}

// Evaluar un paso: devuelve puntaje 0-100
float evaluarPaso(float tiempoEntrepasos, float accelStdDev) {
  // Puntaje ritmo: que tan cerca del objetivo
  float errorMs = fabs(tiempoEntrepasos - walkTiempoObjetivo);
  float puntRitmo = constrain(100.0 - (errorMs / walkTolerancia * 100.0), 0.0, 100.0);

  // Puntaje suavidad: baja stdDev = suave
  float umbral = (float)walkUmbralSuavidad;
  float puntSuav;
  if (accelStdDev <= umbral) {
    puntSuav = 100.0;
  } else if (accelStdDev >= umbral * 3.0) {
    puntSuav = 0.0;
  } else {
    puntSuav = 100.0 - ((accelStdDev - umbral) / (umbral * 2.0) * 100.0);
  }
  puntSuav = constrain(puntSuav, 0.0, 100.0);

  walkSumaRitmo    += puntRitmo;
  walkSumaSuavidad += puntSuav;
  walkPasosEvaluados++;

  float total = (puntRitmo * 0.5) + (puntSuav * 0.5);
  if (total > walkMejorPaso) walkMejorPaso = total;
  if (total < walkPeorPaso)  walkPeorPaso  = total;

  return total;
}

// Obtener etiqueta y color segun puntaje
void getWalkLabel(float puntaje, const char *&label, uint16_t &color) {
  if (puntaje >= 90)      { label = "Maestro";   color = TFT_GREEN;  }
  else if (puntaje >= 70) { label = "Atento";    color = TFT_CYAN;   }
  else if (puntaje >= 50) { label = "Aprendiz";  color = TFT_YELLOW; }
  else if (puntaje >= 30) { label = "Distraido"; color = TFT_ORANGE; }
  else                    { label = "Agitado";   color = TFT_RED;    }
}

// Registra un paso detectado (modo AUTO o TAP): calcula timing, puntaje, haptic, fin sesion
void registrarPaso(unsigned long now) {
  walkPasosContados++;

  float tiempoEntrePasos = (float)(now - walkUltimoPaso);
  walkUltimoPaso   = now;
  walkUltimoTiming = tiempoEntrePasos;

  if (walkPasosContados > 1) {
    float accelStdDev = calcAccelStdDev();
    float puntaje     = evaluarPaso(tiempoEntrePasos, accelStdDev);
    walkUltimoPuntaje = puntaje;
    drvPlayEffect(puntaje >= 60 ? 7 : 12);
    Serial.printf("[WALK] Paso %d: %.1fs (obj:%.1fs) stdDev:%.0f punt:%.0f\n",
                  walkPasosContados, tiempoEntrePasos / 1000.0,
                  walkTiempoObjetivo / 1000.0, accelStdDev, puntaje);
  }

  // Reset buffer acelerometro para siguiente paso
  walkAccelIdx   = 0;
  walkAccelCount = 0;

  // Reiniciar guia haptica desde este paso
  walkNextGuiaTime = now + (unsigned long)walkTiempoObjetivo;

  // Comprobar fin de sesion
  if (walkPasosContados >= walkPasosObjetivo) {
    if (walkPasosEvaluados > 0) {
      walkRitmoFinal    = walkSumaRitmo / walkPasosEvaluados;
      walkSuavidadFinal = walkSumaSuavidad / walkPasosEvaluados;
      walkPuntajeFinal  = (walkRitmoFinal * 0.5) + (walkSuavidadFinal * 0.5);
    }
    walkingMedEjercicio = false;
    walkingMedResultado = true;
    drvStop();
    drvPlayEffect(walkPuntajeFinal >= 70 ? 14 : 4);
    drawWalkingResultScreen();
    Serial.printf("[WALK] COMPLETO! Puntaje: %.0f (Ritmo:%.0f Suav:%.0f)\n",
                  walkPuntajeFinal, walkRitmoFinal, walkSuavidadFinal);
  }
}

void drawWalkingSelScreen() {
  tft->fillScreen(COLOR_BG);

  // Header
  tft->fillRoundRect(0, 0, SCREEN_W, 28, 0, 0x0320);  // verde oscuro
  tft->setTextColor(TFT_GREEN, 0x0320);
  tft->setTextDatum(MC_DATUM);
  tft->drawString("MINDFUL STEPS", SCREEN_W / 2, 14, 2);

  // Subtitulo
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString("Walking Meditation", SCREEN_W / 2, 42, 2);

  // Nivel seleccionado info
  const WalkLevel &lv = walkLevels[walkNivel - 1];
  tft->setTextColor(TFT_CYAN, COLOR_BG);
  tft->drawString(lv.nombre, SCREEN_W / 2, 65, 2);

  char infoBuf[32];
  snprintf(infoBuf, sizeof(infoBuf), "%d pasos - %.1fs/paso", lv.pasosObj, lv.tiempoObj / 1000.0);
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString(infoBuf, SCREEN_W / 2, 85, 1);

  // Botones de nivel en fila horizontal (y=105, h=30)
  int btnW = 66, btnH = 30, btnY = 105, gap = 5;
  int x1 = 5, x2 = x1 + btnW + gap, x3 = x2 + btnW + gap;

  uint16_t c1 = (walkNivel == 1) ? TFT_GREEN : COLOR_BAR_BG;
  tft->fillRoundRect(x1, btnY, btnW, btnH, 6, c1);
  tft->drawRoundRect(x1, btnY, btnW, btnH, 6, COLOR_TEXT_DIM);
  tft->setTextColor(COLOR_TEXT, c1);
  tft->drawString("Lv.1", x1 + btnW / 2, btnY + btnH / 2, 2);

  uint16_t c2 = (walkNivel == 2) ? TFT_GREEN : COLOR_BAR_BG;
  tft->fillRoundRect(x2, btnY, btnW, btnH, 6, c2);
  tft->drawRoundRect(x2, btnY, btnW, btnH, 6, COLOR_TEXT_DIM);
  tft->setTextColor(COLOR_TEXT, c2);
  tft->drawString("Lv.2", x2 + btnW / 2, btnY + btnH / 2, 2);

  uint16_t c3 = (walkNivel == 3) ? TFT_GREEN : COLOR_BAR_BG;
  tft->fillRoundRect(x3, btnY, btnW, btnH, 6, c3);
  tft->drawRoundRect(x3, btnY, btnW, btnH, 6, COLOR_TEXT_DIM);
  tft->setTextColor(COLOR_TEXT, c3);
  tft->drawString("Lv.3", x3 + btnW / 2, btnY + btnH / 2, 2);

  // Botones AUTO / TAP (modo deteccion de pasos)
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString("Deteccion:", SCREEN_W / 2, 140, 1);

  uint16_t cAuto = (walkDeteccionMode == 0) ? TFT_GREEN : COLOR_BAR_BG;
  uint16_t cTap  = (walkDeteccionMode == 1) ? TFT_GREEN : COLOR_BAR_BG;
  tft->fillRoundRect(20,  152, 90, 26, 6, cAuto);
  tft->drawRoundRect(20,  152, 90, 26, 6, COLOR_TEXT_DIM);
  tft->setTextColor(COLOR_TEXT, cAuto);
  tft->drawString("AUTO", 65, 165, 2);

  tft->fillRoundRect(130, 152, 90, 26, 6, cTap);
  tft->drawRoundRect(130, 152, 90, 26, 6, COLOR_TEXT_DIM);
  tft->setTextColor(COLOR_TEXT, cTap);
  tft->drawString("TAP", 175, 165, 2);

  // Boton INICIAR (y=188, h=30)
  tft->fillRoundRect(30, 188, 180, 30, 8, TFT_GREEN);
  tft->drawRoundRect(30, 188, 180, 30, 8, TFT_WHITE);
  tft->setTextColor(TFT_BLACK, TFT_GREEN);
  tft->drawString("INICIAR", SCREEN_W / 2, 203, 2);

  // Boton VOLVER (y=224, h=14)
  tft->fillRoundRect(60, 224, 120, 14, 4, TFT_RED);
  tft->setTextColor(COLOR_TEXT, TFT_RED);
  tft->drawString("VOLVER", SCREEN_W / 2, 231, 1);
}

void drawWalkingScreen() {
  tft->fillScreen(COLOR_BG);

  // Header
  char hdrBuf[24];
  snprintf(hdrBuf, sizeof(hdrBuf), "MINDFUL STEPS - Lv.%d", walkNivel);
  tft->fillRoundRect(0, 0, SCREEN_W, 28, 0, 0x0320);
  tft->setTextColor(TFT_GREEN, 0x0320);
  tft->setTextDatum(MC_DATUM);
  tft->drawString(hdrBuf, SCREEN_W / 2, 14, 2);

  // Pasos: "12 / 20"
  char pasosBuf[16];
  snprintf(pasosBuf, sizeof(pasosBuf), "%d / %d", walkPasosContados, walkPasosObjetivo);
  tft->setTextColor(TFT_WHITE, COLOR_BG);
  tft->drawString("PASOS", SCREEN_W / 2, 40, 2);
  tft->setTextColor(TFT_GREEN, COLOR_BG);
  tft->drawString(pasosBuf, SCREEN_W / 2, 60, 4);

  // Barra de progreso (y=78, h=8)
  int progW = 200;
  int progX = (SCREEN_W - progW) / 2;
  float progPct = (walkPasosObjetivo > 0) ? (float)walkPasosContados / walkPasosObjetivo : 0;
  progPct = constrain(progPct, 0.0, 1.0);
  tft->fillRoundRect(progX, 78, progW, 8, 3, COLOR_BAR_BG);
  if (progPct > 0)
    tft->fillRoundRect(progX, 78, (int)(progW * progPct), 8, 3, TFT_GREEN);

  // Ultimo timing
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString("Ultimo paso:", SCREEN_W / 2, 98, 1);

  if (walkUltimoTiming > 0) {
    char timBuf[16];
    snprintf(timBuf, sizeof(timBuf), "%.1fs", walkUltimoTiming / 1000.0);
    // Color segun precision
    float errorMs = fabs(walkUltimoTiming - walkTiempoObjetivo);
    uint16_t timColor;
    if (errorMs <= walkTolerancia * 0.3)       timColor = TFT_GREEN;
    else if (errorMs <= walkTolerancia * 0.7)  timColor = TFT_YELLOW;
    else if (errorMs <= walkTolerancia)        timColor = TFT_ORANGE;
    else                                       timColor = TFT_RED;
    tft->setTextColor(timColor, COLOR_BG);
    tft->drawString(timBuf, SCREEN_W / 2 - 30, 114, 4);

    // Objetivo referencia
    char objBuf[16];
    snprintf(objBuf, sizeof(objBuf), "/ %.1fs", walkTiempoObjetivo / 1000.0);
    tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft->drawString(objBuf, SCREEN_W / 2 + 50, 114, 2);
  } else {
    tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
    tft->drawString("Esperando...", SCREEN_W / 2, 114, 2);
  }

  // Suavidad actual (stdDev del acelerometro)
  float stdDev = calcAccelStdDev();
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString("Suavidad:", 60, 142, 1);
  // Barra suavidad (y=152, h=10)
  float umbral = (float)walkUmbralSuavidad;
  float suavPct = constrain(1.0 - (stdDev / (umbral * 3.0)), 0.0, 1.0);
  uint16_t suavColor;
  if (suavPct >= 0.7)      suavColor = TFT_GREEN;
  else if (suavPct >= 0.4) suavColor = TFT_YELLOW;
  else                     suavColor = TFT_RED;
  tft->fillRoundRect(20, 152, 200, 10, 3, COLOR_BAR_BG);
  if (suavPct > 0)
    tft->fillRoundRect(20, 152, (int)(200 * suavPct), 10, 3, suavColor);

  // Puntaje parcial
  float puntParcial = 0;
  if (walkPasosEvaluados > 0)
    puntParcial = ((walkSumaRitmo / walkPasosEvaluados) * 0.5) +
                  ((walkSumaSuavidad / walkPasosEvaluados) * 0.5);
  const char *labelP; uint16_t colorP;
  getWalkLabel(puntParcial, labelP, colorP);

  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString("Puntaje:", SCREEN_W / 2, 175, 1);
  char puntBuf[8];
  snprintf(puntBuf, sizeof(puntBuf), "%.0f", puntParcial);
  tft->setTextColor(colorP, COLOR_BG);
  tft->drawString(puntBuf, SCREEN_W / 2 - 20, 192, 4);
  tft->drawString(labelP, SCREEN_W / 2 + 40, 192, 2);

  // Boton PARAR (y=218, h=20)
  tft->fillRoundRect(80, 218, 80, 20, 4, TFT_RED);
  tft->setTextColor(COLOR_TEXT, TFT_RED);
  tft->drawString("PARAR", SCREEN_W / 2, 228, 2);
}

void drawWalkingResultScreen() {
  tft->fillScreen(COLOR_BG);

  // Header
  tft->fillRoundRect(0, 0, SCREEN_W, 28, 0, 0x0320);
  tft->setTextColor(TFT_GREEN, 0x0320);
  tft->setTextDatum(MC_DATUM);
  tft->drawString("SESION COMPLETA", SCREEN_W / 2, 14, 2);

  // Puntaje total grande
  const char *label; uint16_t color;
  getWalkLabel(walkPuntajeFinal, label, color);

  char puntBuf[8];
  snprintf(puntBuf, sizeof(puntBuf), "%.0f", walkPuntajeFinal);
  tft->setTextColor(color, COLOR_BG);
  tft->drawString(puntBuf, SCREEN_W / 2, 50, 4);
  tft->drawString(label, SCREEN_W / 2, 75, 2);

  // Nivel info
  char nivBuf[24];
  snprintf(nivBuf, sizeof(nivBuf), "Nivel %d - %d pasos", walkNivel, walkPasosContados);
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString(nivBuf, SCREEN_W / 2, 108, 1);

  // Desglose
  tft->drawFastHLine(20, 118, 200, COLOR_TEXT_DIM);

  char ritmoBuf[24], suavBuf[24];
  snprintf(ritmoBuf, sizeof(ritmoBuf), "Ritmo: %.0f%%", walkRitmoFinal);
  snprintf(suavBuf,  sizeof(suavBuf),  "Suavidad: %.0f%%", walkSuavidadFinal);
  tft->setTextColor(TFT_CYAN, COLOR_BG);
  tft->drawString(ritmoBuf, SCREEN_W / 2, 132, 2);
  tft->setTextColor(TFT_YELLOW, COLOR_BG);
  tft->drawString(suavBuf, SCREEN_W / 2, 152, 2);

  // Mejor/peor paso
  char mejBuf[24], peoBuf[24];
  snprintf(mejBuf, sizeof(mejBuf), "Mejor: %.0f", walkMejorPaso);
  snprintf(peoBuf, sizeof(peoBuf), "Peor: %.0f",  walkPeorPaso);
  tft->setTextColor(TFT_GREEN, COLOR_BG);
  tft->drawString(mejBuf, 70, 172, 2);
  tft->setTextColor(TFT_RED, COLOR_BG);
  tft->drawString(peoBuf, 170, 172, 2);

  // Boton REPETIR (y=195, h=26)
  tft->fillRoundRect(20, 195, 95, 26, 6, TFT_GREEN);
  tft->setTextColor(TFT_BLACK, TFT_GREEN);
  tft->drawString("REPETIR", 67, 208, 2);

  // Boton SALIR (y=195, h=26)
  tft->fillRoundRect(125, 195, 95, 26, 6, TFT_RED);
  tft->setTextColor(COLOR_TEXT, TFT_RED);
  tft->drawString("SALIR", 172, 208, 2);
}

void drawMainScreen() {
  tft->fillScreen(COLOR_BG);

  // ── HEADER ──
  tft->fillRoundRect(0, 0, SCREEN_W, 28, 0, COLOR_HEADER);
  tft->setTextColor(COLOR_TEXT, COLOR_HEADER);
  tft->setTextDatum(MC_DATUM);
  tft->drawString("NEURO-CRAWLER  MindWave", SCREEN_W / 2, 14, 2);

  // ── BOTÓN MENU (esquina superior derecha) ──
  tft->fillRoundRect(180, 1, 58, 26, 4, TFT_BLUE);
  tft->setTextColor(COLOR_TEXT, TFT_BLUE);
  tft->setTextDatum(MC_DATUM);
  tft->drawString("Menu", 209, 14, 2);

  // ── SEÑAL ──
  int sigY = 32;
  uint16_t sigColor = (poorSignal == 0) ? COLOR_SIGNAL_OK :
                      (poorSignal < 50) ? TFT_YELLOW : COLOR_SIGNAL_BAD;
  tft->fillCircle(12, sigY + 8, 6, sigColor);
  tft->setTextColor(COLOR_TEXT, COLOR_BG);
  tft->setTextDatum(TL_DATUM);

  char sigBuf[32];
  if (poorSignal == 0)        snprintf(sigBuf, sizeof(sigBuf), "Senal: OK");
  else if (poorSignal == 200) snprintf(sigBuf, sizeof(sigBuf), "Sin contacto");
  else                        snprintf(sigBuf, sizeof(sigBuf), "Ruido: %d", poorSignal);
  tft->drawString(sigBuf, 24, sigY + 2, 2);

  // Paquetes recibidos
  tft->setTextDatum(TR_DATUM);
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  char pktBuf[16];
  snprintf(pktBuf, sizeof(pktBuf), "P:%lu", totalPackets);
  tft->drawString(pktBuf, SCREEN_W - 5, sigY + 2, 2);

  // ── ATENCIÓN ──
  int attY = 54;
  tft->setTextDatum(TL_DATUM);
  tft->setTextColor(COLOR_ATTENTION, COLOR_BG);
  tft->drawString("ATENCION", 5, attY, 2);

  char attBuf[8];
  snprintf(attBuf, sizeof(attBuf), "%3d", attention);
  tft->setTextDatum(TR_DATUM);
  tft->setTextColor(eSenseColor(attention), COLOR_BG);
  tft->drawString(attBuf, SCREEN_W - 5, attY, 4);

  drawBar(5, attY + 26, SCREEN_W - 10, 12, attention, COLOR_ATTENTION);

  tft->setTextDatum(TL_DATUM);
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString(eSenseLabel(attention), 5, attY + 40, 1);

  // ── MEDITACIÓN ──
  int medY = 104;
  tft->setTextDatum(TL_DATUM);
  tft->setTextColor(COLOR_MEDITATE, COLOR_BG);
  tft->drawString("MEDITACION", 5, medY, 2);

  char medBuf[8];
  snprintf(medBuf, sizeof(medBuf), "%3d", meditation);
  tft->setTextDatum(TR_DATUM);
  tft->setTextColor(eSenseColor(meditation), COLOR_BG);
  tft->drawString(medBuf, SCREEN_W - 5, medY, 4);

  drawBar(5, medY + 26, SCREEN_W - 10, 12, meditation, COLOR_MEDITATE);

  tft->setTextDatum(TL_DATUM);
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString(eSenseLabel(meditation), 5, medY + 40, 1);

  // ── ÍNDICES NEURO-CRAWLER ──
  int ncY = 156;
  tft->drawFastHLine(5, ncY, SCREEN_W - 10, COLOR_TEXT_DIM);
  ncY += 4;

  tft->setTextColor(COLOR_FATIGA, COLOR_BG);
  tft->setTextDatum(TL_DATUM);
  char ifnBuf[32];
  snprintf(ifnBuf, sizeof(ifnBuf), "IFN: %.2f", indiceFatiga);
  tft->drawString(ifnBuf, 5, ncY, 2);

  const char* fLabel = (indiceFatiga < 1.0) ? "Alerta" :
                       (indiceFatiga < 2.0) ? "Normal" :
                       (indiceFatiga < 3.0) ? "Cansancio" : "DESCANSAR";
  tft->setTextDatum(TR_DATUM);
  tft->drawString(fLabel, SCREEN_W - 5, ncY, 2);

  ncY += 20;
  tft->setTextColor(COLOR_CARGA, COLOR_BG);
  tft->setTextDatum(TL_DATUM);
  char rcBuf[32];
  snprintf(rcBuf, sizeof(rcBuf), "RC:  %.2f", ratioCarga);
  tft->drawString(rcBuf, 5, ncY, 2);

  const char* cLabel = (ratioCarga < 0.5) ? "Relajado" :
                       (ratioCarga < 1.5) ? "Activo" : "Intenso";
  tft->setTextDatum(TR_DATUM);
  tft->drawString(cLabel, SCREEN_W - 5, ncY, 2);

  // ── MINI BANDAS EEG ──
  int eegY = 200;
  tft->drawFastHLine(5, eegY, SCREEN_W - 10, COLOR_TEXT_DIM);
  eegY += 3;

  const char* labels[] = {"d","t","lA","hA","lB","hB","lG","mG"};
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->setTextDatum(TL_DATUM);
  for (int k = 0; k < 8; k++) {
    tft->drawString(labels[k], 5 + k * 29, eegY, 1);
  }

  uint32_t eegVals[8] = { eegDelta, eegTheta, eegLowAlpha, eegHighAlpha,
                           eegLowBeta, eegHighBeta, eegLowGamma, eegMidGamma };
  uint32_t maxVal = 1;
  for (int k = 0; k < 8; k++) { if (eegVals[k] > maxVal) maxVal = eegVals[k]; }

  uint16_t eegColors[8] = { TFT_BLUE, TFT_PURPLE, TFT_GREEN, TFT_DARKGREEN,
                             TFT_YELLOW, TFT_ORANGE, TFT_RED, TFT_MAGENTA };
  int barW = 22, barMaxH = 22, barStartY = eegY + 12;
  for (int k = 0; k < 8; k++) {
    int bx = 5 + k * 29;
    int bh = map(constrain(eegVals[k], 0, maxVal), 0, maxVal, 1, barMaxH);
    tft->fillRect(bx, barStartY + (barMaxH - bh), barW, bh, eegColors[k]);
    tft->drawRect(bx, barStartY, barW, barMaxH, COLOR_BAR_BG);
  }
}

void drawMenuScreen() {
  tft->fillScreen(COLOR_BG);

  // Header
  tft->fillRoundRect(0, 0, SCREEN_W, 28, 0, COLOR_HEADER);
  tft->setTextColor(COLOR_TEXT, COLOR_HEADER);
  tft->setTextDatum(MC_DATUM);
  tft->drawString("MENU", SCREEN_W / 2, 14, 2);

  // ── Batería ──────────────────────────────────────────────
  float batVolt   = power->getBattVoltage();
  // Calcular % desde voltaje (getBattPercentage() devuelve 127 sin calibrar)
  int   batPct    = constrain((int)((batVolt - 3000.0) / (4200.0 - 3000.0) * 100.0), 0, 100);
  bool  charging  = power->isChargeing();
  bool  connected = power->isBatteryConnect();

  // Color según nivel
  uint16_t batColor;
  if (!connected)         batColor = COLOR_TEXT_DIM;
  else if (batPct >= 60)  batColor = TFT_GREEN;
  else if (batPct >= 30)  batColor = TFT_YELLOW;
  else                    batColor = TFT_RED;

  // Porcentaje grande
  char batBuf[16];
  snprintf(batBuf, sizeof(batBuf), "%d%%", connected ? batPct : 0);
  tft->setTextColor(batColor, COLOR_BG);
  tft->setTextDatum(MC_DATUM);
  tft->drawString(batBuf, SCREEN_W / 2, 36, 4);

  // Voltaje y estado de carga
  char voltBuf[32];
  if (!connected) {
    snprintf(voltBuf, sizeof(voltBuf), "Sin bateria");
  } else if (charging) {
    snprintf(voltBuf, sizeof(voltBuf), "Cargando %.0fmV", batVolt);
  } else if (batPct >= 100) {
    snprintf(voltBuf, sizeof(voltBuf), "Carga completa");
  } else {
    snprintf(voltBuf, sizeof(voltBuf), "%.0f mV", batVolt);
  }
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString(voltBuf, SCREEN_W / 2, 58, 1);

  // Línea separadora
  tft->drawFastHLine(10, 68, SCREEN_W - 20, COLOR_TEXT_DIM);

  // ── Umbral vibracion ──────────────────────────────────────
  char umbBuf[32];
  if (umbralVibracion == 0) snprintf(umbBuf, sizeof(umbBuf), "Vibra: OFF");
  else snprintf(umbBuf, sizeof(umbBuf), "Vibra: >= %d", umbralVibracion);
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString(umbBuf, SCREEN_W / 2, 78, 2);

  // ── Botones umbral en fila horizontal (y=94, h=28) ────────
  int btnW = 66, btnH = 28, btnY = 94, gap = 5;
  int x70 = 5, x80 = x70 + btnW + gap, x90 = x80 + btnW + gap;

  uint16_t c70 = (umbralVibracion == 70) ? TFT_GREEN : COLOR_BAR_BG;
  tft->fillRoundRect(x70, btnY, btnW, btnH, 6, c70);
  tft->drawRoundRect(x70, btnY, btnW, btnH, 6, COLOR_TEXT_DIM);
  tft->setTextColor(COLOR_TEXT, c70);
  tft->drawString("70", x70 + btnW / 2, btnY + btnH / 2, 4);

  uint16_t c80 = (umbralVibracion == 80) ? TFT_GREEN : COLOR_BAR_BG;
  tft->fillRoundRect(x80, btnY, btnW, btnH, 6, c80);
  tft->drawRoundRect(x80, btnY, btnW, btnH, 6, COLOR_TEXT_DIM);
  tft->setTextColor(COLOR_TEXT, c80);
  tft->drawString("80", x80 + btnW / 2, btnY + btnH / 2, 4);

  uint16_t c90 = (umbralVibracion == 90) ? TFT_GREEN : COLOR_BAR_BG;
  tft->fillRoundRect(x90, btnY, btnW, btnH, 6, c90);
  tft->drawRoundRect(x90, btnY, btnW, btnH, 6, COLOR_TEXT_DIM);
  tft->setTextColor(COLOR_TEXT, c90);
  tft->drawString("90", x90 + btnW / 2, btnY + btnH / 2, 4);

  // Línea separadora
  tft->drawFastHLine(10, 130, SCREEN_W - 20, COLOR_TEXT_DIM);

  // ── Botón FIN (y=136, h=26) — finalizar sesión ──
  tft->fillRoundRect(20, 136, 200, 26, 6, TFT_ORANGE);
  tft->drawRoundRect(20, 136, 200, 26, 6, TFT_YELLOW);
  tft->setTextColor(COLOR_TEXT, TFT_ORANGE);
  tft->drawString("FIN SESION", SCREEN_W / 2, 149, 2);

  // ── Botón WALK MED (y=168, h=24) ──
  tft->fillRoundRect(20, 168, 200, 24, 6, 0x0320);
  tft->drawRoundRect(20, 168, 200, 24, 6, TFT_GREEN);
  tft->setTextColor(TFT_GREEN, 0x0320);
  tft->drawString("WALK MED", SCREEN_W / 2, 180, 2);

  // ── Botón RELOJ (y=198, h=24) — ajustar fecha/hora ──
  tft->fillRoundRect(20, 198, 200, 24, 6, 0x4208);  // gris oscuro
  tft->drawRoundRect(20, 198, 200, 24, 6, TFT_YELLOW);
  tft->setTextColor(TFT_YELLOW, 0x4208);
  tft->drawString("RELOJ", SCREEN_W / 2, 210, 2);

  // Botón EXIT (y=228, h=12)
  tft->fillRoundRect(70, 228, 100, 12, 4, TFT_RED);
  tft->setTextColor(COLOR_TEXT, TFT_RED);
  tft->drawString("EXIT", SCREEN_W / 2, 234, 1);
}

void drawConnectScreen(const char* status, const char* detail) {
  tft->fillScreen(COLOR_BG);

  tft->setTextDatum(MC_DATUM);
  tft->setTextColor(COLOR_ATTENTION, COLOR_BG);
  tft->drawString("NEURO-CRAWLER", SCREEN_W / 2, 30, 4);

  tft->setTextColor(COLOR_TEXT, COLOR_BG);
  tft->drawString("MindWave BT SPP", SCREEN_W / 2, 65, 2);

  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString("A4:DA:32:6F:EE:BA", SCREEN_W / 2, 85, 2);

  tft->setTextColor(COLOR_CARGA, COLOR_BG);
  tft->drawString("Baud: 57600", SCREEN_W / 2, 105, 2);

  tft->setTextColor(TFT_YELLOW, COLOR_BG);
  tft->drawString(status, SCREEN_W / 2, 135, 2);

  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString(detail, SCREEN_W / 2, 158, 2);

  char attBuf[32];
  snprintf(attBuf, sizeof(attBuf), "Intento: %d", connectAttempts);
  tft->drawString(attBuf, SCREEN_W / 2, 180, 2);

  // BT callback status
  tft->setTextColor(COLOR_CARGA, COLOR_BG);
  tft->drawString(btStatus.c_str(), SCREEN_W / 2, 200, 1);

  // Animación
  int dotPhase = (millis() / 300) % 4;
  for (int d = 0; d < 4; d++) {
    uint16_t dotColor = (d == dotPhase) ? TFT_CYAN : COLOR_BAR_BG;
    tft->fillCircle(90 + d * 20, 225, 5, dotColor);
  }
}

// ════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  Serial.println("\n══════════════════════════════════════");
  Serial.println("  NEURO-CRAWLER x T-Watch x MindWave");
  Serial.println("  Bluetooth SPP @ 57600 baud  [DUAL-CORE]");
  Serial.println("══════════════════════════════════════");

  // ── Crear mutexes FreeRTOS ──
  neuroMutex    = xSemaphoreCreateMutex();
  sdMutex       = xSemaphoreCreateMutex();
  rtcCacheMutex = xSemaphoreCreateMutex();
  Serial.println("[RTOS] Mutexes creados (neuro, sd, rtcCache)");

  // ── Inicializar T-Watch ──
  watch = TTGOClass::getWatch();
  watch->begin();
  watch->openBL();

  tft   = watch->tft;
  power = watch->power;

  tft->setRotation(2);
  tft->fillScreen(COLOR_BG);
  tft->setSwapBytes(true);
  //watch->setBrightness(180);
  watch->setBrightness(100);

  // ── Inicializar motor vibración (DRV2605L via I2C) ──
  watch->enableDrv2650();       // AXP202 GPIO0 alimenta el DRV2605L
  drv = watch->drv;
  drv->selectLibrary(1);
  drv->setMode(DRV2605_MODE_INTTRIG);

  // ── Inicializar BMA423 (acelerómetro + podómetro) ──
  BMA *bma = watch->bma;
  bma->begin();
  Acfg accelCfg;
  accelCfg.odr       = BMA4_OUTPUT_DATA_RATE_100HZ;
  accelCfg.range      = BMA4_ACCEL_RANGE_2G;
  accelCfg.bandwidth  = BMA4_ACCEL_NORMAL_AVG4;
  accelCfg.perf_mode  = BMA4_CONTINUOUS_MODE;
  bma->accelConfig(accelCfg);
  bma->enableAccel();
  bma->enableFeature(BMA423_STEP_CNTR, 1);
  bma->resetStepCounter();
  Serial.println("[BMA] Acelerometro + Step Counter inicializados");

  // ── Inicializar SD Card ──
  if (watch->sdcard_begin()) {
    sdReady = true;
    Serial.println("[SD] SD Card inicializada correctamente");
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("[SD] No se detecto tarjeta SD");
      sdReady = false;
    } else {
      Serial.printf("[SD] Tipo: %s, Tamano: %llu MB\n",
        (cardType == CARD_MMC) ? "MMC" :
        (cardType == CARD_SD)  ? "SDSC" :
        (cardType == CARD_SDHC)? "SDHC" : "UNKNOWN",
        SD.cardSize() / (1024 * 1024));
    }
  } else {
    sdReady = false;
    Serial.println("[SD] Error: No se pudo inicializar SD Card");
  }

  // ── Mostrar pantalla de seleccion de usuario ──
  pantallaUsuario = true;
  drawUserSelectScreen();
  Serial.println("[INFO] Selecciona usuario para continuar.");
}

// ════════════════════════════════════════════════════════════════════════════
//  INICIAR BLUETOOTH (se ejecuta al pulsar CONECTAR)
// ════════════════════════════════════════════════════════════════════════════

void iniciarBluetooth() {
  drawConnectScreen("Iniciando BT...", "Configurando SPP");

  // ══════════════════════════════════════════════════════════════════════
  // ORDEN CRÍTICO PARA CONEXIÓN SPP COMO MAESTRO:
  //   1. setPin() ANTES de begin()
  //   2. Registrar callback para debug
  //   3. begin() con nombre y true = modo maestro
  // ══════════════════════════════════════════════════════════════════════

  // 1. Configurar PIN (MindWave usa "0000")
  SerialBT.setPin("0000");
  Serial.println("[BT] PIN configurado: 0000");

  // 2. Registrar callback de eventos SPP
  SerialBT.register_callback(btCallback);
  Serial.println("[BT] Callback registrado");

  // 3. Iniciar BT como MAESTRO (true)
  if (!SerialBT.begin("NeuroCrawler", true)) {
    Serial.println("[ERROR] Fallo al inicializar Bluetooth!");
    tft->setTextColor(TFT_RED, COLOR_BG);
    tft->setTextDatum(MC_DATUM);
    tft->drawString("ERROR: BT INIT FALLO", SCREEN_W / 2, 140, 2);
    delay(3000);
    drawInicioScreen();
    pantallaInicio = true;
    return;
  }

  Serial.println("[BT] Bluetooth iniciado como MAESTRO SPP");
  Serial.println("[BT] Nombre: NeuroCrawler");
  Serial.printf("[BT] Target: A4:DA:32:6F:EE:BA @ %d baud\n", MINDWAVE_BAUDRATE);

  btStatus = "BT Maestro OK";
  btIniciado = true;
  drawConnectScreen("BT Listo", "Buscando MindWave...");

  Serial.println("\n[INFO] ─────────────────────────────────────────");
  Serial.println("[INFO] INSTRUCCIONES:");
  Serial.println("[INFO] 1. Enciende el MindWave (switch ON)");
  Serial.println("[INFO] 2. Si LED parpadea rapido = modo normal");
  Serial.println("[INFO] 3. Si no conecta, manten switch arriba");
  Serial.println("[INFO]    4-5 seg para modo discoverable");
  Serial.println("[INFO]    (LED parpadea doble)");
  Serial.println("[INFO] 4. El ESP32 intentara conectar cada 8s");
  Serial.println("[INFO] ─────────────────────────────────────────\n");

  connectAttemptTime = millis();
}

// ════════════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════════════

void loop() {
  unsigned long now = millis();

  // ════════════════════════════════════════════════════════════════════════
  // WALKING MEDITATION - Lógica del ejercicio
  // ════════════════════════════════════════════════════════════════════════

  if (walkingMedActivo && walkingMedEjercicio) {
    BMA *bma = watch->bma;

    // 1. Leer acelerómetro cada 20ms -> buffer suavidad
    if (now - walkLastAccelRead >= 20) {
      walkLastAccelRead = now;
      Accel acc;
      if (bma->getAccel(acc)) {
        float mag = sqrt((float)acc.x * acc.x + (float)acc.y * acc.y + (float)acc.z * acc.z);
        walkAccelSamples[walkAccelIdx] = mag;
        walkAccelIdx = (walkAccelIdx + 1) % WALK_ACCEL_BUF_SIZE;
        if (walkAccelCount < WALK_ACCEL_BUF_SIZE) walkAccelCount++;
      }
    }

    // 2. Deteccion de pasos segun modo
    if (walkDeteccionMode == 0) {
      // Modo AUTO: detectar pico de aceleracion (impacto del paso)
      // Usamos la ultima magnitud ya calculada en el bloque acelerometro
      if (walkAccelCount > 0) {
        int lastIdx = (walkAccelIdx - 1 + WALK_ACCEL_BUF_SIZE) % WALK_ACCEL_BUF_SIZE;
        float mag = walkAccelSamples[lastIdx];
        bool debounceOk = (now - walkLastPeakTime) >= WALK_PEAK_DEBOUNCE;

        if (mag >= WALK_PEAK_THRESHOLD && !walkPeakAbove && debounceOk) {
          // Flanco ascendente — nuevo paso detectado
          walkPeakAbove    = true;
          walkLastPeakTime = now;
          if (!walkingMedResultado) registrarPaso(now);
        } else if (mag < WALK_PEAK_THRESHOLD) {
          walkPeakAbove = false;
        }
      }
    }
    // Modo TAP: los pasos se registran desde el touch handler del ejercicio

    // 3. Guía háptica (solo nivel 1): vibrar cuando toca dar paso
    if (walkGuiaActiva && now >= walkNextGuiaTime && walkPasosContados < walkPasosObjetivo) {
      drvPlayEffect(14);  // Buzz 100% — señal de paso (mayor intensidad)
      walkNextGuiaTime = now + (unsigned long)walkTiempoObjetivo;
    }

    // 3b. Touch durante ejercicio: PARAR o TAP-paso
    {
      int16_t tx, ty;
      if (watch->getTouch(tx, ty) && (now - lastTouchTime > 400)) {
        lastTouchTime = now;
        if (ty >= 218 && ty <= 238 && tx >= 80 && tx <= 160) {
          // Boton PARAR
          if (walkPasosEvaluados > 0) {
            walkRitmoFinal    = walkSumaRitmo / walkPasosEvaluados;
            walkSuavidadFinal = walkSumaSuavidad / walkPasosEvaluados;
            walkPuntajeFinal  = (walkRitmoFinal * 0.5) + (walkSuavidadFinal * 0.5);
          }
          walkingMedEjercicio = false;
          walkingMedResultado = true;
          drvStop();
          drvPlayEffect(walkPuntajeFinal >= 70 ? 14 : 4);
          drawWalkingResultScreen();
        } else if (walkDeteccionMode == 1) {
          // Modo TAP: cualquier toque fuera de PARAR = nuevo paso
          registrarPaso(now);
        }
      }
    }

    // 4. Actualizar pantalla cada 300ms
    if (now - walkLastScreenUpdate >= 300) {
      walkLastScreenUpdate = now;
      drawWalkingScreen();
    }

    // Si BT conectado, seguir leyendo datos y actualizando cache RTC
    if (btConnected && btTaskRunning) {
      readNeuroData();
      if (now - lastRtcCacheUpdate >= 1000) {
        lastRtcCacheUpdate = now;
        actualizarCacheRTC();
      }
    }

    return;  // no procesar pantalla principal durante ejercicio
  }

  // ════════════════════════════════════════════════════════════════════════
  // FASE -1: SELECCION DE USUARIO
  // ════════════════════════════════════════════════════════════════════════

  if (pantallaUsuario) {
    int16_t tx, ty;
    if (watch->getTouch(tx, ty) && (now - lastTouchTime > 400)) {
      lastTouchTime = now;
      // Boton Rodrigo: x:30-210, y:90-130
      if (tx >= 30 && tx <= 210 && ty >= 90 && ty <= 130) {
        usuarioActual = 0;
      }
      // Boton Julieth: x:30-210, y:140-180
      else if (tx >= 30 && tx <= 210 && ty >= 140 && ty <= 180) {
        usuarioActual = 1;
      }
      // Boton Ambos: x:30-210, y:190-230
      else if (tx >= 30 && tx <= 210 && ty >= 190 && ty <= 230) {
        usuarioActual = 2;
      }

      if (usuarioActual >= 0) {
        pantallaUsuario = false;
        pantallaGapaxion = true;
        gapaxionActual = -1;
        Serial.printf("[INFO] Usuario: %s\n", usuarioNombres[usuarioActual]);
        drawGapaxionSelectScreen();
      }
    }
    return;
  }

  // ════════════════════════════════════════════════════════════════════════
  // FASE -0.5: SELECCION DE EJERCICIO GAPAXION
  // ════════════════════════════════════════════════════════════════════════

  if (pantallaGapaxion) {
    int16_t tx, ty;
    if (watch->getTouch(tx, ty) && (now - lastTouchTime > 400)) {
      lastTouchTime = now;

      // Grid de botones G1-G10 (5x2)
      int btnW = 42, btnH = 32, gap = 4;
      int startX = 7;
      int y1 = 68, y2 = 108;

      // Detectar toque en botones G1-G5 (fila 1)
      if (ty >= y1 && ty <= y1 + btnH) {
        for (int i = 0; i < 5; i++) {
          int bx = startX + i * (btnW + gap);
          if (tx >= bx && tx <= bx + btnW) {
            gapaxionActual = i;
            drawGapaxionSelectScreen();
            break;
          }
        }
      }
      // Detectar toque en botones G6-G10 (fila 2)
      else if (ty >= y2 && ty <= y2 + btnH) {
        for (int i = 0; i < 5; i++) {
          int bx = startX + i * (btnW + gap);
          if (tx >= bx && tx <= bx + btnW) {
            gapaxionActual = 5 + i;
            drawGapaxionSelectScreen();
            break;
          }
        }
      }
      // Boton CONTINUAR (y:170, h:36) — solo si hay gapaxion seleccionado
      else if (gapaxionActual >= 0 && ty >= 170 && ty <= 206 && tx >= 40 && tx <= 200) {
        pantallaGapaxion = false;
        pantallaInicio = true;
        Serial.printf("[INFO] Ejercicio: %s\n", gapaxionNombres[gapaxionActual]);
        drawInicioScreen();
      }
      // Boton VOLVER (y:216, h:22)
      else if (ty >= 216 && ty <= 238 && tx >= 70 && tx <= 170) {
        pantallaGapaxion = false;
        pantallaUsuario = true;
        usuarioActual = -1;
        gapaxionActual = -1;
        drawUserSelectScreen();
      }
    }
    return;
  }

  // ════════════════════════════════════════════════════════════════════════
  // FASE 0: PANTALLA DE INICIO (espera pulsar CONECTAR)
  // ════════════════════════════════════════════════════════════════════════

  if (pantallaInicio) {
    int16_t tx, ty;
    if (watch->getTouch(tx, ty) && (now - lastTouchTime > 400)) {
      lastTouchTime = now;

      if (walkingMedActivo) {
        // Touch en pantallas Walking Meditation
        if (walkingMedSeleccion) {
          // Botones nivel Lv.1/2/3 (y=105, h=30)
          if (ty >= 105 && ty <= 135) {
            if (tx >= 5 && tx <= 71)        { walkNivel = 1; drawWalkingSelScreen(); }
            else if (tx >= 76 && tx <= 142) { walkNivel = 2; drawWalkingSelScreen(); }
            else if (tx >= 147 && tx <= 213){ walkNivel = 3; drawWalkingSelScreen(); }
          }
          // Botones AUTO/TAP (y=152, h=26)
          else if (ty >= 152 && ty <= 178) {
            if (tx >= 20 && tx <= 110)       { walkDeteccionMode = 0; drawWalkingSelScreen(); }
            else if (tx >= 130 && tx <= 220) { walkDeteccionMode = 1; drawWalkingSelScreen(); }
          }
          // Boton INICIAR (y=188, h=30)
          else if (ty >= 188 && ty <= 218 && tx >= 30 && tx <= 210) {
            iniciarWalkingExercise();
          }
          // Boton VOLVER (y=224, h=14)
          else if (ty >= 224 && ty <= 238 && tx >= 60 && tx <= 180) {
            finalizarWalkingMed();
          }
        } else if (walkingMedEjercicio) {
          // Touch durante ejercicio manejado en el bloque loop de ejercicio (return temprano)
        } else if (walkingMedResultado) {
          // Boton REPETIR (y=195, h=26, x=20-115)
          if (ty >= 195 && ty <= 221 && tx >= 20 && tx <= 115) {
            walkingMedResultado = false;
            walkingMedSeleccion = true;
            drawWalkingSelScreen();
          }
          // Boton SALIR (y=195, h=26, x=125-220)
          else if (ty >= 195 && ty <= 221 && tx >= 125 && tx <= 220) {
            finalizarWalkingMed();
          }
        }
      } else if (pantallaReloj) {
        // Touch en pantalla de configuración de reloj
        int colW = 38, gapR = 2;
        int startX = (SCREEN_W - 6 * colW - 5 * gapR) / 2;

        // Botones [+] (y=68, h=28)
        if (ty >= 68 && ty <= 96) {
          for (int i = 0; i < 6; i++) {
            int bx = startX + i * (colW + gapR);
            if (tx >= bx && tx <= bx + colW) {
              relojCampoSel = i;
              // Incrementar con limites
              switch (i) {
                case 0: relojCampos[0] = constrain(relojCampos[0] + 1, 2020, 2099); break;
                case 1: relojCampos[1]++; if (relojCampos[1] > 12) relojCampos[1] = 1; break;
                case 2: relojCampos[2]++; if (relojCampos[2] > 31) relojCampos[2] = 1; break;
                case 3: relojCampos[3]++; if (relojCampos[3] > 23) relojCampos[3] = 0; break;
                case 4: relojCampos[4]++; if (relojCampos[4] > 59) relojCampos[4] = 0; break;
                case 5: relojCampos[5]++; if (relojCampos[5] > 59) relojCampos[5] = 0; break;
              }
              drawRelojScreen();
              break;
            }
          }
        }
        // Botones [-] (y=128, h=28)
        else if (ty >= 128 && ty <= 156) {
          for (int i = 0; i < 6; i++) {
            int bx = startX + i * (colW + gapR);
            if (tx >= bx && tx <= bx + colW) {
              relojCampoSel = i;
              // Decrementar con limites
              switch (i) {
                case 0: relojCampos[0] = constrain(relojCampos[0] - 1, 2020, 2099); break;
                case 1: relojCampos[1]--; if (relojCampos[1] < 1) relojCampos[1] = 12; break;
                case 2: relojCampos[2]--; if (relojCampos[2] < 1) relojCampos[2] = 31; break;
                case 3: relojCampos[3]--; if (relojCampos[3] < 0) relojCampos[3] = 23; break;
                case 4: relojCampos[4]--; if (relojCampos[4] < 0) relojCampos[4] = 59; break;
                case 5: relojCampos[5]--; if (relojCampos[5] < 0) relojCampos[5] = 59; break;
              }
              drawRelojScreen();
              break;
            }
          }
        }
        // Boton GUARDAR (y=192, h=28)
        else if (ty >= 192 && ty <= 220 && tx >= 20 && tx <= 220) {
          watch->rtc->setDateTime(
            (uint16_t)relojCampos[0], (uint8_t)relojCampos[1], (uint8_t)relojCampos[2],
            (uint8_t)relojCampos[3],  (uint8_t)relojCampos[4], (uint8_t)relojCampos[5]
          );
          Serial.printf("[RTC] Reloj actualizado: %04d-%02d-%02d %02d:%02d:%02d\n",
                        relojCampos[0], relojCampos[1], relojCampos[2],
                        relojCampos[3], relojCampos[4], relojCampos[5]);
          // Feedback visual
          tft->fillRoundRect(20, 192, 200, 28, 6, TFT_WHITE);
          tft->setTextColor(COLOR_BG, TFT_WHITE);
          tft->setTextDatum(MC_DATUM);
          tft->drawString("GUARDADO!", SCREEN_W / 2, 206, 2);
          delay(800);
          pantallaReloj = false;
          menuActivo = true;
          drawMenuScreen();
        }
        // Boton VOLVER (y=224, h=14)
        else if (ty >= 222 && ty <= 240 && tx >= 70 && tx <= 170) {
          pantallaReloj = false;
          menuActivo = true;
          drawMenuScreen();
        }
      } else if (menuActivo) {
        // Touch en pantalla menú (desde inicio)
        if (ty >= 94 && ty <= 122) {
          if (tx >= 5 && tx <= 71)        { umbralVibracion = (umbralVibracion == 70) ? 0 : 70; drawMenuScreen(); }
          else if (tx >= 76 && tx <= 142) { umbralVibracion = (umbralVibracion == 80) ? 0 : 80; drawMenuScreen(); }
          else if (tx >= 147 && tx <= 213){ umbralVibracion = (umbralVibracion == 90) ? 0 : 90; drawMenuScreen(); }
        } else if (ty >= 136 && ty <= 162) {          // Botón FIN SESION
          finalizarSesion();
        } else if (ty >= 168 && ty <= 192) {          // Botón WALK MED
          walkingMedActivo = true;
          walkingMedSeleccion = true;
          menuActivo = false;
          drawWalkingSelScreen();
        } else if (ty >= 198 && ty <= 222) {          // Botón RELOJ
          menuActivo = false;
          pantallaReloj = true;
          // Cargar valores actuales del RTC
          RTC_Date dt = watch->rtc->getDateTime();
          relojCampos[0] = dt.year;
          relojCampos[1] = dt.month;
          relojCampos[2] = dt.day;
          relojCampos[3] = dt.hour;
          relojCampos[4] = dt.minute;
          relojCampos[5] = dt.second;
          relojCampoSel = 0;
          drawRelojScreen();
        } else if (ty >= 228 && ty <= 240) {          // Botón EXIT
          menuActivo = false;
          drawInicioScreen();
        }
      } else {
        // Boton USER: x:1-59, y:1-27 — vuelve a seleccion de gapaxion
        if (tx >= 1 && tx <= 59 && ty >= 1 && ty <= 27) {
          pantallaInicio = false;
          pantallaGapaxion = true;
          gapaxionActual = -1;
          drawGapaxionSelectScreen();
        }
        // Boton MENU: x:180-238, y:1-27
        else if (tx >= 180 && tx <= 238 && ty >= 1 && ty <= 27) {
          menuActivo = true;
          drawMenuScreen();
        }
        // Boton CONECTAR: x:40-200, y:140-190
        else if (tx >= 40 && tx <= 200 && ty >= 140 && ty <= 190) {
          pantallaInicio = false;
          iniciarBluetooth();
        }
      }
    }
    return;
  }

  // ════════════════════════════════════════════════════════════════════════
  // FASE 1: CONEXIÓN BLUETOOTH
  // ════════════════════════════════════════════════════════════════════════

  if (!btConnected) {

    // Intentar conexión cada 8 segundos
    if (now - connectAttemptTime > 8000 || connectAttempts == 0) {
      connectAttempts++;
      connectAttemptTime = now;

      Serial.printf("\n[BT] === Intento #%d ===\n", connectAttempts);
      Serial.println("[BT] Iniciando discovery + connect SPP...");
      drawConnectScreen("Conectando...", "Discovery SPP...");

      // ══════════════════════════════════════════════════════════════
      // connect(address, channel, sec_mask, role):
      //   - address: MAC del MindWave
      //   - channel 0: auto-discover del canal SPP correcto
      //   - ESP_SPP_SEC_NONE: sin autenticación adicional
      //   - ESP_SPP_ROLE_MASTER: ESP32 como iniciador
      //
      // Internamente hace:
      //   1. esp_bt_gap_start_discovery() - busca el dispositivo
      //   2. esp_spp_start_discovery()    - busca servicio SPP (UUID)
      //   3. esp_spp_connect()            - conecta al canal encontrado
      //
      // El MindWave transmite a 57600 baud sobre SPP/RFCOMM,
      // pero este rate es transparente al nivel de BluetoothSerial:
      // los bytes se reciben directamente sin configurar baud.
      // ══════════════════════════════════════════════════════════════

      bool result = SerialBT.connect(neuroskyMAC, 0, ESP_SPP_SEC_NONE, ESP_SPP_ROLE_MASTER);

      if (result) {
        Serial.println("[BT] connect() retorno TRUE - CONECTADO!");
        btConnected = true;

        // Mostrar confirmación
        tft->fillScreen(COLOR_BG);
        tft->setTextDatum(MC_DATUM);
        tft->setTextColor(TFT_GREEN, COLOR_BG);
        tft->drawString("CONECTADO!", SCREEN_W / 2, 80, 4);
        tft->setTextColor(COLOR_TEXT, COLOR_BG);
        tft->drawString("MindWave vinculado", SCREEN_W / 2, 120, 2);
        tft->setTextColor(COLOR_CARGA, COLOR_BG);
        tft->drawString("Baud: 57600", SCREEN_W / 2, 145, 2);
        tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
        tft->drawString("Esperando datos...", SCREEN_W / 2, 175, 2);
        delay(2000);

        // Reset parser
        parserState    = WAIT_SYNC1;
        lastPacketTime = now;
        dataReceived   = false;
        totalPackets   = 0;
        badChecksums   = 0;

        // Iniciar logging en SD
        iniciarLogSD();

        // Cachear timestamp RTC antes de iniciar Core 0
        actualizarCacheRTC();

        // Iniciar tarea Core 0 para BT+SD
        if (!btTaskHandle) {
          btTaskRunning = true;
          xTaskCreatePinnedToCore(
            btSdTask,        // funcion de la tarea
            "BT_SD",         // nombre
            8192,            // stack (bytes)
            NULL,            // parametro
            1,               // prioridad
            &btTaskHandle,   // handle
            0                // Core 0
          );
          Serial.println("[CORE0] Tarea BT+SD lanzada en Core 0");
        }

      } else {
        Serial.println("[BT] connect() retorno FALSE");
        Serial.println("[BT] Verifica:");
        Serial.println("[BT]   - MindWave encendido?");
        Serial.println("[BT]   - LED azul parpadeando?");
        Serial.println("[BT]   - MAC correcta? (A4:DA:32:6F:EE:BA)");
        btStatus = "Conexion fallida";
        drawConnectScreen("Sin conexion", "Reintento en 8s...");
      }
    }

    // Actualizar animación de puntos
    if (now - lastDisplayUpdate > 300) {
      lastDisplayUpdate = now;
      int dotPhase = (now / 300) % 4;
      for (int d = 0; d < 4; d++) {
        tft->fillCircle(90 + d * 20, 225, 5,
                        (d == dotPhase) ? TFT_CYAN : COLOR_BAR_BG);
      }
    }

    return;
  }

  // ════════════════════════════════════════════════════════════════════════
  // FASE 2: LEER DATOS COMPARTIDOS DESDE CORE 0
  // ════════════════════════════════════════════════════════════════════════

  // btConnected puede ser puesto a false por Core 0 si pierde conexion
  if (!btConnected) {
    btStatus = "Desconectado";
    connectAttemptTime = 0;
    drawConnectScreen("Desconectado", "Reconectando...");
    return;
  }

  // Traer datos frescos parseados por Core 0
  readNeuroData();

  // Actualizar cache RTC cada ~1 segundo (I2C seguro en Core 1)
  if (now - lastRtcCacheUpdate >= 1000) {
    lastRtcCacheUpdate = now;
    actualizarCacheRTC();
  }

  // ════════════════════════════════════════════════════════════════════════
  // FASE 3: TOUCH — MENÚ Y BOTONES
  // ════════════════════════════════════════════════════════════════════════

  int16_t tx, ty;
  if (watch->getTouch(tx, ty) && (now - lastTouchTime > 400)) {
    lastTouchTime = now;

    if (walkingMedActivo && walkingMedSeleccion) {
      // Touch en pantalla selección Walking Meditation (estando conectado)
      if (ty >= 105 && ty <= 135) {
        if (tx >= 5 && tx <= 71)        { walkNivel = 1; drawWalkingSelScreen(); }
        else if (tx >= 76 && tx <= 142) { walkNivel = 2; drawWalkingSelScreen(); }
        else if (tx >= 147 && tx <= 213){ walkNivel = 3; drawWalkingSelScreen(); }
      }
      else if (ty >= 152 && ty <= 178) {
        if (tx >= 20 && tx <= 110)       { walkDeteccionMode = 0; drawWalkingSelScreen(); }
        else if (tx >= 130 && tx <= 220) { walkDeteccionMode = 1; drawWalkingSelScreen(); }
      }
      else if (ty >= 188 && ty <= 218 && tx >= 30 && tx <= 210) {
        iniciarWalkingExercise();
      }
      else if (ty >= 224 && ty <= 238 && tx >= 60 && tx <= 180) {
        finalizarWalkingMed();
        lastDisplayUpdate = 0; // forzar redibujado pantalla principal
      }
    } else if (walkingMedActivo && walkingMedResultado) {
      // Touch en pantalla resultado Walking Meditation (estando conectado)
      if (ty >= 195 && ty <= 221 && tx >= 20 && tx <= 115) {
        walkingMedResultado = false;
        walkingMedSeleccion = true;
        drawWalkingSelScreen();
      }
      else if (ty >= 195 && ty <= 221 && tx >= 125 && tx <= 220) {
        finalizarWalkingMed();
        lastDisplayUpdate = 0; // forzar redibujado pantalla principal
      }
    } else if (menuActivo) {
      // Touch en pantalla menú — botones 70/80/90 en fila horizontal
      if (ty >= 94 && ty <= 122) {
        if (tx >= 5 && tx <= 71)        { umbralVibracion = (umbralVibracion == 70) ? 0 : 70; drawMenuScreen(); }
        else if (tx >= 76 && tx <= 142) { umbralVibracion = (umbralVibracion == 80) ? 0 : 80; drawMenuScreen(); }
        else if (tx >= 147 && tx <= 213){ umbralVibracion = (umbralVibracion == 90) ? 0 : 90; drawMenuScreen(); }
      } else if (ty >= 140 && ty <= 170) {           // Botón FIN SESION
        finalizarSesion();
        return;
      } else if (ty >= 180 && ty <= 206) {           // Botón WALK MED
        walkingMedActivo = true;
        walkingMedSeleccion = true;
        menuActivo = false;
        drawWalkingSelScreen();
      } else if (ty >= 210 && ty <= 236) {           // Botón Exit
        menuActivo = false;
        lastDisplayUpdate = 0; // forzar redibujado
      }
    } else {
      // Touch en pantalla principal o de conexión — botón Menu (x:180-238, y:1-27)
      if (tx >= 180 && tx <= 238 && ty >= 1 && ty <= 27) {
        menuActivo = true;
        drawMenuScreen();
      }
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  // FASE 4: ACTUALIZAR PANTALLA
  // ════════════════════════════════════════════════════════════════════════

  if (!menuActivo && !walkingMedActivo && now - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = now;
    drawMainScreen();

    // Log serial
    Serial.printf("[DATA] SQ:%3d ATT:%3d MED:%3d BLK:%3d IFN:%.2f RC:%.2f Pkts:%lu Bad:%lu\n",
                  poorSignal, attention, meditation, blinkStrength,
                  indiceFatiga, ratioCarga, totalPackets, badChecksums);

    if (eegDelta > 0) {
      Serial.printf("[EEG] d:%lu t:%lu lA:%lu hA:%lu lB:%lu hB:%lu lG:%lu mG:%lu\n",
                    eegDelta, eegTheta, eegLowAlpha, eegHighAlpha,
                    eegLowBeta, eegHighBeta, eegLowGamma, eegMidGamma);
    }

    // SD Card: escritura movida a Core 0 (btSdTask)
  }

  // ════════════════════════════════════════════════════════════════════════
  // FASE 5: VIBRACIÓN POR MEDITACIÓN
  // ════════════════════════════════════════════════════════════════════════

  if (umbralVibracion > 0 && meditation >= umbralVibracion) {
    if (meditation >= 100) {
      // Meditación = 100: vibración intermitente 300ms on/off durante 3 segundos
      if (!vibActiva) {
        vibActiva = true;
        vibStartTime = now;
        vibLastToggle = now;
        vibEstado = true;
        drvPlayEffect(15);  // 750ms Alert 100%
      }
    } else {
      // Meditación >= umbral pero < 100: vibración continua
      if (!motorOn) {
        motorOn = true;
        drvPlayEffect(52);  // Pulsing Strong 1 100%
      }
      // Re-disparar efecto cada 800ms para mantener vibración continua
      if (motorOn && (now - vibLastToggle >= 800)) {
        vibLastToggle = now;
        drvPlayEffect(52);
      }
    }
  } else {
    // Meditación bajo umbral: apagar motor (si no hay intermitente activa)
    if (!vibActiva && motorOn) {
      motorOn = false;
      drvStop();
    }
  }

  // Manejar vibración intermitente med=100 (sin delay)
  if (vibActiva) {
    if (now - vibStartTime >= 3000) {
      // Pasaron 3 segundos: terminar
      vibActiva = false;
      vibEstado = false;
      drvStop();
    } else if (now - vibLastToggle >= 300) {
      // Toggle cada 300ms
      vibLastToggle = now;
      vibEstado = !vibEstado;
      if (vibEstado) {
        drvPlayEffect(15);  // 750ms Alert 100%
      } else {
        drvStop();
      }
    }
  }

  // ════════════════════════════════════════════════════════════════════════
  // FASE 6: TIMEOUT
  // ════════════════════════════════════════════════════════════════════════

  if (lastPacketTime > 0 && (now - lastPacketTime > 10000)) {
    Serial.println("[WARN] Sin paquetes validos por 10s");
    tft->setTextDatum(MC_DATUM);
    tft->setTextColor(TFT_RED, COLOR_BG);
    tft->drawString("SIN DATOS 10s", SCREEN_W / 2, SCREEN_H - 15, 1);
    lastPacketTime = now;
  }
}
