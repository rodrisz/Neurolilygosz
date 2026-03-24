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
//hola prueba git
#include <LilyGoWatch.h>
#include "BluetoothSerial.h"

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

// ─── ESTADO CONEXIÓN ───────────────────────────────────────────────────────
bool          btConnected       = false;
bool          dataReceived      = false;
unsigned long lastPacketTime    = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long connectAttemptTime= 0;
int           connectAttempts   = 0;
uint32_t      totalPackets      = 0;
uint32_t      badChecksums      = 0;

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
        } else {
          badChecksums++;
        }
        parserState = WAIT_SYNC1;
      }
      break;
  }
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
  int   batPct    = power->getBattPercentage();
  float batVolt   = power->getBattVoltage();
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
  tft->drawString(batBuf, SCREEN_W / 2, 44, 4);

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
  tft->drawString(voltBuf, SCREEN_W / 2, 70, 1);

  // Línea separadora
  tft->drawFastHLine(10, 82, SCREEN_W - 20, COLOR_TEXT_DIM);

  // ── Umbral activo ────────────────────────────────────────
  char umbBuf[32];
  if (umbralVibracion == 0) snprintf(umbBuf, sizeof(umbBuf), "Vibra: OFF");
  else snprintf(umbBuf, sizeof(umbBuf), "Vibra: >= %d", umbralVibracion);
  tft->setTextColor(COLOR_TEXT_DIM, COLOR_BG);
  tft->drawString(umbBuf, SCREEN_W / 2, 93, 2);

  // ── Botones umbral (h=32, compactos) ────────────────────
  // Botón 70 (y=108)
  uint16_t c70 = (umbralVibracion == 70) ? TFT_GREEN : COLOR_BAR_BG;
  tft->fillRoundRect(20, 108, 200, 32, 6, c70);
  tft->drawRoundRect(20, 108, 200, 32, 6, COLOR_TEXT_DIM);
  tft->setTextColor(COLOR_TEXT, c70);
  tft->drawString("70", SCREEN_W / 2, 124, 4);

  // Botón 80 (y=146)
  uint16_t c80 = (umbralVibracion == 80) ? TFT_GREEN : COLOR_BAR_BG;
  tft->fillRoundRect(20, 146, 200, 32, 6, c80);
  tft->drawRoundRect(20, 146, 200, 32, 6, COLOR_TEXT_DIM);
  tft->setTextColor(COLOR_TEXT, c80);
  tft->drawString("80", SCREEN_W / 2, 162, 4);

  // Botón 90 (y=184)
  uint16_t c90 = (umbralVibracion == 90) ? TFT_GREEN : COLOR_BAR_BG;
  tft->fillRoundRect(20, 184, 200, 32, 6, c90);
  tft->drawRoundRect(20, 184, 200, 32, 6, COLOR_TEXT_DIM);
  tft->setTextColor(COLOR_TEXT, c90);
  tft->drawString("90", SCREEN_W / 2, 200, 4);

  // Botón EXIT (y=222)
  tft->fillRoundRect(20, 222, 200, 15, 4, TFT_RED);
  tft->setTextColor(COLOR_TEXT, TFT_RED);
  tft->drawString("EXIT", SCREEN_W / 2, 229, 1);
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
  Serial.println("  Bluetooth SPP @ 57600 baud");
  Serial.println("══════════════════════════════════════");

  // ── Inicializar T-Watch ──
  watch = TTGOClass::getWatch();
  watch->begin();
  watch->openBL();

  tft   = watch->tft;
  power = watch->power;

  tft->setRotation(2);
  tft->fillScreen(COLOR_BG);
  tft->setSwapBytes(true);
  watch->setBrightness(180);

  // ── Inicializar motor vibración (DRV2605L via I2C) ──
  watch->enableDrv2650();       // AXP202 GPIO0 alimenta el DRV2605L
  drv = watch->drv;
  drv->selectLibrary(1);
  drv->setMode(DRV2605_MODE_INTTRIG);

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
    while (1) { delay(1000); }
  }

  Serial.println("[BT] Bluetooth iniciado como MAESTRO SPP");
  Serial.println("[BT] Nombre: NeuroCrawler");
  Serial.printf("[BT] Target: A4:DA:32:6F:EE:BA @ %d baud\n", MINDWAVE_BAUDRATE);

  btStatus = "BT Maestro OK";
  drawConnectScreen("BT Listo", "Buscando MindWave...");

  // ══════════════════════════════════════════════════════════════════════
  // INSTRUCCIONES PARA EL USUARIO:
  // ══════════════════════════════════════════════════════════════════════
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
  // FASE 2: LECTURA Y PARSEO THINKGEAR
  // ════════════════════════════════════════════════════════════════════════

  // Verificar conexión activa
  if (!SerialBT.connected()) {
    Serial.println("[BT] Conexion perdida! Reconectando...");
    btConnected = false;
    btStatus = "Desconectado";
    connectAttemptTime = 0;
    drawConnectScreen("Desconectado", "Reconectando...");
    return;
  }

  // Leer TODOS los bytes disponibles del stream BT
  // El MindWave envía ~513 paquetes/segundo a 57600 baud
  // (512 paquetes raw de 7 bytes + 1 paquete grande ~1/seg)
  int bytesAvailable = SerialBT.available();
  while (bytesAvailable > 0) {
    uint8_t b = SerialBT.read();
    parseByte(b);
    bytesAvailable--;
  }

  // ════════════════════════════════════════════════════════════════════════
  // FASE 3: TOUCH — MENÚ Y BOTONES
  // ════════════════════════════════════════════════════════════════════════

  int16_t tx, ty;
  if (watch->getTouch(tx, ty) && (now - lastTouchTime > 400)) {
    lastTouchTime = now;

    if (menuActivo) {
      // Touch en pantalla menú
      if (ty >= 108 && ty <= 140) {                  // Botón 70
        umbralVibracion = (umbralVibracion == 70) ? 0 : 70;
        drawMenuScreen();
      } else if (ty >= 146 && ty <= 178) {           // Botón 80
        umbralVibracion = (umbralVibracion == 80) ? 0 : 80;
        drawMenuScreen();
      } else if (ty >= 184 && ty <= 216) {           // Botón 90
        umbralVibracion = (umbralVibracion == 90) ? 0 : 90;
        drawMenuScreen();
      } else if (ty >= 222 && ty <= 237) {           // Botón Exit
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

  if (!menuActivo && now - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
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
