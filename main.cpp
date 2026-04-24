/*
 * ============================================================
 *  PAPER AIRPLANE PAYLOAD — STM32G030 + RFM98 + TinyGPS++
 *  Packet ID = 0  (PKT_AIRPLANE)
 * ============================================================
 *
 *  State machine:
 *    0 = Booting
 *    1 = Waiting for GPS lock
 *    2 = Dormant / ascending  (high power, slow TX ~10 s)
 *    3 = Flight mode          (high power, fast TX ~500 ms)
 *
 *  Flight mode triggered by:
 *    GPS altitude >= FLIGHT_ALT_M only
 *
 *  LED states (PA0):
 *    0 Booting          --> Solid ON
 *    1 GPS lock wait    --> Fast blink 100ms on / 100ms off
 *    2 Dormant/ascent   --> Single blip: 50ms on, 1950ms off
 *    3 Flight mode      --> Double blip: 50ms on, 100ms off, 50ms on, 800ms off
 *    Fatal LoRa error   --> Solid OFF forever
 *
 *  Battery:
 *    10k/10k voltage divider on PB7
 *    Vbat(mV) = ADC_raw * 6600 / 4095
 * ============================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>

// ── LoRa RF parameters ──────────────────────────────────────
#define LORA_FREQ       434200000
#define SF              10
#define BW              125       // kHz
#define CR              8
#define TX_POWER        20        // dBm — always high power

// ── Timing ──────────────────────────────────────────────────
#define LOOP_DELAY_MS   100       // base loop tick (ms)
#define CYCLES_FAST     1         // TX every 5 ticks  --> ~500 ms (flight mode)
#define CYCLES_SLOW     50       // TX every 100 ticks --> ~10 s  (dormant mode)

// ── Altitude arm threshold ──────────────────────────────────
#define FLIGHT_ALT_M    10L    // metres — triggers flight mode

// ── Battery ADC ─────────────────────────────────────────────
//  10k/10k divider on PB7 --> ADC sees Vbat/2
//  STM32G030: 12-bit ADC, Vref = 3.3 V
//  Vbat(mV) = (raw / 4095) * 3300 * 2
#define BAT_PIN         PB7
#define BAT_SCALE_NUM   6583UL 
#define BAT_SCALE_DEN   4095UL

// ── Pin definitions ─────────────────────────────────────────
#define LED_PIN         PA0
#define LORA_CS         PA4
#define LORA_MISO       PA6
#define LORA_MOSI       PA7
#define LORA_SCK        PA5
#define LORA_RST        PA1
#define LORA_DIO0       PA12

// ── Packet ID ────────────────────────────────────────────────
#define PKT_AIRPLANE    0

// ── LED timing constants (ms) ────────────────────────────────
#define LED_FAST_PERIOD     200
#define LED_DORMANT_ON      50
#define LED_DORMANT_PERIOD  2000
#define LED_FLIGHT_ON       50
#define LED_FLIGHT_GAP      100
#define LED_FLIGHT_PERIOD   1000

// ─────────────────────────────────────────────────────────────
//  Shared telemetry struct
//  MUST be byte-identical on every node in the system
// ─────────────────────────────────────────────────────────────
#pragma pack(push, 1)
struct Telemetry {
  uint8_t  id;          // PKT_* constant
  uint8_t  state;       // sender's state machine value
  uint8_t  satellites;
  int32_t  lat_e7;      // latitude  × 1e7
  int32_t  lng_e7;      // longitude × 1e7
  int32_t  altitude;    // metres MSL
  uint32_t frame;       // rolling TX counter
  int16_t  speed_knots;
  uint16_t voltage_mv;  // battery voltage in mV
};
#pragma pack(pop)

// ─────────────────────────────────────────────────────────────
//  Globals
// ─────────────────────────────────────────────────────────────
TinyGPSPlus    gps;
HardwareSerial SerialGPS(USART2);

Telemetry telemetry;

uint8_t  nodeState = 0;
uint32_t cycle     = 0;

// ─────────────────────────────────────────────────────────────
//  Forward declarations
// ─────────────────────────────────────────────────────────────
void     initGPS();
void     initLoRa();
void     feedGPS();
void     buildTelemetry();
void     sendTelemetry();
void     manageState();
void     updateLED();
uint16_t readBatteryMV();

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  // LED first — immediate visual feedback
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);    // State 0: solid ON = booting
  delay(500);

  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);

  pinMode(BAT_PIN, INPUT_ANALOG);

  memset(&telemetry, 0, sizeof(telemetry));
  telemetry.id = PKT_AIRPLANE;

  initGPS();
  initLoRa();

  nodeState = 1;  // Booting done --> waiting for GPS lock
}

// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  cycle++;

  feedGPS();
  buildTelemetry();
  manageState();
  updateLED();

  bool doTX = false;
  if (nodeState == 3) {
    doTX = (cycle % CYCLES_FAST == 0);
  } else if (nodeState >= 2) {
    doTX = (cycle % CYCLES_SLOW == 0);
  }

  if (doTX) sendTelemetry();

  delay(LOOP_DELAY_MS);
}

// ─────────────────────────────────────────────────────────────
//  STATE MACHINE
// ─────────────────────────────────────────────────────────────
void manageState() {
  switch (nodeState) {

    case 1:   // Waiting for GPS lock
      if (gps.satellites.value() >= 4 && gps.location.isValid()) {
        nodeState = 2;
      }
      break;

    case 2:   // Dormant / ascending — watch altitude
      if (gps.altitude.isValid() && gps.altitude.meters() >= FLIGHT_ALT_M) {
        nodeState = 3;
      }
      break;

    case 3:   // Flight mode — latched forever, no exit
      break;
  }
}

// ─────────────────────────────────────────────────────────────
//  LED — non-blocking, millis() driven, unique per state
// ─────────────────────────────────────────────────────────────
void updateLED() {
  uint32_t t = millis();

  switch (nodeState) {

    case 0:   // Booting — solid ON
      digitalWrite(LED_PIN, HIGH);
      break;

    case 1: { // GPS wait — fast blink 100ms on / 100ms off
      bool on = ((t / (LED_FAST_PERIOD / 2)) % 2 == 0);
      digitalWrite(LED_PIN, on ? HIGH : LOW);
      break;
    }

    case 2: { // Dormant — single blip: 50ms ON, 1950ms OFF
      uint32_t phase = t % LED_DORMANT_PERIOD;
      digitalWrite(LED_PIN, phase < LED_DORMANT_ON ? HIGH : LOW);
      break;
    }

    case 3: { // Flight — double blip within 1s cycle:
              // 0–50ms ON, 50–150ms OFF, 150–200ms ON, 200–1000ms OFF
      uint32_t phase = t % LED_FLIGHT_PERIOD;
      bool on = (phase < LED_FLIGHT_ON) ||
                (phase >= (uint32_t)(LED_FLIGHT_ON + LED_FLIGHT_GAP) &&
                 phase <  (uint32_t)(LED_FLIGHT_ON + LED_FLIGHT_GAP + LED_FLIGHT_ON));
      digitalWrite(LED_PIN, on ? HIGH : LOW);
      break;
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  GPS
// ─────────────────────────────────────────────────────────────
void initGPS() {
  SerialGPS.begin(9600);
  delay(200);
  // Switch CASIC GPS to high altitude mode
  SerialGPS.print("$PCAS11,5*18\r\n");
  delay(100);
  SerialGPS.print("$PCAS11,5*18\r\n");
  delay(100);
  SerialGPS.print("$PCAS11,5*18\r\n");
  delay(100);
}

void feedGPS() {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }
}

void buildTelemetry() {
  telemetry.frame++;
  telemetry.id          = PKT_AIRPLANE;
  telemetry.state       = nodeState;
  telemetry.satellites  = (uint8_t)gps.satellites.value();
  telemetry.lat_e7      = (int32_t)(gps.location.lat() * 1e7);
  telemetry.lng_e7      = (int32_t)(gps.location.lng() * 1e7);
  telemetry.altitude    = (int32_t)gps.altitude.meters();
  telemetry.speed_knots = (int16_t)gps.speed.knots();
  telemetry.voltage_mv  = readBatteryMV();
}

// ─────────────────────────────────────────────────────────────
//  LORA
// ─────────────────────────────────────────────────────────────
void initLoRa() {
  SPI.setMOSI(LORA_MOSI);
  SPI.setMISO(LORA_MISO);
  SPI.setSCLK(LORA_SCK);
  SPI.begin();

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    digitalWrite(LED_PIN, LOW);   // Fatal — solid OFF forever
    while (1);
  }

  LoRa.setTxPower(TX_POWER);
  //LoRa.setGain(6);
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BW * 1000UL);
  LoRa.setCodingRate4(CR);
  LoRa.enableCrc();

  // No receive needed — this node is TX only now
}

void sendTelemetry() {
  // Hard-reset radio before every TX for reliability
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  initLoRa();

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&telemetry, sizeof(telemetry));
  LoRa.endPacket();   // Blocking — returns when TX complete
}

// ─────────────────────────────────────────────────────────────
//  BATTERY
// ─────────────────────────────────────────────────────────────
uint16_t readBatteryMV() {
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(BAT_PIN);
    delay(1);
  }
  uint32_t raw = sum / 8;
  return (uint16_t)((raw * BAT_SCALE_NUM) / BAT_SCALE_DEN);
}
