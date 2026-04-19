#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
HardwareSerial SerialGPS(USART2);

#define LED_PIN    PA0

// LoRa pins
#define LORA_CS    PA4
#define LORA_MISO  PA6
#define LORA_MOSI  PA7
#define LORA_SCK   PA5
#define LORA_RST   PA1
#define LORA_DIO0  PA12

int state = 0; 

// 0 = booting
// 1 = waiting for GPS lock
// 2 = flight mode, low power mode
// 3 = descent mode

#pragma pack(push, 1)
struct Telemetry {
  uint8_t satellites;     // 0–255 is plenty
  int32_t lat_e7;         // latitude * 1e7
  int32_t lng_e7;         // longitude * 1e7
  int32_t altitude;       // meters
  uint32_t frame;         // frame counter
  int16_t speed_knots;    // knots
};
#pragma pack(pop)

Telemetry telemetry;

void setup() {
  pinMode(LED_PIN, OUTPUT);

  // LED ON = booting
  digitalWrite(LED_PIN, HIGH);

  initLoRa(); // Initialize LoRa
  initGPS(); // Init GPS

  state = 1; // waiting for GPS lock
}

void loop() {
  GPSCallback();
  
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  }
}

void GPSCallback(){
  while (SerialGPS.available()) {
    char c = SerialGPS.read();
    gps.encode(c);
  }
  telemetry.frame++; // Begin a new frame

  telemetry.satellites = gps.satellites.value();
  telemetry.lat_e7 = (int32_t)(gps.location.lat() * 1e7);
  telemetry.lng_e7 = (int32_t)(gps.location.lng() * 1e7);
  telemetry.altitude = gps.altitude.meters();
  telemetry.speed_knots = gps.speed.knots();

  if (state == 1 && gps.satellites.value() > 4){
    state = 2;
  }

}

void initGPS(){
  SerialGPS.begin(9600);
  delay(100);
  Serial.print("$PCAS11,5*18\r\n");
  delay(100);
  Serial.print("$PCAS11,5*18\r\n");
  delay(100);
  Serial.print("$PCAS11,5*18\r\n");
  delay(100);
}

void initLoRa(){
  // Configure SPI for LoRa
  SPI.setMOSI(LORA_MOSI);
  SPI.setMISO(LORA_MISO);
  SPI.setSCLK(LORA_SCK);
  SPI.begin();

  // Configure LoRa pins
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  delay(500);

  // Initialize LoRa
  if (!LoRa.begin(434E6)) {
    // FAIL = solid OFF
    digitalWrite(LED_PIN, LOW);
    while (1);
  }

  // SUCCESS = blink once
  digitalWrite(LED_PIN, LOW);
  delay(200);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  LoRa.setTxPower(0);
}
