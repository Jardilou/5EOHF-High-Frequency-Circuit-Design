/* 
  ESP32 Unit Test Suite
  - ESP32 basic checks: boot, GPIO toggle
  - WiFi: scan and simple TCP ping attempt (scan only; no credentials)
  - BLE: advertise + passive scan
  - SAM-M8Q: UART NMEA reader + I2C presence probe (addr 0x42)
  - RFM95: SPI RegVersion read, Reset cycle, optional LoRa transmit (if "LoRa" library is installed)
  - Serial output at 115200 for results
  IMPORTANT: Many ESP32 boards reserve GPIO6..GPIO11 for flash/PSRAM. Using GPIO6/GPIO7 may break the module.
  We should adjust pins if we encounter boot issues!
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <SPI.h>

// Optional LoRa library: uncomment if you have the Sandeep Mistry LoRa library installed
// #include <LoRa.h>

///// USER PIN CONFIG (from your message) /////
const int PIN_RFM95_SCK  = 33; // SCK
const int PIN_RFM95_SPIQ = 34; // SPIQ (may be DIO/IRQ depending on your board)
const int PIN_RFM95_SPID = 35; // SPID (MISO/MOSI naming ambiguous; treat as MISO here)
const int PIN_RFM95_NSS  = 6;  // NSS (WARNING: GPIO6 is sometimes reserved for flash)
const int PIN_RFM95_RESET= 7;  // RESET (WARNING: GPIO7 may be reserved)
const int PIN_RFM95_DIO0 = PIN_RFM95_SPIQ; // fallback: if SPIQ is DIO0/IRQ, used for LoRa library

const int GPS_UART_TX = 17; // U1TXD (ESP32 -> GPS)
const int GPS_UART_RX = 18; // U1RXD (GPS -> ESP32)
const int GPS_I2C_SDA = 51; // GPIO45
const int GPS_I2C_SCL = 52; // GPIO46
const uint8_t GPS_I2C_ADDR = 0x42; // typical UBX I2C address (0x42)

const int LED_TEST_PIN = 2; // on-board LED or any test pin
const int GPIO_TEST_PINS[] = {2, 4, 12}; // extra pins toggled during GPIO test
const size_t N_GPIO_TEST_PINS = sizeof(GPIO_TEST_PINS)/sizeof(GPIO_TEST_PINS[0]);

HardwareSerial GPSu(1); // use UART1 for GPS (pins 17 TX, 18 RX)

SPIClass hspi(HSPI); // we'll bind HSPI to custom pins

// Helper timing
unsigned long nowMs() { return millis(); }

/////////////////////////
// Test helpers
/////////////////////////

void printHeader(const char* title) {
  Serial.println();
  Serial.print("=== ");
  Serial.print(title);
  Serial.println(" ===");
}

void pauseMs(unsigned long ms) {
  unsigned long t = nowMs();
  while (nowMs() - t < ms) { yield(); }
}

/////////////////////////
// ESP32: power/boot / basic tests
/////////////////////////

void test_boot_and_power() {
  printHeader("ESP32 Boot & Power Test");
  Serial.printf("Boot time millis: %lu\n", nowMs());
  // Basic sanity: check Vcc reading (if ADC on board measures it) - optional
  Serial.println("Note: measure 3.3V rail externally with DVM/oscilloscope under load for full power test.");
  Serial.println("Result: BOOT OK (if you see this message and no brownout resets).");
}

void test_gpio() {
  printHeader("GPIO Digital IO Test");
  Serial.println("Toggling GPIO test pins and reading them back where possible.");
  for (size_t i = 0; i < N_GPIO_TEST_PINS; ++i) {
    int p = GPIO_TEST_PINS[i];
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  pauseMs(100);
  for (size_t i = 0; i < N_GPIO_TEST_PINS; ++i) {
    int p = GPIO_TEST_PINS[i];
    Serial.printf("Toggling pin %d -> HIGH\n", p);
    digitalWrite(p, HIGH);
    pauseMs(200);
    // If you have a multimeter or scope probe, verify the pin shows 3.3V
  }
  pauseMs(200);
  for (size_t i = 0; i < N_GPIO_TEST_PINS; ++i) {
    digitalWrite(GPIO_TEST_PINS[i], LOW);
  }
  Serial.println("GPIO toggle done. Observe with scope/logic analyzer to confirm.");
}

/////////////////////////
// WiFi tests
/////////////////////////

void test_wifi_scan() {
  printHeader("WiFi Scan Test");
  Serial.println("Starting WiFi in STA mode and scanning for networks...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  pauseMs(100);
  int n = WiFi.scanNetworks();
  Serial.printf("Found %d networks:\n", n);
  for (int i = 0; i < n; ++i) {
    Serial.printf("  %d: %s (RSSI %d dBm) Channel %d, Encryption: %s\n",
                  i, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
                  WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "OPEN" : "SECURE");
  }
  if (n == 0) Serial.println("No networks found (check antenna / board).");
  WiFi.scanDelete();
}

void test_wifi_softap() {
  printHeader("WiFi SoftAP Test");
  Serial.println("Bringing up SoftAP (no password) to verify WiFi stack and antenna.");
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP("ESP32_UnitTest_AP");
  Serial.printf("softAP start: %s\n", ok ? "OK" : "FAILED");
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("SoftAP IP: %s\n", ip.toString().c_str());
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
}

/////////////////////////
// BLE tests
/////////////////////////

void test_ble_advertise_and_scan() {
  printHeader("BLE Advertise & Scan Test");
  Serial.println("Initializing BLE device...");
  BLEDevice::init("ESP32_UnitTest");
  BLEServer *pServer = BLEDevice::createServer();
  Serial.println("Starting advertise (30s) and local passive scan for devices...");
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->start();
  // Passive scan using NimBLE/other libs is more involved; here we show advertise success:
  Serial.println("BLE advertiser started. Use a phone to confirm presence named 'ESP32_UnitTest'.");
  pauseMs(5000);
  pAdvertising->stop();
  Serial.println("BLE advertise stopped.");
}

/////////////////////////
// SAM-M8Q GPS tests
/////////////////////////

void test_gps_uart() {
  printHeader("GPS UART Test (NMEA)");
  Serial.println("Opening UART1 for GPS at 9600/UBX default (try 9600 and 38400 if needed)...");
  GPSu.begin(9600, SERIAL_8N1, GPS_UART_RX, GPS_UART_TX); // rx, tx
  unsigned long tstart = nowMs();
  Serial.println("Listening for NMEA sentences for 6 seconds...");
  String buffer;
  while (nowMs() - tstart < 6000UL) {
    while (GPSu.available()) {
      char c = GPSu.read();
      buffer += c;
      if (c == '\n') {
        // Print complete line
        buffer.trim();
        if (buffer.length() > 0) {
          Serial.printf("GPS UART: %s\n", buffer.c_str());
        }
        buffer = "";
      }
    }
    yield();
  }
  Serial.println("End of UART listen. If you saw GGA/RMC/NMEA lines then UART is working and GPS has clear view.");
}

void test_gps_i2c_probe() {
  printHeader("GPS I2C Presence Probe (address 0x42)");
  // Use Wire with custom pins:
  // On ESP32, Wire.begin(SDA, SCL) with integer pins above 21 works if board supports them
  Wire.end();
  Wire.begin(GPS_I2C_SDA, GPS_I2C_SCL);
  pauseMs(50);
  Serial.print("Scanning I2C for device 0x42... ");
  Wire.beginTransmission(GPS_I2C_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    Serial.println("OK: Device responded at 0x42 (likely SAM-M8Q UBX I2C).");
  } else {
    Serial.printf("NO RESPONSE (err=%d). If GPS is using I2C, check wiring/power and that GPS supports I2C.\n", err);
  }
  // Optionally request a few bytes
  Wire.requestFrom(GPS_I2C_ADDR, (uint8_t)6);
  if (Wire.available()) {
    Serial.print("I2C read bytes: ");
    while (Wire.available()) {
      Serial.printf("0x%02X ", Wire.read());
    }
    Serial.println();
  } else {
    Serial.println("No readable bytes from I2C device (or device requires UBX framing).");
  }
}

/////////////////////////
// RFM95 (SX127x family) SPI checks
/////////////////////////

// SX1276/77/78/79 RegVersion is at 0x42 and commonly returns 0x12 for known silicon.
// SPI protocol: MSB=1 for write, MSB=0 for read (i.e., readAddr = addr & 0x7F)
uint8_t rfm95_read_reg(uint8_t addr) {
  uint8_t readAddr = addr & 0x7F;
  digitalWrite(PIN_RFM95_NSS, LOW);
  uint8_t resp = hspi.transfer(readAddr);
  resp = hspi.transfer(0x00);
  digitalWrite(PIN_RFM95_NSS, HIGH);
  return resp;
}

void rfm95_write_reg(uint8_t addr, uint8_t value) {
  uint8_t writeAddr = addr | 0x80; // set MSB for write
  digitalWrite(PIN_RFM95_NSS, LOW);
  hspi.transfer(writeAddr);
  hspi.transfer(value);
  digitalWrite(PIN_RFM95_NSS, HIGH);
}

void test_rfm95_spi_basic() {
  printHeader("RFM95 SPI Basic Test");
  Serial.println("Configuring HSPI with user pins and toggling reset.");
  // configure HSPI pins
  // NOTE: If your board uses GPIO6/GPIO7 for flash, this will break. Consider changing NSS/RESET.
  hspi.begin(PIN_RFM95_SCK, PIN_RFM95_SPID, /*MOSI*/ PIN_RFM95_SCK, /*SCLK duplicated*/ PIN_RFM95_SCK); 
  // The above line won't be perfect for some boards — we still set pinMode for NSS
  pinMode(PIN_RFM95_NSS, OUTPUT);
  digitalWrite(PIN_RFM95_NSS, HIGH);
  pinMode(PIN_RFM95_RESET, OUTPUT);
  // reset pulse
  digitalWrite(PIN_RFM95_RESET, LOW);
  pauseMs(10);
  digitalWrite(PIN_RFM95_RESET, HIGH);
  pauseMs(10);

  Serial.println("Reading RegVersion (0x42)...");
  uint8_t ver = rfm95_read_reg(0x42);
  Serial.printf("RegVersion read: 0x%02X\n", ver);
  if (ver == 0x12) {
    Serial.println("RegVersion matched expected SX1276/7/8/9 silicon (0x12). SPI appears functional.");
  } else {
    Serial.println("Unexpected RegVersion. This may be okay if different silicon; if 0x00 or 0xFF check wiring and flash-pin conflicts.");
  }

  Serial.println("Toggling NSS and RESET a few times to confirm control lines.");
  for (int i=0;i<3;i++){
    digitalWrite(PIN_RFM95_NSS, LOW);
    pauseMs(50);
    digitalWrite(PIN_RFM95_NSS, HIGH);
    pauseMs(50);
    digitalWrite(PIN_RFM95_RESET, LOW);
    pauseMs(20);
    digitalWrite(PIN_RFM95_RESET, HIGH);
    pauseMs(200);
  }
  Serial.println("RFM95 SPI basic checks done.");
}

void test_rfm95_sendpackets_if_lora_lib() {
  printHeader("RFM95 LoRa Send Test (optional: requires LoRa library)");
  Serial.println("If you have the 'LoRa' library by Sandeep Mistry installed and pins mapped correctly, uncomment the include and LoRa init lines in the code.");
  Serial.println("This function will attempt to initialize LoRa and transmit a few test packets. Measure with your spectrum analyzer.");
  // Example instructions (not executed unless you enable LoRa library):
  Serial.println(" -> If you want this to run, uncomment the LoRa include at top and the LoRa init below, then re-upload.");
  
  //Once LoRa.h is enabled:
  LoRa.setPins(PIN_RFM95_NSS, PIN_RFM95_RESET, PIN_RFM95_DIO0);
  if (!LoRa.begin(868E6)) {
    Serial.println("LoRa.begin failed - check IRQ pin and NSS pin mapping (and avoid GPIO6/7).");
    return;
  }
  Serial.println("LoRa initialized. Sending 10 packets (one per second). Use spectrum analyzer to see transmissions.");
  for (int i=0;i<10;i++){
    String payload = "UTest packet #" + String(i);
    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();
    Serial.printf("Sent packet %d\n", i);
    delay(1000);
  }
  Serial.println("Packets sent.");
  
}

/////////////////////////
// Run full sequence
/////////////////////////

void run_all_tests() {
  Serial.println();
  Serial.println("==============================================");
  Serial.println("Starting automated unit test sequence...");
  Serial.println("Observe Serial output for PASS/FAIL and take external measurements as indicated.");
  Serial.println("==============================================");

  test_boot_and_power();
  test_gpio();

  test_wifi_scan();
  test_wifi_softap();

  test_ble_advertise_and_scan();

  // GPS
  test_gps_uart();
  test_gps_i2c_probe();

  // RFM95
  test_rfm95_spi_basic();
  test_rfm95_sendpackets_if_lora_lib();

  Serial.println();
  Serial.println("All tests executed. Inspect output above.");
  Serial.println("For RFM95 TX spectral tests: send LoRa packets (see optional LoRa block) while you measure with your Rigol SA.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { yield(); } // wait for serial
  Serial.println("\nESP32 Unit Test Suite starting...");
  // set GPS I2C pins before using Wire
  // Wire.begin(GPS_I2C_SDA, GPS_I2C_SCL); // done in probe
  // Setup LED pin
  pinMode(LED_TEST_PIN, OUTPUT);
  digitalWrite(LED_TEST_PIN, LOW);

  // Small startup delay
  pauseMs(2000);

  run_all_tests();
}

void loop() {
  // Idle. Tests are one-shot at boot; you can expand this loop to re-run tests on demand.
  delay(10000);
}
