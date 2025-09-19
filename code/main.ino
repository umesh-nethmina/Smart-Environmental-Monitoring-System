#include <Wire.h>
#include <Adafruit_AHTX0.h>          // Updated AHT sensor library (supports AHT10 and AHT20)
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SPIMemory.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <math.h>

// === Sensor and Communication Setup ===
Adafruit_AHTX0 aht;                  // Changed from Adafruit_AHT10
Adafruit_BMP280 bmp;
RTC_DS3231 rtc;
SPIFlash flash(5);                  // FLASH_CS = 5
SoftwareSerial sim900(16, 17);     // SIM900A TX/RX
HardwareSerial pmsSerial(1);        // PMS5003
HardwareSerial co2Serial(2);        // MH-Z19C

// === Constants ===
#define PMS_RX_PIN 26
#define PMS_TX_PIN 27
#define MHZ_RX_PIN 14
#define MHZ_TX_PIN 13
#define CO_SENSOR_PIN 34

const float Vref = 3.3;
const float RL = 10.0;
const float R0 = 250.0;

const char* APN = "dialogbb";
const char* THINGSPEAK_API_KEY = "ET58DCXGGCXFSEH2";

// === Timing ===
const unsigned long SENSOR_INTERVAL = 5 * 60 * 1000UL;
const unsigned long RESEND_INTERVAL = 20000;
unsigned long lastSensorMillis = 0;
unsigned long lastResendMillis = 0;

// === Flags and Flash Addresses ===
bool gsmConnected = false;
bool resendInProgress = false;
uint32_t flashAddress = 0;
uint32_t resendAddr = 0;

// === GSM Disconnect Tracking ===
bool wasDisconnected = false;
uint32_t disconnectStart = 0;

// === PM Sensor Values ===
uint16_t pm2_5 = 0, pm10 = 0;

// === Data Structure ===
struct SensorData {
  float temp, hum, pressure, co_ppm;
  int co2_ppm;
  uint16_t pm25, pm10;
  uint32_t timestamp;
};

// === Function Prototypes ===
void connectGPRS();
bool sendToThingSpeak(SensorData data);
void bufferData(SensorData data);
float readRsAverage(int samples = 5);
float calculateCOppm(float rs, float r0);
int readMHZ19C();
void readPMS5003();
void flashWrite(uint32_t addr, SensorData& data);
void flashRead(uint32_t addr, SensorData& data);
bool waitForOK(String tag);
void flushSerial();
void sendReconnectSMS(uint32_t duration, SensorData data);

void setup() {
  Serial.begin(115200);
  sim900.begin(19200);
  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  co2Serial.begin(9600, SERIAL_8N1, MHZ_RX_PIN, MHZ_TX_PIN);

  Wire.begin(21, 22);  // Initialize I2C with your SDA/SCL pins

  Serial.println("🔌 Initializing sensors...");
  if (!rtc.begin()) {
    Serial.println("❌ RTC failed to initialize!");
    while (1);
  }

  if (!aht.begin()) {
    Serial.println("❌ Failed to initialize AHT sensor!");
    while (1);
  }

  if (!bmp.begin(0x76)) {
    Serial.println("❌ BMP280 failed to initialize!");
    while (1);
  }

  if (!flash.begin()) {
    Serial.println("❌ Flash memory failed to initialize!");
    while (1);
  }

  Serial.println("✅ All sensors initialized.");
  delay(2000);
  connectGPRS();
  if (gsmConnected) resendInProgress = true;
}

void loop() {
  unsigned long now = millis();

  if (now - lastSensorMillis >= SENSOR_INTERVAL || lastSensorMillis == 0) {
    lastSensorMillis = now;

    sensors_event_t humEvent, tempEvent;
    aht.getEvent(&humEvent, &tempEvent);
    float temp = tempEvent.temperature;
    float hum = humEvent.relative_humidity;
    float pressure = bmp.readPressure() / 100.0;
    float rs = readRsAverage();
    float co_ppm = (rs == INFINITY) ? -1 : calculateCOppm(rs, R0);
    int co2_ppm = readMHZ19C();
    readPMS5003();

    SensorData data = { temp, hum, pressure, co_ppm, co2_ppm, pm2_5, pm10, rtc.now().unixtime() };

    Serial.printf("📊 Temp: %.2f C | Hum: %.2f %% | Press: %.2f hPa\n", temp, hum, pressure);
    Serial.printf("💨 PM2.5: %d | PM10: %d | CO: %.2f ppm | CO₂: %d ppm\n", pm2_5, pm10, co_ppm, co2_ppm);

    if (gsmConnected) {
      if (!sendToThingSpeak(data)) {
        bufferData(data);
        gsmConnected = false;
        resendInProgress = false;
        wasDisconnected = true;
        disconnectStart = rtc.now().unixtime();
      }
    } else {
      bufferData(data);
      connectGPRS();
      if (gsmConnected) {
        uint32_t nowTs = rtc.now().unixtime();
        if (wasDisconnected) {
          uint32_t duration = nowTs - disconnectStart;
          if (duration >= 600) {
            sendReconnectSMS(duration, data);
          }
          wasDisconnected = false;
        }
        resendAddr = 0;
        resendInProgress = true;
      }
    }
  }

  if (resendInProgress && now - lastResendMillis >= RESEND_INTERVAL) {
    lastResendMillis = now;

    if (resendAddr < flashAddress) {
      SensorData data;
      flashRead(resendAddr, data);
      Serial.printf("📤 Resending: %.2f C, %.2f %%, %d PM2.5, CO: %.2f, CO₂: %d\n",
        data.temp, data.hum, data.pm25, data.co_ppm, data.co2_ppm);

      if (sendToThingSpeak(data)) {
        resendAddr += sizeof(SensorData);
      } else {
        Serial.println("❌ Resend failed. Retrying...");
        gsmConnected = false;
        resendInProgress = false;
      }
    } else {
      Serial.println("✅ All buffered data sent. Erasing flash...");
      flash.eraseSector(0);
      flashAddress = 0;
      resendAddr = 0;
      resendInProgress = false;
    }
  }
}

// === Sensor + Flash Functions ===
void readPMS5003() {
  if (pmsSerial.available() >= 32 && pmsSerial.peek() == 0x42) {
    uint8_t buf[32];
    int len = pmsSerial.readBytes(buf, 32);
    if (len == 32 && buf[0] == 0x42 && buf[1] == 0x4D) {
      uint16_t checksum = (buf[30] << 8) | buf[31];
      uint16_t sum = 0;
      for (int i = 0; i < 30; i++) sum += buf[i];
      if (sum == checksum) {
        pm2_5 = (buf[12] << 8) | buf[13];
        pm10 = (buf[14] << 8) | buf[15];
      }
    } else {
      pmsSerial.read();
    }
  }
}

float readRsAverage(int samples) {
  float sum = 0; int count = 0;
  for (int i = 0; i < samples; i++) {
    int adc = analogRead(CO_SENSOR_PIN);
    float v = (adc / 4095.0) * Vref;
    if (v > 0.01) {
      float rs = ((Vref - v) * RL) / v;
      sum += rs;
      count++;
    }
    delay(50);
  }
  return count ? (sum / count) : INFINITY;
}

float calculateCOppm(float rs, float r0) {
  if (rs == INFINITY) return -1;
  return 99.79 * pow(rs / r0, -2.334);
}

int readMHZ19C() {
  byte cmd[9] = {0xFF, 0x01, 0x86, 0,0,0,0,0,0x79};
  byte resp[9];
  co2Serial.write(cmd, 9);
  delay(10);
  if (co2Serial.available() >= 9) {
    co2Serial.readBytes(resp, 9);
    if (resp[0] == 0xFF && resp[1] == 0x86)
      return (resp[2] << 8) + resp[3];
  }
  return -1;
}

// === GPRS and Upload ===
void connectGPRS() {
  Serial.println("📶 Connecting GPRS...");
  sim900.println("AT"); if (!waitForOK("AT")) return;
  sim900.println("AT+SAPBR=0,1"); delay(1000);
  sim900.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""); if (!waitForOK("CONTYPE")) return;
  sim900.print("AT+SAPBR=3,1,\"APN\",\""); sim900.print(APN); sim900.println("\""); if (!waitForOK("APN")) return;
  sim900.println("AT+SAPBR=1,1"); delay(3000);
  sim900.println("AT+SAPBR=2,1");
  gsmConnected = waitForOK("GPRS Ready");
  if (gsmConnected) Serial.println("✅ GPRS connected");
  else Serial.println("❌ GPRS failed");
}

bool sendToThingSpeak(SensorData d) {
  String url = "http://api.thingspeak.com/update?api_key=" + String(THINGSPEAK_API_KEY);
  url += "&field1=" + String(d.temp, 2);
  url += "&field2=" + String(d.hum, 2);
  url += "&field3=" + String(d.pressure, 2);
  url += "&field4=" + String(d.pm25);
  url += "&field5=" + String(d.pm10);
  url += "&field6=" + ((d.co_ppm >= 0) ? String(d.co_ppm, 2) : "0");
  url += "&field7=" + ((d.co2_ppm >= 0) ? String(d.co2_ppm) : "0");

  sim900.println("AT+HTTPINIT");
  if (!waitForOK("HTTPINIT")) return false;

  sim900.println("AT+HTTPPARA=\"CID\",1");
  if (!waitForOK("CID")) return false;

  sim900.print("AT+HTTPPARA=\"URL\",\""); sim900.print(url); sim900.println("\"");
  if (!waitForOK("URL")) return false;

  sim900.println("AT+HTTPACTION=0");
  delay(6000);

  if (sim900.find("+HTTPACTION:")) {
    int method = sim900.parseInt();
    int status = sim900.parseInt();
    Serial.print("📡 HTTP Status: "); Serial.println(status);
    if (status == 200) {
      sim900.println("AT+HTTPREAD"); delay(1000);
      flushSerial();
      sim900.println("AT+HTTPTERM");
      return true;
    }
  }

  sim900.println("AT+HTTPTERM");
  return false;
}

// === Flash Functions ===
void bufferData(SensorData data) {
  flashWrite(flashAddress, data);
  flashAddress += sizeof(SensorData);
  Serial.println("📦 Buffered to flash.");
}

void flashWrite(uint32_t addr, SensorData& data) {
  uint8_t* ptr = (uint8_t*)&data;
  for (size_t i = 0; i < sizeof(SensorData); i++) {
    flash.writeByte(addr + i, ptr[i]);
  }
}

void flashRead(uint32_t addr, SensorData& data) {
  uint8_t* ptr = (uint8_t*)&data;
  for (size_t i = 0; i < sizeof(SensorData); i++) {
    ptr[i] = flash.readByte(addr + i);
  }
}

// === Utilities ===
bool waitForOK(String tag) {
  long timeout = millis() + 5000;
  while (millis() < timeout) {
    if (sim900.available()) {
      String res = sim900.readString();
      if (res.indexOf("OK") >= 0) {
        Serial.println("✅ " + tag);
        return true;
      } else if (res.indexOf("ERROR") >= 0) {
        Serial.println("❌ ERROR: " + tag);
        return false;
      }
    }
  }
  Serial.println("⏳ Timeout: " + tag);
  return false;
}

void flushSerial() {
  while (sim900.available()) Serial.write(sim900.read());
}

// === SMS Alert Function ===
void sendReconnectSMS(uint32_t duration, SensorData data) {
  String msg = "GSM reconnected after " + String(duration / 60) + " min.\n";
  msg += "T: " + String(data.temp, 1) + "C\n";
  msg += "H: " + String(data.hum, 1) + "%\n";
  msg += "P: " + String(data.pressure, 1) + "hPa\n";
  msg += "PM2.5: " + String(data.pm25) + " | PM10: " + String(data.pm10) + "\n";
  msg += "CO: " + String(data.co_ppm, 1) + "ppm\n";
  msg += "CO2: " + String(data.co2_ppm) + "ppm";

  Serial.println("📩 Sending SMS...");
  sim900.println("AT+CMGF=1"); delay(1000);
  sim900.println("AT+CSCS=\"GSM\""); delay(500);
  sim900.print("AT+CMGS=\"+947xxxxxxxx\"\r"); delay(1000);
  sim900.print(msg); delay(1000);
  sim900.write(26); // CTRL+Z to send
  delay(5000);
  Serial.println("✅ SMS sent.");
}
