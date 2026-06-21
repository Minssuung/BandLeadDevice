#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <WiFiUdp.h>

// Wi-Fi 설정
const char* ssid = "PinkLAB";
const char* password = "pinklab0523!";

// PC 수신기 설정
const char* pcIP = "192.168.0.6";
const uint16_t udpPort = 4210;

// WT61 IMU UART
// 센서 TX -> ESP RX(GPIO14), 센서 RX -> ESP TX(GPIO12)
// 일부 보드/코어에서는 D5, D6 매크로가 없어서 GPIO 번호로 fallback 처리
#if defined(D5) && defined(D6)
const uint8_t IMU_RX_PIN = D5;
const uint8_t IMU_TX_PIN = D6;
#else
const uint8_t IMU_RX_PIN = 14;  // GPIO14
const uint8_t IMU_TX_PIN = 12;  // GPIO12
#endif

SoftwareSerial imuSerial(IMU_RX_PIN, IMU_TX_PIN);
WiFiUDP udp;

const uint32_t IMU_BAUD_CANDIDATES[] = {9600, 115200, 57600, 38400, 19200};
const uint8_t IMU_BAUD_COUNT = sizeof(IMU_BAUD_CANDIDATES) / sizeof(IMU_BAUD_CANDIDATES[0]);

struct ImuData {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float wx = 0.0f;
  float wy = 0.0f;
  float wz = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
};

ImuData imu;
uint8_t frame[11];
uint8_t frameIdx = 0;
unsigned long lastSendMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastImuFrameMs = 0;
unsigned long lastBaudSwitchMs = 0;
unsigned long imuBytesTotal = 0;
unsigned long imuValidFrames = 0;
uint8_t imuBaudIdx = 0;
uint32_t currentImuBaud = IMU_BAUD_CANDIDATES[0];

void beginImuSerialAt(uint8_t idx) {
  if (idx >= IMU_BAUD_COUNT) {
    return;
  }
  imuBaudIdx = idx;
  currentImuBaud = IMU_BAUD_CANDIDATES[idx];
  imuSerial.begin(currentImuBaud);
  frameIdx = 0;
  Serial.print("IMU baud set: ");
  Serial.println(currentImuBaud);
}

int16_t toInt16(uint8_t lo, uint8_t hi) {
  int16_t v = (int16_t)((hi << 8) | lo);
  return v;
}

bool parseWT61Byte(uint8_t b) {
  if (frameIdx == 0 && b != 0x55) {
    return false;
  }

  frame[frameIdx++] = b;

  if (frameIdx < 11) {
    return false;
  }

  frameIdx = 0;

  uint8_t sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += frame[i];
  }
  if (sum != frame[10]) {
    return false;
  }

  uint8_t type = frame[1];
  int16_t d0 = toInt16(frame[2], frame[3]);
  int16_t d1 = toInt16(frame[4], frame[5]);
  int16_t d2 = toInt16(frame[6], frame[7]);

  if (type == 0x51) {
    imu.ax = (float)d0 / 32768.0f * 16.0f;
    imu.ay = (float)d1 / 32768.0f * 16.0f;
    imu.az = (float)d2 / 32768.0f * 16.0f;
  } else if (type == 0x52) {
    imu.wx = (float)d0 / 32768.0f * 2000.0f;
    imu.wy = (float)d1 / 32768.0f * 2000.0f;
    imu.wz = (float)d2 / 32768.0f * 2000.0f;
  } else if (type == 0x53) {
    imu.roll = (float)d0 / 32768.0f * 180.0f;
    imu.pitch = (float)d1 / 32768.0f * 180.0f;
    imu.yaw = (float)d2 / 32768.0f * 180.0f;
    return true;
  }

  return false;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, ESP IP: ");
  Serial.println(WiFi.localIP());
}

void sendImuPacket() {
  char payload[256];
  snprintf(
    payload,
    sizeof(payload),
    "{\"ts\":%lu,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"wx\":%.3f,\"wy\":%.3f,\"wz\":%.3f,\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f}",
    millis(),
    imu.ax,
    imu.ay,
    imu.az,
    imu.wx,
    imu.wy,
    imu.wz,
    imu.roll,
    imu.pitch,
    imu.yaw
  );

  udp.beginPacket(pcIP, udpPort);
  udp.write((const uint8_t*)payload, strlen(payload));
  udp.endPacket();

  Serial.println(payload);
}

void sendStatusPacket() {
  char payload[192];
  snprintf(
    payload,
    sizeof(payload),
    "{\"type\":\"status\",\"ts\":%lu,\"wifi\":%d,\"rssi\":%d,\"imu_available\":%d,\"imu_last_ms\":%lu,\"imu_baud\":%lu,\"imu_bytes\":%lu,\"imu_frames\":%lu}",
    millis(),
    (int)WiFi.status(),
    (int)WiFi.RSSI(),
    (lastImuFrameMs > 0 ? 1 : 0),
    lastImuFrameMs,
    currentImuBaud,
    imuBytesTotal,
    imuValidFrames
  );

  udp.beginPacket(pcIP, udpPort);
  udp.write((const uint8_t*)payload, strlen(payload));
  udp.endPacket();

  Serial.println(payload);
}

void setup() {
  Serial.begin(115200);
  beginImuSerialAt(0);
  connectWiFi();
  Serial.println("IMU streaming ready");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  while (imuSerial.available() > 0) {
    uint8_t b = (uint8_t)imuSerial.read();
    imuBytesTotal++;
    bool angleFrameReady = parseWT61Byte(b);

    if (angleFrameReady) {
      lastImuFrameMs = millis();
      imuValidFrames++;
      unsigned long now = millis();
      if (now - lastSendMs >= 50) {
        sendImuPacket();
        lastSendMs = now;
      }
    }
  }

  unsigned long now = millis();

  // 부팅 후 IMU 프레임이 없으면 후보 보레이트를 순환하며 자동 탐색
  if (lastImuFrameMs == 0 && now - lastBaudSwitchMs >= 3000) {
    uint8_t nextIdx = (uint8_t)((imuBaudIdx + 1) % IMU_BAUD_COUNT);
    beginImuSerialAt(nextIdx);
    lastBaudSwitchMs = now;
  }

  if (now - lastStatusMs >= 1000) {
    sendStatusPacket();
    lastStatusMs = now;
  }
}
