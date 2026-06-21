#include <WiFi.h>
#include <WiFiUdp.h>

// Wi-Fi 설정
const char* ssid = "PinkLAB";
const char* password = "pinklab0523!";

// PC 수신기 설정
const char* pcIP = "192.168.0.6";  // same-subnet broadcast
const uint16_t udpPort = 4210;

// ESP32 + MAX485 배선 기본값(LOLIN D32)
// RO -> ESP32 RX2(GPIO16)
// DI -> ESP32 TX2(GPIO17)
// RE/DE -> GND 직결 (수신 고정 테스트 - GPIO 제어 없음)
const int IMU_RX_PIN = 16;
const int IMU_TX_PIN = 17;
const int RS485_RE_DE_PIN = 21;  // DE+RE -> GPIO21 (TX시 HIGH, RX시 LOW)

// WT901C485 설정
// WT901C485는 RS485 Modbus RTU 전용 - 자동 스트리밍 없음, 반드시 폴링 필요
const bool USE_MODBUS_POLL = true;
const uint8_t IMU_MODBUS_SLAVE_IDS[] = {0x50, 0x01};
const uint8_t IMU_MODBUS_SLAVE_COUNT = sizeof(IMU_MODBUS_SLAVE_IDS) / sizeof(IMU_MODBUS_SLAVE_IDS[0]);
const uint16_t IMU_ANGLE_START_REG = 0x003D;  // Roll, Pitch, Yaw
const uint16_t IMU_ANGLE_REG_COUNT = 3;

HardwareSerial imuSerial(2);
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
unsigned long imuHeaderHits = 0;
unsigned long imuChecksumFails = 0;
unsigned long imuPollOk = 0;
unsigned long imuPollFail = 0;
uint8_t imuBaudIdx = 0;
uint32_t currentImuBaud = IMU_BAUD_CANDIDATES[0];
unsigned long lastPollMs = 0;
uint8_t imuSlaveIdx = 0;
uint8_t lastGoodSlaveId = 0;

uint16_t modbusCrc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void setRs485TxMode(bool tx) {
  if (RS485_RE_DE_PIN >= 0) {
    digitalWrite(RS485_RE_DE_PIN, tx ? HIGH : LOW);
  }
}

bool pollWt901c485Angle(uint8_t slaveId) {
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x03;
  req[2] = (uint8_t)(IMU_ANGLE_START_REG >> 8);
  req[3] = (uint8_t)(IMU_ANGLE_START_REG & 0xFF);
  req[4] = (uint8_t)(IMU_ANGLE_REG_COUNT >> 8);
  req[5] = (uint8_t)(IMU_ANGLE_REG_COUNT & 0xFF);
  uint16_t crc = modbusCrc16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  imuSerial.flush();
  setRs485TxMode(true);
  delayMicroseconds(200);
  imuSerial.write(req, sizeof(req));
  imuSerial.flush();
  delayMicroseconds(200);
  setRs485TxMode(false);

  uint8_t resp[32];
  uint8_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 60) {
    while (imuSerial.available() > 0 && n < sizeof(resp)) {
      resp[n++] = (uint8_t)imuSerial.read();
      imuBytesTotal++;
    }
    if (n >= 11) {
      break;
    }
  }

  if (n < 11) {
    return false;
  }

  if (resp[0] != slaveId || resp[1] != 0x03 || resp[2] != 6) {
    return false;
  }

  uint16_t respCrc = (uint16_t)resp[n - 2] | ((uint16_t)resp[n - 1] << 8);
  uint16_t calcCrc = modbusCrc16(resp, n - 2);
  if (respCrc != calcCrc) {
    imuChecksumFails++;
    return false;
  }

  int16_t rollRaw = (int16_t)(((uint16_t)resp[3] << 8) | resp[4]);
  int16_t pitchRaw = (int16_t)(((uint16_t)resp[5] << 8) | resp[6]);
  int16_t yawRaw = (int16_t)(((uint16_t)resp[7] << 8) | resp[8]);

  // WT901C 계열 각도 레지스터는 보통 0.01 deg 스케일
  imu.roll = (float)rollRaw / 100.0f;
  imu.pitch = (float)pitchRaw / 100.0f;
  imu.yaw = (float)yawRaw / 100.0f;

  return true;
}

void beginImuSerialAt(uint8_t idx) {
  if (idx >= IMU_BAUD_COUNT) {
    return;
  }
  imuBaudIdx = idx;
  currentImuBaud = IMU_BAUD_CANDIDATES[idx];
  imuSerial.begin(currentImuBaud, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
  frameIdx = 0;
  Serial.print("IMU baud set: ");
  Serial.println(currentImuBaud);
}

int16_t toInt16(uint8_t lo, uint8_t hi) {
  int16_t v = (int16_t)((hi << 8) | lo);
  return v;
}

bool parseWT61Byte(uint8_t b) {
  if (b == 0x55) {
    imuHeaderHits++;
  }

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
    imuChecksumFails++;
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
  char payload[320];
  uint8_t activeSlaveId = IMU_MODBUS_SLAVE_IDS[imuSlaveIdx];
  snprintf(
    payload,
    sizeof(payload),
    "{\"type\":\"status\",\"ts\":%lu,\"wifi\":%d,\"rssi\":%d,\"imu_available\":%d,\"imu_last_ms\":%lu,\"imu_baud\":%lu,\"imu_slave\":%u,\"imu_last_good_slave\":%u,\"imu_bytes\":%lu,\"imu_frames\":%lu,\"imu_headers\":%lu,\"imu_csum_fail\":%lu,\"imu_poll_ok\":%lu,\"imu_poll_fail\":%lu}",
    millis(),
    (int)WiFi.status(),
    (int)WiFi.RSSI(),
    (lastImuFrameMs > 0 ? 1 : 0),
    lastImuFrameMs,
    currentImuBaud,
    activeSlaveId,
    lastGoodSlaveId,
    imuBytesTotal,
    imuValidFrames,
    imuHeaderHits,
    imuChecksumFails,
    imuPollOk,
    imuPollFail
  );

  udp.beginPacket(pcIP, udpPort);
  udp.write((const uint8_t*)payload, strlen(payload));
  udp.endPacket();

  Serial.println(payload);
}

void setup() {
  Serial.begin(115200);

  if (RS485_RE_DE_PIN >= 0) {
    pinMode(RS485_RE_DE_PIN, OUTPUT);
    digitalWrite(RS485_RE_DE_PIN, LOW);
  }

  beginImuSerialAt(0);  // 9600 baud

  // GPIO16 수신 핀 자체 테스트
  // RO 핀을 3.3V에 연결하면 0xFF 바이트가 들어와야 함
  // RO 핀을 GND에 연결하면 0x00 바이트가 들어와야 함
  Serial.println("=== GPIO16 RX pin test (3초 대기) ===");
  delay(3000);
  unsigned long rxCount = 0;
  while (imuSerial.available() > 0) {
    uint8_t b = imuSerial.read();
    rxCount++;
    if (rxCount <= 5) {
      Serial.print("RX byte: 0x");
      Serial.println(b, HEX);
    }
  }
  Serial.print("GPIO16 test bytes received: ");
  Serial.println(rxCount);
  if (rxCount == 0) {
    Serial.println("!!! GPIO16 수신 없음: RO->GPIO16 연결 불량 또는 센서 전원 없음");
  } else {
    Serial.println("GPIO16 수신 OK");
  }
  Serial.println("=== test end ===");

  connectWiFi();
  Serial.print("UDP target: ");
  Serial.print(pcIP);
  Serial.print(":");
  Serial.println(udpPort);
  Serial.println("IMU streaming ready");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!USE_MODBUS_POLL) {
    // 스트리밍 모드: 수신된 모든 바이트를 로그 (디버깅)
    static unsigned long lastDebugMs = 0;
    while (imuSerial.available() > 0) {
      uint8_t b = (uint8_t)imuSerial.read();
      imuBytesTotal++;
      
      // 수신된 바이트를 16진수로 출력 (매 100ms마다 5개 바이트씩)
      if (millis() - lastDebugMs >= 100) {
        Serial.print("RX: ");
        for (int i = 0; i < 5 && imuSerial.available() > 0; i++) {
          Serial.print(b, HEX);
          Serial.print(" ");
          b = (uint8_t)imuSerial.read();
          imuBytesTotal++;
        }
        Serial.println();
        lastDebugMs = millis();
      }
      
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
  } else {
    unsigned long now = millis();
    if (now - lastPollMs >= 100) {
      uint8_t slaveId = IMU_MODBUS_SLAVE_IDS[imuSlaveIdx];
      bool ok = pollWt901c485Angle(slaveId);
      if (ok) {
        lastImuFrameMs = now;
        imuValidFrames++;
        imuPollOk++;
        lastGoodSlaveId = slaveId;
        sendImuPacket();
      } else {
        imuPollFail++;
        imuSlaveIdx = (uint8_t)((imuSlaveIdx + 1) % IMU_MODBUS_SLAVE_COUNT);
      }
      lastPollMs = now;
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
