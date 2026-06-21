#include <WiFi.h>
#include <WiFiUdp.h>

// Wi-Fi 설정
const char* ssid = "PinkLAB";
const char* password = "pinklab0523!";

// PC 수신기 설정
const char* pcIP = "192.168.0.12";  // same-subnet broadcast (PC Wi-Fi IP)
const uint16_t udpPort = 4210;

// ESP32 + MAX485 배선 기본값(LOLIN D32)
// RO -> ESP32 RX2(GPIO16)
// DI -> ESP32 TX2(GPIO17)
// RE/DE -> GPIO21 (TX시 HIGH, RX시 LOW)
const int IMU_RX_PIN = 16;
const int IMU_TX_PIN = 17;
const int RS485_RE_DE_PIN = 21;  // DE+RE -> GPIO21 (TX시 HIGH, RX시 LOW)

// 현재 연결 상태 기준: RS485 Modbus 폴링 모드 사용
const bool USE_MODBUS_POLL = true;
// 센서 융합 모드 A/B 토글: true=6축(지자기OFF, 자석환경 강함/빠른동작 yaw킥),
//                          false=9축(지자기ON, 자석 멀면 자기치유/근처면 ~100° 홉)
const bool IMU_USE_6AXIS = false;  // A/B 2라운드: 9축(지자기 ON) — 사람 착용은 서보에서 멀어서 테스트
const uint8_t IMU_MODBUS_SLAVE_IDS[] = {0x50, 0x51, 0x52};  // 폴링할 IMU의 Modbus 슬레이브 ID 목록
const uint8_t IMU_MODBUS_SLAVE_COUNT = sizeof(IMU_MODBUS_SLAVE_IDS) / sizeof(IMU_MODBUS_SLAVE_IDS[0]);
const uint16_t IMU_ANGLE_START_REG = 0x003D;  // Roll, Pitch, Yaw
const uint16_t IMU_ANGLE_REG_COUNT = 3;
// WT901C485 쿼터니언 레지스터 Q0~Q3 (값 = raw/32768). 짐벌락 해결용.
// 0x52는 ‖q‖≈0.9로 오답 확인됨 → 0x51로 변경. 재업로드 후 viz의 ‖q‖이 1.00인지 확인.
const uint16_t IMU_QUAT_START_REG = 0x0051;
const uint16_t IMU_QUAT_REG_COUNT = 4;

HardwareSerial imuSerial(2);
WiFiUDP udp;

// 115200을 첫 번째로: 설정 완료 후에는 115200에서 실행
const uint32_t IMU_BAUD_CANDIDATES[] = {115200, 9600, 57600, 38400, 19200};
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
  // 쿼터니언 (기본값 = 단위 쿼터니언). 센서에서 직접 읽어 짐벌락 회피.
  float q0 = 1.0f;
  float q1 = 0.0f;
  float q2 = 0.0f;
  float q3 = 0.0f;
};

ImuData imus[IMU_MODBUS_SLAVE_COUNT];
uint8_t frame[11];
uint8_t frameIdx = 0;
unsigned long lastSendMs = 0;
unsigned long lastStatusMs = 0;
unsigned long lastImuFrameMs = 0;
unsigned long imuLastFrameMs[IMU_MODBUS_SLAVE_COUNT] = {0};
unsigned long lastBaudSwitchMs = 0;
unsigned long imuBytesTotal = 0;
unsigned long imuValidFrames = 0;
unsigned long imuHeaderHits = 0;
unsigned long imuChecksumFails = 0;
unsigned long imuPollOk = 0;
unsigned long imuPollFail = 0;
// 통합읽기 진단 카운터 + 적응형 스킵 (실패 반복 센서는 타임아웃 낭비 안 하게)
unsigned long imuCombinedOk = 0;
unsigned long imuCombinedFail = 0;
bool combinedSupported[8] = {true, true, true, true, true, true, true, true};
uint8_t combinedFailStreak[8] = {0};
// 쿼터니언-온리 폴링 (1차 경로): 응답 13B vs 통합 53B → IMU당 ~3.5ms 단축, 32→~90Hz.
// 오일러는 ESP에서 역산해 JSON 키 유지 (PC 파서가 roll 키 없으면 전 패킷 드롭 — FC 검증).
unsigned long imuQuatOnlyOk = 0;
unsigned long imuQuatOnlyFail = 0;
bool quatOnlySupported[8] = {true, true, true, true, true, true, true, true};
uint8_t quatOnlyFailStreak[8] = {0};
// FC 수정: ①죽은 센서 백오프 — 전 경로 실패 센서는 500ms 폴 스킵(타임아웃 캐스케이드로
// 멀쩡한 센서까지 끌려가는 것 방지) ②10초마다 강등(quat-only/combined OFF) 재프로브
// (일시 글리치 5연속으로 영구 저속 강등되는 비가역성 해소)
unsigned long sensorBackoffUntil[8] = {0};
unsigned long lastReprobeMs = 0;
uint8_t imuBaudIdx = 0;
uint32_t currentImuBaud = IMU_BAUD_CANDIDATES[0];
unsigned long lastPollMs = 0;
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

// Modbus FC06: Write Single Register (WT901C 설정 변경용)
bool writeModbusSingleReg(uint8_t slaveId, uint16_t regAddr, uint16_t value) {
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x06;
  req[2] = (uint8_t)(regAddr >> 8);
  req[3] = (uint8_t)(regAddr & 0xFF);
  req[4] = (uint8_t)(value >> 8);
  req[5] = (uint8_t)(value & 0xFF);
  uint16_t crc = modbusCrc16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  while (imuSerial.available() > 0) (void)imuSerial.read();

  imuSerial.flush();
  setRs485TxMode(true);
  delayMicroseconds(100);
  imuSerial.write(req, sizeof(req));
  imuSerial.flush();
  delayMicroseconds(200);
  setRs485TxMode(false);

  // FC06 응답은 요청의 에코 (8바이트)
  uint8_t resp[16];
  uint8_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 30) {
    while (imuSerial.available() > 0 && n < sizeof(resp)) {
      resp[n++] = imuSerial.read();
    }
    if (n >= 8) break;
  }

  if (n < 8) return false;
  uint16_t respCrc = (uint16_t)resp[6] | ((uint16_t)resp[7] << 8);
  return respCrc == modbusCrc16(resp, 6) && resp[0] == slaveId && resp[1] == 0x06;
}

// WT901C-485의 Baud Rate를 115200으로 설정하고 저장
void configureImusToBaud115200() {
  // Baud 레지스터: 0x0004  (0=2400, 1=4800, 2=9600, 3=19200, 4=38400, 5=57600, 6=115200)
  // SAVE  레지스터: 0x0000  (0x0000 쓰면 설정 저장)
  const uint16_t BAUD_REG   = 0x0004;
  const uint16_t SAVE_REG   = 0x0000;
  const uint16_t BAUD_115200 = 0x0006;
  // WitMotion은 설정 레지스터 쓰기 전 KEY(0x69)에 0xB588 '잠금해제' 필수!
  //   (이게 없어서 기존 승급이 무시되고 9600에 머물렀음 → 5-6Hz 병목의 근본 원인)
  const uint16_t KEY_REG    = 0x0069;
  const uint16_t KEY_UNLOCK = 0xB588;

  Serial.println("=== IMU baud 9600->115200 설정 시도 ===");
  // 9600 baud로 시작해 현재 설정 시도
  imuSerial.begin(9600, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
  delay(200);
  while (imuSerial.available() > 0) (void)imuSerial.read();

  for (uint8_t i = 0; i < IMU_MODBUS_SLAVE_COUNT; i++) {
    uint8_t id = IMU_MODBUS_SLAVE_IDS[i];
    Serial.print("  IMU 0x");
    Serial.print(id, HEX);
    Serial.print(" baud 설정... ");
    writeModbusSingleReg(id, KEY_REG, KEY_UNLOCK);  // 잠금해제
    delay(20);
    bool ok = writeModbusSingleReg(id, BAUD_REG, BAUD_115200);
    if (ok) {
      delay(20);
      writeModbusSingleReg(id, KEY_REG, KEY_UNLOCK);  // SAVE 전에도 잠금해제
      delay(20);
      writeModbusSingleReg(id, SAVE_REG, 0x0000);  // 저장
      Serial.println("OK (재부팅 후 115200 적용)");
    } else {
      Serial.println("응답없음 (이미 115200이거나 연결 문제)");
    }
    delay(50);
  }
  Serial.println("115200 baud로 장치 재부팅 대기...");
  delay(500);  // 센서가 저장하고 재부팅할 시간
  Serial.println("=== IMU baud 설정 완료 ===");
}

// WT901 6축 모드(지자기 OFF) 설정 — 실내/서보 자석 환경에서 9축 지자기 융합은
// 상대 yaw 드리프트/홉을 만들어 손목롤·팔꿈치에 ~100° 오프셋 오염 (지그 실측 확정).
// 6축은 yaw가 천천히 매끄럽게만 흘러 [기준 설정] 리셋으로 충분.
// 단일 레지스터 읽기 (FC03) — 설정 리드백 검증용
bool readModbusSingleReg(uint8_t slaveId, uint16_t regAddr, uint16_t &outVal) {
  uint8_t req[8];
  req[0] = slaveId; req[1] = 0x03;
  req[2] = (uint8_t)(regAddr >> 8); req[3] = (uint8_t)(regAddr & 0xFF);
  req[4] = 0x00; req[5] = 0x01;
  uint16_t crc = modbusCrc16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF); req[7] = (uint8_t)(crc >> 8);
  while (imuSerial.available() > 0) (void)imuSerial.read();
  imuSerial.flush();
  setRs485TxMode(true); delayMicroseconds(100);
  imuSerial.write(req, sizeof(req)); imuSerial.flush();
  delayMicroseconds(200); setRs485TxMode(false);
  uint8_t resp[16]; uint8_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 60) {
    while (imuSerial.available() > 0 && n < sizeof(resp)) resp[n++] = imuSerial.read();
    if (n >= 7) break;
  }
  if (n < 7 || resp[0] != slaveId || resp[1] != 0x03 || resp[2] != 2) return false;
  uint16_t rc = (uint16_t)resp[5] | ((uint16_t)resp[6] << 8);
  if (rc != modbusCrc16(resp, 5)) return false;
  outVal = ((uint16_t)resp[3] << 8) | resp[4];
  return true;
}

void configureImus6Axis(uint32_t baud) {
  // 7차 fact-checker 검증: AXIS6=0x24(0=9축,1=6축), KEY=0x69(0xB588), SAVE=0x00
  //   (WitMotion 공식 Modbus SDK REG.h 확인). 설정 후 리드백으로 적용 확인 필수.
  const uint16_t KEY_REG    = 0x0069;
  const uint16_t KEY_UNLOCK = 0xB588;
  const uint16_t AXIS_REG   = 0x0024;   // 0=9축(지자기 융합), 1=6축
  const uint16_t SAVE_REG   = 0x0000;

  Serial.println("=== IMU 6축 모드(지자기 OFF) 설정 ===");
  imuSerial.begin(baud, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
  delay(200);
  while (imuSerial.available() > 0) (void)imuSerial.read();

  for (uint8_t i = 0; i < IMU_MODBUS_SLAVE_COUNT; i++) {
    uint8_t id = IMU_MODBUS_SLAVE_IDS[i];
    Serial.print("  IMU 0x");
    Serial.print(id, HEX);
    Serial.print(" 6축 설정... ");
    uint16_t cur = 0xFFFF;
    if (readModbusSingleReg(id, AXIS_REG, cur) && cur == (IMU_USE_6AXIS ? 1 : 0)) {
      Serial.println(IMU_USE_6AXIS ? "이미 6축 (스킵)" : "이미 9축 (스킵)");
      continue;
    }
    writeModbusSingleReg(id, KEY_REG, KEY_UNLOCK);
    delay(20);
    bool ok = writeModbusSingleReg(id, AXIS_REG, IMU_USE_6AXIS ? 0x0001 : 0x0000);
    if (ok) {
      delay(20);
      writeModbusSingleReg(id, KEY_REG, KEY_UNLOCK);
      delay(20);
      writeModbusSingleReg(id, SAVE_REG, 0x0000);  // 저장
      delay(100);
      // 리드백 검증 (FC#1): 적용 안 됐으면 크게 알림
      uint16_t v = 0xFFFF;
      if (readModbusSingleReg(id, AXIS_REG, v) && v == (IMU_USE_6AXIS ? 1 : 0)) {
        Serial.println(IMU_USE_6AXIS ? "OK (리드백 확인 = 6축)" : "OK (리드백 확인 = 9축)");
      } else {
        Serial.print("!!! 리드백 실패/불일치 (v=");
        Serial.print(v);
        Serial.println(") — 9축으로 남아있을 수 있음!");
      }
    } else {
      Serial.println("응답없음");
    }
    delay(50);
  }
  Serial.println("=== 6축 설정 완료 ===");
}

// 지연 단축 설정 (체감지연 최대 범인 = 센서 내부 필터):
//   BANDWIDTH 0x1F: 기본 0x04(20Hz) → 0x02(98Hz). 내부 칼만 평활화 군지연 ~수십ms 감소.
//   RRATE 0x03: 0x0D(출력 안 함)로 고정. ★주의: 0x0B(200Hz) 실험 결과 Modbus 배선에서도
//     능동 스트리밍(WIT 프레임 자동송출)이 켜져 버스 플러딩 → poll_fail 폭증 (2026-06-12 실측).
//     레지스터 갱신은 RRATE와 무관하게 내부 융합속도로 이뤄짐(실측 지연 ≤40ms) — 올릴 이유 없음.
const uint16_t IMU_KEY_REG    = 0x0069;
const uint16_t IMU_KEY_UNLOCK = 0xB588;
const uint16_t IMU_SAVE_REG   = 0x0000;

// 잠금해제→쓰기→리드백을 성공할 때까지 재시도. 플러딩 중인 버스에서는 요청/응답이
// 스트림과 충돌해 자주 깨지므로 1회 시도로는 복구 불가 — 재시도가 핵심.
bool writeRegVerifiedRetry(uint8_t id, uint16_t reg, uint16_t val, uint8_t tries) {
  for (uint8_t t = 0; t < tries; t++) {
    uint16_t cur = 0xFFFF;
    if (readModbusSingleReg(id, reg, cur) && cur == val) {
      return true;  // 이미 설정됨 / 쓰기 적용 확인
    }
    writeModbusSingleReg(id, IMU_KEY_REG, IMU_KEY_UNLOCK);
    delay(20);
    writeModbusSingleReg(id, reg, val);
    delay(20);
  }
  uint16_t v = 0xFFFF;
  return readModbusSingleReg(id, reg, v) && v == val;
}

void configureImusLowLatency(uint32_t baud) {
  struct RegItem { uint16_t reg; uint16_t val; const char* name; };
  const RegItem items[] = {
    { 0x0003, 0x000D, "rate silent" },     // 플러딩 차단이 최우선 — 반드시 첫 번째
    { 0x001F, 0x0002, "bandwidth 98Hz" },
  };

  Serial.println("=== IMU 저지연 설정 (rate silent + bandwidth 98Hz) ===");
  imuSerial.begin(baud, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
  delay(200);
  while (imuSerial.available() > 0) (void)imuSerial.read();

  for (uint8_t i = 0; i < IMU_MODBUS_SLAVE_COUNT; i++) {
    uint8_t id = IMU_MODBUS_SLAVE_IDS[i];
    bool anyOk = false;
    for (const RegItem& item : items) {
      Serial.print("  IMU 0x");
      Serial.print(id, HEX);
      Serial.print(" ");
      Serial.print(item.name);
      Serial.print("... ");
      if (writeRegVerifiedRetry(id, item.reg, item.val, 8)) {
        anyOk = true;
        Serial.println("OK (리드백 확인)");
      } else {
        Serial.println("!!! 실패 (8회 재시도 후에도 미적용)");
      }
    }
    if (anyOk) {
      writeModbusSingleReg(id, IMU_KEY_REG, IMU_KEY_UNLOCK);
      delay(20);
      writeModbusSingleReg(id, IMU_SAVE_REG, 0x0000);
      delay(100);
    }
    delay(50);
  }
  Serial.println("=== 저지연 설정 완료 ===");
}

bool pollWt901c485Angle(uint8_t slaveId, ImuData& outData) {
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

  // 요청 전 수신 버퍼를 비워 이전 잡음/잔여 바이트를 제거
  while (imuSerial.available() > 0) {
    (void)imuSerial.read();
  }

  imuSerial.flush();
  setRs485TxMode(true);
  delayMicroseconds(100);  // RE/DE 핀 안정화는 100μs면 충분
  imuSerial.write(req, sizeof(req));
  imuSerial.flush();
  delayMicroseconds(200);  // TX→RX 전환 가드타임
  setRs485TxMode(false);

  uint8_t resp[64];
  uint8_t n = 0;
  unsigned long t0 = millis();
  // 115200: 11B≈1ms → 8ms 충분 / 9600 폴백: ~11.5ms → 30ms.
  // 무응답 센서의 타임아웃이 사이클을 지배(FC: 캐스케이드시 전체 27Hz)하므로 짧게.
  unsigned long timeoutMs = (currentImuBaud >= 115200) ? 8 : 30;
  while (millis() - t0 < timeoutMs) {
    while (imuSerial.available() > 0 && n < sizeof(resp)) {
      resp[n++] = (uint8_t)imuSerial.read();
      imuBytesTotal++;
    }
    if (n >= 11) break;  // 응답 충분히 수신 시 즉시 탈출
  }

  if (n < 11) {
    return false;
  }

  // 버퍼 안에서 유효한 Modbus 응답 프레임(11바이트) 탐색
  for (uint8_t i = 0; i + 10 < n; i++) {
    if (resp[i] != slaveId || resp[i + 1] != 0x03 || resp[i + 2] != 6) {
      continue;
    }

    uint16_t respCrc = (uint16_t)resp[i + 9] | ((uint16_t)resp[i + 10] << 8);
    uint16_t calcCrc = modbusCrc16(&resp[i], 9);
    if (respCrc != calcCrc) {
      imuChecksumFails++;
      continue;
    }

    int16_t rollRaw = (int16_t)(((uint16_t)resp[i + 3] << 8) | resp[i + 4]);
    int16_t pitchRaw = (int16_t)(((uint16_t)resp[i + 5] << 8) | resp[i + 6]);
    int16_t yawRaw = (int16_t)(((uint16_t)resp[i + 7] << 8) | resp[i + 8]);

    // ★스케일 수정(2026-06-12): WT901C485 각도 레지스터 = raw/32768×180 (±180°).
    //   기존 /100은 1.82배 과장 — 실기 쿼터니언·오일러 쌍 6값 대조로 확정.
    outData.roll = (float)rollRaw / 32768.0f * 180.0f;
    outData.pitch = (float)pitchRaw / 32768.0f * 180.0f;
    outData.yaw = (float)yawRaw / 32768.0f * 180.0f;
    return true;
  }

  return false;
}

// WT901C485 쿼터니언(Q0~Q3) 폴링. 오일러 폴링과 별도 트랜잭션이라
// 기존 오일러 경로는 전혀 건드리지 않는다. 실패해도 단위 쿼터니언 유지.
bool pollWt901c485Quat(uint8_t slaveId, ImuData& outData) {
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x03;
  req[2] = (uint8_t)(IMU_QUAT_START_REG >> 8);
  req[3] = (uint8_t)(IMU_QUAT_START_REG & 0xFF);
  req[4] = (uint8_t)(IMU_QUAT_REG_COUNT >> 8);
  req[5] = (uint8_t)(IMU_QUAT_REG_COUNT & 0xFF);
  uint16_t crc = modbusCrc16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  while (imuSerial.available() > 0) {
    (void)imuSerial.read();
  }

  imuSerial.flush();
  setRs485TxMode(true);
  delayMicroseconds(100);
  imuSerial.write(req, sizeof(req));
  imuSerial.flush();
  delayMicroseconds(200);
  setRs485TxMode(false);

  // 응답: addr(1)+fc(1)+bytecount(1=8)+data(8)+crc(2) = 13바이트
  // 115200: 13B≈1.1ms → 8ms 충분. 무응답 타임아웃이 사이클 지배 방지(FC).
  uint8_t resp[32];
  uint8_t n = 0;
  unsigned long t0 = millis();
  unsigned long timeoutMs = (currentImuBaud >= 115200) ? 8 : 30;
  while (millis() - t0 < timeoutMs) {
    while (imuSerial.available() > 0 && n < sizeof(resp)) {
      resp[n++] = (uint8_t)imuSerial.read();
      imuBytesTotal++;
    }
    if (n >= 13) break;
  }

  if (n < 13) {
    return false;
  }

  for (uint8_t i = 0; i + 12 < n; i++) {
    if (resp[i] != slaveId || resp[i + 1] != 0x03 || resp[i + 2] != 8) {
      continue;
    }
    uint16_t respCrc = (uint16_t)resp[i + 11] | ((uint16_t)resp[i + 12] << 8);
    if (respCrc != modbusCrc16(&resp[i], 11)) {
      continue;
    }
    int16_t q0Raw = (int16_t)(((uint16_t)resp[i + 3] << 8) | resp[i + 4]);
    int16_t q1Raw = (int16_t)(((uint16_t)resp[i + 5] << 8) | resp[i + 6]);
    int16_t q2Raw = (int16_t)(((uint16_t)resp[i + 7] << 8) | resp[i + 8]);
    int16_t q3Raw = (int16_t)(((uint16_t)resp[i + 9] << 8) | resp[i + 10]);
    outData.q0 = (float)q0Raw / 32768.0f;
    outData.q1 = (float)q1Raw / 32768.0f;
    outData.q2 = (float)q2Raw / 32768.0f;
    outData.q3 = (float)q3Raw / 32768.0f;
    return true;
  }

  return false;
}

// 통합 폴링: 오일러(0x3D~0x3F)+쿼터니언(0x51~0x54)을 한 트랜잭션(24레지스터)으로.
//   트랜잭션 2회→1회 = 턴어라운드/타임아웃 윈도 절반 → poll_fail 감소 + 주기 단축.
//   실패 시 호출부가 기존 개별 폴링으로 폴백하므로 안전.
const uint16_t IMU_COMBINED_START_REG = 0x003D;
const uint16_t IMU_COMBINED_REG_COUNT = 24;  // 0x3D..0x54

bool pollWt901c485Combined(uint8_t slaveId, ImuData& outData) {
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x03;
  req[2] = (uint8_t)(IMU_COMBINED_START_REG >> 8);
  req[3] = (uint8_t)(IMU_COMBINED_START_REG & 0xFF);
  req[4] = (uint8_t)(IMU_COMBINED_REG_COUNT >> 8);
  req[5] = (uint8_t)(IMU_COMBINED_REG_COUNT & 0xFF);
  uint16_t crc = modbusCrc16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  while (imuSerial.available() > 0) {
    (void)imuSerial.read();
  }

  imuSerial.flush();
  setRs485TxMode(true);
  delayMicroseconds(100);
  imuSerial.write(req, sizeof(req));
  imuSerial.flush();
  delayMicroseconds(200);
  setRs485TxMode(false);

  // 응답: addr(1)+fc(1)+bytecount(1=48)+data(48)+crc(2) = 53바이트
  // 115200: 53B≈4.6ms → 15ms 충분 / 9600 폴백: ≈55ms → 90ms 필요
  const uint8_t NEED = 53;
  unsigned long timeoutMs = (currentImuBaud >= 115200) ? 15 : 90;
  uint8_t resp[80];
  uint8_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (imuSerial.available() > 0 && n < sizeof(resp)) {
      resp[n++] = (uint8_t)imuSerial.read();
      imuBytesTotal++;
    }
    if (n >= NEED) break;
  }

  if (n < NEED) {
    return false;
  }

  for (uint8_t i = 0; i + (NEED - 1) < n; i++) {
    if (resp[i] != slaveId || resp[i + 1] != 0x03 || resp[i + 2] != 48) {
      continue;
    }
    uint16_t respCrc = (uint16_t)resp[i + NEED - 2] | ((uint16_t)resp[i + NEED - 1] << 8);
    if (respCrc != modbusCrc16(&resp[i], NEED - 2)) {
      imuChecksumFails++;
      continue;
    }
    const uint8_t* d = &resp[i + 3];
    int16_t rollRaw  = (int16_t)(((uint16_t)d[0] << 8) | d[1]);   // 0x3D
    int16_t pitchRaw = (int16_t)(((uint16_t)d[2] << 8) | d[3]);   // 0x3E
    int16_t yawRaw   = (int16_t)(((uint16_t)d[4] << 8) | d[5]);   // 0x3F
    // ★스케일 수정(2026-06-12): raw/32768×180 (기존 /100은 1.82배 과장 — 실기 대조 확정)
    outData.roll  = (float)rollRaw / 32768.0f * 180.0f;
    outData.pitch = (float)pitchRaw / 32768.0f * 180.0f;
    outData.yaw   = (float)yawRaw / 32768.0f * 180.0f;
    // 쿼터니언: 0x51-0x3D = 20레지스터 오프셋 → 데이터 바이트 40..47
    int16_t q0Raw = (int16_t)(((uint16_t)d[40] << 8) | d[41]);
    int16_t q1Raw = (int16_t)(((uint16_t)d[42] << 8) | d[43]);
    int16_t q2Raw = (int16_t)(((uint16_t)d[44] << 8) | d[45]);
    int16_t q3Raw = (int16_t)(((uint16_t)d[46] << 8) | d[47]);
    outData.q0 = (float)q0Raw / 32768.0f;
    outData.q1 = (float)q1Raw / 32768.0f;
    outData.q2 = (float)q2Raw / 32768.0f;
    outData.q3 = (float)q3Raw / 32768.0f;
    return true;
  }

  return false;
}

// 쿼터니언 → 오일러(ZYX, WitMotion 규약과 동일) 역산. 쿼터니언-온리 폴링시
// 오일러 레지스터를 안 읽으므로 PC 호환 키(roll/pitch/yaw)를 여기서 채운다.
void quatToEuler(ImuData& imu) {
  float q0 = imu.q0, q1 = imu.q1, q2 = imu.q2, q3 = imu.q3;
  // 역산은 단위 쿼터니언 가정 — 게이트(0.8~1.2)를 통과한 약간의 비정규도 정규화
  float nrm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (nrm > 1e-6f) {
    q0 /= nrm; q1 /= nrm; q2 /= nrm; q3 /= nrm;
  }
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  imu.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.29578f;
  imu.pitch = asinf(sinp) * 57.29578f;
  imu.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.29578f;
}

// 쿼터니언 유효성: 0 응답/깨진 값/시작주소 오정렬로 오일러 역산하지 않게 노름 가드.
// FC 지적: 과거 실측된 '‖q‖≈0.9 오답'(레지스터 오정렬)이 0.5~2.0 게이트는 통과 →
// n² 0.8~1.2로 조임 (WT901 정상 quat은 단위 ±LSB라 안전).
bool quatLooksValid(const ImuData& imu) {
  float n2 = imu.q0 * imu.q0 + imu.q1 * imu.q1 + imu.q2 * imu.q2 + imu.q3 * imu.q3;
  return n2 > 0.8f && n2 < 1.2f;
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
    imus[0].ax = (float)d0 / 32768.0f * 16.0f;
    imus[0].ay = (float)d1 / 32768.0f * 16.0f;
    imus[0].az = (float)d2 / 32768.0f * 16.0f;
  } else if (type == 0x52) {
    imus[0].wx = (float)d0 / 32768.0f * 2000.0f;
    imus[0].wy = (float)d1 / 32768.0f * 2000.0f;
    imus[0].wz = (float)d2 / 32768.0f * 2000.0f;
  } else if (type == 0x53) {
    imus[0].roll = (float)d0 / 32768.0f * 180.0f;
    imus[0].pitch = (float)d1 / 32768.0f * 180.0f;
    imus[0].yaw = (float)d2 / 32768.0f * 180.0f;
    return true;
  }

  return false;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // modem sleep OFF: DTIM 대기로 인한 UDP 수십ms 지터 제거
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
  char payload[1024];
  const ImuData& imu1 = imus[0];
  ImuData imu2;
  ImuData imu3;
  if (IMU_MODBUS_SLAVE_COUNT > 1) {
    imu2 = imus[1];
  }
  if (IMU_MODBUS_SLAVE_COUNT > 2) {
    imu3 = imus[2];
  }
  snprintf(
    payload,
    sizeof(payload),
    "{\"ts\":%lu,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"wx\":%.3f,\"wy\":%.3f,\"wz\":%.3f,\"roll\":%.3f,\"pitch\":%.3f,\"yaw\":%.3f,"
    "\"imu1_roll\":%.3f,\"imu1_pitch\":%.3f,\"imu1_yaw\":%.3f,"
    "\"imu2_roll\":%.3f,\"imu2_pitch\":%.3f,\"imu2_yaw\":%.3f,"
    "\"imu3_roll\":%.3f,\"imu3_pitch\":%.3f,\"imu3_yaw\":%.3f,"
    "\"imu1_q0\":%.5f,\"imu1_q1\":%.5f,\"imu1_q2\":%.5f,\"imu1_q3\":%.5f,"
    "\"imu2_q0\":%.5f,\"imu2_q1\":%.5f,\"imu2_q2\":%.5f,\"imu2_q3\":%.5f,"
    "\"imu3_q0\":%.5f,\"imu3_q1\":%.5f,\"imu3_q2\":%.5f,\"imu3_q3\":%.5f,"
    // D1a: 센서별 신선도(ms). PC가 한 IMU라도 stale이면 freeze (틀린자세 추종 방지).
    //   imuLastFrameMs=0(한 번도 성공 못함)이면 -1로 보내 '없음' 명시.
    "\"imu1_age\":%ld,\"imu2_age\":%ld,\"imu3_age\":%ld}",
    millis(),
    imu1.ax, imu1.ay, imu1.az,
    imu1.wx, imu1.wy, imu1.wz,
    imu1.roll, imu1.pitch, imu1.yaw,
    imu1.roll, imu1.pitch, imu1.yaw,
    imu2.roll, imu2.pitch, imu2.yaw,
    imu3.roll, imu3.pitch, imu3.yaw,
    imu1.q0, imu1.q1, imu1.q2, imu1.q3,
    imu2.q0, imu2.q1, imu2.q2, imu2.q3,
    imu3.q0, imu3.q1, imu3.q2, imu3.q3,
    (imuLastFrameMs[0] > 0 ? (long)(millis() - imuLastFrameMs[0]) : -1L),
    (IMU_MODBUS_SLAVE_COUNT > 1 && imuLastFrameMs[1] > 0 ? (long)(millis() - imuLastFrameMs[1]) : -1L),
    (IMU_MODBUS_SLAVE_COUNT > 2 && imuLastFrameMs[2] > 0 ? (long)(millis() - imuLastFrameMs[2]) : -1L)
  );

  udp.beginPacket(pcIP, udpPort);
  udp.write((const uint8_t*)payload, strlen(payload));
  udp.endPacket();

  // ★레이트 병목 수정: JSON(~400자) 시리얼 출력 = 115200서 ~35ms 블로킹/패킷!
  //   (9-14Hz의 주범이었음) → 2초에 1회만 디버그 출력
  static unsigned long lastPayloadDbgMs = 0;
  if (millis() - lastPayloadDbgMs >= 2000) {
    lastPayloadDbgMs = millis();
    Serial.println(payload);
  }
}

void sendStatusPacket() {
  // FC: 카운터 자릿수 최악 기준 581자 가능 — 512면 잘려서 PC json 파싱 무음 실패
  char payload[768];
  uint8_t imu1SlaveId = IMU_MODBUS_SLAVE_IDS[0];
  uint8_t imu2SlaveId = (IMU_MODBUS_SLAVE_COUNT > 1) ? IMU_MODBUS_SLAVE_IDS[1] : 0;
  uint8_t imu3SlaveId = (IMU_MODBUS_SLAVE_COUNT > 2) ? IMU_MODBUS_SLAVE_IDS[2] : 0;
  unsigned long imu2LastMs = (IMU_MODBUS_SLAVE_COUNT > 1) ? imuLastFrameMs[1] : 0;
  unsigned long imu3LastMs = (IMU_MODBUS_SLAVE_COUNT > 2) ? imuLastFrameMs[2] : 0;
  snprintf(
    payload,
    sizeof(payload),
    "{\"type\":\"status\",\"ts\":%lu,\"wifi\":%d,\"rssi\":%d,"
    "\"imu_available\":%d,\"imu_last_ms\":%lu,\"imu_baud\":%lu,"
    "\"imu1_slave\":%u,\"imu1_available\":%d,\"imu1_last_ms\":%lu,"
    "\"imu2_slave\":%u,\"imu2_available\":%d,\"imu2_last_ms\":%lu,"
    "\"imu3_slave\":%u,\"imu3_available\":%d,\"imu3_last_ms\":%lu,"
    "\"imu_last_good_slave\":%u,\"imu_bytes\":%lu,\"imu_frames\":%lu,"
    "\"imu_headers\":%lu,\"imu_csum_fail\":%lu,\"imu_poll_ok\":%lu,\"imu_poll_fail\":%lu,"
    "\"comb_ok\":%lu,\"comb_fail\":%lu,\"qo_ok\":%lu,\"qo_fail\":%lu}",
    millis(),
    (int)WiFi.status(),
    (int)WiFi.RSSI(),
    (lastImuFrameMs > 0 ? 1 : 0),
    lastImuFrameMs,
    currentImuBaud,
    imu1SlaveId,
    (imuLastFrameMs[0] > 0 ? 1 : 0),
    imuLastFrameMs[0],
    imu2SlaveId,
    (imu2LastMs > 0 ? 1 : 0),
    imu2LastMs,
    imu3SlaveId,
    (imu3LastMs > 0 ? 1 : 0),
    imu3LastMs,
    lastGoodSlaveId,
    imuBytesTotal,
    imuValidFrames,
    imuHeaderHits,
    imuChecksumFails,
    imuPollOk,
    imuPollFail,
    imuCombinedOk,
    imuCombinedFail,
    imuQuatOnlyOk,
    imuQuatOnlyFail
  );

  udp.beginPacket(pcIP, udpPort);
  udp.write((const uint8_t*)payload, strlen(payload));
  udp.endPacket();

  // 115200 USB시리얼에 ~450자 = ~40ms 블로킹 → 5초당 1회만 (UDP 전송은 매초 유지)
  static unsigned long lastStatusDbgMs = 0;
  if (millis() - lastStatusDbgMs >= 5000) {
    lastStatusDbgMs = millis();
    Serial.println(payload);
  }
}

void setup() {
  Serial.begin(115200);

  Serial.println("=== Firmware boot: esp32_imu_to_pc3 ===");
  Serial.print("IMU_MODBUS_SLAVE_COUNT=");
  Serial.println(IMU_MODBUS_SLAVE_COUNT);
  Serial.print("IMU_MODBUS_SLAVE_IDS=");
  for (uint8_t i = 0; i < IMU_MODBUS_SLAVE_COUNT; i++) {
    Serial.print("0x");
    if (IMU_MODBUS_SLAVE_IDS[i] < 16) {
      Serial.print("0");
    }
    Serial.print(IMU_MODBUS_SLAVE_IDS[i], HEX);
    if (i + 1 < IMU_MODBUS_SLAVE_COUNT) {
      Serial.print(",");
    }
  }
  Serial.println();

  if (RS485_RE_DE_PIN >= 0) {
    pinMode(RS485_RE_DE_PIN, OUTPUT);
    digitalWrite(RS485_RE_DE_PIN, LOW);
  }

  // WT901C-485 baud를 115200으로 설정 (이미 115200이면 응답없음 메시지 출력)
  configureImusToBaud115200();
  // 6축 모드 설정: 9600(승급 실패시)과 115200(승급 성공시) 둘 다 시도
  configureImus6Axis(115200);
  configureImus6Axis(9600);
  // 저지연 설정: 115200만 (보레이트 승급이 위에서 선행되므로 9600 폴백 불필요 —
  // 플러딩 복구 재시도 8회 × 9600 패스까지 돌리면 부팅이 수십초 길어짐)
  configureImusLowLatency(115200);

  beginImuSerialAt(0);  // 115200 baud (첫 번째 콴디데이트)

  // GPIO16 수신 핀 자체 테스트
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
    Serial.println("GPIO16 수신 0바이트 (rate silent 모드에선 정상 — 폴링 응답으로 판정할 것)");
  } else {
    Serial.println("GPIO16 수신 OK");
  }
  Serial.println("=== test end ===");

  connectWiFi();
  Serial.print("UDP target: ");
  Serial.print(pcIP);
  Serial.print(":");
  Serial.println(udpPort);
  // setup이 3초+ 걸려서 loop 첫 사이클에 보레이트 자동탐색이 즉시 발동하는 경합 방지:
  // 115200에 온전한 3초 윈도를 준다 (폴 1회 실패만으로 9600 전환되던 문제)
  lastBaudSwitchMs = millis();
  Serial.println("IMU streaming ready");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!USE_MODBUS_POLL) {
    static unsigned long lastDebugMs = 0;
    static uint8_t rxDebugBuf[16];
    static uint8_t rxDebugLen = 0;
    while (imuSerial.available() > 0) {
      uint8_t b = (uint8_t)imuSerial.read();
      imuBytesTotal++;

      if (rxDebugLen < sizeof(rxDebugBuf)) {
        rxDebugBuf[rxDebugLen++] = b;
      }
      if (millis() - lastDebugMs >= 200) {
        if (rxDebugLen > 0) {
          Serial.print("RX(");
          Serial.print(rxDebugLen);
          Serial.print("):");
          for (uint8_t i = 0; i < rxDebugLen; i++) {
            Serial.print(" ");
            Serial.print(rxDebugBuf[i], HEX);
          }
          Serial.println();
          rxDebugLen = 0;
        }
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
    // 115200 + 통합읽기: IMU당 ~7ms × 3 = ~21ms 사이클 → 폴 주기 5ms면 전송속도가 상한
    if (now - lastPollMs >= 5) {
      bool anyOk = false;
      // 10초마다 강등 센서 재프로브 (일시 글리치로 인한 영구 저속 방지 — FC)
      if (now - lastReprobeMs >= 10000) {
        lastReprobeMs = now;
        for (uint8_t k = 0; k < IMU_MODBUS_SLAVE_COUNT; k++) {
          quatOnlySupported[k] = true;
          quatOnlyFailStreak[k] = 0;
          combinedSupported[k] = true;
          combinedFailStreak[k] = 0;
        }
      }
      for (uint8_t i = 0; i < IMU_MODBUS_SLAVE_COUNT; i++) {
        uint8_t slaveId = IMU_MODBUS_SLAVE_IDS[i];
        // 죽은 센서 백오프: 500ms간 스킵 → 멀쩡한 센서들은 고레이트 유지 (FC)
        if (now < sensorBackoffUntil[i]) {
          continue;
        }
        bool ok = false;
        // 1차: 쿼터니언-온리(13B 응답, IMU당 ~3.5ms 단축) + 오일러 ESP 역산.
        //   임시본에 폴링: 무효 quat(CRC 통과+노름 불량)가 imus[i]를 오염 못 하게 (FC)
        if (quatOnlySupported[i]) {
          ImuData tmp = imus[i];
          ok = pollWt901c485Quat(slaveId, tmp) && quatLooksValid(tmp);
          if (ok) {
            quatToEuler(tmp);
            imus[i] = tmp;
            imuQuatOnlyOk++;
            quatOnlyFailStreak[i] = 0;
          } else {
            imuQuatOnlyFail++;
            if (++quatOnlyFailStreak[i] >= 5) {
              quatOnlySupported[i] = false;  // 10초 후 재프로브로 복귀
            }
          }
        }
        // 2차: 통합읽기(오일러+쿼터니언 24레지스터)
        if (!ok && combinedSupported[i]) {
          ok = pollWt901c485Combined(slaveId, imus[i]);
          if (ok) {
            imuCombinedOk++;
            combinedFailStreak[i] = 0;
          } else {
            imuCombinedFail++;
            if (++combinedFailStreak[i] >= 5) {
              combinedSupported[i] = false;  // 10초 후 재프로브로 복귀
            }
          }
        }
        // 3차: 개별 폴링 폴백 (quat은 노름 가드 통과 시에만 커밋)
        if (!ok) {
          ok = pollWt901c485Angle(slaveId, imus[i]);
          if (ok) {
            ImuData tq = imus[i];
            if (pollWt901c485Quat(slaveId, tq) && quatLooksValid(tq)) {
              imus[i] = tq;
            }
          }
        }
        // 전 경로 실패 → 500ms 백오프 (타임아웃 캐스케이드 차단)
        if (!ok) {
          sensorBackoffUntil[i] = now + 500;
        }
        if (ok) {
          imuLastFrameMs[i] = now;
          lastImuFrameMs = now;
          imuValidFrames++;
          imuPollOk++;
          lastGoodSlaveId = slaveId;
          anyOk = true;
        } else {
          imuPollFail++;
        }
      }

      if (anyOk) {
        sendImuPacket();
      }
      lastPollMs = now;
    }
  }

  unsigned long now = millis();

  // poll 성공 전까지는 모드에 따라 보레이트 자동 탐색
  if (lastImuFrameMs == 0 && now - lastBaudSwitchMs >= 3000) {
    uint8_t nextIdx = (uint8_t)((imuBaudIdx + 1) % IMU_BAUD_COUNT);
    beginImuSerialAt(nextIdx);
    while (imuSerial.available() > 0) {
      (void)imuSerial.read();
    }
    lastBaudSwitchMs = now;
  }

  if (now - lastStatusMs >= 1000) {
    sendStatusPacket();
    lastStatusMs = now;
  }
}
