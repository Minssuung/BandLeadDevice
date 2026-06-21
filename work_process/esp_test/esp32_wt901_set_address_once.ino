#include <Arduino.h>

// ESP32 (LOLIN D32) + MAX485 wiring
// RO -> RX2 (GPIO16)
// DI -> TX2 (GPIO17)
// RE+DE -> GPIO21 (HIGH: TX, LOW: RX)
static const int IMU_RX_PIN = 16;
static const int IMU_TX_PIN = 17;
static const int RS485_RE_DE_PIN = 21;

// Default flow: 0x50 -> 0x51 at 9600 baud
static const uint32_t IMU_BAUD = 9600;
static const uint8_t OLD_ADDR = 0x50;
static const uint8_t NEW_ADDR = 0x51;

// Known candidate registers for WT901 address change
static const uint16_t ADDR_REGS[] = {
  0x001A,
  0x00FF,
  0x1001,
};

// Save config register/value used by many WT901 variants
static const uint16_t SAVE_REG = 0x0000;
static const uint16_t SAVE_VAL = 0x0000;
static const uint16_t UNLOCK_REG = 0x0069;
static const uint16_t UNLOCK_KEY = 0xB588;

HardwareSerial imuSerial(2);

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
  digitalWrite(RS485_RE_DE_PIN, tx ? HIGH : LOW);
}

void rs485Send(const uint8_t* data, uint8_t len) {
  while (imuSerial.available() > 0) {
    (void)imuSerial.read();
  }

  setRs485TxMode(true);
  delayMicroseconds(800);
  imuSerial.write(data, len);
  imuSerial.flush();
  delayMicroseconds(800);
  setRs485TxMode(false);
}

uint8_t readResp(uint8_t* buf, uint8_t maxLen, uint16_t waitMs) {
  uint8_t n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < waitMs) {
    while (imuSerial.available() > 0 && n < maxLen) {
      buf[n++] = (uint8_t)imuSerial.read();
    }
  }
  return n;
}

bool hasValidFrame(const uint8_t* resp, uint8_t n, uint8_t slaveId, uint8_t fc, uint8_t frameLen) {
  if (n < frameLen) {
    return false;
  }

  for (uint8_t i = 0; i + frameLen <= n; i++) {
    if (resp[i] != slaveId || resp[i + 1] != fc) {
      continue;
    }

    uint16_t crcRecv = (uint16_t)resp[i + frameLen - 2] | ((uint16_t)resp[i + frameLen - 1] << 8);
    uint16_t crcCalc = modbusCrc16(&resp[i], frameLen - 2);
    if (crcRecv == crcCalc) {
      return true;
    }
  }
  return false;
}

bool pingReadAngle(uint8_t slaveId) {
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x03;
  req[2] = 0x00;
  req[3] = 0x3D; // roll/pitch/yaw start
  req[4] = 0x00;
  req[5] = 0x01; // read one register for quick ping
  uint16_t crc = modbusCrc16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  rs485Send(req, sizeof(req));

  uint8_t resp[64];
  uint8_t n = readResp(resp, sizeof(resp), 120);
  return hasValidFrame(resp, n, slaveId, 0x03, 7);
}

bool pingReadAngleRetry(uint8_t slaveId, uint8_t tries, uint16_t gapMs) {
  for (uint8_t i = 0; i < tries; i++) {
    if (pingReadAngle(slaveId)) {
      return true;
    }
    delay(gapMs);
  }
  return false;
}

bool writeSingleRegister(uint8_t slaveId, uint16_t reg, uint16_t value) {
  uint8_t req[8];
  req[0] = slaveId;
  req[1] = 0x06;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = (uint8_t)(value >> 8);
  req[5] = (uint8_t)(value & 0xFF);
  uint16_t crc = modbusCrc16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  rs485Send(req, sizeof(req));

  uint8_t resp[64];
  uint8_t n = readResp(resp, sizeof(resp), 180);
  if (hasValidFrame(resp, n, slaveId, 0x06, 8)) {
    return true;
  }

  // Some devices do not echo reliably after config writes.
  return n == 0;
}

bool saveConfig(uint8_t addr) {
  bool any = false;
  // 모델별로 save 명령 해석이 다를 수 있어 여러 패턴을 순차 시도
  any = writeSingleRegister(addr, SAVE_REG, SAVE_VAL) || any;
  any = writeSingleRegister(addr, SAVE_REG, 0x0001) || any;
  any = writeSingleRegister(addr, 0x0001, 0x0000) || any;
  any = writeSingleRegister(addr, 0x0001, 0x0001) || any;
  return any;
}

bool unlockConfig(uint8_t addr) {
  return writeSingleRegister(addr, UNLOCK_REG, UNLOCK_KEY);
}

bool tryChangeAddress(uint8_t oldAddr, uint8_t newAddr) {
  Serial.printf("[TRY] unlock cfg reg 0x%04X <- 0x%04X via old addr 0x%02X ... ",
                UNLOCK_REG, UNLOCK_KEY, oldAddr);
  bool unlockOk = unlockConfig(oldAddr);
  Serial.println(unlockOk ? "DONE" : "NO ECHO");
  delay(100);

  for (uint8_t i = 0; i < sizeof(ADDR_REGS) / sizeof(ADDR_REGS[0]); i++) {
    uint16_t reg = ADDR_REGS[i];
    Serial.printf("[TRY] write addr reg 0x%04X <- 0x%02X ... ", reg, newAddr);

    if (!writeSingleRegister(oldAddr, reg, newAddr)) {
      Serial.println("FAIL");
      continue;
    }

    Serial.println("OK");
    delay(150);

    bool oldOkAfterWrite = pingReadAngle(oldAddr);
    bool newOkAfterWrite = pingReadAngle(newAddr);
    Serial.printf("[CHK] after write old=0x%02X:%s  new=0x%02X:%s\n",
                  oldAddr, oldOkAfterWrite ? "OK" : "NO",
                  newAddr, newOkAfterWrite ? "OK" : "NO");

    // 주소가 즉시 바뀌는 모델이 있어 저장 명령은 새 주소를 우선 사용한다.
    if (newOkAfterWrite) {
      Serial.printf("[TRY] save via new addr 0x%02X ... ", newAddr);
      bool s1 = saveConfig(newAddr);
      Serial.println(s1 ? "DONE" : "NO ECHO");

      // Flash commit 여유를 주기 위해 save를 반복하고 충분히 대기
      delay(300);
      (void)saveConfig(newAddr);
      delay(300);
      (void)saveConfig(newAddr);
      delay(1200);

      // 일부 모델은 old 주소에서도 save를 한 번 더 받아야 영구 반영됨
      if (pingReadAngleRetry(oldAddr, 2, 80)) {
        Serial.printf("[TRY] extra save via old addr 0x%02X ... ", oldAddr);
        bool sx = saveConfig(oldAddr);
        Serial.println(sx ? "DONE" : "NO ECHO");
        delay(500);
      }

      // 즉시 relock이 commit을 방해하는 모델이 있어 여기서는 relock하지 않음
      return true;
    }

    if (oldOkAfterWrite) {
      Serial.printf("[TRY] save via old addr 0x%02X ... ", oldAddr);
      bool s2 = saveConfig(oldAddr);
      Serial.println(s2 ? "DONE" : "NO ECHO");
      delay(300);
      (void)saveConfig(oldAddr);
      delay(1200);

      // 일부 모델은 저장 직후 주소가 적용되므로 다시 확인
      delay(200);
      if (pingReadAngle(newAddr)) {
        return true;
      }
      continue;
    }

    // 둘 다 응답이 없으면 재부팅 중일 수 있어 다음 후보로 바로 넘어가지 않음
    delay(400);
    if (pingReadAngle(newAddr)) {
      Serial.printf("[INFO] new addr 0x%02X responded after short wait\n", newAddr);
      (void)saveConfig(newAddr);
      delay(1200);
      return true;
    }
  }

  return false;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(RS485_RE_DE_PIN, OUTPUT);
  setRs485TxMode(false);

  imuSerial.begin(IMU_BAUD, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
  delay(150);

  Serial.println();
  Serial.println("========================================");
  Serial.println(" WT901 RS485 one-shot address changer");
  Serial.println("========================================");
  Serial.printf("Target: 0x%02X -> 0x%02X @ %lu baud\n", OLD_ADDR, NEW_ADDR, (unsigned long)IMU_BAUD);
  Serial.println("IMPORTANT: connect only IMU2 on the RS485 bus.");

  Serial.printf("[STEP1] ping old addr 0x%02X ... ", OLD_ADDR);
  bool oldOk = pingReadAngle(OLD_ADDR);
  Serial.println(oldOk ? "OK" : "NO RESPONSE");

  if (!oldOk) {
    Serial.println("Cannot confirm old address. Check power, wiring, baud, and bus isolation.");
    return;
  }

  Serial.printf("[STEP2] change address 0x%02X -> 0x%02X\n", OLD_ADDR, NEW_ADDR);
  bool changed = tryChangeAddress(OLD_ADDR, NEW_ADDR);
  Serial.println(changed ? "Address change sequence sent." : "All address register attempts failed.");

  Serial.println("[STEP3] wait 5 seconds for sensor reboot...");
  delay(5000);

  Serial.printf("[STEP4] ping new addr 0x%02X ... ", NEW_ADDR);
  bool newOk = pingReadAngleRetry(NEW_ADDR, 5, 150);
  Serial.println(newOk ? "OK" : "NO RESPONSE");

  if (newOk) {
    Serial.println("SUCCESS: address change confirmed.");
    Serial.println("Now restore slave list in esp32_imu_to_pc2.ino to {0x50, 0x51}.");
  } else {
    Serial.printf("[STEP4b] ping old addr 0x%02X ... ", OLD_ADDR);
    bool oldStillOk = pingReadAngleRetry(OLD_ADDR, 5, 150);
    Serial.println(oldStillOk ? "OK" : "NO RESPONSE");

    Serial.println("FAILED: new address did not respond.");
    if (oldStillOk) {
      Serial.println("Old address is still active: write/save register map is likely different for this model.");
    } else {
      Serial.println("Neither old nor new responded: check power/wiring and rerun after power-cycle.");
    }
  }

  Serial.println("One-shot run complete. loop() does nothing.");
}

void loop() {
  delay(1000);
}
