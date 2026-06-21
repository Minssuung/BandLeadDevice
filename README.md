# BandLeadDevice (밴드형 리드 디바이스)

IMU 3개를 팔(윗팔·아랫팔·손)에 차고 사람 팔 자세를 측정해 **7DOF 로봇팔(OpenArm)을 실시간 추종**시키는 착용형 텔레오퍼레이션("리드") 디바이스. 핑크랩 **케어링** 프로젝트의 모방학습 데이터 수집용.

## 시스템 구성 · 데이터 흐름

```
IMU×3 (WT901C485, RS485/Modbus) ─► ESP32(LOLIN D32) ─Wi-Fi UDP :4210─► PC ──► 로봇팔
   윗팔 / 아랫팔 / 손                (MAX485, 5V 배터리)         bridge.calib_ik   JointTrajectory(ROS2)
```

- 윗팔 IMU → 어깨 3DOF, 아랫팔 IMU → 팔꿈치 1DOF, 손 IMU → 손목 3DOF, 버튼 → 그리퍼.
- PC: 인접 IMU 상대 쿼터니언 → **swing-twist 시드 + 멀티스타트 LM IK**로 관절각 분해 → 부호/매핑·관절한계 클램프 → 발행.

## 하드웨어

- IMU ×3: WitMotion **WT901C485** (RS485, 직렬 버스 → 주소 `0x50/0x51/0x52` 개별 설정 필요)
- MCU: **LOLIN D32** (ESP32-WROOM-32) + RS485 변환 **MAX485**
- 입력: KY-023 조이스틱, 홀센서+자석(그리퍼), 마이크로스위치(리프트), 택트버튼(토크 OFF)
- 대상 로봇: **OpenArm v1** (검증 단계는 DYNAMIXEL XL430/XL330 "미니 팔로워" 리그 사용)

## 디렉터리 구조

```
openarm_teleop_adapter.py   IMU → 실물 OpenArm(ROS2 JointTrajectory) 텔레옵 어댑터 (메인)
viz/                        three.js + URDF 실시간 뷰어 & bridge.py(calib_ik 분해)
sim_mujoco/                 MuJoCo 물리 검증
work_process/               개발 로그·테스트 보고서·ESP 펌웨어(esp_test/)·제어 테스트
  ├─ work_process.md          개발 프로세스 개요
  ├─ teleop_개발로그.md        기술 해결 로그(짐벌락·IK·캘리브)
  └─ imu_arm_mapping_table.md  IMU↔관절 매핑표
DynamixelSDK/               벤더 SDK (미니 팔로워 구동용)
requirements.txt
```

## 설치

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt     # PyQt5, pyserial  (DynamixelSDK는 ./DynamixelSDK/python/src 를 .pth로 로드)
```

ESP32 펌웨어는 `work_process/esp_test/`의 `.ino`를 Arduino IDE로 업로드.
IMU 주소는 `esp32_wt901_set_address_once.ino`로 `0x50/0x51/0x52` 지정(직렬 버스 충돌 방지).

## 실행 (OpenArm 텔레옵)

ROS2 Jazzy + `caring_openarm`(형제 디렉터리 `~/dev_ws/caring_openarm`) 필요:

```bash
source /opt/ros/jazzy/setup.bash
source ~/dev_ws/caring_openarm/install/setup.bash
export ROS_DOMAIN_ID=34

# DRY-RUN: 실물 안 움직이고 매핑/부호만 검증
~/dev_ws/caring_openarm/.venv/bin/python openarm_teleop_adapter.py --side right

# 검증 후 실제 저속 구동
~/dev_ws/caring_openarm/.venv/bin/python openarm_teleop_adapter.py --side right --execute --scale 0.5 --exec-time 0.4
```

> 현재 경로가 `~/dev_ws/BandLeadDevice` + 형제 `caring_openarm` 기준으로 일부 하드코딩돼 있음(어댑터 상단 `_BL`).

## 핵심 성과 (검증)

- 종단 지연 250~300ms → **113ms**, 갱신율 30.8 → **54Hz**
- 추종 정확도(어깨) 14~15° → **3.1°**, IK 계산 실패율 8.8% → **0%**
- 짐벌락 / IK 폭주(j6 +2326°) → 센서 쿼터니언 직접읽기(0x51) + 워밍스타트 제거로 **감김 구조적 불가**
- 손목 이상회전 = 지자기 간섭 규명 → **9축(나침반 ON) 확정**

## 안전

DRY-RUN 기본, `--execute` 시에만 실구동 · 관절한계 클램프 · 속도제한 180°/s · stale-freeze 워치독 · 시작 점프 방지.
⚠️ OpenArm은 **토크 OFF 시 낙하**(미니 팔로워와 반대) → 위치유지/홈복귀 정책 필요.

## 관련 문서

- 케어링 Confluence: `기술검토 > 리드 디바이스 / Bandi 장비 및 성능 검토` (정본)
- 상세 개발 기록: `work_process/` 하위 `.md` (일일보고·테스트플랜·검증보고서)
