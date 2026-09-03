
# 🏔️ 산지기 (SanJigi)
> Off-Grid 기반 산악 위치 추적 및 구조 시스템
> *Mountain Emergency Location Tracking & Rescue System for Off-Grid Environments*

---

## 1. 프로젝트 개요 (Project Overview)

산지기(SanJigi)는 GPS가 연결되지 않는 산악 음영 지역(Off-Grid)에서 등산객의 안전을 확보하기 위해 개발된 **LoRa 기반 산악 위치 추적 시스템**입니다.

기압 센서 기반의 고도 변화 데이터를 실시간 수집·분석하여, 독립적인 RF 네트워크를 통해 위치 좌표를 전송합니다.

---

## 2. 핵심 기능 (Key Features)

- **Off-Grid LoRa 장거리 통신**
  - 기지국 없는 산악 음영 지역에서도 비상 통신망을 구축하여 위치 데이터 및 SOS 신호 전송
- **기압 기반 z축 위치 보정**
  - 기압 센서를 활용한 고도 변화 모니터링으로 3D거리 보정
- **저전력 시스템 설계 (Power Optimization)**
  - MCU Deep Sleep 및 인터럽트를 적용하여 야외 산악 환경에서 장시간 동작 가능

---

## 3. 시스템 아키텍처 (System Architecture)
<img width="6564" height="2260" alt="Image" src="https://github.com/user-attachments/assets/9b7b39a8-5609-4d04-ab3e-6ad79ebdc6e6" />

---

## 4. 하드웨어 및 개발 환경 (Hardware & Tech Stack)

### Hardware
- **MCU**: ESP32-S3
- **LoRa Module**: SX1262 Transceiver
- **Sensors**: BMP280
- **Power**: Li-Ion

### Software & Firmware
- **Development Environment**: VS Code + PlatformIO
- **Language**: C++
- **Protocols**: LoRa RF, SPI, I2C


## License

Copyright (c) 2026 SanJigi Team. All rights reserved.
