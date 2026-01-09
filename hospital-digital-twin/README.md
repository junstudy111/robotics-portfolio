# Digital Twin 기반 검체 무인 이송 로봇

고위험 병원 환경에서의 안전성 검증 및 로봇 제어 최적화를 위한 디지털 트윈 시스템

## 📌 프로젝트 개요
- **기간:** 2024.12.08 ~ 12.19
- **팀:** 4명
- **역할:** Vision & UI & FastAPI 브릿지

## 🛠 Tech Stack
- Python
- ROS2
- ArUco Marker
- FastAPI
- OpenCV
- Digital Twin (ISAAC SIM)

## 📁 Packages
- `hospital_interfaces/` - 커스텀 메시지 & 서비스 정의
- `hospital_robot_ui/` - UI Node & FastAPI 브릿지
- `my_pkg/` - 추가 패키지

## ✨ 핵심 기여

### 1. ArUco 기반 6D Pose 추정
- 마커 인식을 통한 정확한 위치 추정
- 자율 파지 시퀀스 설계
- base_link 기준 좌표 변환

### 2. ROS2-FastAPI 통합 관제 시스템
- 웹 대시보드 실시간 통신
- 로봇 상태 모니터링
- 예외 처리 로직 구현

### 3. Mock-up 기반 독립 검증
- 모듈별 독립 테스트
- 통합 지연 문제 해결

## 🎯 주요 성과
- ArUco 마커 기반 정확한 파지 구현
- ROS2-FastAPI 브릿지 안정적 동작
- 모듈 독립 검증 체계 구축

## 💡 배운 점
- 인식 데이터와 제어 시퀀스 동기화
- 독립적 모듈 검증의 중요성
- 좌표 변환 및 행렬 연산 실전 적용
