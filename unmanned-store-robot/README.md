# 무인 매장 AI 로봇 시스템

음성 명령으로 상품을 찾아 전달하는 End-to-End 로봇 시스템

## 📌 프로젝트 개요
- **기간:** 2024.11.24 ~ 12.05
- **팀:** 6명
- **역할:** LLM 통합 & 로봇 제어 노드

## 🛠 Tech Stack
- Python
- ROS2
- LLM (GPT)
- STT (Speech-to-Text)
- Doosan Manipulator

## 📁 Packages
- `od_msg/` - 객체 감지 메시지
- `pick_and_place_text/` - 텍스트 기반 제어
- `pick_and_place_voice/` - 음성 기반 제어 (나의 역할)
- `rokey/` - 로봇 기본 패키지

## ✨ 핵심 기능
- STT → LLM → 로봇 명령 연계
- 음성 기반 상품 탐색
- 자동 Pick & Place
- Intent 분석 및 로봇 제어
