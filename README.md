# TV_Smart_Factory

## 개발 환경
- Ubuntu 20.04, Raspberry Pi 5
- Python 3.8
- Dobot Magician
- Intel RealSense D435i
- YOLOv5
- ROS2 Humble
- RoboDk
## 프로젝트 동작도
![프로젝트_동작도](https://github.com/user-attachments/assets/cb6091a5-afcc-4ed8-88c9-2e3c6d89e86a)
## 주요 기능
1. Dobot Magician 제어
   - Dobot이 지정된 위치로 이동할 수 있도록 위치 티칭 작업과 Suction Cup을 사용해 Pick & Place 모션을 구현합니다.
   - 사용자가 작동 명령을 내리면 Dobot Magician 로봇 암이 Panel을 탐지 위치로 옮깁니다.
3. 객체 탐지
   - Dobot Magician을 통해 Panel들이 탐지 위치로 이동합니다.
   - Intel RealSense D435i를 통해 실시간 이미지 데이터를 받아옵니다.
   - YOLOv5를 통해 Back Panel과 Board Panel을 분류되며, 이때 
4. 컨베이어 벨트 제어
