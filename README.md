# Smart Factory Solution for TV Manufacturing***
## 개발 환경
- **운영체제** : Ubuntu 20.04 
- **프로그래밍 언어** : Python 3.8  
- **하드웨어** :  
  - Dobot Magician  
  - Intel RealSense D435i
  - Raspberry Pi 5 
- **소프트웨어 및 프레임워크** :  
  - YOLOv5  
  - ROS2 Humble  
  - RoboDK

## 프로젝트 동작도
![프로젝트_동작도](https://github.com/user-attachments/assets/cb6091a5-afcc-4ed8-88c9-2e3c6d89e86a)

## 주요 기능
1. Dobot Magician 제어
   - Dobot이 지정된 위치로 이동할 수 있도록 위치 티칭 작업과 Suction Cup을 사용해 Pick & Place 모션을 구현합니다.
   - 사용자가 작동 명령을 내리면 Dobot Magician 로봇 암이 Panel을 탐지 위치로 옮깁니다.

  <img src = "https://github.com/Junoquu/TV_Smart_Factory/blob/main/image/%EB%B0%B1%ED%8C%90%EB%84%AC.gif">
  <img src = "https://github.com/Junoquu/TV_Smart_Factory/blob/main/image/%EB%B3%B4%EB%93%9C%ED%8C%90%EB%84%AC.gif">

2. 객체 탐지
   - Dobot Magician을 통해 Panel들이 탐지 위치로 이동합니다.
   - Intel RealSense D435i를 통해 실시간 이미지 데이터를 받아옵니다.
   - YOLOv5를 통해 Back Panel과 Board Panel을 탐지하며, 컨베이어 벨트와 분류기를 통해 분류됩니다.
  
  - Panel 탐지가 안되었을 경우
  <img src = "https://github.com/Junoquu/TV_Smart_Factory/blob/main/image/realsense_yolo%20(Unknown%20%ED%83%90%EC%A7%80).png">

  - Panel 탐지가 되었을 경우
  <img src = "https://github.com/Junoquu/TV_Smart_Factory/blob/main/image/realsense_yolo%20(Panel%20%ED%83%90%EC%A7%80).png">
  

3. 컨베이어 벨트 제어
   - Raspberry Pi GPIO를 사용해 스텝 모터와 서보 모터를 제어하여 Panel들을 이동 및 분류 합니다.
  <img src = "https://github.com/Junoquu/TV_Smart_Factory/blob/main/image/%EC%98%81%EC%83%81%EC%B2%98%EB%A6%AC.gif">

4. 소켓 통신
   - Panel 데이터를 실시간 수집하고 로봇 암 작업과 컨베이어 벨트 작업을 제어합니다.
