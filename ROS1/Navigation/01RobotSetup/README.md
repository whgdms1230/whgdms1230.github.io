---
sort: 1
---

# RobotSetup

<img src="RobotSetup.png"  width="900" height="400">

위 다이어그램은 Navigation 구성 요소들을 나타낸다. 따라서 Navigation을 운용하기 위해서는 기본적으로 다음을 필요로 한다.

{% include list.liquid all=true %}

## 하드웨어 요구사항
1. Differential Drive 형태의 로봇 전용
2. Laser 또는 Lidar 등 Mapping과 Localization에 사용될 수 있는 센서
3. 직사각형 또는 원형의 단순한 모형으로 단순화하여 제어되므로, 공간에 비해 로봇의 크기가 크면 제한됨