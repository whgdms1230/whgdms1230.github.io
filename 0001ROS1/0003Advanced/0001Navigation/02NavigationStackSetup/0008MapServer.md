---
sort: 8
---

# map_server

## 0. 참고 문헌

*- [map_server 위키페이지](http://wiki.ros.org/map_server)*

## 1. map_server
map_server는 지도 데이터를 제공하는 ROS 노드이며, map_saver를 통해 맵 파일을 저장하는 역할도 수행한다.

## 2. Map Format
맵은 YAML파일과 이미지파일을 한 쌍의 파일 형태로 저장된다. YAML파일은 맵의 메타정보를 저장하고, 이미지 파일의 이름을 저장한다. 이미지파일은 점유데이터를 가져온다.

### 2.1 Image Format
이미지파일은 각 픽셀의 색상으로 장애물을 나타낸다. 흰섹 픽셀은 장애물이 없는 공간이고, 검은색 픽셀은 장애물이 점유하고 있는 공간이다. 그 이외의 색깔의 픽셀은 알 수 없는 공간으로 표현된다.

### 2.2 YAML Format
YAML 파일의 형태의는 다음과 같다.
```
image: testmap.png
resolution: 0.1
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```
* image는 이미지 파일의 경로를 나타낸다.
* resolution은 맵의 해상도를 나타낸다. meter/pixel
* origin은 맵의 x, y, yaw 초기 위치를 나타낸다. (0, 0)은 지도의 중심, yaw는 회전방향
* occupied_thresh는 이 값보다 큰 픽셀은 장애물임을 나타내는 값이다.
* free_thresh는 이 값보다 작은 픽셀은 장애물이 없는 공간임을 나타내는 값이다.
* negate는 흰색/검은색 픽셀의 반전 여부를 나타내는 것으로 thresholds 값은 영향을 받지 않는다.

## 3. Command-line Tools

### 3.1 map_server
map_server는 ROS 서비스를 통해 맵을 읽어오는 노드이며, 다음과 같이 맵을 불러올 수 있다.
```bash
rosrun map_server map_server map.yaml
```

#### 3.1.1 Published Topics
* map_metadata([nav_msgs/MapMetaData](http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html)) : 이 메세지를 통해 맵의 메타 정보를 가져옴
* map([nav_msgs/OccupancyGrid](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html)) : 이 메세지를 통해 맵을 받아옴

#### 3.1.2 Services
* static_map([nav_msgs/GetMap](http://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html)) : 지도를 가져옴

#### 3.1.3 Parameters
* ~frame_id(string, default: "map") : 퍼블리쉬 된 맵의 앞의 frame 이름을 설정

### 3.2 map_saver
map_saver는 맵을 저장하는데 사용되며, SLAM으로 그린 맵을 저장하는데 주로 사용된다.
```bash
rosrun map_server map_saver [-occ <threshold_occupied>] [--free <threshold_free>] [-f <mapname>] map:=/your/costmap/topic
```

#### 3.2.1 Subscribed Topics
* map([nav_msgs/OccupancyGrid](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html)) : 이 메세지를 통해 맵을 검색