---
sort: 8
---

# Navigation Tunning Guide

## 0. 참고 문헌

*- [Navigation Tunning Guide 위키 페이지](http://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide)*

본 내용은 Kaiyu Zheng의 [ROS Navigation Tunning Guide](https://kaiyuzheng.me/documents/navguide.pdf)를 기반으로 작성하였다.

## 1. Velocity and Acceleration
먼저, ROS Navigation에서 로봇의 주행의 실질적인 데이터 형태인 속도와 가속도의 설정에 대한 내용이다. local planner에서 받는 토픽인 `odom`과 그 결과로 나오는 토픽인 `cmd_vel`은 모두 속도 및 가속도 기반의 토픽이기 때문이다.

### 1.1 Maximum Velocity의 선정
실제 환경에서 로봇이 직선운동 및 제자리에서 회전운동 시의 최대 속도를 기준으로 선정하며, Safety를 위해서 실제 최대 속도보다 약간 낮은 값으로 parameter를 선정한다.

### 1.2 Maximum Accleceration의 선정
최대 가속도 측정하기 위해, 속도가 0에서 최대 속도까지 도달하는데 시간을 기준으로 선정한다.

### 1.3 Minimum Value 선정
DWA 기준으로 설명하면, 선속도의 경우 Minimum velocity이 음수일 경우 후진을 하며, 0일 경우 후진을 하지 않는다고 볼 수 있다. 회전 속도의 경우 절댓값을 취하기 때문에 최소 회전 속도는 음수로 지정하지 않아도 된다.

### 1.4 x, y 방향 Velocity
x 및 y 방향의 선속도를 나타내며, x 방향은 로봇의 주행방향이고, y 방향은 로봇 주행 방향에서 왼쪽으로 90도 방향이다. 일반적으로 differential wheeled robot의 경우 y 방향의 속도값은 사용하지 않는다.

## 2. Global Planner

### 2.1 Global Planner의 선정
move_base에서는 3가지 타입의 Global Planner를 제공한다.
* carrot_planner
* navfn
* global_planner

이 중에서 이번 장에서는 global_planner에 대해 서술한다.

### 2.2 global_planner parameter
global_planner는 파라미터 선정을 통해 Dijkstra, A*, Standard Behavior, Grid Path 와 같은 planning 방법을 선택할 수 있다.
추가적으로 경로 생성 시 cost를 계산에 사용되는 parameter로, `cost_factor`, `neutral_cost`, `lethal_cost`가 있다. 다음은 cost를 계산하는 식으로, `costmap_cost_value`는 0에서 252로 나타나는 costmap의 cost 값이다.
```
cost = COST_NEUTRAL + COST_FACTOR * costmap_cost_value
```
`cost_factor`, `neutral_cost`가 너무 낮거나 높으면 경로 생성의 수준이 낮아지므로, 적당한 수준으로 설정해야 한다. `lethal_cost`의 값이 낮게 설정할 경우 feasible한 경로를 찾더라도, 경로 생성이 되지 않을 수 있으므로 주의해야 한다.

## 3. Local Planner

### 3.1 Dynamic Window Approach(DWA)

DWA의 동작 단계를 다음과 같이 정의할 수 있다.
1. 로봇의 제어 공간 내에서 속도(dx, dy, dtheta)에 대한 샘플을 취한다.
2. 샘플로 선정한 각각의 속도에 대해서 시뮬레이션을 한다.
3. 각 샘플에 대한 시뮬레이션을 통해 샘플에 대한 점수를 부여한다.
4. 가장 높은 점수를 받은 궤적을 선정한다.
5. 위 과정을 목적지에 도착할 때까지 계속 반복한다.

위의 동작을 수행하기 위해 DWA를 사용하는 데 있어 설정해야 할 parameter들에 대해서 정리한다.

### 3.1.1 Forward Simulation
local planner가 속도 샘플을 통해 simulation 하는 과정에 필요한 parameter들을 정리한다.
`sim_time`은 simulation 시간을 말하며, `sim_time`이 낮을 경우에는 simulation 과정이 짧으므로 좁은공간을 지나는 경우에 성능이 떨어질 수 있다. 반면 높게 설정되는 경우 simulation을 통해 생성되는 경로가 유연하지 않아 좁은 공간에서 충돌을 회피하거나 경로를 재생성 하는데 좋지 않은 영향을 줄 수 있다.

다음으로 `vx_sample`, `vy_sample`, `vth_sample`은 각 방향의 sample 갯수를 설정하는 parameter로 sample의 갯수는 많으면 좋지만 computing power에 따라 적절하게 선정해야 한다.

`sim_granularity`는 경로의 점 사이의 간격 크기를 설정한다. 이는 경로의 각 점이 장애물과 교차하는지 여부를 판단하며, 값이 작을 수록 많은 수의 검사가 필요하므로 이도 computing power에 따라 적절하게 선정해야 한다.

### 3.1.2 Trajectory Scoring
simulation을 통해서 여러 개의 경로 중 가장 점수가 높은 경로를 선택하게 되는데, 경로의 점수를 부여하는 식은 다음과 같다.

```
cost
= path_distance_bias*(distance to path from the endpoint of the trajectory)
+ goal_distance_bias*(distance to local goal from the endpoint of the trajectory)
+ occdist_scale*(maximum obstacle cost along the trajectory in obstacle cost(0-254))
```

따라서 `path_distance_bias`, `goal_distance_bias`, `occdist_scale`을 어떻게 선정하는가에 따라 다르게 동작하게 된다.
`path_distance_bias`는 값이 높을 수록 global_planner에 의해 생성된 경로에서 벗어나지 않게 local planer을 하게 한다. `goal_distance_bias`는 경로에 상관 없이 local goal에 다가갈 수 있도록 한다. `occdist_scale`은 장애물을 피하기 위한 동작 수준을 결정한다.

### 3.1.3 Goal Distance Tolerance
해당 파라미터들은 목적지에 도착했을 때 도착 여부에 대한 판단 기준을 내리는 파라미터들이다.
`yaw_goal_tolerance`와 `xy_goal_tolerance`는 yaw 방향과 x,y 방향에 대해 오차 범위를 설정하는 파라미터이고, `latch_xy_goal_tolerance`는 true로 설정이 된다면 목표 허용오차를 잠그게 되는 파라미터로, 목적지에 도착해서도 허용오차가 없기 때문에 제자리에서 계속 회전하게 된다. 따라서 일반적으로 false로 사용한다.

### 3.1.4 Oscillation Reset
만약 local planner가 양 방향으로 진동하는 경로를 만들게 되면, 진동하는 수준에 따라 recovery behavior로 넘어가게 된다. `oscillation_reset_dist`는 그 진동하는 수준에 대해 설정하는 파라미터이다.

## 4. Costmap
costmap은 크게 `static map layer`, `obstacle map layer`, `inflation layer`로 나뉜다. `static map layer`는 저장되어 있는 맵 기반으로 형성되는 맵이고, `obstacle map layer`는 장애물에 의해 동적으로 생기는 맵이다. 이 때, laser와 같은 2D 데이터를 이용해 생성될 수도 있고, camera를 이용해 3D 데이터를 이용한 `voxel layer`를 생성할 수도 있다. `inflation layer`는 2D 맵 상의 장애물에 cost를 부여하는 맵이다. 이 `inflation layer`에 의해 생성되는 cost는 global costmap에서는 맵 기반으로 생성이 되며, local costmap에서는 로봇의 센서를 이용하 실시간으로 생성이 된다.

### 4.1 footprint
`footprint`는 로봇의 윤곽선을 나타내는 것으로, 장애물과 접촉 여부를 판단하기 위해서 사용된다. `footprint` 설정은 x, y 좌표의 list로 설정할 수 있다. 일반적으로 로봇의 외형보다 더 크게 설정해야 장애물에 부딪히지 않고 안전하게 사용할 수 있다.

### 4.2 inflation
inflation layer는 0에서 255의 cost를 map의 cell 상에 부여한다. `inflation_radius`와 `cost_scaling_factor` 파라미터에 의해 inflation layer가 설정된다. `inflation_radius`는 장애물로부터 얼마나 먼 거리까지 cost를 부여할 것인가를 설정하며, `cost_scaling_factor`는 셀의 cost를 계산하는데 사용하는 factor이며, 높은 값을 선정하면 장애물에 가까울수록 가파르게 cost가 증가하게 된다.

### 4.3 costmap resolution
`resolution` 파라미터는 맵핑한 지도의 resolution 수준과 동일하게 설정하면 된다. 만약 resolution이 낮게 설정이 된다면 좁은 공간에서 장애물의 크기가 크게 설정되어 지나갈 수 없다고 판단할 수도 있다.

### 4.4 obstacle layer and voxel layer
두 개의 layer는 장애물에 대한 코스트를 나타내기 위해 사용된다. 둘의 차이 점은 각각 2d와 3d로 표현된다는 점이다. 
--> voxel layer에 대한 내용은 추후 추가

## 5. Adaptive Monte Carlo Localization(AMCL)
AMCL은 ROS에서 로봇의 localization을 위해 제공되는 패키지이다.

laser scanner model로 부터 받은 센서 데이터와 계산된 odom 데이터를 이용한 localization을 위해서는 적절한 파라미터들의 수정이 필요하다. 그러나 파라미터의 선정이 잘못된다면, 로봇의 위치가 부정확하거나, 발산하는 경우가 발생하니 주의해야 한다.