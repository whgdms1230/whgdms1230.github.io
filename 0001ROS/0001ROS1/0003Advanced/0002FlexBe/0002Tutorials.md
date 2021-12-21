---
sort: 2
---

# FlexBe Tutorials

## 0. 참고 문헌

*- [FlexBe 홈페이지](http://philserver.bplaced.net/fbe/)*

*- [FlexBe Tutorials ROS 위키 페이지](http://wiki.ros.org/flexbe/Tutorials)*

## 1. Creating a New Behavior
다음의 roslaunch 명령을 통해 flexbe app을 실행시킨다.
```bash
roslaunch flexbe_app flexbe_ocs.launch
```

flexbe_ocs를 실행하면 Behavior Deshboard가 보이는데, [위키 페이지](http://wiki.ros.org/flexbe/Tutorials/Creating%20a%20New%20Behavior) 참조하여 다음과 같이작성한다.

<img src="flexbe_tutorials.png"  width="1000" height="600">

* Overview : 해당 behavior의 이름과 그에 대한 개요를 작성한다.
  * 이 때 이름은 위 예제처럼 대문자 및 공백을 만들지 않고, 공백 없는 소문자로 작성하는 것이 좋다.
* Private Configuration : behavior 동작에 필요한 상수값을 정의한다.
* Behavior Parameters : behavior 동작에 사용되는 매개변수를 정의한다.
* State Machine Userdata : 한 state에서 다음 state로 전달되는 userdata 값으로, 각 state의 input_keys와 output_keys를 정의한다.
* State Machine Interface : 전체 state machine의 outcomes와 input_keys, output_keys를 정의한다.

## 2. Using the Statemachine Editor
다음으로 Statemachine Editor로 이동하여 동작에 필요한 state를 정의한다. state의 세부 생성 방법은 [위키 페이지](http://wiki.ros.org/flexbe/Tutorials/Using%20the%20Statemachine%20Editor) 참조하여 작성하면 된다. 여기서는 새로운 state를 만드는 것이 아니라 flexbe에서 제공하는 state를 이용하여 정의하는 것이다.

<img src="flexbe_tutorials2.png"  width="1000" height="600">

* Initial_Wait는 WaitState state를 이용하여 정의되었고, WaitState에서 정의되어야 하는 파라미터는 wait_time이다.
  * wait_time을 앞서 Behavior Parameters에서 정의한 매개변수인 waiting_time으로 지정하였다.
  * Initial_Wait는 waiting_time의 값 만큼 대기하는 state이다.
  * 앞서 정의한 파라미터들은 self. 로 시작해야 하며, 이는 파라미터와 상수를 구분하기 위해 사용된다.
* Print_Gretting은 LogState state를 이용하여 정의되었고, LogState에서 정의되어야 하는 파라미터는 text와 serverity이다.
  * text는 앞서 정의한 상수인 hello로 정의하였고, serverity는 디폴트로 정의되어 있는 Logger.REPORT_HINT를 사용한다.
  * Required Autonomy Levels는 High로 지정한다.
  * Print_Gretting은 ‘Hello World!’를 출력하는 state이다.
  * 여기서 상수는 파라미터와 다르게 self.를 붙지이 않은 것을 알 수 있다.
* 시작점부터 각 state를 연결하고, 마지막엔 finished에 연결한다. 여기서 failed는 사용하지 않았다.
* [Save Behavior] 버튼을 통해 만든 behavior를 저장할 수 있고, 해당 behavior는 이전에 만든 repository에 생성됨을 확인할 수 있다.

## 3. Execution of a Behavior
behavior를 만들었따면, 해당 behavior를 동작하기 위해 다음의 명령으로 flexbe 전체 behavior engine을 실행할 수 있다.
```bash
roslaunch flexbe_app flexbe_full.launch
```

실행시키면 flexbe_ocs와 동일한 화면이 나오는 것을 알 수 있다. 여기서 [Load Behavior] 버튼을 통해 위해서 만든 behavior를 가져올 수 있다.

<img src="flexbe_tutorials3.png"  width="1000" height="600">

RuntimeControl 탭으로 넘어가면 앞서 만든 behavior를 실행시켜 볼 수 있다.(자세한 실행 내용은 [위키 페이지](http://wiki.ros.org/flexbe/Tutorials/Execution%20of%20a%20Behavior) 참조)

<img src="flexbe_tutorials4.png"  width="1000" height="600">

여기서 Autonomy Level의 개념을 정의하는데,
* Autonomy Level은 각 state의 수행 수준을 정의하는데 사용된다.
* 실행 수준은 No, Low, High, Full의 네 가지 수준으로, 해당 수준의 이상인 state는 사용자가 직접 화살표를 클릭하여 진행시켜야 하며, 해당 수준 이하인 state는 자동적으로 진행이된다.
  * No인 경우에는 모든 state를 사용자가 직접 수행시키며, Low와 High는 각각 state의 Autonomy Level이 Low 또는 High 이상인 경우만 사용자가 직접 수행시키고, Full의 경우 모든 state가 자동으로 진행된다.