---
sort: 220603
---

# ROS2 C++ 쓰레드 생성과 소멸

다음은 ROS2 에서 쓰레드 생성 및 소멸에 대한 한 가지 방법이다.

## 쓰레드 생성

```cpp
std::shared_ptr<std::thread> thread_;

const unsigned int thread_ms = 100;

thread_ = std::make_shared<std::thread>(std::bind(&thread_function, this, thread_ms));

void thread_funcion(unsigned int ms){
  const auto wait_duration = std::chrono::milliseconds(ms);

  while(true){
    std::this_thread::sleep_for(wait_duration);

    // 쓰레드 프로세스 동작 정의
  }
}
```

## 쓰레드 소멸

클래스의 소멸자에 다음을 정의해야 클래스 객체가 소멸될 때 쓰레드도 정상적으로 소멸된다.
만약 쓰레드를 소멸시키지 않고 클래스가 객체가 소멸되면 에러 발생한다.

```cpp
thread_->detach();
if(task_thread_->joinable()) thread_->joint();
```