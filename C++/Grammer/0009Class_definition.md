---
sort: 9
---

# Class 정의

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. class

`class`는 객체의 특성을 정의한 것이다. `C++`에서 클래스를 선언하는 코드는 주로 헤더 파일에 작성하고, 구체적으로 구현하는 코드는 소스 파일에 작성한다.

클래스 안에는 데이터 멤버와 메서드(동작)를 선언한다. 각각의 데이터 멤버와 메서드마다 `public`, `protected`, `private` 등으로 접근 수준을 지정한다.

`public`은 클래스 밖에서 접근이 가능하다. 반면, `private`는 클래스 외부에서 접근할 수 없다. 따라서 값을 가져오는 `getter`와 값을 설정하는 `setter`를 정의하고 이를 `public`으로 지정한다. 따라서 일반적으로 데이터 멤버는 `private`로 지정하고, 메서드는 `public`에 선언한다.

`protected`는 상속과 관련해서 사용한다....

## 2. 생성자와 소멸자

생성자(`constructor`)는 클래스와 이름이 같고, 리턴 타입이 없는 메서드이다. 이 메서드는 해당 클래스 객체를 생성할 때 자동으로 호출된다.

소멸자(`destructor`)는 생성자와 형태는 같지만 앞에 `~`를 붙인 메서드이다. 이 메서드는 해당 클래스 객체가 제거될 때 자동으로 호출된다.

### 2.1. 데이터 멤버 초기화 방법

생성자로 데이터 멤버를 초기화 하는 방법은 두 가지다.

첫 번째 방법은 생성자 이니셜라이저를 사용하는 것으로, 권장되는 방법이다.
```cpp
AirlineTicket::AirlineTicket()
  : mPassengerName("Unknown Passenger")
  , mNumberOfMiles(0)
  , mHasEliteSuperRewardsStatus(false)
{
}
```

두 번째 방법은 생성자의 본문에서 초기화하는 방법이다.
```cpp
AirlineTicket::AirlineTicket()
{
  mPassengerName = "Unknown Passenger";
  mNumberOfMiles = 0;
  mHasEliteSuperRewardsStatus = false;
}
```

## 3. 스택 / 힙 기반의 클래스 생성

클래스를 생성하는 예로 스택 기반으로 생성하는 방법과, 힙 기반으로 생성하는 방법이 있다.

다음은 AirlineTicket 객체를 스택 기반으로 생성하는 예시이다.
```cpp
AirlineTicket myyTicket;
myTicket.setPassengerName("Sherman T. Socketwrench");
myTicket.setNumberOfMiles(700);
...
```

다음은 AirlineTicket 객체를 힙 기반으로 생성하는 예시이다.
```cpp
// 스마트 포인터 기반
auto myTicket = make_unique<AirlineTicket>();
myTicket->setPassengerName("Sherman T. Socketwrench");
myTicket->setNumberOfMiles(700);
...

// 스마트 포인터를 사용하지 않는 경우
AirlineTicket* myTicket = new AirlineTicket();
...
delete myTicket; // 이 경우에는 delete 키워드로 힙 객체를 삭제
```