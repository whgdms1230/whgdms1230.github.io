---
sort: 1
---

# Working With Dynamic Memory

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 메모리 작동 과정

먼저 스택에 저장되는 자동 변수(automatic variable)의 생성과정이다. 이 변수는 선언된 스코프를 벗어나면 할당된 메모리가 자동으로 해제됨.

```cpp
int i = 7;
```

다음으로 힙 메모리 할당되는 변수의 생성 과정이다. ptr 변수를 스택에 생성하고, nullptr로 초기화한 뒤 ptr가 동적으로 생성된 힙 메모리를 가리키도록 설정한다.

```cpp
int* ptr = nullptr;
ptr = new int;
```

위의 코드를 한 줄로 표현하면 다음과 같다.

```cpp
int* ptr = new int;
```

이 경우는 ptr 변수는 스택에 있지만, ptr이 가리키는 값은 힙에 있다.

다음 코드는 포인터가 스택과 힙에 모두 있는 예이다.

```cpp
int** handle = nullptr;
handle = new int*;
*handle = new int;
```

여기서는 먼저 정수 포인터에 대한 포인터를 handle이란 변수로 선언했다. 그런 다음 정수 포인터를 담는 데 충분한 크기로 메모리를 할당한 뒤 그 메모리에 대한 포인터를 handle에 저장했다. 이어서 이 메모리(*handle)에 정수를 담기 충분한 크기의 힙 메모리를 동적으로 할당했다. 이렇게 두 포인터 중 하나(handle)는 스택에, 다른 하나(*handle)는 힙에 있도록 두 단계로 구성한 상태를 보여준다.

## 2. 메모리 할당과 해제

### 2.1 new와 delete
변수에 필요한 메모리 블록을 할당하려면 new에 그 변수의 타입을 지정해서 호출한다. 그러면 할당된 메모리에 대한 포인터가 리턴된다.

new의 리턴값을 무시하거나 그 포인터를 담았던 변수가 스코프를 벗어나면 할당했던 메모리에 접근할 수 없게 되는데, 이는 메모리 누수라고 한다.

다음은 int 크기의 공간에 대한 메모리 누수의 예를 보여준다.

```cpp
void leaky()
{
    new int;    // 버그! 메모리 누수가 발생한다.
    cout << "방금 int 하나를 잃어버렸다." << endl;
}
```

속도가 빠른 메모리를 무한 공급하지 않는 한 객체에 할당했던 메모리를 다른 용도로 사용할 수 있도록 해제해야 한다. 메모리 누수를 막기 위해 힙 메모리를 해제하려면 delete를 이용한다.

```cpp
int* ptr = new int;
delete ptr;
ptr = nullptr;
```

> 메모리를 해제한 포인터는 nullptr로 초기화해야 해제된 메모리를 가리키는 포인터를 사용하는 실수를 방지할 수 있다.

### 2.2 malloc()
C에서는 malloc() 함수를 이용하여 동적할당을 했다. malloc()은 인수로 지정한 바이트 수 만큼 메모리를 할당한다. C++에서도 여전히 malloc()을 지원하지만, new를 사용하는 것이 바람직하다.

new는 단순히 메모리를 할당하는 데 그치지 않고 객체까지 만든다.

예를 들어 Foo 라는 클래스의 객체를 생성하는 코드를 살펴보자.
```cpp
Foo* myFoo = (Foo*)malloc(sizeof(Foo));
Foo* myOtherFoo = new Foo();
```

이 코드는 Foo 객체를 저장하는 데 충분한 크기의 힙 영역을 할당시키고, 이를 가리키는 포인터 myFoo와 myOtherFoo에 저장한다. 두 포인터로 모두 Foo의 데이터 멤버와 메서드에 접근할 수 있지만, myFoo가 가리키는 Foo 객체는 아직 생성되지 않았기 때문에 정상적인 객체라고 볼 수 없다.

이렇듯 malloc()은 메모리에 일정한 영역만 따로 빼놓을 뿐 객체를 생성하지 않는다. 반면 new를 호출한 문장은 메모리 할당 뿐 아니라 Foo의 생성자를 호출해서 객체를 생성한다.

> free()와 delete의 관계도 이와 비슷하다. free()는 객체의 소멸자를 호출하지 않는 반면 delete는 소멸자를 호출해서 객체를 정상적으로 제거한다.

### 2.3 메모리 할당에 실패한 경우
new를 사용한 메모리 할당이 실패하는 경우는, 메모리가 부족한 상황이다. 이런 상황에서는 프로그램의 동작을 예측할 수 없기 때문에 정확한 상태를 가늠하기 힘들다.

따라서 기본적으로 new가 실패하면 프로그램이 종료된다. new로 요청한 만큼의 메모리가 없어서 익셉션이 발생한다.

또는 익셉션이 발생하지 않는 버전은 익셉션 대신 nullptr를 리턴한다. 문법은 다음과 같다.

```cpp
int* ptr = new(nothrow) int;
```

nullptr를 리턴하는 경우에는 해당 상황을 처리해줘야 하며, 처리를 하지 못하는 경우에 버그가 발생할 가능성이 높다.

> new가 실패하는 경우 익셉션을 던지는 표준 버전의 new를 사용하는 것이 바람직하다.
