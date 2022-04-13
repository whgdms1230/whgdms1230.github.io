---
sort: 2
---

# Keyword Confusion

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. const 키워드

const 키워드는 변경되면 안 될 대상을 선언할 때 사용한다. 이 키워드는 변수 또는 매개변수와 메서드에 적용할 수 있다.

### 1.1 const 변수와 매개변수

변수에 const를 붙이면 그 값이 변하지 않게 보호할 수 있다. 이 키워드는 `#define`으로 정의할 상수를 표현할 때 가장 많이 사용한다.

```cpp
const double PI = 3.141592
```

전역 변수나 클래스의 데이터 멤버뿐만 아니라 모든 종류의 변수에 const를 붙일 수 있다.

또한 함수나 메서드에 대한 매개변수도 그 값이 변하지 않도록 const로 지정할 수 있다.

```cpp
void func(const int param)
{
    // param을 변경할 수 없다.
}
```


#### 1.1.1 const 포인터

변수가 여러 단계의 간접 참조 연산을 거쳐야 하는 포인터로 선언됐다면 const를 적용하기 까다롭다.

```cpp
int* ip;
ip = new int[10];
ip[4] = 5;
```

여기서 ip를 const로 지정하고 싶다고 하자. 이때 const로 지정할 대상이 ip 변수인지 아니면 이 변수가 가리키는 값인지부터 구분해야 한다. 다시 말해 위 코드에서 const로 보호할 대상이 두 번째 줄인지 아니면 세 번째 줄인지 정해야 한다.

포인터로 가리키는 값이 수정되지 않게(세 번째 문장처럼 할 수 없게) 하려면 다음과 같이 const 키워드를 ip 변수의 포인터 앞에 붙인다.

```cpp
const int* ip; // 또는 int const* ip;
ip = new int[10];
ip[4] = 5; // 컴파일 에러 발생
```

반면 변경하지 않게 하려는 대상이 ip가 가리키는 값이 아니라 ip 자체라면 다음과 같이 const를 ip 변수 바로 앞에 붙인다.

```cpp
int* const ip = nullptr;
ip = new int[10]; // 컴파일 에러 발생
ip[4] = 5; // 에러 : 널 포인터 역참조
```

이렇게 하면 ip 자체를 변경할 수 없게 되기 때문에 이 변수를 선언과 초기화해야 한다.

```cpp
int* const ip = new int[10];
ip[4] = 5;
```

또한 다음과 같이 ip와 ip 포인터가 가리키는 값을 모두 const로 지정할 수도 있다.

```cpp
int const* const ip = nullptr; // 또는 const int* const ip = nullptr;
```

> const 키워드는 항상 바로 왼쪽에 나온 대상에 적용된다. const 변수가 복잡하게 선언됐을 때 오른쪽부터 왼쪽으로 읽으면 쉽게 파악할 수 있다. 예를 들어 `int* const ip`라고 선언한 문장을 오른쪽부터 읽으면 ip는 int에 대한 포인터에 const를 적용한 것이라고 해석한다. 반면 `int const* ip`는 ip는 const가 적용된 int에 대한 포인터라고 해석한다.

#### 1.1.2 const 레퍼런스

레퍼런스는 기본적으로 const 속성을 갖고 있다. 다시 말해 가리키는 대상을 변경하지 않는다. 그래서 명시적으로 const로 지정할 필요가 없다.

또한 레퍼런스에 대한 레퍼런스를 만들 수 없기 때문에 참조가 한 단계뿐이다. 여러 단계로 참조하는 유일한 경우는 포인터에 대한 레퍼런스를 만들 때 뿐이다.

이와 같은 레퍼런스의 특징 때문에 C++에서 `const 레퍼런스`라고 부르는 것은 대부분 다음과 같은 경우를 의미한다.

```cpp
int z;
const in& zRef = z; // 또는 int const& zRef = z;
zRef = 4; // 컴파일 에러 발생
```

int&에 const를 지정하면 zRef는 다른 값을 대입할 수 없다. 하지만 주의할 점은 zRef를 const라고 지정한 것과 z는 별개다. 그래서 zRef를 거치지 않고 z에 곧바로 접근하면 값을 변경할 수 있다.

const 레퍼런스는 주로 매개변수에 적용한다. 인수를 효율적으로 전달하도록 레퍼런스를 쓰고 싶은데 값을 변경할 수 없게 만들기 위해 사용한다.

```cpp
void doSomething(const BigClass& arg)
{
    // 구현 코드 작성
}
```

> 매개변수로 전달할 대상이 객체라면 기본적으로 const 레퍼런스로 선언한다. 전달할 객체를 변경할 일이 있을 때만 const를 생략한다.

### 1.2 const 메서드

[클레스 메서드를 const로 지정할 수 있다.](/0003Software/0001Languages/0001C++/0001Professional_C++/Part3/Chapter9/0003Method.html#2-const-메서드) 그러면 그 클래스에서 mutable로 선언하지 않은 데이터 멤버는 변경할 수 없다.

### 1.3 constexpr 키워드

C++에서 제공하는 상수 표현식이란 개념이 필요할 때가 있다. 예를 들어 배열을 정의할 때 크기를 상수 표현식으로 지정해야 한다. 그러므로 다음과 같이 작성하면 에러가 발생한다.

```cpp
const int getArraySize() { return 32; }

int main()
{
    int myArray[getArraySize()]; // C++에서 허용하지 않는 표현
    return 0;
}
```

constexpr 키워드를 사용하면 앞에 나온 getArraySize() 함수를 상수 표현식으로 다시 정의할 수 있다.

```cpp
constexpr int getArraySize() { return 32; }

int main()
{
    int myArray[getArraySize()]; // OK
    return 0;
}
```

다음과 같이 작성할 수도 있다.

```cpp
int myArray[getArraySize() + 1];
```

하지만 함수에 constexpr를 적용하면 그 함수에 다음과 같이 몇 가지제약사항이 적용된다.

* 함수 본문에는 goto 문, try/catch 블록, 초기화하지 않은 변수, 리터럴 타입이 아닌 변수 정의 등이 없어야 하고, 익셉션을 던져도 안 된다. 다른 constexpr 함수를 호출할 수는 있다.
* 리턴 타입이 반드시 리터럴 타입이어야 한다.
* 클래스의 멤버가 constexpr 함수일 때는 virtual로 선언할 수 없다.
* constexpr 함수의 매개변수는 반드시 리터럴 타입이어야 한다.
* 컴파일러는 항상 모든 정의를 완전히 알아야 컴파일 할 수 있으므로 constexpr 함수의 구현 코드를 컴파일러가 해석하기 전에는 호출할 수 없다.
* dynamic_cast()와 reinterpret_cast()를 사용할 수 없다.
* new와 delete를 사용할 수 없다.

사용자 정의 타입으로 된 상수 표현식 변수를 만들고 싶다면 constexpr 생성자를 정의한다. 생성자에 constexpr를 적용할 때도 함수와 마찬가지로 여러 가지 제약사항이 적용된다.

* 가상 베이스 클래스를 가질 수 없다.
* 생성자의 매개변수가 모두 리터럴 타입이어야 한다.
* 생성자 본문을 함수 try 블록으로 만들 수 없다.
* 생성자 본문을 명시적으로 디폴트로 지정하거나, constexpr 함수의 본문과 똑같은 요구사항을 만족해야 한다.
* 모든 데이터 멤버를 상수 표현식으로 초기화해야 한다.

다음과 같은 Rect 클래스는 앞서 나열한 요구사항에 맞게 constexpr 생성자를 정의했다.

```cpp
class Rect
{
    public:
        constexpr Rect(size_t width, size_t height)
            : mWidth(width), mHeight(height) {}

        constexpr size_t getArea() const { return mWidth * mHeight; }

    private:
        size_t mWidth, mHeight;
};
```

constexpr 객체는 다음과 같이 선언할 수 있다.

```cpp
constexpr Rect r(8, 2);
int myArray[r.getArea()];
```

## 2. static 키워드

C++ 코드에서 static 키워드의 용도는 다양하다. static 키워드를 다양한 문맥에서 사용하게 만든 이유는 키워드가 늘어나는 것을 피하기 위해서다.

### 2.1 static 데이터 멤버와 메서드

static으로 선언한 데이터 멤버는 static으로 지정하지 않은 멤버와 달리 객체에 속하지 않는다. 다시 말해 static 데이터 멤버는 객체 외부에 단 하나만 존재한다.

static 메서드도 객체가 아닌 클래스에 속한다는 점은 같다. static 메서드는 특정 객체를 통해 실행되지 않는다.

[static 데이터 멤버](/0003Software/0001Languages/0001C++/0001Professional_C++/Part3/Chapter9/0004DataMember.html#1-static-데이터-멤버) 및 [메서드](/0003Software/0001Languages/0001C++/0001Professional_C++/Part3/Chapter9/0003Method.html#1-static-메서드) 자료 참고

### 2.2 static 링크

##### 링크

C++는 코드를 소스 파일 단위로 컴파일해서 그 결과로 나온 오브젝트 파일들을 링크 단계에서 서로 연결한다. C++ 소스 파일(함수나 전역 변수 포함)마다 정의된 이름은 외부 링크나 내부 링크를 통해 서로 연결된다.

외부 링크로 연결되면 다른 소스 파일에서 이름을 사용할 수 있고, 내부 링크로 연결되면 같은 파일에서만 사용할 수 있다.

함수나 전역 변수는 기본적으로 외부 링크가 적용된다. 하지만 선언문 앞에 static 키워드를 붙이면 내부 링크가 적용된다.

##### static 링크 예제

예를 들어 FirstFile.cpp와 AnotherFile.cpp란 소스파일이 있다고 하자.

FirstFile.cpp는 다음과 같다. 이 파일은 f()의 프로토타입만 있고 정의하는 코드는 없다.

```cpp
void f();

int main()
{
    f();
    return 0;
}
```

AnotherFile.cpp는 다음과 같다. 이 파일은 f()를 선언하는 코드와 정의하는 코드가 모두 있다.

```cpp
#include <iostream>

void f();

void f()
{
    std::cout << "f\n";
}
```

앞에 나온 소스 파일은 아무런 에러 없이 컴파일되고 링크된다. f()가 외부 링크로 처리되어 main()에서 다른 파일에 있는 함수를 호출할 수 있기 때문이다.

> 이렇게 같은 함수에 대한 프로토타입을 여러 파일에 작성해도 된다. 전처리기는 소스 파일에 있는 `#include` 파일을 보고 헤더 파일에 나온 프로토타입을 소스 파일에 추가한다. 헤더 파일을 사용하는 이유는 프로토타입을 여러 파일에서 일관성 있게 유지하기 위해서다. 하지만 이 예제에서는 헤더 파일을 사용하지 않는다.

이번에는 AnotherFile.cpp에서 f()의 선언문 앞에 static을 붙여보자. 참고로 f()를 정의하는 코드에서는 static 키워드를 생략해도 된다. 단, 함수 선언문이 함수를 정의하는 코드보다 앞에 나와야 한다.

```cpp
#include <iostream>

static void f();

void f()
{
    std::cout << "f\n";
}
```

컴파일 과정에는 아무런 에러가 발생하지 않지만 링크 단계에서 에러가 발생한다. f()를 static으로 지정해서 내부 링크로 변경되 FirstFile.cpp에서 찾을 수 없기 때문이다.

##### 익명 네임스페이스

static 대신 익명 네임스페이스를 이용하여 내부 링크가 적용되게 할 수도 있다. 익명 네임스페이스에 속한 항목은 이를 선언한 소스 파일 안에서는 얼마든지 접근 가능하지만 다른 소스 파일에서는 접근할 수 없다. 이는 static 키워드의 효과와 같다.

```cpp
#include <iostream>

namespace {
    void f();

    void f()
    {
        std::cout << "f\n";
    }
}
```

##### extern 키워드

const와 typedef는 기본적으로 내부 링크로 처리된다. 여기에 extern을 붙이면 외부 링크가 적용된다.

하지만 extern의 적용 과정은 좀 복잡하다. 어떤 이름을 extern으로 지정하면 컴파일러는 이를 정의가 아닌 선언문으로 취급한다. 변수를 extern으로 지정하면 컴파일러는 그 변수에 대해 메모리를 할당하지 않는다. 따라서 그 변수를 정의하는 문장을 따로 작성해야 한다.

예를 들어 AnotherFile.cpp에 나온 다음 문장이 그렇다.

```cpp
extern int x;
int x = 3;
```

아니면 다음과 같이 extern으로 선언하는 동시에 초기화해도 된다. 그러면 선언과 정의를 한 문장에서 처리할 수 있다. 하지만 이 경우는 extern이란 키워드를 붙일 필요가 없다.

```cpp
extern int x = 3;
```

extern이 반드시 필요한 경우는 다음과 같이 FirstFile.cpp과 같은 다른 소스 파일에서 x에 접근하게 만들 때다. FirstFile.cpp에서 x를 extern으로 선언했기 때문에 다른 파일에 있던 x를 여기서 사용할 수 있는 것이다.

```cpp
#include <iostream>

extern int x;

int main()
{
    std::cout << x << std:: endl;
}
```

### 2.3 함수 안의 static 변수

함수 안에서 static으로 지정한 변수는 그 함수만 접근할 수 있는 전역 변수와 같다. 주로 어떤 함수에서 초기화 작업 수행 여부를 기억하는 용도로 많이 사용한다.

```cpp
void performTask()
{
    static bool initialized = false;
    if(!initialized) {
        cout << "initializing" << endl;
        // 초기화 수행
        initialized = true;
    }

    // 원하는 작업 수행
}
```

그런데 이렇게 static 변수를 사용하면 헷갈리기 쉽다. static 변수를 정의할 필요가 없도록 코드 구조를 변경하는 것이 바람직하다. 이 예제의 경우 특수한 초기화 작업을 수행하는 생성자를 클래스에 별도로 정의한다.

> 메이어의 싱글턴 패턴과 같은 경우에 이렇게 static 변수를 사용하는 것이 좋을 때가 있다.

## 3. 비로컬 변수의 초기화 순서

전역 변수와 static 클래스 데이터 멤버는 모두 main()이 시작하기 전에 초기화된다. 이러한 변수는 소스 파일에 선언된 순서대로 초기화된다.

다음 예시에서는 Demo::x가 y보다 먼저 초기화된다.

```cpp
class Demo
{
    public:
        static int x;
};

int Demo::x = 3;
int y = 4;
```

그런데 C++ 표준은 비로컬 변수가 여러 소스 파일에 선언됐을 때 초기화하는 순서는 따로 정해두지 않았다. 어떤 소스 파일에 x란 전역 변수가 있고, 다른 파일에는 y란 전역 변수가 있을 때 어느 것이 먼저 초기화되는지 알 수 없다.

간혹 전역 변수나 static 변수가 서로 의존 관계에 있을 때 문제가 발생할 수 있다. 어떤 전역 객체의 생성자 안에서 다른 전역 객체에 접근할 수 있는데, 두 소스 파일에 정의된 전역 객체 중 어느 것이 먼저 생성될지 알 수 없고, 초기화되는 순서도 제어할 수 없다.

## 4. 비로컬 변수의 소멸 순서

비로컬 변수는 생성된 순서와 반대로 소멸된다. 그런데 여러 소스 파일에 있는 비로컬 변수의 초기화 순서는 생성 순서와 마찬가지로 표준에 정해져 있지 않아서 정확한 소멸 순서를 알 수 없다.