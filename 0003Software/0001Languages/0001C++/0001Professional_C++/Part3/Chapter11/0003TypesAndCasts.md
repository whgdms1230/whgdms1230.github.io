---
sort: 3
---

# Types And Casts

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 타입 앨리어스

타입 앨리어스란 기존에 선언된 타입에 다른 이름을 붙이는 것이다.

다음 예는 `int*` 타입을 `IntPtr` 이란 이름으로 부르게 만든다. 이는 기존 타입으로 선언할 수도 있고, 정의한 앨리어스를 이용하여 타입을 선언할 수 있으며, 두 타입은 완전히 같다.

```cpp
using IntPtr = int*;

int* p1;
IntPtr p2;
```

타입 앨리어스는 복잡하게 선언된 타입 표현을 간편하게 만들기 위한 용도로 많이 사용한다. 흔히 템플릿을 이용할 때 이런 경우가 많다.

다음 예시에서는 `std::vector<std::string>`과 같이 복잡한 타입을 타입 앨리어스를 이용하여 짧고 의미가 드러나게 표현한 것이다.

```cpp
using StringVector = std::vector<std::string>

void processVector(const StringVector& vec)
{
    // 코드 생략
}

int main()
{
    StringVector myVector;
    processVector(myVector);
    return 0;
}
```

> 타입 앨리어스를 정의할 때 스코프 지정자도 포함시킬 수 있다. 앞에 나온 예제에 정의한 StringVector는 std란 스코프를 담고 있다.

## 2. 함수 포인터에 대한 타입 앨리어스

함수 포인터의 타입은 매개변수 타입과 리턴 타입에 따라 결정된다. 함수 포인터를 다루는 방법 중 하나는 앨리어스를 사용하는 것이다. 타입 앨리어스를 사용하면 특정한 속성을 가진 함수들을 타입 이름 하나로 부를 수 있다.

예를 들어 int 매개변수 두 개와 bool 리턴값 하나를 가진 함수의 타입을 MatchFunction이란 이름으로 정의하려면 다음과 같이 작성한다.

```cpp
using MatchFunction = bool(*)(int, int);
```

이렇게 타입 이름을 별도로 정의하면 MatchFunction을 매개변수로 받는 함수를 작성할 수 있다.

예를 들어 다음 함수는 int 배열 두 개와 각각의 크기, MatchFunction을 인수로 받는다. 전달받은 두 배열에 대해 루프를 돌면서 각 원소를 MatchFunction의 인수로 전달하고 호출해서 true가 되면 특정한 메시지를 화면에 출력한다.

```cpp
void findMatches(int values1[], int values2[], size_t numValues, MatchFunction matcher)
{
    for(size_t i = 0; i < numValues; i++){
        if(matcher(values1[i], values2[i])){
            cout << "Match found at position " << i << " (" << values1[i] << ", " << values2[i] << ")" << endl;
        }
    }
}
```

타입이 MatchFunction와 일치하는 함수라면 어떤 것도 findMatches() 함수의 인수로 전달할 수 있다.

```cpp
bool intEqual(int item1, int item2)
{
    return item1 == item2;
}
```

intEqual() 함수는 MatchFunction 타입과 일치하기 때문에 findMatches()의 마지막 인수로 전달할 수 있다.

```cpp
int arr1[] = { 2, 5, 6, 9, 10, 1, 1 };
int arr2[] = { 4, 4, 2, 9, 0, 3, 4 };

size_t arrSize = std::size(arr1);
cout << "Calling findMatches() using intEqual():" << endl;
findMatches(arr1, arr2, arrSize, &intEqual);
```

> 여기서 intEqual() 함수를 findMatches() 함수에 포인터로 전달했는데, 정확한 문법에 따르면 위와 같이 `&`를 붙여야 하지만, 생략하고 이름만 적어도 컴파일러는 이 값이 주소라고 판단한다.

이처럼 함수 포인터를 사용하면 findMatches()란 함수를 matcher란 매개변수의 값에 따라 다양한 용도로 커스터마이즈할 수 있다.

##### C++에서의 함수 포인터 사용 예

C++에서는 함수 포인터를 잘 사용하지는 않지만 간혹 함수 포인터가 필요할 때가 있다. 대표적으로 동적 링크 라이브러리(Dynamic Linked Library, DLL)에 있는 함수에 대한 포인터를 구할 때가 있다.

`hardware.dll`이란 DLL에 Connect()란 함수가 있을 때 이 Connect()를 호출하려면 먼저 DLL을 불러와야 한다고 하자. 실행 시간에 DLL을 불러오는 작업은 다음과 같이 윈도우에서 제공하는 LoadLibrary()란 커널 함수로 처리한다.

```cpp
HMODULE lib = ::LoadLibrary("hardware.dll");
```

Connect()의 프로토타입이 다음과 같이 int 값 하나를 리턴하고, 매개변수를 세 개(bool, int, C 스타일 스트링) 받는다고 하자.

```cpp
int __stdcall Connect(bool b, int n, const char* p);
```

> 여기서 `__stdcall`은 마이크로소프트에서 정의한 지시자로서 함수에 매개변수가 전달되는 방식과 메모리를 해제하는 방법을 지정한다.

타입 앨리어스를 이용해 앞에 나온 프르토타입을 가진 함수에 대한 포인터에 이름 (ConnectFunction)을 정의한다.

```cpp
using ConnectFunction = int(__stdcall*)(bool, int, const char*);
```

DLL을 불러오는 과정에 문제가 없고 함수 포인터도 정의했다면 DLL에 있는 함수에 대한 포인터를 다음과 같이 구할 수 있다.

```cpp
ConnectFunction connect = (ConnectFunction)::GetProcAddress(lib, "Connect");
```

정상적으로 처리됐다면 방금 받은 함수 포인터로 다음과 같이 호출할 수 있다.

```cpp
connect(true, 3, "Hello world");
```

## 3. 메서드와 데이터 멤버를 가리키는 포인터에 대한 타입 앨리어스

메서드나 데이터 멤버를 포인터로 접근하려면 반드시 해당 객체의 문맥에서 포인터를 역참조해야 한다.

```cpp
Employee employee;
int (Employee::*methodPtr) () const = &Employee:getSalary;
cout << (employee.*methodPtr)() << endl;
```

두 번째 줄은 methodPtr란 변수를 선언하는데, 이 변수의 타입은 Employee에 있는 `non-static const`를 가리키는 포인터다. 이 메서드는 인수를 받지 않고 int 값을 리턴한다. 여기서는 선언과 동시에 변수의 값을 Emplyee 클래스의 getSalary() 메서드에 대한 포인터로 초기화했다. 이는 *methodPtr 앞에 `Employee::`가 붙은 점만 빼면 함수 포인터를 정의하는 문법과 비슷하다.

세 번째 줄은 employee 객체를 통해 methodPtr 포인터로 getSalary() 메서드를 호출한다. 메서드 이름 뒤에 나온 `()`는 `*`보다 우선순위가 높기 때문에 `employee.*methodPtr`를 소괄호로 묶어야 이 부분을 먼저 처리한다.

해당 코드에서 타입 앨리어스를 활용하면 두 번째 줄을 다음과 같이 좀 더 읽기 쉽게 작성할 수 있다.

```cpp
Employee employee;
using PtrToGet = int (Employee::*) () const;
PtrToGet methodPtr = &Employee:getSalary;
cout << (employee.*methodPtr)() << endl;
```

auto를 쓰면 훨씬 더 간결해진다.

```cpp
Employee employee;
auto methodPtr = &Employee:getSalary;
cout << (employee.*methodPtr)() << endl;
```

메서드나 데이터 멤버에 대한 포인터를 사용할 일은 많지 않지만, `non-static` 메서드나 데이터 멤버에 대한 포인터는 객체를 거치지 않고서는 역참조할 수 없다는 사실을 반드시 명심한다. 프로그래밍을 하다보면 `qsort()`와 같이 함수 포인터를 받는 함수에 non-static 메서드의 포인터를 전달하는 실수를 저지르기 쉬운데, 이렇게 작성하면 동작하지 않는다.

> C++에서는 객체를 거치지 않고서도 static 데이터 멤버나 메서드의 포인터를 역참조할 수 있다.

## 4. typedef

typedef도 타입 앨리어스와 마찬가지로 기존에 선언된 타입에 이름을 붙여준다.

먼저 다음은 타입 앨리어스로 정의된 코드이다.

```cpp
using IntPtr = int*;
```

타입 앨리어스를 사용하기 전에는 다음과 같이 typedef로 비슷한 효과를 냈다. 하지만, 가독성이 훨씬 더러진다.

```cpp
typedef int* IntPtr;
```

또 다른 예로 다음과 같이 정의된 타입 앨리어스가 있다고 하자.

```cpp
using FunctionType = int (*)(char, double);
```

이를 typedef로 표현하면 다음과 같다.

```cpp
typdef int (*FunctionType)(char, double)
```

FunctionType이란 이름이 중간에 나오기 때문에 헷갈린다.

타입 앨리어스와 typedef가 완전히 똑같은 것은 아니다. 템플릿에 활용할 때는 typedef보다 타입 앨리어스를 사용하는 것이 훨씬 유리하다.

## 5. 캐스팅

C++는 `const_cast()`, `static_cast()`, `reinterpret_cast()`, `dynamic_cast()`라는 네 가지 캐스팅 방법을 제공한다.

`()`를 이용하는 C 스타일 캐스팅도 C++에 계속 지원하나, 의도가 분명히 드러나지 않아서 에러가 발생하거나 예상과 다른 결과가 나올 수 있다. 따라서 C++ 스타일로 캐스팅하는 것이 훨씬 안전하고 문법도 깔끔하다.

### 5.1 const_cast()

const_cast()는 변수에 const 속성을 추가하거나 제거할 때 사용한다. 정석대로라면 const로 지정한 부분을 항상 일관성 있게 유지해야 하지만 서드파티 라이브러리와 같이 마음대로 수정할 수 없을 때는 부득이 const 속성을 일시적으로 제거할 수밖에 없다. 단, 호출할 함수가 객체를 변경하지 않는다고 보장될 때만 이렇게 처리해야 한다.

```cpp
extern void ThirdPartyLibraryMethod(char* str);

void f(const char* str)
{
    ThirdPartyLibraryMethod(const_cast<char*>(str));
}
```

C++17부터 `std::as_const()`란 헬퍼 메서드가 추가됐다. 이 메서드는 `<utility>` 헤더에 정의돼 있으며, 레퍼런스 매개변수를 const 레퍼런스 버전으로 변환해준다. 기본적으로 `as_const(obj)`는 `const_cast<const T&>(obj)`와 같다. 여기서 `T`는 obj의 타입이다.

```cpp
std::string str = "C++";
const std::string& constStr = std::as_const(str);
```

as_const()와 auto를 조합할 때 주의점이 있다. auto는 레퍼런스와 const 속성을 제거한다. 따라서 다음과 같이 작성하면 result의 변수 타입은 `const std::string&`가 아닌 `std::string`이 된다.

```cpp
auto result = std::as_const(str);
```

### 5.2 static_cast()

static_ast()는 명시적 변환 기능을 수행한다. 예를 들어 다음 코드처럼 정수에 대한 나눗셈이 아닌 부동소수점에 대한 나눗셈으로 처리하도록 int를 double로 변환해야 할 때가 있다. 이때 static_ast()를 사용하면 된다.

```cpp
int i = 3;
int j = 4;
double result = static_cast<double>(i) / j;
```

사용자 정의 생성자나 변환 루틴에서 허용하는 명시적 변환을 수행할 때도 static_cast()를 사용할 수 있다. 예를 들어 A 클래스의 생성자 중에 B 클래스 객체를 인수로 받는 버전이 있다면 B 객체를 A 객체로 변환하는 데 static_cast()를 이용할 수 있다. 그런데 이런 변환은 대부분 컴파일러가 알아서 처리해준다.

상속 계층에서 하위 타입으로 다운캐스팅할 때도 static_cast()를 사용한다.

```cpp
class Base
{
    public:
        virtual ~Base() = default;
};

class Derived : public Base
{
    public:
        virtual ~Derived() = default;
};

int main()
{
    Base* b;
    Derived* d = new Derived();
    b = d; // 상속 계층의 상위 타입으로 업캐스팅할 필요 없다.
    d = static_cast<Derived*>(b); // 상속 계층의 하위 타입으로 다운캐스팅해야 한다.

    Base base;
    Derived derived;
    Base& br = derived;
    Derived& dr = static_cast<Derived*>(br);

    return 0;
}
```

이러한 캐스팅은 포인터나 레퍼런스에도 적용할 수 있다. 단, 객체 자체에는 적용할 수 없다.

static_cast()는 실행 시간에 타입 검사를 수행하지 않는다. 실행 시간에 캐스팅할 때는 Base와 Derived가 실제로 관련이 없어도 Base 포인터나 레퍼런스를 모두 Derived 포인터나 레퍼런스로 변경한다. 예를 들어 다음과 같이 작성하면 컴파일 과정과 실행 과정에 아무런 문제가 발생하지 않지만 포인터 d를 사용하다가 객체의 범위를 벗어난 영역의 메모리를 덮어쓰는 심각한 문제가 발생할 수 있다.

```cpp
Base* b = new Base();
Derived* d = static_cast<Derived*>(b);
```

타입을 안전하게 캐스팅하도록 실행 시간에 타입 검사를 적용하려면 dynamic_cast()를 사용한다.

static_cast()는 그리 강력하지 않다. 포인터의 타입이 서로 관련 없을 때는 적용할 수 없다. 또한 변환 생성자가 제공되지 않는 타입의 객체에도 적용할 수 없다. const 타입을 non-const 타입으로 변환할 수도 없고, int에 대한 포인터에도 적용할 수 없다.

### 5.3 reinterpret_cast()

reinterpret_cast()는 static_cast()보다 강력하지만 안전성은 떨어진다.

C++ 타입 규칙에서 허용하지 않더라도 상황에 따라 캐스팅하는 것이 적합할 때 적용할 수 있다. 예를 들어 서로 관련이 없는 레퍼런스끼리 변환할 수도 있다.

상속 계층에서 아무런 관련이 없는 포인터 타입끼리도 변환할 수 있다. 이런 포인터는 흔히 `void*` 타입으로 캐스팅한다. 이 작업은 내부적으로 처리되기 때문에 명시적으로 캐스팅하지 않아도 된다. 하지만 이렇게 `void*`로 변환한 것을 다시 원래 타입으로 캐스팅할 때는 reinterpret_cast()를 사용해야 한다.

`void*` 포인터는 메모리의 특정 지점을 가리키는 포인터일 뿐 `void*` 포인터 자체에는 아무런 타입 정보가 없기 때문이다.

```cpp
class X {};
class Y {};

int main()
{
    X x;
    Y y;
    X* xp = &x;
    Y* yp = &y;

    // 서로 관련 없는 클래스 타입의 포인터를 변환할 때는 reinterpret_cast()를 써야 한다.
    // static_cast()는 작동하지 않는다.
    xp = reinterpret_cast<X*>(yp);
    // 포인터를 void*로 변환할 때는 캐스팅하지 않아도 된다.
    void* p = xp;
    // 변환된 void*를 다시 원래 포인터로 복원할 때는 reinterpret_cast()를 써야 한다.
    xp = reinterpret_cast<X*>(p);

    // 서로 관련 없는 클래스 타입의 레퍼런스를 변환할 때는 reinterpret_cast()를 써야 한다.
    // static_cast()는 작동하지 않는다.
    X& xr = x;
    Y& yr = reinterpret_cast<Y&>(x);

    return 0;
}
```

reinterpret_cast()는 타입 검사를 하지 않고 변환할 수 있기 때문에 주의해야 한다.

> 포인터를 int 타입으로 변환하거나 그 반대로 변환할 때도 reinterpret_cast()를 사용할 수 있다. 단, 이때 int의 크기가 포인터를 담을 정도로 충분히 커야 한다. 예를 들어 64비트 포인터를 32비트 int로 변환하는 작업을 reinterpret_cast()로 처리하면 컴파일 에러가 발생한다.

### 5.4 dynamic_cast()

dynamic_cast()는 같은 상속 계층에 속한 타입끼리 캐스팅할 때 실행 시간에 타입을 검사한다.

포인터나 레퍼런스를 캐스팅할 때 이를 이용할 수 있다. dynamic_cast()는 내부 객체의 타입 정보를 실행 시간에 검사한다. 그래서 캐스팅이 적합하지 않다고 판단하면 포인터에 대해서는 널 포인터를 리턴하고, 레퍼런스에 대해서는 `std::bad_cast` 익셉션을 발생시킨다.

```cpp
class Base
{
    public:
        virtual ~Base() = default;
};

class Derived : public Base
{
    public:
        virtual ~Derived() = default;
};
```

이때 dynamic_cast()에 대한 올바른 사용 예는 다음과 같다.

```cpp
Base* b;
Derived* d = new Derived();
b = d;
d = dynamic_cast<Derived*>(b);
```

반면 레퍼런스에 대해 다음과 같이 dynamic_cast()를 적용하면 익셉션이 발생한다.

```cpp
Base base;
Derived derived;
Base& br = base;

try{
    Derived& dr = dynamic_cast<Derived&>(br);
} catch(const bad_cast&){
    cout << "Bad cast!" << endl;
}
```

static_cast()나 reinterpret_cast()로도 같은 상속 계층의 하위 타입으로 캐스팅할 수 있다. 차이점은 dynamic_cast()는 실행 시간에 타입 검사를 수행하는 반면 static_cast()나 reinterpret_cast()는 문제가 되는 타입도 그냥 캐스팅해버린다는 것이다.

실행 시간의 타입 정보는 객체의 vtable에 저장된다. 따라서 dynamic_cast()를 적용하려면 클래스에 virtual 메서드가 최소한 한 개 이상 있어야 한다.

### 5.5 캐스팅 정리

| 상황 | 캐스팅 방법 |
|-|-|
const 속성 제거 | const_cast()|
언어에서 허용하는 명시적변환(int->double, int->double) | static_cast()
사용자 정의 생성자나 변환 연산자에서 지원하는 명시적 변환 | static_cast()
서로 관련 없는 타입의 객체끼리 변환 | 불가능
같은 상속 계층에 있는 클래스 타입의 객체 포인터/레퍼런스 사이의 변환 | dynamic_cast() 권장, static_cast()도 가능
서로 관련 없는 타입의 포인터/래퍼런스 사이의 변환 | reinterpret_cast()
함수 포인터 사이의 변환 | reinterpret_cast()

