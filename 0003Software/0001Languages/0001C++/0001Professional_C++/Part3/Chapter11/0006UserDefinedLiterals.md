---
sort: 6
---

# User-Defined Literals

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 사용자 정의 리터럴

C++는 표준 리터럴을 다양하게 제공한다. 이런 리터럴은 프로그램에서 곧바로 사용할 수 있다.

* 'a' : 문자
* "character array" : 0으로 끝나는 문자 배열, 즉 C 스타일 스트링
* 3.14f : 부동소수점(float) 값
* 0xabc : 16진수 숫자

C++은 사용자가 리터럴을 직접 정의하는 기능도 제공한다. 이러한 사용자 정의 리터럴은 반드시 언더스코어 `_`로 시작해야 한다. 그리고 언더스코어 바로 다음에 나오는 첫 문자는 반드시 소문자여야 한다. 예를 들어 _i, _s, _km, _miles 등과 같다.

사용자 정의 리터럴은 리터럴 연산자를 정의하는 방식으로 구현한다. 리터럴 연산자는 미가공 모드(rqw mode)나 가공 모드(cooked mode)로 작동한다.

미가공 모드에서는 단순히 문자가 나열된 것으로 취급하지만, 가공 모드에서는 특정한 타입의 값으로 해석한다. 123 이란 리터럴을 미가공 모드의 리터럴 연산자는 '1', '2', '3'이란 문자의 나열로 받고, 가공 모드의 리터럴 연산자는 123이란 정숫값으로 처리한다.

0x23 이란 리터럴을 미가공 모드에서는 '0', 'x', '2', '3'이란 문자열의 나열로 처리하고, 가공 모드에서는 35라는 정수로 처리한다.

가공 모드의 리터럴 연산자는 다음 두 조건 중 하나를 만족해야 한다.

* 숫자값을 처리하려면 타입이 unsigned long long, long double, char, wchar_t, char16_t, char32_t 중 하나로 된 매개변수가 필요하다.
* 스트링을 처리하려면 매개변수가 두 개 있어야 한다. 첫 번째는 문자 배열을, 두 번째는 그 배열의 길이를 지정한다(예: const char* str, size_t len).

예를 들어 복소수 리터럴을 정의하는 가공 모드 리터럴 연산자를 다음과 같이 _i 라는 사용자 정의 리터럴로 구현할 수 있다.

```cpp
std::complex<long double> operator"" _i(long double d)
{
    return std::complex<long double>(0, d);
}
```

_i 리터럴을 다음과 같이 사용할 수 있다.

```cpp
std::complex<long double> c1 = 9.634_i;
auto c2 = 1.23_i; // c2의 타입은 std::complex<long double>
```

또 다른 예로 std::string 리터럴을 정의하는 사용자 정의 리터럴 _s에 대한 가공 모드의 리터럴 연산자를 다음과 같이 구현할 수 있다.

```cpp
std::string operator"" _s(const char* str, size_t len)
{
    return std::string(str, len);
}
```

_s 리터럴을 다음과 같이 사용할 수 있다.

```cpp
std::string str1 = "Hello World"_s;
auto str2 = "Hello World"_s; // str2의 타입은 std::string
```

미가공 모드 리터럴 연산자를 정의하려면 0으로 끝나는 C 스타일 스트링을 const char* 타입으로 받는 매개변수를 한 개 정의해야 한다. 다음은 _i라는 리터럴을 미가공 모드로 작동하도록 정의한다.

```cpp
std::complex<long double> operator"" _i(const char* p)
{
    // 구현 코드 생략
    // C 스타일 스트링을 파싱해서 복소수로 변환하는 코드를 구현
}
```

미가공 모드 리터럴 연산자를 사용하는 방법은 가공 모드 리터럴 연산자와 동일하다.

## 2. 사용자 정의 리터럴에 대한 표준

C++는 다음과 같은 표준 사용자 정의 리터럴을 제공한다. 표준 사용자 정의 리터럴은 언더스코어로 시작하지 않는다는 점에 주의한다.

* s: std::string 리터럴을 생성한다.
    ```cpp
    auto myString = "Hello World"s;
    ```
    이때 먼저 `using namespace std::string_literals;`를 선언해야 한다.
* sv: std::string_view 리터럴을 생성한다.
    ```cpp
    auto myStringView = "Hello World"sv;
    ```
    이때 먼저 `using namespace std::string_view_literals;`를 선언해야 한다.
* h, min, s, ms, us, ns: 시간 간격을 포현하는 std::chrono::duration 리터럴을 생성한다.
    ```cpp
    auto myDuration = 42min;
    ```
    이때 먼저 `using namespace std::chrono_literals;`를 선언해야 한다.
* i, il, if: 각각 `complex<double>`, `complex<long double>`, `complex<float>` 타입의 복소수 리터럴을 생성한다.
    ```cpp
    auto myComplexNumber = 1.3i;
    ```
    이때 먼저 `using namespace std::complex_literals;`를 선언해야 한다.