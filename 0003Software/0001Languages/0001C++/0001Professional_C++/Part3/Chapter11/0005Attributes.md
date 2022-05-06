---
sort: 5
---

# Attributes

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 어트리뷰트

어트리뷰트란 특정 벤더에서만 제공하는 정보나 옵션을 소스 코드에 추가하는 메커니즘이다.

C++ 표준에 어트리뷰트가 추가되기 전에는 벤더마다 `__attribute__`, `__declspec`과 같은 속성을 나름대로 정했다. C++11 부터는 이러한 속성을 모두 이중 대괄호를 이용하여 `[[어트리뷰트]]`와 같은 문법으로 표기하도록 표준화됐다.

C++ 표준은 여섯 가지 어트리뷰트만 지원한다.(여기서는 `[[carries_dependency]]`란 어트리뷰트는 제외하고 설명)

## 2. \[[noreturn]]

`[[noreturn]]`은 함수가 호출한 측으로 제어를 리턴하지 않는다는 것을 의미한다.

주로 프로세스나 스레드를 종료시키거나 익셉션을 던지는 함수에 이 어트리뷰트를 지정한다. 함수에 이 어트리뷰트를 붙이면 의도가 명확히 드러나기 때문에 컴파일러는 경고나 에러를 발생시키지 않는다.

```cpp
[[noreturn]] void forceProgramTermination()
{
    std::exit(1);
}

bool isDongleAvailable()
{
    bool isAvailable = false;
    // 라이선싱 동글이 있는지 검사한다.
    return isAvailable;
}

bool isFeatureLicensed(int featureId)
{
    if (!isDongleAvailable()) {
        // 라이선싱 동글을 찾지 못해서 프로그램 실행을 중단한다.
        forceProgramTermination();
    } else {
        bool isLicensed = false;
        // 동글을 발견하면 라이선스를 검사해서 주어진 기능을 제공하는지 확인한다.
        return isLicensed;
    }
}

int main()
{
    bool isLicensed = isFeatureLicensed(42);
}
```

이 예제 코드를 컴파일 하면 에러나 경고가 발생하지 않으나, `[[noreturn]]` 어트리뷰트를 제거하면 경고 메시지가 출력된다.

## 3. \[[deprecated]]

`[[deprecated]]`는 더 이상 지원하지 않는 대상을 지정할 때 사용한다. 다시 말해 현재 사용할 수는 있지만 권장하지 않는 기능임을 표시한다. 이 어트리뷰트에 지원 중단 사유를 인수로 지정할 수 있다.

```cpp
[[deprecated("Unsafe method, please use xyz")]] void func();
```

지원이 중단돼서 `[[deprecated]]`를 붙인 함수를 코드에서 사용하면 컴파일 에러 또는 경고 메시지가 발생한다.

## 4. \[[fallthrough]]

C++17부터 `[[fallthrough]]`란 어트리뷰트가 추가됐다. 이 어트리뷰트는 switch 문에서 의도적으로 fallthorugh를 적용하고 싶을 때 사용한다. 의도적으로 fallthrough를 적용할 switch 문에 이 어트리뷰트를 지정하지 않으면 컴파일러에서 경고 메시지를 출력할 수 있다.

> 빈 케이스 문에는 지정하지 않아도 된다.

```cpp
switch (backgroundColor) {
    case Color::DarkBlue:
        doSomethingForDarkBlue();
        [[fallthrough]];
    case Color::Black:
        // 배경색이 다크 블루나 블랙일 때 실행된다.
        doSomethingForBlackOrDrakBlue();
        break;
    case Color::Red:
    case Color::Green:
        // 배경색이 레드나 그린일 때 실행된다.
        break;
}
```

## 5. \[[nodiscard]]

값을 리턴하도록 정의된 함수에 `[[nodiscard]]`를 지정하면 그 함수를 이용하는 코드에서 리턴 값을 사용하지 않을 때 경고 메시지가 발생한다.

```cpp
[[nodiscard]] int func()
{
    return 42;
}

int main()
{
    func();
    return 0;
}
```

이렇게 작성하면 컴파일러는 경고 메시지를 출력한다.

이 기능은 에러 코드를 리턴하는 함수에 적용하면 좋다. 이런 함수에 `[[nodiscard]]`를 붙이면 에러 코드를 무시할 수 없게 만들 수 있다.

## 6. \[[maybe_unused]]

`[[maybe_unused]]`는 프로그램에서 사용하지 않는 코드를 발견해도 경고 메시지를 출력하지 말라고 컴파일러에 지시할 때 사용한다.

```cpp
int func(int param1, [[maybe_unused]] int param2)
{
    return 42;
}
```

이렇게 작성하면 두 번째 매개변수를 사용하지 않더라도 경고 메시지를 출력하지 않는다.

## 7. 벤더 정의 어트리뷰트

벤더에서 이런 어트리뷰트를 정의할 때는 프로그램의 의미를 변경하는 용도가 아닌 컴파일러에서 코드를 최적화하거나 에러를 검사하는 데 도움 되기 위한 목적으로 제공해야 한다. 벤더마다 제공하는 어트리뷰트끼리 충돌할 수 있기 때문에 `[[clang::noduplicate]]`와 같이 벤더 이름을 붙이는 것이 좋다.