---
sort: 7
---

# Header Files

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 헤더 파일

헤더 파일이란 서브시스템이나 코드에 추상 인터페이스를 제공하는 메커니즘이다. 헤더를 사용할 때 헤더 파일이 중복되거나 순환 참조가 발생하지 않도록 주의해야 한다.

인클루드 가드라는 메커니즘을 사용하면 중복 정의를 피할 수 있다. `#ifndef` 지시자를 이용하여 특정한 키가 이미 정의됐는지 검사하고, 정의되었다면 `#endif` 문장까지 건너뛴다. 만약 정의되지 않았다면 바로 다음 문장에 `#define`으로 정의하여 나중에 같은 내용을 인클루드할 때 건너뛰게 만들 수 있다.

```cpp
#ifndef LOGGER_H
#define LOGGER_H

class Logger
{
    // ...
};

#endif // LOGGER_H
```

요즘 나온 컴파일러는 거의 모두 `#pragma once` 디렉티브를 제공하기 때문에 이렇게 인클루드 가드를 작성하지 않아도 된다.

```cpp
#pragma once

class Logger
{
    // ...
};
```

헤더 파일에 관련된 문제를 방지하기 위한 또 다른 기능으로 전방 선언(포워드 선언)이란 것도 있다. 어떤 클래스를 참조해야 하는데 그 클래스에 대한 헤더 파일을 인클루드 할 수 없다면, 그 클래스에 대한 정의를 `#include` 메커니즘으로 불러오지 않고도 사용하게 만들 수 있다.

예를 들어 Logger 클래스에서 사용자 설정사항을 관리하도록 Preferences 란 클래스를 사용한다고 하자. 그런데 Preferances 클래스는 다시 Logger 클래스를 이용해서 순환 의존 관계가 발생한다. 이런 상황은 인클루드 가드로는 해결할 수 없고, 전방 선언을 해야한다.

다음 코드를 보면 헤더 파일을 인클루드하지 않고도 Preferences 클래스를 사용하도록 Logger.h 헤더 파일에 Preferences 클래스를 전방 선언한다.

```cpp
#pragma once

#include <string_view>

class Perferences; // 전방 선언

class Logger
{
    public:
        static void setPreferences(const Preferences& prefs);
        static void logError(std::string_view error);
};
```

헤더 파일에 다른 헤더 파일을 인클루드 하는 대신 전방 선언을 적용하는 것이 바람직하다. 그러면 작성한 헤더 파일이 다른 헤더 파일을 의존하는 것을 제거해서 컴파일 시간이 크게 줄어들기 때문이다. 물론 전방 선언한 타입을 선언한 헤더 파일을 구현 파일에서 정확히 인클루드 해야 한다.

C++17 부터 특정한 헤더 파일이 존재하는지 확인할 수 있도록 `__has_include("파일명")`과 `__has_include(<파일명>)`이란 전처리 상수가 추가됐다. 이 상수는 해당 헤더 파일이 존재하면 1, 그렇지 않으면 0이 된다.

예를 들어 C++17에서 `<optional>` 헤더 파일이 정식으로 지원되기 전에 `<experimental/optional>`이란 초기 버전의 헤더 파일이 제공됐다. 이때 `__has_include()`를 이용하면 현재 시스템에서 둘 중 어느 헤더 파일을 지원하는지 확인할 수 있다.

```cpp
#if __has_include(<optional>)
    #include <optional>
#elif __has_include(<experimental/optional>)
    #include <experimental/optional>
#endif
```