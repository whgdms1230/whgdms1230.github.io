---
sort: 4
---

# Different Kind of Data Member

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. static 데이터 멤버

static 데이터 멤버는 객체가 아닌 클래스에 속한다. 이는 전역 변수와 비슷하지만 자신이 속한 클래스 범위를 벗어날 수 없다.

static 데이터 멤버로 카운터를 구현하여, 각각의 스프레드시트 객체에 숫자로 된 고유한 ID를 부여하도록 Spreadsheet 클래스를 정의하면 다음과 같다.

```cpp
class Spreadsheet
{
    // ...
    private:
        static size_t sCounter;
    // ...
};
```

static 클래스 멤버를 정의하면 소스 파일에서 이 멤버에 대한 공간을 할당해야 한다. 이 작업은 주로 해당 클래스의 메서드를 정의하는 소스 파일에서 처리한다. 기본적으로 0으로 초기화 되며, static 포인터는 nullptr로 초기화된다.

```cpp
size_t Spreadsheet::sCounter;
size_t Spreadsheet::sCounter = 0;   // 0 으로 초기화 명시
```

### 1.1 인라인 변수

C++17부터 static 데이터 멤버를 inline으로 선언할 수 있다. 그러면 소스 파일에 공간을 따로 할당하지 않아도 된다.

```cpp
class Spreadsheet
{
    // ...
    private:
        static inline size_t sCounter = 0;
    // ...
};
```

클래스에서 inline으로 선언하면 소스 파일에서 이 멤버에 대한 공간 할당을 하지 않아도 된다.

### 1.2 클래스 메서드에서 static 데이터 멤버 접근하기

클래스 메서드 안에서는 stati 데이터 멤버를 일반 데이터 멤버인 것 처럼 사용한다.

다음 예시는 Spreadsheet에 mId란 데이터 멤버를 만들고, 이 값을 Spreadsheet 생성자에서 sCounter의 값으로 치기화 하는 경우이다.

```cpp
class Spreadsheet
{
    public:
        size_t getId() const;
    private:
        static size_t sCounter;
        size_t mId = 0;
};

// 생성자 구현 코드
Spreadsheet::Spreadsheet(size_t width, size_t height) : mId(sCounter++), mWidth(width), mHeight(height)
{
    mCells = new Spreadsheetell*[mWidth];
    for(size_t i = 0; i < mWith; i++){
        mCells[i] new SpreadsheetCell[mHeight];
    }
}
```

### 1.3 메서드 밖에서 static 데이터 멤버 접근하기

static 데이터 멤버에 대해서 접근 제한자/지정자를 적용할 수 있다. sCounter를 private로 선언하면 클래스 메서드 밖에서 접근할 수 없으며, public으로 선언하면 클래스 메서드 밖에서 접근할 수 있다.

public으로 선언된 static 데이터 멤버를 클래스 메서드 밖에서 접근할 때는 스코프 지정 연산자를 붙이면 된다.

하지만 데이터 멤버를 public으로 선언하는 것은 바람직하지 않으며, 반드시 public get/set 메서드로 접근해야 한다. static 데이터 멤버를 외부에서 접근하려면 static get/set 메서드를 사용하는 방식으로 구현한다.

## 2. const static 데이터 멤버

특정 클래스에만 적용되는 상수인 클래스 상수를 정의할 때는 전역 상수로 선언하지 않고 반드시 static const(또는 const static) 데이터 멤버로 선언한다.

정수 및 열거 타입의 static const 데이터 멤버는 별도로 인라인 변수로 지정할 필요 없이 클래스 정의 코드에서 선언과 동시에 초기화할 수 있다.

예를 들어 스프레드시트의 최대 높이와 폭을 지정하는 경우, 이 값을 클래스의 static const 멤버로 정의한다.

```cpp
class Spreadsheet
{
    public:
        static const size_t kMaxHeight = 100;
        static const size_t kMaxWidth = 100;
};
```

이렇게 선언한 상수를 생성자에서 다음과 같이 활용할 수 있다.

```cpp
Spreadsheet::Spreadsheet(size_t width, size_t height)
    : mId(sCounter++)
    , mWidth(std::min(width, kMaxWidth))
    , mHeight(std::min(height, kMaxHeight))
{
    mCells = new Spreadsheetell*[mWidth];
    for(size_t i = 0; i < mWith; i++){
        mCells[i] new SpreadsheetCell[mHeight];
    }
}
```

## 3. 레퍼런스 데이터 멤버

하나의 스프레드시트 어플래케이션에서 여러 스프레드시트를 관리하기 위해 스프레드시트와 통신할 수 있어야 한다. 마찬가치로 스프레드시트마다 애플리케이션 객체에 대한 레퍼런스를 저장할 수도 있다. 이를 위해 Spreadsheet 클래스와 SpreadsheetApplication 클래스가 서로에 대해 알아야 한다.

`#include` 문으로 서로를 참조하게 되면 순환 참조가 발생하여 해결할 수 없게 된다. 이런 경우 헤더 파일 중 어느 한 곳에서 포워드 선언을 해야한다. 컴파일러가 SpreadsheetApplication에 대해 알 수 있도록 포워드 선언을 적용해서 Spreadsheet 클래스를 다시 정의하면 다음과 같다.

```cpp
class SpreadsheetApplication;   // 포워드 선언

class Spreadsheet
{
    public:
        Spreadsheet(size_t width, size_t height, SpreadsheetApplication& theApp);
    private:
        SpreadsheetApplication& mTheApp;
};
```

SpreadsheetApplication 레퍼런스를 데이터 멤버로 추가했다. 이때 포인터보다 레퍼런스를 사용하는 것이 바람직한데, Spreadsheet는 항상 SpreadsheetApplication을 참조하기 때문이다. 포인터를 사용하면 이런 관계를 보장할 수 없다.

생성자의 구현 코드에서 다음과 같이 생성자 이니셜라이저에서 레퍼런스를 지정한다.

```cpp
Spreadsheet::Spreadsheet(size_t width, size_t height, SpreadsheetApplication& theApp)
    : mId(sCounter++)
    , mWidth(std::min(width, kMaxWidth))
    , mHeight(std::min(height, kMaxHeight))
    , mTheApp(theApp)
{
    // ...
}
```

> 레퍼런스 멤버를 반드시 복제 생성자에서도 초기화해야 한다.

> 레퍼런스를 초기화한 뒤에는 그 레퍼런스가 가리키는 객체를 변경할 수 없다. 대입 연산자로 레퍼런스에 값을 대입할 수 없다. 때로는 현재 클래스에서 레퍼런스 데이터 멤버에 대해 대입 연산자를 제공할 수 없을 수도 있다. 이때는 대입 연산자가 deleted로 지정된다.

## 4. const 레퍼런스 데이터 멤버

일반 레퍼런스와 마찬가지로 레퍼런스 멤버도 const 객체를 가리킬 수 있다. 예를 들어 Spreadsheet가 애플리케이션 객체에 대해 const 레퍼런스만 가지도록 하면 다음과 같다.

```cpp
class Spreadsheet
{
    public:
        Spreadsheet(size_t width, size_t height, const SpreadsheetApplication& theApp);
    private:
        const SpreadsheetApplication& mTheApp;
};
```

const 레퍼런스 SpreadsheetApplication 데이터 멤버는 SpreadsheetApplication 객체의 const 메서드만 호출할 수 있다.

> 레퍼런스 멤버를 static이나 static const로 지정할 수도 있으나, 이렇게 사용할 일은 거의 없다.