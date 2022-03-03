---
sort: 8
---

# Building Stable Interfaces

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 인터페이스 클래스와 구현 클래스

인터페이스를 간결하게 구성하고 구현 세부사항을 모두 숨겨서 인터페이스를 안정적으로 유지하기 위하여 작성할 클래스마다 인터페이스 클래스와 구현 클래스를 따로 정의한다.

구현 클래스는 흔히 작성하는 클래스를 말하며, 인터페이스 클래스는 구현 클래스와 똑같이 public 메서드를 제공하되 구현 클래스 객체에 대한 포인터를 갖는 데이터 멤버 하나만 정의한다. 이를 핌플 이디엄(pimpl idiom, private implementation idiom) 또는 브릿지 패턴(bridge pattern)이라 부른다.

인터페이스 클래스 메서드는 단순히 구현 클래스 객체에 있는 동일한 메서드를 호출하도록 구현한다. 그러면 구현 코드가 변해도 public 메서드로 구성된 인터페이스 클래스는 영향을 받지 않는다. 따라서 다시 컴파일할 일이 줄어든다.

주의할 점은 인터페이스 클래스에 존재하는 유일한 데이터 멤버를 구현 클래스에 대한 포인터로 정의해야 제대로 효과를 발휘한다는 것이다. 데이터 멤버가 포인터가 아닌 값 타입이면 구현 클래스가 변경될 때마다 다시 컴파일해야 한다.

## 2. Spreadsheet 클래스에 적용

Spreadsheet 클래스에 이 방식을 적용하려면 먼저 다음과 같이 Spreadsheet 클래스를 public 인터페이스 클래스로 정의한다.

```cpp
#include "SpreadsheetCell.h"
#include <memory>

// 포워드 선언
class SpreadsheetApplication;

class Spreadsheet
{
    public:
        Spreadsheet(const SpreadsheetApplication& theApp,
                    size_t width = kMaxWidth,
                    size_t height = kMaxHeight);
        Spreadsheet(const Spreadsheet& src);
        ~Spreadsheet();

        Spreadsheet& operator=(const Spreadsheet& rhs);

        void setCellAt(size_t x, size_t y, const SpreadsheetCell& cell);
        SpreadsheetCell& getCellAt(size_t x, size_t y);

        size_t getId() const;

        static const size_t kMaxHeight = 100;
        static const size_t kMaxWidth = 100;

        friend void swap(Spreadsheet& first, Spreadsheet& second) noexcept;

    private:
        class Impl;
        std::unique_ptr<Impl> mImpl;
};
```

구현 코드는 Impl이란 이름으로 private 중첩 클래스로 정의한다. Spreadsheet 클래스 말고는 구현 클래스에 대해 알 필요가 없기 때문이다. 이렇게 하면 Spreadsheet 클래스는 Impl 인스턴스에 대한 포인터인 데이터 멤버 하나만 갖게 된다.

중첩 클래스인 Spreadsheet::Impl의 인터페이스는 기존 Spreadsheet 클래스와 거의 같다. 하지만 Impl 클래스는 Spreadsheet의 private 중첩 클래스이기 때문에 다음과 같이 두 Spreadsheet::Impl 객체를 맞바꾸는 전역 friend swap() 함수를 가질 수 없다.

```cpp
friend void swap(Spreadsheet::Impl& first, Spreadsheet::Impl& second) noexcept;
```

따라서 다음과 같이 Spreadsheet::Impl 클래스에서 swap()을 private 메서드로 정의해야 한다.

```cpp
void swap(Impl& other) noexcept;

// 구현 코드
void Spreadsheet::Impl::swap(ImplU& other) noexcept
{
    using std::swap;

    swap(mWidth, other.mWidth);
    swap(mHeight, other.mHeight);
    swap(mCells, other.mCells);
}
```

Spreadsheet 클래스는 구현 클래스를 가리키는 unique_ptr를 가지고 있기 때문에 사용자 선언 소멸자가 있어야 한다. 이 소멸자는 특별히 할 일이 없기 때문에 구현 파일에 다음과 같이 디폴트로 지정한다.

```cpp
Spreadsheet::~Spreadsheet() = default;
```

setCellAt()이나 getCellAt()과 같은 Spreadsheet의 메서드에 대한 구현 코드는 들어온 요청을 내부 Impl 객체로 그냥 전달하면 된다.

```cpp
void Spreadsheet::setCellAt(size_t x, size_t y, const SpreadsheetCell& cell)
{
    mImpl->setCellAt(x, y, cell);
}

SpreadsheetCell& spreadsheet::getCellAt(size_t x, size_t y)
{
    return mImpl->getCellAt(x, y);
}
```

이렇게 하려면 Spreadsheet의 생성자에서 반드시 Impl 객체를 생성하도록 구현해야 한다.

```cpp
Spreadsheet::Spreadsheet(const SpreadsheetApplication& theApp,
                        size_t widht,
                        size_t height)
{
    mImpl = std::make_unique<Impl>(theApp, width, height);
}

Spreadsheet::Spreadsheet(const Spreadsheet& src)
{
    mImpl = std::make_unique<Impl>(*src.mImpl);
}
```

복제 생성자는 원본 스프레드시트(src)의 내부 Impl 객체를 복제해야하기 때문에 Impl에 대한 포인터가 아닌 레퍼런스를 인수로 받는다. 따라서 mImpl 포인터를 역참조해서 객체 자체에 접근해야 생성자를 호출할 때 이 레퍼런스를 받을 수 있다.

Spreadsheet의 대입 연산자도 마찬가지로 내부 Impl의 대입 연산자로 전달한다.

```cpp
Spreadsheet& Spreadsheet::operator=(const Spreadsheet& rhs)
{
    *mImpl = *rhs.mImpl;
    return *this;
}
```

대입 연산자는 현재 호출을 Impl의 대입 연산자로 포워딩해야 하는데, 이 연산자는 객체를 직접 복제할 때만 구동된다. mImpl 포인터를 역참조하면 강제로 직접 객체 대입 방식을 적용하기 때문에 Impl의 대입 연산자를 호출할 수 있다.

swap() 함수는 다음과 같이 단순히 데이터 멤버를 맞바꾸기만 한다.

```cpp
void swap(Spreadsheet& first, Spreadsheet& second) noexcept
{
    using std::swap;

    swap(first.mImpl, second.mImpl);
}
```