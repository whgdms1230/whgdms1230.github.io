---
sort: 2
---

# Dynamic Memory Allocation in Objects

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. Spreadsheet 클래스
Spreadsheet 클래스의 첫 버전은 다음과 같다.
* Spreadsheet를 SpreadsheetCell 타입의 2차원 배열로 만든다.
* Spreadsheet에서 특정 위치에 있는 셀을 설정하거나 조회하는 메서드를 정의한다.
* 상용 스프레드시트 애플리케이션은 셀의 위치를 표시할 때 두 개의 축은 모두 숫자로 표시한다.

```cpp
#include <cstddef>
#include "SpreadsheetCell.h"

class Spreadsheet
{
    public:
        Spreadsheet(size_t width, size_t height);
        void setCellAt(size_t x, size_t y, const SpreadsheetCell& cell);
        SpreadsheetCell& getCellAt(size_t x, size_t y);
    private:
        bool inRange(size_t value, size_t upper) const;
        size_t mWidth = 0;
        size_t mHeight = 0;
        SpreadsheetCell** mCells = nullptr;
};
```

여기서 멤버의 타입을 SpreadsheetCell 타입의 표준 2차원 배열이 아니라 SpreadsheetCell** 타입으로 정의했다. 그 이유는 Spreadsheet 객체마다 크기가 다를 수 있기 때문에 이 클래스의 생성자에서 클라이언트가 지정한 높이와 너비에 맞게 2차원 배열을 동적으로 할당하기 위해서이다.

2차원 배열을 동적으로 할당하기 위해 생성자를 다음과 같이 작성한다.

```cpp
Spreadsheet::Spreadsheet(size_t width, size_t height) : mWidth(width), mHeight(height)
{
    mCells = new SpreadsheetCell*[mWidth];
    for (size_t i =0; i < mWidth; i++) {
        mCells[i] = new SpreadsheetCell[mHeight];
    }
}
```

다음은 셀 하나를 읽고 쓰는 메서드를 구현한 코드다.

```cpp
void Spreadsheet::setCellAt(int x, int y, const SpreadsheetCell& cell)
{
    if (!inRange(x, mWidth) || inRange(y, mHeight)) {
        throw std::out_of_range("");
    }
    mCells[x][y] = cell;
}

SpreadsheetCell& Spreadsheet::getCellAt(int x, int y)
{
    if (!inRange(x, mWidth) || inRange(y, mHeight)) {
        throw std::out_of_rnage("");
    }
    return mCells[x][y];
}
```

## 2. 소멸자로 메모리 해제하기

객체 안에서 동적으로 할당한 메모리는 소멸자에서 해제하는 것이 바람직하다. 소멸자의 이름은 클래스의 이름과 같고, 그 앞에 틸드(~) 기호를 붙인다. 소멸자는 인수를 받지 않으며 단 하나만 존재한다.

```cpp
class Spreadsheet
{
    public:
        Spreadsheet(size_t width, size_t height);
        ~Spreadsheet();
        // ...
};

Spreadsheet::~Spreadsheet()
{
    for(size_t i = 0; i < mWidth; i++) {
        delete [] mCells[i];
    }
    delete [] mCells;
    mCells = nullptr;
}
```

## 3. 복제와 대입 처리하기

객채의 복제 또는 대입 시 int, double, 포인터와 같은 기본 타입에 대해서는 비트 단위 복제, 얕은 복제 또는 대입이 적용된다. 하지만 동적으로 할당한 객체를 포함한다면, 복제와 대입에 대해 따로 처리해주어야 한다.

객체의 잘못된 복제 코드를 보면 다음과 같다.

```cpp
#include "Spreadsheet.h"

void printSpreadsheet(Spreadsheet s)
{
    // ...
}

int main()
{
    Spreadsheet s1(4, 3);
    printSpreadsheet(s1);
    return 0;
}
```

Spreadsheet는 mCells라는 포인터 변수 하나만 갖고있는데, 얕은 복제를 적용하면 대상 객체를 mCells에 담긴 데이터가 아닌 mCells 포인터의 복제본만 받는다. 따라서 s와 s1이 같은 데이터를 가리키는 포인터가 되는 상황이 발생한다.

이 상태에서는 s가 변경하면 mCells가 가리키는 대상을 변경할 수 있을 뿐만 아니라, printSpreadsheet() 함수가 리턴할 때 s의 소멸자가 호출되면 mCells가 가리키던 메모리를 해제해 버린다. 이는 s1도 올바른 메모리를 가리키지 않게 되는데, 이런 포인터를 댕글링 포인터라 부른다.

다음으로 객체의 잘못된 대입 연산을 보면 다음과 같다.

```cpp
Spreadsheet s1(2, 2), s2(4, 3);
s1 = s2;
```

이는 s1과 s2에 있는 mCells 포인터가 가리키는 메모리가 똑같아지며, s1에서 mCells가 가리키던 메모리는 미아가 되어 메모리 누수가 된다. 그래서 대입 연산자에서는 반드시 깊은 복제를 적용해야 한다.

### 3.1 Spreadsheet 복제 생성자

Spreadsheet 클래스에 다음과 같이 복제 생성자를 작성한다.

```cpp
class Spreadsheet
{
    public:
        Spreadsheet(const Spreadsheet& src);
        // ...
};

Spreadsheet::Spreadsheet(const Spreadsheet& src) : Spreadsheet(src.mWidth, src.mHeight)
{
    for(size_t i = 0; i < mWidth; i++) {
        for(size_t j = 0; j < mHeight; j++) {
            mCells[i][j] = src.mCells[i][j];
        }
    }
}
```

이 코드는 위임 생성자를 사용했다. 생성자 이니셜라이저는 적절한 양의 메모리를 할당하는 작업을 비복제 버전의 생성자에 맡긴다. 그러고 나서 실제로 값을 복제하는 작업을 수행한다. 이는 동적으로 할당된 2차원 배열인 mCells의 깊은 복제 처리 방법을 보여준다.

### 3.2 Spreadsheet 대입 연산자

다음은 Spreadsheet 클래스의 대입 연산자를 작성한다.

```cpp
class Spreadsheet
{
    public:
        Spreadsheet& operator=(const Spreadsheet& rhs);
        // ...
};

Spreadsheet& Spreadsheet::operator=(const Spreadsheet& rhs)
{
    // 자신을 대입하는지 확인
    if (this == &rhs) {
        return *this;
    }

    // 기존 메모리를 해제
    for (size_t i = 0; i < mWidth; i++) {
        delete [] mCells[i];
    }
    delete [] mCells;
    mCells = nullptr;

    // 메모리를 새로 할당
    mWidth = rhs.mWidht;
    mHeight = rhs.mHeight;

    mCells = new SpreadsheetCell*[mWidth];
    for (size_t i = 0; i < mWidth; i++) {
        mCells[i] = new SpreadsheetCell[mHeight];
    }

    // 데이터 복제
    for(size_t i = 0; i < mWidth; i++) {
        for(size_t j = 0; j < mHeight; j++) {
            mCells[i][j] = rhs.mCells[i][j];
        }
    }

    return *this;
}
```

이 코드는 `this` 객체가 비정상적인 상태가 될 수 있다. 예를 들어 메모리를 정상적으로 해제해서 mWidth와 mHeight는 제대로 설정됐지만 메모리를 할당하는 루프문에서 익셉션이 발생했다고 하자. 그러면 이 메서드의 나머지 코드를 건너뛰고 리턴해버린다. 이렇게 Spreadsheet 인스턴스가 손상됐기 때문에 mCells는 필요한 만큼의 메모리를 갖지 못한다. 결국 이 코드는 익셉션이 발생하면 문제가 생긴다.

이런 경우 모두 정상적으로 처리하거나, 그렇지 못하면 `this` 객체를 건드리지 않아야 한다. 예외가 발생해도 문제가 생기지 않게 대입 연산자를 구현하려면 복제 후 맞바꾸기 패턴을 적용하는 것이 좋다.

##### 복제 후 맞바꾸기

비멤버 swap() 함수를 만들고 Spreadsheet 클래스의 프렌드로 지정한다.

> 비멤버 함수로 구현하는 이유는 다양한 표준 라이브러리 알고리즘에서 활용하기 위함이다.

```cpp
class Spreadsheet
{
    public:
        Spreadsheet& operator=(const Spreadsheet& rhs);
        friend void swap(Spreadsheet& first, Spreadsheet& second) noexcept;
        // ...
};

void swap(Spreadsheet& first, Spreadsheet& second) noexcept
{
    using std::swap;

    swap(first.mWidth, second.mWidth);
    swap(first.mHeight, second.mHeight);
    swap(first.mCells, second.mCells);
}
```

복제 후 맞바꾸기 패턴을 익셉션에 안전하게 구현하려면 swap() 함수에서 절대로 익셉션을 던지면 안되기 때문에 `noexcept`로 지정한다. swap() 함수에서 실제로 데이터 멤버를 교체하는 작업은 `<utility>` 헤더에서 제공하는 std::swap()으로 처리한다.

이렇게 swap() 함수를 익셉션에 안전하게 만들면 대입 연산자를 다음과 같이 구현할 수 있다.

```cpp
Spreadsheet& Spreadsheet::operator=(const Spreadsheet& rhs)
{
    // 자신을 대입하는지 호가인
    if(this == &rhs) {
        return *this;
    }

    Spreadsheet temp(rhs); // 모든 작업을 임시 인스턴스에서 처리
    swap(*this, temp); // 익셉션을 발생하지 않는 연산으로만 작업을 처리
    return *this;
}
```

대입 연산자는 복제 후 맞바꾸기 패턴에 따라 구현하는 것이 바람직하다. 그래야 익셉션에 대한 안전성을 높일 수 있기 때문이다.

### 3.3 대입과 값 전달 방식 금지

대입이나 값 전달 방식을 금지하려면 Spreadsheet 클래스를 다음과 같이 정의하면 된다.

```cpp
class Spreadsheet
{
    public:
        Spreadsheet(size_t width, size_t height);
        Spreadsheet(const Spreadsheet& src) = delete;
        ~Spreadsheet();
        Spreadsheet& operator=(const Spreadshset& rhs) = delete;
        // ...
};
```

만약 이렇게 작성한 Spreadsheet 객체를 복제하거나 대입하려고 하면 에러 메시지가 출력된다.

## 4. 이동 의미론으로 이동 처리하기

객체에 이동 의미론을 적용하려면 이동 생성자와 이동 대입 연산자를 정의해야 한다. 그러면 컴파일러는 원본 객체를 임시 객체로 만들어서 대입 연산을 수행한 뒤 임시 객체를 제거한다. 이 과정에서 이동 생성자와 이동 대입 연산자를 활용한다.

원본 객체에 있는 데이터 멤버를 새 객체로 이동시킬 때, 원본 객체의 데이터 멤버는 대부분 널 값으로 초기화된다. 이렇게 메모리를 비롯한 리소스의 소유권을 다른 객체로 이동시킨다.

이 과정은 멤버 변수에 대한 얕은 복제와 비슷하며 할당된 메모리나 다른 리소스에 대한 소유권을 전환함으로써 댕글링 포인터나 메모리 누수를 방지한다

### 4.1 우측값 레퍼런스

C++에서 우측값은 리터럴, 임시 객체, 값처럼 좌측값이 아닌 모든 대상을 가리킨다.

우측값 레퍼런스란 우측값에 대한 레퍼런스를 말한다. 우측값이 임시 객체일 때 적용되는 개념으로, 임시 객체에 대해 적용할 함수를 컴파일러가 선택하기 위한 용도로 사용한다.

우측값 레퍼런스로 구현하려면 크기가 큰 값을 복사하는 연산이 나오더라도 컴파일러는 이 값이 나중에 삭제될 임시 객체라는 점을 이용하여 그 값에 우측값에 대한 포인터를 복사하는 방식으로 실행할 수 있다.

함수의 매개변수에 `&&`를 붙여서 우측값 레퍼런스로 만들 수 있다. 다음 예제에서 handleMessage() 함수의 좌측값 레퍼런스를 받는 정의와 우측값 레퍼런스를 받는 정의를 작성한다.

```cpp
// 좌측값 레퍼런스 매개변수
void handleMessage(std::string& message)
{
    cout << "handleMessage with lvalue reference: " << message << endl;
}

// 우측값 레퍼런스 매개변수
void handleMessage(std::string&& message)
{
    cout << "handleMessage with rvalue reference: " << message << endl;
}
```

다음으로 handleMessage() 함수를 호출하는 부분이다. 
```cpp
std::string a = "Hello ";
std::string b = "World";

handleMessage(a); // handleMessage(string& value) 호출

handleMessage(a + b); // handleMessage(string&& value) 호출

handleMessage("Hello World"); // handleMessage(string&& value) 호출

handleMessage(std::move(b)); // handleMessage(string&& value) 호출
```

a + b 의 경우 표현식으로 임시변수가 생성되므로 우측값 레퍼런스 버전이 호출된다. 또한 리터럴을 전달해도 우측값 레퍼런스 버전이 호출된다. 리터럴은 좌측값이 될 수 없기 때문이다.

좌측값을 우측값으로 캐스팅하는 std::move()를 사용하면 좌측값인 변수를 이용하여 우측값 레퍼런스 버전의 handleMessage()를 호출하게 만들 수 있다.

> handleMessage() 함수 안에서 우측값 레퍼런스 타입인 message 매개변수는 좌측값이기 때문에 해당 함수 안에서 다른 함수에 우측값으로 전달하려면 std::move()를 이용해야 한다.

### 4.2 이동 의미론 구현 방법

이동 의미론은 우측값 레퍼런스로 구현한다. 클래스에 이동 의미론을 추가하려면 이동 생성자와 이동 대입 연산자를 구현해야 한다. 이때 이동 생성자와 이동 대입 연산자를 noexcept로 지정해서 두 메서드에서 익셉션이 절대로 발생하지 않는다고 컴파일러에 알려줘야 한다. 특히 표준 라이브러리와 호환성을 유지하려면 반드시 이렇게 해야 한다.

Spreadsheet 클래스에 이동 생성자와 이동 대입 연산자를 추가한 코드이다. 여기에 cleanup()과 moveForm()이란 헬퍼 메서드도 추가한다. cleanup()은 소멸자와 이동 대입 연산자에서 사용하고, moveFrom()은 원본 객체의 멤버 변수를 대상 객체로 이동시킨 뒤 원본 객체를 리셋한다.

```cpp
class Spreadsheet
{
    public:
        Spreadsheet(Spreadsheet&& src) noexcept; // 이동 생성자
        Spreadsheet& operator=(Spreadsheet&& rhs) noexcept; // 이동 대입 연산자
        // ...
    private:
        void cleanup() noexcept;
        void moveFrom(Spreadsheet& src) noexcept;
        // ...
};

void Spreadsheet::cleanup() noexcept
{
    for(size_t i = 0; i < mWidth; i++) {
        delete[] mCells[i];
    }
    delete[] mCells;
    mCells = nullptr;
    mWidth = mHeight = 0;
}

void Spreadsheet::moveFrom(Spreadsheet& src) noexcept
{
    // 데이터에 대한 얕은 복제
    mWidth = src.mWidth;
    mHeight = src.mHeight;
    mCells = ssrc.mCells;

    // 소유권이 이전됐기 때문에 소스 객체를 리셋한다.
    src.mWidth = 0;
    src.mHeight = 0;
    src.mCells = nullptr;
}

// 이동 생성자
Spreadsheet::Spreadsheet(Spreadsheet&& src) noexcept
{
    moveFrom(src);
}

// 이동 대입 연산자
Spreadsheet Spreadsheet::operator=(Spreadsheet&& rhs) noexcept
{
    // 자기 자신을 대입하는지 확인한다.
    if(this == &rhs) {
        return *this;
    }

    // 예전 메모리를 해제한다.
    cleanup();

    moveFrom(rhs);

    return *this;
}
```

이동 생성자와 이동 대입 연산자는 모두 mCells에 대한 메모리 소유권을 원본 객체에서 새로운 객체로 이동시킨다. 그리고 원본 객체의 소멸자가 이 메모리를 해제하지 않도록 원본 객체의 mCells 포인터를 널 포인터로 리셋한다.

이와 같이 이동 의미론은 원본 객체를 삭제할 때만 유용하다.

#### 4.2.1 명시적으로 삭제하거나 디폴트로 만들기

이동 생성자와 이동 대입 연산자도 명시적으로 삭제하거나 디폴트로 만들 수 있다.

사용자가 클래스에 복제 생성자, 복제 대입 연산자, 이동 대입 연산자, 소멸자를 직접 선언하지 않았다면 컴파일러가 디폴트 이동 생성자를 만들어준다. 또한 사용자가 클래스에 복제 생성자, 이동 생성자, 복제 대입 연산자, 소멸자를 직접 선언하지 않았다면 컴파일러는 디폴트 이동 대입 연산자를 만들어 준다.

> 5의 규칙 : 클래스에 동적 할당 메모리를 사용하는 코드를 작성하면 소멸자, 복제 생성자, 이동 생성자, 복제 대입 연산자, 이동 대입 연산자를 반드시 구현한다.

#### 4.2.2 객체 데이터 멤버 이동하기

만약 데이터 멤버가 객체라면 std::move()로 이동시켜야 한다. 만약, Spreadsheet 클래스에 mName이란 이름의 std::string 타입의 멤버가 있다면, 다음과 같이 std::move()를 이용하여 moveFrom() 함수를 구현한다.

```cpp
void Spreadsheet::moveFrom(Spreadsheet& src) noexcept
{
    // 객체 데이터 멤버를 이동 시킴
    mName = std::move(src.mName);

    // 데이터에 대한 얕은 복제
    mWidth = src.mWidth;
    mHeight = src.mHeight;
    mCells = ssrc.mCells;

    // 소유권이 이전됐기 때문에 소스 객체를 리셋한다.
    src.mWidth = 0;
    src.mHeight = 0;
    src.mCells = nullptr;
}
```

#### 4.2.3 swap() 함수로 구현한 이동 생성자와 이동 대입 연산자

만약, Spreadsheet 클래스에 데이터 멤버를 새로 추가할 때 swap() 함수와 moveFrom() 메서드를 동시에 수정해야 한다. 만약 둘 중에 하나라도 수정하지 않으면 버그가 발생하게 되는데, 이러한 문제를 막기 위해 이동 생성자와 이동 대입 연산자를 디폴트 생성자와 swap() 함수로 구현한다.

```cpp
class Spreadsheet
{
    private:
        Spreadsheet() = default;
        // ...
};

Spreadsheet::Spreadsheet(Spreadsheet&& src) noexcept : Spreadsheet()
{
    swap(*this, src);
}

Spreadsheet& Spreadsheet::operator=(Spreadsheet&& rhs) noexcept
{
    Spreadsheet temp(std::move((rhs));
    swap(*this, temp);
    return *this;
}
```

먼저, Spreadsheet 클래스에 디폴트 생성자를 추가한다. 이 클래스의 사용자가 디폴트 생성자를 직접 사용할 일은 없기 때문에 private로 지정한다. 다음으로 cleanup()과 moveFrom() 헬퍼 메서드를 삭제한다. cleanup() 메서드에 있던 코드를 소멸자로 옮긴다.

이동 생성자는 디폴트 생성자에 작업을 위임한다. 그런 다음 디폴트 생성자가 만든 *this를 원본 객체와 맞바꾼다. 이동 대입 연산자는 먼저 rhs로 이동 생성해서 Spreadsheet에 대한 로컬 인스턴스를 만든다. 그러고 나서 이렇게 이동 생성된 로컬 Spreadsheet 인스턴스를 *this와 맞바꾼다.

디폴트 생성자와 swap() 함수로 이동 생성자와 이동 대입 연산자를 구현하면, 효율성은 떨어질 수 있으나, 버그 발생 확률을 낮출 수 있다.

### 4.3 Spreadsheet의 이동 연산자 테스트

테스트 결과는 책 참고

```cpp
Spreadsheet createObject()
{
    return Spreadsheet(3, 2);
}

int main()
{
    vector<Spreadsheet> vec;
    for(int i = 0; i < 2; ++i) {
        cout << "Iteration " << i << endl;
        vec.push_back(Spreadsheet(100, 100));
        cout << endl;
    }

    Spreadsheet s(2,3);
    s = creageObject();

    Spreadsheet s2(5,6);
    s2 = s;
    return 0;
}
```

### 4.4 이동 의미론으로 swap 함수 구현하기

이동 의미론으로 성능을 높이는 또 다른 예제로 두 객체를 스왑하는 swap() 함수를 살펴보자. 다음에 나온 swapCopy() 함수는 이동 의미론을 적용하지 않았다.

```cpp
void swapCopy(T& a, T& b)
{
    T temp(a);
    a = b;
    b = temp;
}
```

먼저 a를 temp에 복제한 뒤, b를 a에 복제하고, 마지막으로 temp를 b에 복제했다. 그런데 만약 T가 복제하기에 상당히 무거우면 성능이 크게 떨어진다. 이럴 때는 다음과 같이 이동 의미론을 적용해서 복제가 발생하지 않도록 구현한다.

```cpp
void swapMove(T& a, T& b)
{
    T temp(std::move(a));
    a = std::move(b);
    b = std::move(temp);
}
```

표준 라이브러리의 std::swap()이 바로 이렇게 구현되어 있다.

## 5. 영의 규칙

영의 규칙이란 다섯 가지 특수 멤버 함수(소멸자, 복제 생성자, 이동 생성자, 복제 대입 연산자, 이동 대입 연산자)를 구현할 필요가 없도록 클래스를 디자인해야 한다는 것이다.

이는 메모리를 동적으로 할당하지 말고 표준 라이브러리 컨테이너와 같은 최신 구문을 활용해야 한다.

예를 들어 `SpreadsheetCell**` 이란 데이터 멤버 대신 `vector<vector<SpreadsheetCell>>`을 사용하여 `vector`가 베모리를 자동으로 관리한다는 특성을 이용함으로써, 다섯 가지 특수 멤버의 필요성을 없앤다.

이렇게 최신 C++로 작성할 때에는 영의 규칙을 적용한다.