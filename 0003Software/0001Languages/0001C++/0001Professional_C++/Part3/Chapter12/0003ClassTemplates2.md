---
sort: 3
---

# Class Templates - 추가 설명

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 컴파일러에서 템플릿을 처리하는 방식

컴파일러는 템플릿 메서드를 정의하는 코드를 발견하면 문법 검사만 하고 템플릿 코드를 실제로 컴파일하지 않는다. 템플릿 정의만 보고서 그 안에서 어떤 타입을 사용할지 알 수 없기 때문이다. 다시 말해 x = y 란 문장에서 x와 y의 타입을 모르면서 컴파일러가 코드를 생성할 수 없다.

컴파일러가 Grid\<int> myIntGrid 처럼 템플릿을 인스턴스화하는 코드를 발견하면 Grid 템플릿의 매개변수 T에 int를 대입해서 int 버전의 Grid 클래스를 생성한다.

이처럼 템플릿 기능이 없을 때는 원소의 타입마다 일일이 클래스를 따로 정의했어야 할 작업을 컴파일러가 대신해주는 것이다. 만약 클래스 템플릿을 정의하는 코드만 작성하고 특정 타입에 대해 인스턴스를 만드는 코드를 작성하지 않으면 클래스 메서드를 정의하는 코드는 컴파일되지 않는다.

### 1.1 선택적 인스턴스화

컴파일러는 항상 제네릭 클래스에 있는 모든 가상 메서드에 대한 코드를 생성한다. 하지만 virtual로 선언하지 않는 다른 메서드는 그중에서 특정 타입에 대해 호출하는 메서드만 컴파일한다. 이를 선택적 인스턴스화라 부른다.

예를 들어 앞에 나온 Grid 클래스 템플릿을 이용하는 코드를 main() 함수에서 다음과 같이 작성한 경우를 보자.

```cpp
Grid<int> myIntGrid;
myIntGrid.at(0, 0) = 10;
```

그러면 컴파일러는 int 버전의 Grid에서 제로 인수 생성자, 소멸자, non-cast at() 메서드만 컴파일한다. 복제 생성자나 대입 연산자, getHeight()에 대한 코드는 생성하지 않는다.

### 1.2 템플릿에 사용할 타입의 요건

타입에 독립적인 코드를 작성하려면 적용할 타입에 대해 어느 정도 고려해야 한다.

예를 들어 Grid 템플릿을 작성할 때 T에 지정한 타입의 원소는 언제든지 소멸될 수 있다는 점을 고려해야 한다. 특히 템플릿의 타입 매개변수에 맞게 대입 연산자를 제공해야 한다면 고려할 사항이 많다.

어떤 템플릿을 인스턴스화할 때 그 템플릿에 있는 연산을 모두 지원하지 않으면 컴파일 에러가 발생한다. 그런데 템플릿을 인스턴스화할 타입이 그 템플릿에 정의된 모든 연산에 적용할 수 없다면 선택적 인스턴스화를 이용하여 그중 일부 메서드를 사용하게 만들면 된다.

## 2. 템플릿 코드를 여러 파일로 나누기

### 2.1 헤더 파일에 템플릿 정의하기

메서드 정의 코드를 클래스 정의 코드가 있는 헤더 파일에 함께 적는 방법이 있다. 그러면 이 템플릿을 사용하는 소스파일에 `#include` 문으로 헤더 파일만 불러오면 컴파일러는 클래스 정의와 메서드 정의를 모두 참조할 수 있다. Grid 예제가 바로 이렇게 처리된다.

또 다른 방법은 템플릿 메서드 정의 코드를 다른 헤더 파일에 적고, 그 헤더 파일을 클래스 정의를 담은 헤더 파일에서 `#include` 문으로 불러오는 것이다. 이때 메서드 정의가 담긴 헤더를 추가하는 `#include` 문은 반드시 클래스 정의 코드 뒤에 적어야 한다. 그렇지 않으면 에러가 발생한다.

```cpp
template <typename T>
class Grid
{
    // 클래스 정의 코드 생략
};

#include "GridDefinitions.h"
```

이처럼 메서드 정의와 클래스 정의를 두 헤더 파일에 나눠서 작성하면 클래스 정의와 메서드 정의를 명확히 구분할 수 있다는 장점이 있다.

### 2.2 소스 파일에 템플릿 정의하기

메서드 정의 코드를 소스 파일에 작성하는 경우, 템플릿을 사용하는 코드에서 메서드 정의 코드를 볼 수 있어야 한다. 이를 위해 클래스 템플릿 정의가 있는 헤더 파일에 메서드 구현 코드가 있는 소스 파일을 추가하는 `#include` 문을 작성하면 된다. 

```cpp
template <typename T>
class Grid
{
    // 클래스 정의 코드 생략
};

#include "Grid.cpp"
```

이때 Grid.cpp 파일이 프로젝트 빌드 목록에 추가되지 않도록 주의한다. 이 파일은 추가하면 안 될 뿐만 아니라 추가할 방법도 없으며 따로 컴파일할 수도 없다. 반드시 헤더 파일의 `#include` 문에 추가해야 한다.

### 2.3 클래스 템플릿의 인스턴스화 제한하기

클래스 템플릿을 특정한 타입에만 적용하게 만들고 싶다면 다음과 같이 적용한다.

예를 들어 Grid 클래스를 int, double, vector\<int>에 대해서만 인스턴스화할 수 있게 만들고 싶다면 다음과 같이 작성한다.

```cpp
template <typename T>
class Grid
{
    // 클래스 정의 코드 생략
};
```

이 헤더 파일은 메서드 정의도 없고, 마지막에 `#include` 문도 없다. 이렇게 작성하면 실제 메서드 정의 코드가 담긴 `.cpp` 파일을 빌드 목록에 추가해야 한다. `.cpp` 파일은 다음과 같이 작성한다.

```cpp
#include "Grid.h"
#include <utility>

template <typename T>
Grid<T>::Grid(size_t width, size_t height)
    : mWidth(width), mHeight(height)
{
    mCells.resize(mWidth);
    for(auto& column : mCells){
        column.resize(mHeight);
    }
}

// 나머지 메서드 정의 코드는 생략
```

그러고 나서 이 메서드를 사용할 수 있게 하려면 템플릿에서 허용하는 타입으로 명시적으로 인스턴스화해둬야 한다. 예를 들어 앞에 나온 `.cpp` 파일의 마지막에 다음과 같이 작성한다.

```cpp
// 인스턴스화를 허용할 타입을 명시적으로 나열한다.
template class Grid<int>;
template class Grid<double>;
template class Grid<std::vector<int>>;
```

이렇게 명시적으로 인스턴스화해두면 해당 타입에 대해서만 빌드하기 때문에 다른 타입으로 인스턴스화할 수 없게 된다.

> 이러한 명시적 클래스 템플릿 인스턴스화 기법을 적용하면 클래스 템플릿에 있는 메서드를 실제로 사용하지 않더라도 그 템플릿에 있는 모든 메서드를 컴파일한다.

## 3. 템플릿 매개변수

### 3.1 비타입 템플릿 매개변수

비타입 매개변수란 int나 포인터처럼 함수나 메서드에서 흔히 사용하는 종류의 매개변수를 말한다. 하지만 정수 계열의 타입(char, int, long 등), 열거 타입, 포인터, 레퍼런스, std::nullptr_t 등만 비타입 매개변수로 사용할 수 있다.

> C++17부터 auto, auto& auto* 등도 비타입 매개변수로 사용할 수 있게 됐는데, 이렇게 지정하면 구체적인 타입을 컴파일러가 알아서 지정한다.

Grid 클래스 템플릿의 예에서 그리드의 높이와 너비를 생성자에서 지정하지 않고 비타입 템플릿 매개변수로 표현할 수 있다. 이렇게 생성자 대신 템플릿 목록에서 비타입 매개변수를 사용하면 코드를 컴파일하기 전에 값을 알 수 있다는 장점이 있다.

컴파일러는 템플릿 메서드의 코드를 생성하기 전에 먼저 템플릿 매개변수를 대입한다. 따라서 코드에서 이차원 배열을 vector에 대한 vector가 아닌 기존 정적 배열로 작성하더라도 크기를 동적으로 조절할 수 있다. Grid 클래스 템플릿 코드를 수정하면 다음과 같다.

```cpp
template <typename T, size_t WIDTH, size_t HEIGHT>
class Grid
{
    public:
        Grid() = default;
        virtual ~Grid() = default;

        // 복제 생성자와 대입 연산자를 명시적으로 디폴트로 지정한다.
        Grid(const Grid& src) = default;
        Grid<T, WIDTH, HEIGHT>& operator=(const Grid& rhs) = default;

        std::optional<T>& at(size_t x, size_t y);
        const std::optional<T>& at(size_t x, size_t y) const;

        size_t getHeight() const { return HEIGHT; }
        size_t getWidth() const { return WIDTH; }

    private:
        void verifyCoordinate(size_t x, size_t y) const;

        std::optional<T> mCells[WIDTH][HEIGHT];
};
```

여기서 템플릿 매개변수 목록으로 Grid에 저장할 객체의 타입, 그리드의 너비와 높이를 지정해야 한다. 너비와 높이는 객체를 지정할 이차원 배열을 생성하는 데 필요하다. 메서드를 정의하는 코드는 다음과 같다.

```cpp
template <typename T, size_t WIDTH, size_t HEIGHT>
void Grid<T, WIDTH, HEIGHT>::verifyCoordinate(size_t x, size_t y) const
{
    if(x >= WIDTH || y >= HEIGHT){
        throw std::out_of_range("");
    }
}

template <typename T, size_t WIDTH, size_t HEIGHT>
const std::optional<T>& Grid<T, WIDTH, HEIGHT>::at(size_t x, size_t y) const
{
    verifyCoordinate(x, y);
    return mCells[x][y];
}

template <typename T, size_t WIDTH, size_t HEIGHT>
std::optional<T>& Grid<T, WIDTH, HEIGHT>::at(size_t x, size_t y)
{
    return const_cast<std::optional<T>&>(std::as_const(*this).at(x,y));
}
```

이렇게 변경한 템플릿은 다음과 같이 사용한다.

```cpp
Grid<int, 10, 10> myGrid;
Grid<int, 10, 10> anotherGrid;
myGrid.at(2, 3) = 42;
anotherGrid = myGrid;
cout << anotherGrid.at(2, 3).value_or(0);
```

해당 코드는 의도와 달리 제약사항이 더 많아졌다. 높이와 너비 값에 non-const 정수를 지정할 수 없다는 것이다.

```cpp
size_t height = 10;
Grid<int, 10, height> testGrid; // 컴파일 에러 발생
```

만약 여기서 height를 상수로 정의하면 문제없이 컴파일된다.

```cpp
const size_t height = 10;
Grid<int, 10, height> testGrid; // 컴파일 성공
```

리턴 타입을 정확히 지정한 constexpr 함수로 표현해도 된다. 예를 들어 size_t 타입의 값을 리턴하는 constexpr 함수로 높이에 대한 템플릿 매개변수를 초기화할 수 있다.

```cpp
constexpr size_t getHeight() { return 10; }
...
Grid<double, 2, getHeight()> myDoubleGrid;
```

또 다른 제약사항은 높이와 너비가 템플릿 매개변수라서 두 값이 그리드 타입의 일부가 된다. 즉 `Grid<int, 10, 10>`과 `Grid<int, 10, 11>`은 서로 다른 타입이다. 그래서 두 타입의 객체는 서로 대입할 수 없고, 함수나 메서드에 전달할 때도 호환되지 않는다.

### 3.2 타입 매개변수의 디폴트값

템플릿 매개변수에 디폴트값을 지정하는 문법은 생성자와 비슷하다. 또한 타입 매개변수 T에도 디폴트값을 지정할 수 있다.

```cpp
template <typename T = int, size_t WIDTH = 10, size_t HEIGHT = 10>
class Grid
{
    // 나머지 코드는 이전과 같다.
};
```

메서드를 정의하는 코드에서는 템플릿 선언문에 T, WIDTH, HEIGHT의 디폴트값을 생략해도 된다. 예를 들어 at() 메서드를 다음과 같이 구현할 수 있다.

```cpp
template <typename T, size_t WIDTH, size_t HEIGHT>
const std::optional<T>& Grid<T, WIDTH, HEIGHT>::at(size_t x, size_t y) const
{
    verifyCoordinate(x, y);
    return mCells[x][y];
}
```

이렇게 작성된 Grid를 인스턴스화하는 코드는 다음과 같이 다양하게 표현할 수 있다.

```cpp
Grid<> myIntGrid;
Grid<int> myGrid;
Grid<int, 5> anotherGrid;
Grid<int, 5, 5> aFourthGrid;
```

클래스 템플릿 매개변수의 디폴트 인수를 지정할 때는 함수나 메서드와 동일하게 매개변수 목록에서 오른쪽 끝에서 왼쪽 방향으로 중간에 건너뛰지 않고 디폴트값을 지정해야 한다.

### 3.3 생성자에 대한 템플릿 매개변수 추론 과정

C++17부터 클래스 템플릿 생성자에 전달된 인수를 보고 템플릿 매개변수를 자동으로 추론하는 기능이 추가됐다.

예를 들어 표준 라이브러리에는 `<utility>` 헤더에 정의된 std::pair란 클래스 템플릿이 있다. 이를 다음과 같이 템플릿 매개변수로 지정한다.

```cpp
std::pair<int, double> pair1(1, 2.3);
```

C++는 템플릿 매개변수를 일일이 적는 번거로움을 덜어 주기 위해 std::make_pair()라는 헬퍼 함수 템플릿을 제공한다. 함수 템플릿은 항상 전달된 인수를 보고 템플릿 매개변수를 알아서 결정한다. 따라서 make_pair()도 전달된 값을 보고 템플릿 타입 매개변수를 자동으로 알아낸다. 예를 들어 다음과 같이 호출하면 컴파일러는 템플릿 매개변수가 `pair<int, double>`이라고 유추한다.

```cpp
auto pair2 = std::make_pair(1, 2.3);
```

C++17부터는 이런 헬퍼 함수 템플릿을 더 이상 사용할 필요가 없다. 생성자에 전달된 인수를 보고 템플릿의 타입 매개변수를 자동으로 알아내기 때문이다.

```cpp
std::pair pair3(1, 2.3);
```

물론 클래스 템플릿의 모든 템플릿 매개변수에 디폴트값을 지정했거나 생성자에서 이 매개변수를 사용할 때만 자동으로 추론할 수 있다.

##### 사용자 정의 추론 방식

템플릿 매개변수를 추론하는 규칙을 사용자가 직접 정할 수도 있다.

```cpp
template <typename T>
class SpreadsheetCell
{
    public:
        SpreadsheetCell(const T& t) : mContent(t) { }

        const T& getContent() const { return mContent; }

    private:
        T mContent;
};
```

자동 템플릿 매개변수 추론 기능을 이용하면 std::string 타입에 대한 SpreadsheetCell을 생성하는 코드를 다음과 같이 작성할 수 있다.

```cpp
std::string myString = "Hello World!";
SpreadsheetCell cell(myString);
```

이때 SpreadsheetCell 생성자에 스트링을 `const char*` 타입으로 전달하면 원래 의도와 달리 T의 타입을 `const char*`로 결정해버린다. 이럴 때 다음과 같은 규칙을 직접 지정해서 생성자의 인수를 `const char*` 타입으로 전달할 때 T를 `std::string`으로 추론하게 만든다.

```cpp
SpreadsheetCell(const char*) -> SpreadsheetCell<std::string>;
```

이 문장은 반드시 클래스 정의 밖에 적어야 하며, 네임스페이스는 SpreadsheetCell 클래스와 같아야 한다.

기본 문법은 다음과 같다. 여기서 explicit 키워드는 생략해도 된다. 효과는 단일 매개변수 생성자에 대해 explicit을 지정할 때와 같다. 따라서 매개변수가 하나일 때만 적용할 수 있다.

```cpp
explicit 템플릿_이름(매개변수_목록) -> 추론된_템플릿;
```