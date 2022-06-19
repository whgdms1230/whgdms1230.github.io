---
sort: 4
---

# Class Templates - 활용

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 메서드 템플릿

클래스뿐만 아니라 메서드도 템플릿화할 수 있다. 이러한 메서드 템플릿은 클래스 템플릿 안에 정의해도 되고, 비템플릿 클래스 안에 정의해도 된다.

메서드를 템플릿으로 제공하면 한 메서드를 다양한 타입에 대한 버전으로 만들 수 있다. 메서드 템플릿은 클래스 템플릿에 복제 생성자와 대입 연산자를 정의할 때 특히 유용하다.

> 가상 메서드와 소멸자는 메서드 템플릿으로 만들 수 없다.

```cpp
Grid<int> myIntGrid;
Grid<double> myDoubleGrid;
```

앞서 만든 클래스 템플릿인 Grid를 이용하여 인스턴스화한 Grid\<int>와 Grid\<double>은 타입이 서로 다르다. Grid\<double> 객체를 받는 함수는 Grid\<int> 객체를 인수로 받을 수 없다. int를 double로 강제 형변환해서 int 원소를 double 원소로 복제할 수는 있지만, Grid\<int> 타입 객체를 Grid\<double> 타입 객체에 대입하거나 Grid\<int>로 Grid\<double> 객체를 만들 수는 없다. 따라서 다음과 같이 작성하면 컴파일 에러가 발생한다.

```cpp
myDoubleGrid = myIntGrid; // 컴파일 에러 발생
Grid<double> newDoubleGrid(myIntGrid); // 컴파일 에러 발생
```

그 이유는 Grid 템플릿에 대한 복제 생성자와 대입 연산자가 다음과 같이 정의됐기 때문이다.

```cpp
Grid(const Grid& src);
Grid<T>& operator=(const Grid& rhs);
```

이를 정확히 표현하면 다음과 같다.

```cpp
Grid(const Grid<T>& src);
Grid<T>& operator=(const Grid<T>& rhs);
```

복제 생성자인 Grid와 대입 연산자인 operator=은 모두 const Grid\<T> 레퍼런스를 인수로 받는다. 그러므로 Grid\<double>을 인스턴스화해서 Grid 복제 생성자와 operator=을 호출하면 컴파일러는 각각에 대한 프로토타입을 다음과 같이 생성한다.

```cpp
Grid(const Grid<double>& src);
grid<double>& operator=(const Grid<double>& rhs);
```

이처럼 Grid\<int>를 받는 생성자나 operator=가 없다.

이를 해결하기 위해 Grid 클래스의 복제 생성자와 대입 연산자를 메서드 템플릿으로 만들면 서로 다른 타입을 처리할 수 있다.

```cpp
template <typename T>
class Grid
{
    public:
        // 코드 생략

        template <typename E>
        Grid(const Grid<E>& src);

        template <typename E>
        Grid<T>& operator=(const Grid<E>& rhs);

        void swap(Grid& other) noexcept;

        // 코드 생략      
};
```

템플릿 선언문에 E(element의 줄임말) 라는 새로운 타입 이름을 지정했다. 클래스는 T라는 타입에 대해 템플릿화되고, 복제 생성자는 T와는 다른 E라는 타입에 대해 템플릿화된다. 두 타입에 대해 템플릿화함으로써 한 타입의 Grid 객체를 다른 타입의 Grid로 복제할 수 있다.

수정된 복제 생성자를 정의하는 코드는 다음과 같다.

```cpp
template<typname T>
template<typename E>
Grid<T>::Grid(const Grid<E>& src)
    : Grid(src.getWidth(), src.getHeight())
{
    // 이 생성자의 생성자 이니셜라이저는
    // 적절한 양의 메모리를 할당하는 작업을 비복제 생성자에 위임한다.

    // 그리고 나서 데이터를 복제한다.
    for(size_t i = 0; i < mWidth; i++){
        for(size_t j = 0; j < mHeight; j++){
            mCells[i][j] = src.at(i, j);
        }
    }
}
```

위 정의 코드는 템플릿 선언문을 먼저 적고 그 뒤에 멤버 템플릿 선언문을 작성하였다. 주의할 점은 다음과 같이 두 문장을 합칠 수 없다.

```cpp
template <typename T, typename E> // 중첩된 템플릿 생성자를 이렇게 적으면 안된다.
```

또한 src의 원소에 접근할 때는 반드시 getWidth(), getHeight(), at()과 같은 public 접근자 메서드를 사용해야 한다. 복제할 원본 객체의 타입은 Grid\<E>이고, 복제할 대상 객체의 타입은 Grid\<T>이기 때문이다. 두 타입은 서로 다르기 때문에 반드시 public 메서드로 다뤄야 한다.

템플릿화한 대입 연산자는 const Grid\<E>& 타입의 인수를 받아서 Grid\<T>& 타입을 리턴한다.

```cpp
template <typename T>
template <typename E>
Grid<T>& Grid<T>::operator=(const Grid<E>& rhs)
{
    // 자기 자신을 대입할 경우에 대한 예외 처리를 할 필요가 없다.
    // 이 버전의 대입 연산자는 T와 E가 서로 같으면 호출되지 않기 때문이다.

    // 복제 후 맞바꾸기 구문
    Grid<T> temp(rhs); // 모든 작업을 임시 인스턴스에서 처리한다.
    swap(temp); // 익셉션이 발생하지 않는 연산으로만 처리한다.
    return *this;
}
```

템플릿 버전의 대입 연산자에서는 자기 대입 여부를 검사할 필요가 없다. 같은 타입끼리 대입하는 연산은 템플릿화하지 않은 원래 버전의 operator=이 처리하기 때문이다.

템플릿 버전전의 대입 연산자는 friend 함수인 swap()(함수 템플릿에서 소개) 대신 swap() 메서드를 사용했다. 이 경우에는 같은 타입의 Grid 객체만 맞바꿀 수 있지만, 여기서 문제가 되지는 않는다. 템플릿화한 대입 연산자는 먼저 템플릿화한 복제 생성자를 이용하여 Grid\<E> 객체를 Grid\<T> 객체로 변환하기 때문이다.

swap() 메서드로 Grid\<T> 타입인 temp를 this와 맞바꾼다. 물론 this의 타입도 Grid\<T>다. 여기 나온 swap() 메서드를 정의하는 코드는 다음과 같다.

```cpp
template <typename T>
void Grid<T>::swap(Grid<T>& other) noexcept
{
    using std::swap;

    swap(mWidth, other.mWidth);
    swap(mHeight, other.mHeight);
    swap(mCells, other.mCells);
}
```

### 1.1 비타입 매개변수를 사용하는 메서드 템플릿

앞의 예제엇 HEIGHT와 WIDTH를 정수 타입 템플릿 매개변수로 지정하면 높이와 너비가 타입의 일부가 되어버리는 심각한 문제가 있었다. 이렇게 하면 높이와 너비가 다른 그리드에 대입할 수 없다.

만약 크기가 다른 그리드끼리 대입하거나 복제해야한다면, 대상 객체를 원본 객체와 완전히 똑같이 만드는 대신 원본 배열의 높이와 너비 둘 다 대상 배열보다 작다면 서로 겹치는 부분만 복제하고 나머지 부분은 디폴트값으로 채워 넣는 방식으로 구현할 수 있다.

```cpp
template <typename T, size_t WIDTH = 10, size_t HEIGHT = 10>
class Grid
{
    public:
        Grid() = default;
        virtual ~Grid() = default;

        // 복제 생성자와 대입 연산자를 명시적으로 디폴트로 지정한다.
        Grid(const Grid& src) = default;
        Grid<T, WIDTH, HEIGHT>& operator=(const Grid& rhs) = default;

        template <typename E, size_t WIDTH2, size_t HEIGHT2>
        Grid(const Grid<E, WIDTH2, HEIGHT2>& src);

        template <typename E, size_t WIDTH2, size_t HEIGHT2>
        Gird<T, WIDTH, HEIGHT>& operator=(const Grid<E, WIDTH2, HEIGHT2>& rhs);

        void swap(Grid& other) noexcept;

        std::optional<T>& at(size_t x, size_t y);
        const std::optional<T>& at(size_t x, size_t y) const;

        size_t getHeight() const { return HEIGHT; }
        size_t getWidth() const { return WIDTH; }

        void verifyCoordinate(size_t x, size_t y) const;

        std::optional<T> mCells[WIDTH][HEIGHT];
};
```

복제 생성자와 대입 연산자에 대한 메서드 템플릿과 swap() 이란 헬퍼 메서드를 갖고 있다. 참고로 템플릿 버전이 아닌 기존 복제 생성자와 대입 연산자를 명시적으로 디폴트로 지정했다(소멸자를 직접 정의했기 때문에). 두 메서드는 단순히 mCells만 복제하거나 대입하는데, 서로 크기가 같은 그리드끼리 대입하거나 복제할 때는 이렇게 처리해야하기 때문이다.

템플릿화한 복제 생성자를 정의하는 코드는 다음과 같다. 이 복제 생성자는 src가 더 크더라도 x와 y축에서 각각 WIDTH와 HEIGHT로 지정된 크기만큼만 원소를 복제하고, 두 축 중 어느 하나가 src보다 작으면 나머지 영역에 있는 std::optional 객체드은 reset() 메서드로 리셋된다.

```cpp
template <typename T, size_t WIDTH, size_t HEIGHT>
template <typename E, size_t WIDTH2, size_t HEIGHT2>
Grid<T, WIDTH, HEIGHT>::Grid(const Grid<E, WIDTH2, HEIGHT2>& src)
{
    for(size_t i = 0; i < WIDTH; i++){
        for(size_t j = 0; j < HEIGHT; j++){
            if (i < WIDTH2 && j < HEIGHT2){
                mCells[i][j] = src.at(i, j);
            } else {
                mCells[i][j].reset();
            }
        }
    } 
}
```

swap()과 operator=의 구현 코드는 다음과 같다.

```cpp
template <typename T, size_t WIDTH, size_t HEIGHT>
void Grid<T, WIDTH, HEIGHT>::swap(Grid<T, WIDTH, HEIGHT>& other) noexcept
{
    using std::swap;
    swap(mCells, other.mCells);
}

template <typename T, size_t WIDTH, size_t HEIGHT>
template <typename E, size_t WIDTH2, size_t HEIGHT2>
Grid<T, WIDTH, HEIGHT>& Grid<T, WIDTH, HEIGHT>::operator=(
    const Grid<E, WIDTH2, HEIGHT2>& rhs)
{
    // 자기 자신을 대입할 경우에 대한 예외 처리를 할 필요가 없다.
    // 이 버전의 대입 연산자는 T와 E가 서로 같으면 호출되지 않기 때문이다.

    // 복제 후 맞바꾸기 구문
    Grid<T, WIDTH, HEIGHT> temp(rhs); // 모든 작업을 임시 인스턴스에서 처리한다.
    swap(temp); // 익셉션이 발생하지 않는 연산으로만 처리한다.
    return *this;
}
```

## 2. 클래스 템플릿의 특수화

특정한 경우에 대해서만 템플릿을 다르게 구현하는 것을 템플릿 특수화라 한다. 클래스 템플릿 특수화 코드를 작성할 때는 이 코드가 템플릿이라는 사실뿐만 아니라 이 템플릿이 특정한 타입에 특화된 버전이라는 것도 반드시 명시해야 한다.

```cpp
// 템플릿 특수화를 적용할 때 원본 템플릿도 반드시 참조할 수 있어야 한다.
// 따라서 특수화한 템플릿과 함께 원본 템플릿도 항상 볼 수 있도록 include 문을 추가한다.
#include "Grid.h"

template<>
class Grid<const char*>
{
    public:
        explicit Grid(size_t width = kDefaultWidth, size_t height = kDefaultHeight);
        virtual ~Grid() = default;

        // 복제 생성자와 대입 연산자를 명시적으로 디폴트로 선언한다.
        Grid(const Grid& src) = default;
        Grid<const char*>& operator=(const Grid& rhs) = default;

        // 이동 생성자와 대입 연산자를 명시적으로 디폴트로 선언한다.
        Grid(Grid&& src) = default;
        Grid<const char*>& operator=(Grid&& rhs) = default;

        std::optional<std::string>& at(size_t x, size_t y);
        const std::optional<std::string>& at(size_t x, size_t y) const;

        size_t getHeight() const { return mHeight; }
        size_t getWidth() const { return mWidth; }

        static const size_t kDefaultWidth = 10;
        static const size_t kDefaultHeight = 10;

    private:
        void verifyCoordinate(size_t x, size_t y) const;

        std::vector<std::vector<std::optional<std::string>>> mCells;
        size_t mWidth, mHeight;
}
```

이렇게 특수화할 때는 T와 같은 타입 매개변수를 적지 않고 곧바로 `const char*`를 지정했다.

```cpp
template<>
class Grid<const char*>
```

이렇게 작성하면 컴파일러는 이 클래스가 `const char*`에 특수화한 Grid라고 판단한다.

특수화의 장점은 특수화됐다는 사실이 사용자에게 드러나지 않는다는 것이다. Grid를 int나 SpreadsheetCell에 대해 인스턴스화하면 컴파일러는 원본 Grid 템플릿을 이용하여 코드를 생성하지만, `const char*`에 대한 Grid를 인스턴스화할 때는 `const char*`에 대한 특수화한 버전을 사용한다. 이 과정은 사용자에게 드러나지 않고 모두 내부적으로 처리한다.

```cpp
Grid<int> myIntGrid; // 원본 Grid 템플릿을 사용
Grid<const char*> stringGrid1(2, 2); // const char*에 대한 특수화 버전 사용

const char* dummy = "dummy";
stringGrid1.at(0, 0) = "hello";
stringGrid1.at(0, 1) = dummy;
stringGrid1.at(1, 0) = dummy;
stringGrid1.at(1, 1) = "there";

Grid<const char*> stringGrid2(stringGrid1);
```

템플릿을 특수화하는 것은 상속과는 다른 개념이다. 특수화할 때는 클래스 전체를 완전히 새로 구현해야 한다. 따라서 상속할 때처럼 메서드의 이름과 동작을 똑같이 적용할 필요가 없다. 물론 템플릿 특수화 기능을 본래 목적과 다르게 남용하는 것이기 때문에 특별한 이유가 없다면 이렇게 작성하면 안 된다.

`const char*`에 대한 특수화 버전의 메서드는 다음과 같이 구현한다. 템플릿 정의 코드와 달리 여기서는 메서드 앞에 `template<>` 구문을 적지않아도 된다.

```cpp
Grid<const char*>::Grid(size_t width, size_t height)
    : mWidth(width), mHeight(height)
{
    mCells.resize(mWidth);
    for(auto& column : mCells){
        column.resize(mHeight);
    }
}

void Grid<const char*>::verifyCoordinate(size_t x, size_t y) const
{
    if(x >= mWidth || y >= mHeight){
        throw std::out_of_range("");
    }
}

const std::optional<std::string>& Grid<const char*>::at(size_t x, size_t y) const
{
    verifyCoordinate(x, y);
    return mCells[x][y];
}

std::optional<std::string>& Grid<const char*>::at(size_t x, size_t y)
{
    return const_cast<std::optional<std::string>&>(std::as_const(*this).at(x,y));
}
```

## 3. 클래스 템플릿 상속하기

클래스 템플릿도 상속할 수 있다. 템플릿을 상속한 파생 클래스도 템플릿이어야 한다. 반면 클래스 템플릿을 특정한 타입으로 인스턴스화한 클래스를 상속할 때는 파생 클래스가 템플릿이 아니어도 된다.

두 경우 중 파생 클래스도 템플릿인 경우를 살펴보자.

제네릭 클래스인 Grid를 게임보드로 활용하기에는 기능이 부족해서 게임보드의 한 지점에서 다른 지점으로 옮기는 `move()` 메서드를 추가한 GameBoard 클래스 템플릿을 다음과 같이 정의한다.

```cpp
#include "Grid.h"

template<typename T>
class GameBoard : public Grid<T>
{
    public:
        explicit GameBoard(size_t width = Grid<T>::kDefaultWidth,
            size_t height = Grid<T>::kDefaultHeight);
        void move(size_t xSrc, size_t ySrc, size_t xDest, size_t yDest);
};
```

템플릿을 상속하는 구문은 베이스 클래스가 Grid가 아닌 Grid\<T\>라는 점만 빼면 기존 상속 구문과 차이가 없어 보인다. GameBoard 템플릿은 제네릭 템플릿인 Grid를 곧바로 상속하는 것이 아니라 GameBoard를 특정한 타입에 대해 인스턴스화할 때마다 그 타입에 대해 Grid를 인스턴스화한 클래스를 상속하는 것이다. 그래서 템플릿 상속 문법이 일반 클래스 상속과 같은 것이다.

예를 들어 GameBoard를 ChessPiece 타입에 대해 인스턴스화하면 컴파일러는 Grid\<ChessPiece\>에 대한 코드도 함께 생성한다.

다음으로 생성자와 `move()` 메서드를 구현하는 코드이다. 베이스 클래스 생성자를 호출할 때 Grid\<T\>를 사용한다.

```cpp
template<typename T>
GameBoard<T>::GameBoard(size_t width, size_t height)
    : Grid<T>(width, height)
{}

void GameBoard<T>::move(size_t xSrc, size_t ySrc, size_t xDest, size_t yDest)
{
    Grid<T>::at(xDest, yDest) = std::move(Grid<T>::at(xSrc, ySrc));
    Grid<T>::at(xSrc, ySrc).reset(); // 원본 셀을 리셋
}
```

이렇게 정의한 GameBoard 템플릿의 사용법은 다음과 같다.

```cpp
GameBoard<ChessPiece> chessboard(8, 8);
ChessPiece pawn;
chessboard.at(0, 0) = pawn;
chessboard.move(0, 0, 0, 1);
```

## 4. 상속과 특수화 비교

||상속|특수화
-|-|-|
코드 재사용|**O**:파생 클래스는 베이스 클래스에 있는 데이터 멤버와 메서드를 모두 가진다.|**X**:특수화를 할 때는 필요한 코드를 모두 다시 작성해야 한다.|
이름 재사용|**X**:파생 클래스의 이름은 반드시 베이스 클래스와 다르게 지어야 한다.|**O**:특수화 템플릿 클래스의 이름은 반드시 원본과 같아야 한다.|
다형성 지원|**O**:파생 클래스의 객체를 베이스 클래스의 객체로 표현할 수 있다.|**X**:템플릿을 인스턴스화한 결과마다 타입이 다르다.| 

## 5. 앨리어스 템플릿

`using`과 `typedef`의 개념을 이용하면 클래스 템플릿에 대해 다른 이름으로 부를 수 있다.

```cpp
template<typename T1, typename T2>
class MyTemplateClass { /* ... */ };

using OtherName = MyTemplateClass<int, double>;
```

여기서 타입 앨리어스 대신 `typedef`를 사용해도 된다.

또한 타입 매개변수 중에서 일부만 지정하고, 나머지 타입은 그대로 템플릿 타입 매개변수 형태로 남겨둘 수 있다. 이를 앨리어스 템플릿이라 부른다.

```cpp
template<typename T1>
using OtherName = MyTemplateClass<T1, double>;
```

이런 문장은 `typedef`로 표현할 수 없다.