---
sort: 2
---

# Class Templates - 작성법

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 클래스 템플릿

클래스 템플릿은 멤버 변수 타입, 메서드의 매개변수 또는 리턴 타입을 매개변수로 받아서 클래스를 만든다.

클래스 템플릿은 주로 객체를 저장하는 컨테이너나 데이터 구조에서 많이 사용한다.

## 2. 클래스 템플릿 작성법

체스, 체커, 틱택토와 같은 이차원 게임에서 범용적으로 사용할 수 있는 제네릭 게임보드를 위한 Grid 컨테이너를 구현해보면서 클래스 템플릿에 대해 설명한다.

### 2.1 템플릿 없이 구현한 Grid 클래스

템플릿을 사용하지 않고 제네릭 게임보드를 구현하는 가장 좋은 방법은 다형성을 이용하여 제네릭 GamePiece 객체를 저장하게 만드는 것이다. 그러면 게임의 종류에 따라 GamePiece 클래스를 상속해서 구현할 수 있다.

##### 클래스 정의

예를 들어 체스 게임이라면 GamePiece의 파생 클래스로 ChessPiece를 구현한다. 다형성 덕분에 GamePiece를 저장하도록 정의한 GameBoard를 ChessPiece 저장에도 활용할 수 있다. 이때 GameBoard를 복제할 수 있어야 하기 때문에 GameBoard에서 GamePiece를 복사하는 기능도 구현해야 한다. 이렇게 다형성을 적용하기 위해 다음과 같이 GamePiece 베이스 클래스에 clone()이라는 순수 가상 메서드를 추가한다. GamePiece의 기본 인터페이스는 다음과 같다.

```cpp
class GamePiece
{
    public:
        virtual std::unique_ptr<GamePiece> clone() const = 0;
};
```

GamePiece는 추상 베이스 클래스다. 그래서 ChessPiece와 같은 구체적인 클래스는 GamePiece를 상속할 때 clone() 메서드를 구현해야 한다.

```cpp
class ChessPiece : public GamePiece
{
    public:
        virtual std::unique_ptr<GamePiece> clone() const override;
};

std::unique_ptr<GamePiece> ChessPiece::clone() const
{
    // 복제 생성자를 호출해서 이 인스턴스를 복제한다.
    return std::make_unique<ChessPiece>(*this);
}
```

GameBoard 클래스를 구현할 때 GamePiece를 저장하는 부분을 unique_ptr에 대한 vector의 vector로 작성한다.

```cpp
class GameBoard
{
    public:
        explicit GmaeBoard(size_t width = kDefaultWidth, size_t height = kDefaultHeight);
        GameBoard(const GameBoard& src); // 복제 생성자
        virtual ~GameBoard() = default; // 가상 디폴트 소멸자
        GameBoard& operator=(const GameBoard& rhs); // 대입 연산자

        // 이동 생성자와 대입 연산자를 명시적으로 디폴트로 지정한다.
        GameBoard(GameBoard&& src) = default;
        GameBoard& operator=(GameBoard&& src) = default;

        std::unique_ptr<GamePiece>& at(size_t x, size_t y);
        const std::unique_ptr<GamePiece>& at(size_t x, size_t y) const;

        size_t getHeight() const { return mHeight; }
        size_t getWidth() const { return mWidth; }

        static const size_t kDefaultWidth = 10;
        static const size_t kDefaultHeight = 10;

        friend void swap(GameBoard& first, GameBoard& second) noexcept;

    private:
        void verifyCoordinate(size_t x, size_t y) const;

        std::vector<std::vector<std::unique_ptr<GamePiece>>> mCells;
        size_t mWidth, mHeight;
};
```

at() 메서드는 인수로 지정한 지점에 있는 말을 복제하지 않고 레퍼런스로 리턴한다. GameBoard를 이차원 배열로 추상화하므로 인덱스로 지정한 객체의 복사본이 아닌 실제 객체를 제공하는 방식으로 배열에 접근하게 만들어야 한다. 이렇게 구한 객체 레퍼런스는 나중에 유효하지 않게 될 수 있기 때문에 리턴된 레퍼런스를 저장했다가 다시 쓸 수 없고, 이 레퍼런스가 필요할 때마다 at()을 호출해서 리턴된 레퍼런스를 곧바로 사용해야 한다.

> 여기서 at() 메서드는 레퍼런스를 리턴하는 버전과 const 레퍼런스를 리턴하는 버전으로 제공된다.

##### 클래스 구현

이 클래스의 메서드를 정의하는 코드는 다음과 같다.

> 여기서 대입 연산자를 복제 후 맞바꾸기 패턴으로 구현한 점을 주목하기 바란다. 또한 코드 중복을 피하도록 스콧 메이어의 const_cast() 패턴을 적용했다.

```cpp
GameBoard::GameBoard(size_t width, wize_t height)
    : mWidth(width), mHeight(height)
{
    mCells.resize(mWidth);
    for(auto& column : mCells){
        column.resize(mHeight);
    }
}

GameBoard::GameBoard(const GameBoard& src)
    : GameBoard(src.mWidth, src.mHeight)
{
    // 여기 나온 생성자 이니셜라이저는
    // 먼저 적절한 크기의 메모리를 할당하는 작업을 비복제 생성자에 위임한다.

    // 그러고 나서 데이터를 복제한다.
    for(size_t i = 0; i < mWidth; i++){
        for(size_t j = 0; j < mHeight; j++){
            if(src.mCells[i][j])
                mCells[i][j] = src.mCells[i][j]->clone();
        }
    }
}

void GameBoard::verifyCoordinate(size_t x, size_t y) const
{
    if(x >= mWidth || y >= mHeight){
        throw std::out_of_range("");
    }
}

void swap(GameBoard& first, GameBoard& second) noexcept
{
    using std::swap;

    swap(first.mWidth, second.mWidth);
    swap(first.mHeight, second.mHeight);
    swap(first.mCells, second.mCells);
}

GameBoard& GameBoard::operator=(const GameBoard& rhs)
{
    // 자기 자신을 대입하는지 검사한다.
    if(this == &rhs){
        return *this;
    }

    // 복제 후 맞바꾸기
    GameBoard temp(rhs); // 이 작업은 임시 인스턴스로 처리한다.
    swap(*this, temp); // 예외를 발생하지 않는 연산으로만 작업을 처리한다.
    return *this;
}

const unique_ptr<GamePiece>& GameBoard::at(size_t x, size_t y) const
{
    verifyCoordinate(x, y);
    return mCells[x][y];
}

unique_ptr<GamePiece>& GameBoard::at(size_t x, size_t y)
{
    return const_cast<unique_ptr<GamePiece>&>(as_const(*this).at(x,y));
}
```

##### 클래스 사용 예

```cpp
GameBoard chessBoard(8, 8);
auto pawn = std::make_unique<ChessPiece>();
chessBoard(0, 0) = std::move(pawn);
chessBoard(0, 1) = std::make_unique<ChessPiece>();
chessBoard(0, 1) = nullptr;
```

### 2.2 템플릿으로 구현한 Grid 클래스

GameBoard 클래스를 위와 같이 작성했을 때의 아쉬운 점은 다음과 같다.

1. GameBoard는 원소를 항상 포인터로 지정한다. 그래서 원소를 값으로 지정할 수 없다.
2. 타입 안전성이 떨어진다. GameBoard는 각 셀을 unique_ptr\<GamePiece>로 저장한다. ChessPiece로 저장했던 셀을 요청하기 위해 at()을 호출해도 unique_ptr\<GamePiece>로만 리턴한다. 따라서 GamePiece를 ChessPiece로 다운캐스트해야 ChessPiece의 고유 기능을 활용할 수 있다.
3. int나 double 같은 기본 타입으로 저장할 수 없다. 셀은 GamePiece를 상속한 타입만 저장할 수 있기 때문이다.

따라서 ChessPiece나 SpreadsheetCell 뿐만 아니라, int, double 같은 타입도 모두 수용하려면 Grid를 제네릭 클리스로 만드는 것이 훨씬 좋다.

C++에서 제공하는 클래스 템플릿을 이용하면 특정한 타입에 종속되지 않게 클래스를 구현할 수 있다. 그러면 클라이언트는 원하는 타입에 맞는 클래스를 인스턴스화해서 사용할 수 있다. 이런 방식을 제네릭 프로그래밍이라 부른다.

제네릭 프로그래밍의 가장 큰 장점은 타입 안전성이다. 앞 절에서처럼 다형성을 이용하면 추상 베이스 클래스로 정의해야 하지만, 클래스 템플릿을 활용하면 클래스 안에 있는 메서드를 비롯한 멤버의 타입을 모두 구체적으로 정의할 수 있다.

예를 들어 ChessPiece 뿐만 아니라 TicTacToePiece도 지원한다고 가정하자.

```cpp
class TicTacToePiece : public GamePiece
{
    public:
        virtual std::unique_ptr<GamePiece> clone() const override;
};

std::unique_ptr<GamePiece> clone() const override;
{
    // 복제 생성자를 호출해서 이 인스턴스를 복제한다.
    return std::make_unique<TicTacToePiece>(*this);
}
```

앞 절에서 본 것처럼 제네릭 게임보드를 다형성으로 구현하면 체스보드 객체에 ChessPiece뿐만 아니라 TicTacToePiece마저 저장해버릴 위험이 있다.

```cpp
GameBoard chessBoard(8, 8);
chessBoard.at(0, 0) = std::make_unique<ChessPiece>();
chessBoard.at(0, 1) = std::make_unique<TicTacToePiece>();
```

이렇게 구현하면 저장할 시점에 말의 타입을 기억해두지 않으면 나중에 at() 메서드로 저장된 셀을 가져올 때 정확한 타입으로 다운캐스트를 할 수 없다는 심각한 문제가 발생한다.

##### Grid 클래스 정의

GameBoard 클래스에서 템플릿 기반으로 만든 Grid 클래스를 사용하도록 수정해보자. 이를 위해 클래스 이름을 GameBoard에서 Grid로 바꾼다.

또한 Grid 클래스는 int와 double 같은 기본 타입도 지원해야 한다. 그러기 위해서 GameBoard를 구현할 때처럼 다형성 기반의 포인터 전달 방식으로 구현하는 것보다 다형성을 사용하지 않고 값 전달 방식으로 구현하는 것이 유리하다.

하지만 한 가지 단점이 있다. 포인터 방식과 달리 값 전달 방식을 적용할 때는 셀에 항상 어떤 값이 들어 있어야 하기 때문에 완전히 빈 셀을 만들 수 없다. 이에 반해 포인터 기반으로 구현하면 nullptr로 초기화하는 방식으로 빈 셀을 만들 수 있다. 이는 C++17부터 지원하는 std::optional을 이용하여 값 전달 방식을 지원하는 동시에 빈 셀도 표현할 수 있게 한다. optional은 \<optional> 헤더에 정의돼 있다.

```cpp
template <typename T>
class Grid
{
    public:
        explicit Grid(size_t width kDefaultWidth,
            size_t height = kDefaultHeight);
        virtual ~Grid() = default;

        // 복제 생성자와 대입 연산자를 명시적으로 디폴트로 지정한다.
        Grid(const Grid& src) = default;
        Grid<T>& operator=(const Grid& rhs) = default;

        // 이동 생성자와 대입 연산자를 명시적으로 디폴트로 지정한다.
        Grid(Grid&& src) = default;
        Grid<T>& operator=(Grid&& rhs) = default;

        std::optional<T>& at(size_t x, size_t y);
        const std::optional<T>& at(size_t x, size_t y) const;

        size_t getHeight() const { return mHeight; }
        size_t getWidth() const { return mWidth; }

        static const size_t kDefaultWidth = 10;
        static const size_t kDefaultHeight = 10;

    private:
        void verifyCoordinate(size_t x, size_t y) const;

        std::vector<std::vector<std::optional<T>>> mCells;
        size_t mWidth, mHeight;
};
```

##### 문법

```cpp
template <typename T>
```

해당 문장은 뒤에 나올 클래스 정의가 특정한 타입에 적용할 수 있는 켐플릿이라고 선언하는 문장이다. 템플릿은 타입을 매개변수로 받는다(매개변수화). 함수를 호출할 때 지정하는 인수를 매개변수 이름으로 표현하듯이 템플릿에 적용할 타입도 템플릿의 매개변수 이름으로 표현한다.

> T 라는 이름 자체에는 특별한 의미가 없다. 이름은 마음대로 정할 수 있으며, 관례상 T로 표기한 것이다.

템플릿 지정자는 문장 전체에 적용된다. 앞에 나온 코드에서는 클래스를 정의하는 코드 전체에 적용된다.

> 템플릿 타입 매개변수를 typename 대신 class 키워드로 표기하여 template \<class T>로 표기할 수도 있다. 하지만 템플릿 매개변수를 표현할 때 class란 키워드를 사용하면 타입을 반드시 클래스로 지정해야 한다고 오해할 수 있다. 이는 클래스 뿐만 아니라 struct union, int, double 같은 언어의 기본 타입도 얼마든지 지정할 수 있다.

GameBoard 클래스는 mCells란 데이터 멤버를 포인터 원소에 대한 vector의 vector로 구현했다. 그래서 복제 작업을 특수한 코드(복제 생성자와 복제 대입 연산자)로 처리하려 했다.

반면 템플릿을 이용한 Grid 클래스는 mCells의 타입을 optional 값에 대한 vector의 vector로 정의했다. 그러면 복제 생성자와 대입 연산자를 컴파일러가 만들어주는데, 이렇게 기본으로 생성되는 것으로도 충분하다.

하지만 사용자가 직접 소멸자를 정의하면 복제 생성자나 복제 대입 연산자가 자동으로 생성되지 않는다. 그래서 Grid 클래스 템플릿에서 복제 생성자와 복제 대입 연산자가 자동 생성되도록 명시적으로 디폴트로 지정했다.

마찬가지로 이동 생성자와 이동 대입 연산자도 명시적으로 디폴트로 선언했다.

복제 대입 연산자를 명시적으로 디폴트로 선언하는 문장은 다음과 같다.

```cpp
Grid<T>& operator= (const Grid& rhs) = default;
```

이 문장은 `const GameBoard&` 타입으로 선언했던 rhs 매개변수가 `const Grid&` 타입으로 변경된 것을 알 수 있다. 이 타입을 `const Grid<T>&`로 표기해도 된다. 참고로 클래스 정의 코드 안에서 Grid라고만 적으도 컴파일러는 Grid\<T>로 해석한다. 하지만 클래스 정의 밖에서는 반드시 Grid\<T>라고 적어야 한다.

클래스 템플릿을 작성할 때 Grid가 클래스 이름처럼 보이지만 엄밀히 Grid는 템플릿 이름이다. 실제 클래스를 가리킬 때는 int, SpreadsheetCell, ChessPiece 등과 같은 구체적인 타입으로 템플릿을 인스턴스화한 이름으로 표현해야 한다. 따라서 Grid 클래스 템플릿으로 인스턴스화한 실제 클래스를 가리킬 때는 Grid\<T>로 표현해야 한다.

이제 mCells는 더 이상 포인터가 아닌 optional 타입의 값으로 저장한다. 따라서 at() 메서드의 리턴 타입을 unique_ptr가 아닌 optional\<T>& 또는 const optional\<T>&로 변경한다.

```cpp
std::optional<T>& at(size_t x, size_t y);
const std::optional<T>& at(size_t x, size_t y) const;
```

##### Grid 클래스 메서드 정의

Grid 템플릿에서 메서드를 정의할 때는 반드시 템플릿 지정자를 앞에 적어야 한다.

```cpp
template <typename T>
Grid<T>::Grid(size_t width, size_t height)
    : mWidth(width), mHeight(height)
{
    mCells.resize(mWidth);
    for(auto& column : mCells){
    // 위 문장을 다음과 같이 작성해도 된다.
    // for(std::vector<std::optional<T>>& cloumn : mCells){
    //     column.resize(mHeight); 
    // }
}
```

여기서 `::` 기호 앞의 클래스 이름이 Grid가 아닌 Grid\<T>인 점에 주목한다. 메서드나 static 데이터 멤버를 정의하는 코드는 반드시 클래스 이름을 Grid\<T>와 같이 표기해야 한다. 생성자의 본문은 GameBoard 생성자와 동일하다.

나머지 메서드 정의 코드는 템플릿 지정자와 Grid\<T>를 제외하면 GameBoard와 같다.

```cpp
template <typename T>
void Grid<T>::verifyCoordinate(size_t x, size_t y) const
{
    if(x >= mWidth || y >= mHeight){
        throw std::out_of_range("");
    }
}

template <typename T>
const std::optional<T> Grid<T>::at(size_t x, size_t y) const
{
    verifyCoordinate(x, y);
    return mCells[x][y];
}

template <typename T>
std::optional<T>& Grid<T>::at(size_t x, size_t y)
{
    return const_cast<std::optional<T>&>(std::as_const(*this).at(x,y));
}
```

> 클래스 템플릿 메서드의 구현 코드를 작성할 때 템플릿 타입 매개변수 T에 대해 디폴트값을 지정하려면 T()와 같이 작성해야 한다. T가 클래스 타입이면 T()는 이 클래스의 디폴트 생성자를 호출하고, T가 기본 타입이면 T()는 0을 생성한다. 이렇게 표기하는 방식을 영 초기화 문법이라 부른다. 구체적인 타입을 모르는 변수에 디폴트값을 지정하는 데 유용하다.

### 2.3 Grid 템플릿 사용법

Grid 템플릿으로 Grid 객체를 생성할 때는 타입에 Grid 뿐만 아니라 Grid에 저장할 대상의 타입도 함께 지정해야 한다. 이렇게 클래스 템플릿에 특정한 타입을 지정해서 구체적인 클래스를 만드는 것을 템플릿 인스턴스화라고 한다.

템플릿 인스턴스화는 다음과 같이 객체를 선언하는 과정에 적용할 수 있다.

```cpp
Grid<int> myIntGrid; // int 값을 저장할 Grid 객체를 선언한다.
                     // 이때 생성자에 디폴트 인수를 적용한다.
Grid<double> myDoubleGrid(11, 11); // double 값에 대한 11x11 Grid 선언

myIntGrid.at(0, 0) = 10;
int x = myIntGrid.at(0, 0).value_or(0);

Grid<int> grid2(myIntGrid); // 복제 생성자
Grid<int> anotherIntGrid;
anotherIntGrid = grid2; // 대입 연산자
```

myIntGrid, grid2, anotherIntGrid의 타입은 모두 Grid<int>다. 이렇게 만든 Grid 객체에 SpreadsheetCell이나 ChessPiece 객체를 저장하는 코드를 작성하면 컴파일 에러가 발생한다.

at() 메서드는 std::optional 레퍼런스를 리턴한다. optional에 값이 없는 경우가 있기 때문에, value_or() 메서드를 이용하여 optional에 값이 있을 때만 그 값을 리턴하도록 한다. optional에 값이 없다면 value_or()에 전달한 인수를 리턴한다.

타입을 지정하는 문법이 중요하다. 다음과 같은 경우는 컴파일 에러가 발생한다.

```cpp
Grid test;
Grid<> test;
```

Grid 객체를 받는 함수나 메서드를 선언할 때도 Grid에 저장할 항목의 타입을 구체적으로 지정해야 한다.

```cpp
void processIntGrid(Gird<int>& grid)
{
    // 코드 생략
}
```

아니면, 이후에 설명할 함수 템플릿을 이용해서 Grid의 원소 타입을 매개변수로 표현한 함수로 작성한다.

> Grid\<int> 처럼 매번 Grid 타입에 대한 정식 표기법으로 작성하기 번거롭다면 다음과 같이 타입 앨리어스로 이름을 간단히 표현할 수 있다.
> `using IntGrid = Grid<int>;`

Grid 템플릿은 int 외의 타입 객체도 저장할 수 있다. 예를 들어 다음과 같이 SpreadsheetCell 객체를 저장하도록 Grid를 인스턴스화할 수 있다.

```cpp
Grid<SpreadsheetCell> mySpreadsheet;
SpreadsheetCell myCell(1.234);
mySpreadsheet.at(3, 4) = myCell;
```

다음처럼 포인터 타입 객체도 저장할 수 있다.

```cpp
Grid<const char*> myStringGrid;
myStringGrid.at(2, 2) = "hello";
```

또한 다른 템플릿 타입을 지정할 수도 있다.

```cpp
Grid<vector<int>> gridOfVectors;
vector<int> myVector{1, 2, 3, 4};
gridOfVectors.at(5, 6) = myVector;
```

Grid 템플릿 인스턴스화를 통해 객체를 힙에 동적으로 생성할 수도 있다.

```cpp
auto myGridOnHeap = make_unique<Grid<int>>(2, 2); // 힙에 2x2 Grid 생성
myGridOnHeap->at(0, 0) = 10;
int x = myGridOnHeap->at(0, 0).value_or(0);
```