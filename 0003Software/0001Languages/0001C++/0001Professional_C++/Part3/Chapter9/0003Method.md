---
sort: 3
---

# More About Methods

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. static 메서드
static 메서드는 객체 정보에 접근하지 않는 메서드에 사용한다. static 메서드는 특정 객체에 대해 호출되지 않기 때문에 this 포인터를 가질 수 없으며 어떤 객체의 non-static 멤버에 접근하는 용도로 호출할 수 없다.

static 메서드는 일반 함수와 비슷하지만 클래스의 private static이나 protected static 멤버만 접근할 수 있다는 점이 다르다.

또한 같은 타입의 객체를 포인터나 레퍼런스로 전달하는 방법 등을 사용해서 그 객체를 static 메서드에서 볼 수 있게 만들었다면 클래스에서 non-static private 또는 protected 멤버에 접근하 수 있다.

```cpp
class SpreadsheetCell
{
    private:
        static std::string doubleToString(double inValue);
        static double stringToDouble(std::string_view inString);
        // ...
};
```

같은 클래스 안에서는 static 메서드를 일반 함수처럼 호출할 수 있으며, 클래스 밖에서 호출할 때는 스코프 지정 연산자 `::`를 이용하여 사용할 수 있다.

## 2. const 메서드

const 객체란 값이 바뀌지 않는 객체를 말한다. const 객체나 이에 대한 레퍼런스 또는 포인터를 사용할 때는 그 객체의 데이터 멤버를 절대로 변경하지 않는 메서드만 호출할 수 있다.

const는 메서드를 구현하는 코드에서도 반드시 적어야 한다.
```cpp
class SpreadsheetCell
{
    public:
        double getValue() const;
        std::String getString() const;
        // ...
};

double SpreadsheetCell::getValue() const
{
    return mValue;
}

std::string SpreadsheetCell::getString() const
{
    return doubleToString(mValue);
}
```

static 메서드를 const로 선언해서는 안 된다. static 메서드는 근본적으로 클래스의 인스턴스에 속하지 않아 객체 내부의 값을 변경할 수 없기 때문에 static 메서드에 const 키워드를 붙이는 것은 의미가 없기 때문이다.

만약 객체를 const로 선언했다면 그 객체의 const 메서드만 호출할 수 있다.

```cpp
SpreadsheetCell myCell(5);
cout << myCell.getValeu() << endl;
myCell.setString("6");

const SpreadsheetCell& myCellConstRef = myCell;
cout << myCellConstRef.getValue() << endl;
myCellConstRef.setString("6"); // 에러 발생
```

> const 객체에 대한 레퍼런스를 사용할 수 있도록 객체를 수정하지 않는 메서드는 모두 const로 선언하는 습관을 가져야 한다

### 2.1 mutable 데이터 멤버

때로는 const 메서드에서 객체의 데이터 멤버를 변경하는 경우가 있다. 의미상으로는 사용자 데이터에 아무런 영향을 미치지 않더라도 메서드 내에서 수정이 일어나기 때문에 문제가 발생할 수 있다.

예를 들어 SpreadsheetCell 클래스에 카운터를 두고 getValue()나 getString()이 호출될 때마다 카운터를 업데이트 하는 경우 non-const가 되기 때문에 문제가 발생한다. 이런 경우에 카운터 변수를 `mutable`로 선언하여 const 메서드에서 변경할 수 있다고 명시하면 된다.

```cpp
class SpreadsheetCell
{
    private:
        double mValue = 0;
        mutable size_t mNumAccesses = 0;
};
```

## 3. 메서드 오버로딩

메서드는 함수와 마찬가지로 매개변수의 타입이나 개수만 다르게 지정하여 오버로딩을 할 수 있다.

예를 들어 SpreadsheetCell 클래스의 setString()과 setValue()를 모두 set()으로 통일할 수 있다.

```cpp
class SpreadsheetCell
{
    public:
        void set(double inValue);
        void set(std::string_view inString);
};
```

> getValue()와 getString() 메서드의 경우 리턴 타입이 다르므로, 오버로딩을 할 수 없다.

### 3.1 const 기반 오버로딩

const를 기준으로 오버로딩할 수 있다. 예를 들어 메서드를 두 개 정의할 때 이름과 매개변수는 같지만 하나는 const로 선언한다. 그러면 const 객체에서 이 메서드를 호출하면 const 메서드가 실행된다.

만약 const 버전과 non-const 버전의 구현 코드가 같은 경우 const_cast() 패턴을 적용한다.

SpreadsheetCell 클래스에 SpreadsheetCell 레퍼런스를 리턴하는 getCellAt() 메서드가 있을 때 const을 리턴하는 버전과 non-const 리턴하는 버전에 대하여 오버로딩을 다음과 같이 구현한다.

```cpp
class Spreadsheet
{
    public:
        SpreadsheetCell& getCellAt(size_t x, size_t y);
        const SpreadsheetCell& getCellAt(size_t x, size_t y) const;
        // ...
};

const SpreadsheetCell& Spreadsheet::getCellAt(size_t x, size_t y) const
{
    verifyCoordinate(x, y);
    return mCells[x][y];
}

SpreadsheetCell& Spreadsheet::getCellAt(size_t x, size_t y)
{
    return const_cast<SpreadsheetCell&>(std::as_const(*this).getCellAt(x, y));
}
```

const_cast() 패턴은 const 버전은 그대로 구현하고, non-const 버전은 const 버전을 적절히 캐스팅하여 호출하는 방식으로 구현한다. std::as_const()로 *this를 const Spreadsheet&로 캐스팅하고, const 버전의 getCellAt()을 호출한 다음 const_cast()를 적용해 리턴된 결과에서 const를 제거하는 방식으로 처리한다.

std::as_const() 함수는 C++17 부터 추가되었으므로, 컴파일러에서 지원하지 않는다면 static_cast()를 사용한다.

```cpp
return const_cast<SpreadsheetCell&>(
    static_cast<const Spreadsheet&>(*this).getCellAt(x, y));
```

const 및 non-const 방식의 getCellAt() 메서드를 호출하는 방식은 다음과 같다.

```cpp
Spreadsheet sheet1(5, 6);
SpreadsheetCell& cell1 = sheet1.getCellAt(1, 1);

const Spreadsheet sheet2(5, 6);
const SpreadsheetCell& cell2 = sheet2.getCellAt(1, 1);
```

### 3.2 명시적으로 오버로딩 제거하기

오버로딩된 메서드를 명시적으로 삭제할 수 있다. 그러면 특정한 인수에 대해서는 메서드를 호출하지 못하게 된다.

```cpp
class MyClass
{
    public:
        void foo(int i);
};

// foo() 메서드 호출 예제
MyClass c;
c.foo(123);
c.foo(1.23);
```

위의 예제에서 컴파일러는 double 값 (1.23)을 정수 (1)로 변환해서 `foo(int i)`를 호출한다. 이를 막기 위해서는 foo() 메서드의 double 버전을 명시적으로 삭제하도록 선언해야 한다.

```cpp
class MyClass
{
    public:
        void foo(int i);
        void foo(double d) = delete;
};
```

## 4. 인라인 메서드

메서드를 별도의 코드 블록에 구현해서 호출하지 않고 메서드를 호출하는 부분에서 곧바로 구현 코드를 작성하는 방법을 인라이닝(inlining)이라고 부르며, 이렇게 구현한 메서드를 인라인 메서드라 부른다.

> 일반적으로 #define 매크로보다 인라인 메서드를 사용하는 것이 더 안전하다.

```cpp
inline double SpreadsheetCell::getValue() const
{
    mNumAccesses++;
    return mValue;
}

inline std::string SpreadsheetCell::getString() const
{
    mNumAccesses++;
    return doubleToString(mValue);
}
```

이렇게 작성하면, 컴파일러는 getValue()와 getString()을 호출하는 부분을 함수 호출로 처리하지 않고, 그 함수의 본문을 곧바로 집어넎는다. 만약 인라인 메서드가 성능에 문제가 될 것 같으면 무시할 수도 있다.

컴파일러는 코드 비대화와 같은 몇 가지 기준에 따라 메서드나 함수를 인라인으로 처리할지 판단해서 큰 효과가 없다면 인라인으로 처리하지 않는다. 따라서 간단한 메서드나 함수만 인라인으로 처리해야 한다.

인라인 메서드는 반드시 프로토타입과 구현 코드를 헤더 파일에 작성한다. 또한 inline 키워드를 사용하지 않고 클래스 정의에서 곧바로 메서드 정의 코드를 작성하면 인라인 메서드로 처리해준다.

```cpp
class SpreadsheetCell
{
    public:
        double getValue() const { mNumAccesses++; return mValue; }

        std::string getString() const
        {
            mNumAccesses++;
            return doubleToString(mValue);
        }

        // ...
};
```

## 5. 디폴트 인수

메서드 오버로딩과 비슷한 기능으로 디폴트 인수라는 것도 있다. 이 기능을 이용하여 함수나 메서드의 프로토타입에 매개변수의 디폴트 값을 지정할 수 있다.

즉, 사용자가 다른 값으로 지정한 인수를 전달하면 디폴트 값을 무시하며, 인수를 지정하지 않으면 디폴트 값을 적용한다.

매개변수에 디폴트 값을 지정할 때 반드시 오른쪽 끝의 매개변수부터 시작해서 중간에 건너뛰지 않고 연속적으로 나열해야 한다.

> 디폴트 인수는 함수, 메서드, 생성자에서 지정할 수 있다.

```cpp
class Spreadsheet
{
    public:
        Spreadsheet(size_t width = 100, size_t height = 100);
        // ...
};
```

디폴트 인수는 메서드를 선언하는 코드에서만 지정할 수 있다. 위와 같이 선언하면 Spreadsheet에 비복제 생성자가 하나만 있어도 다음과 같이 인수를 0개, 1개, 2개 지정해서 호출할 수 있다.

```cpp
Spreadsheet s1;
Spreadsheet s2(5);
Spreadsheet s3(5, 6);
```

> 모든 매개변수에 대해 디폴트 값이 지정된 생성자는 디폴트 생성자처럼 쓸 수 있으나, 만약 디폴트 생성자가 있을 때 모든 매개변수에 대폴트 값이 지정된 생성자도 함께 작성하면 컴파일 에러가 발생한다.
