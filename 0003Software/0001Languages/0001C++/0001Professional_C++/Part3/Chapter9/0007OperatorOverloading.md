---
sort: 7
---

# Operator Overloading

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 예제: SpreadsheetCell 덧셈 구현

SpreadsheetCell에 덧셈 기능을 객체지향 방식으로 구현하려면 SpreadsheetCell 객체에 다른 SpreadsheetCell 객체를 더하게 만들어야 한다.

### 1.1 add 메서드

```cpp
class SpreadsheetCell
{
    public:
        SpreadsheetCell add(const SpreadsheetCell& cell) const;
};

// 구현 코드
SreadsheetCell SpreadsheetCell::add(const SpreadsheetCell& cell) const
{
    return SpreadsheetCell(getValue() + cell.getValue());
}

// add() 메서드 사용 예
SpreadsheetCell myCell(4), anotherCell(5);
SpreadsheetCell aThirdCell = myCell.add(anotherCell);
```

add() 메서드는 두 셀을 더해서 그 결과를 새로 생성한 제 3의 셀에 담아서 리턴한다. 원본 셀을 변경하지 않도록 const로 선언하고 인수를 const SpreadsheetCell에 대한 레퍼런스로 받도록 선언한다.

### 1.2 operator+ 오버로딩

```cpp
class SpreadsheetCell
{
    public:
        SpreadsheetCell operator+(const SpreadsheetCell& cell) const;
};

// 구현 코드
SpreadsheetCell SpreadsheetCell::operator+(const SpreadsheetCell& cell) const
{
    return SpreadsheetCell(getValue() + cell.getValue());
}

// operator+ 사용 예
SpreadsheetCell myCell(4), anotherCell(5);
SpreadsheetCell aThirdCell = myCell + anotherCell;
```

operator+ 메서드를 정의함으로써 덧셈 연산자를 이용하여 두 셀을 더할 수 있게 되었다.

> C++ 컴파일러가 프로그램을 파싱할 때 +, -, =, << 와 같은 연산자를 발견하면 매개변수가 일치하는 operator+, operator-, operator=, operator<< 라는 이름의 함수나 메서드가 있는지 확인한다.

##### 묵시적 변환

operator+를 정의하면 셀끼리 더할 수 있을 뿐만 아니라 셀에 string_view, double, int와 같은 값도 더할 수 있다. 컴파일러가 단순히 operator+만 찾는 데 그치지 않고 타입을 정확히 변환할 수 있는 방법도 찾는다. 또한 지정한 타입을 변환할 방법도 찾는다.

```cpp
SpreadsheetCell myCell(4), aThirdCell;
string str = "hello";
aThirdCell = myCell + string_view(str);
aThirdCell = myCell + 5.6;
aThirdCell = myCell + 4;
```

컴파일러가 자신에 double값을 더하는 SpreadsheetCell을 발견하면 먼저 double 타입의 인수를 받는 SpreadsheetCell 생성자를 찾아서 임시 SpreadsheetCell 객체를 생성한 뒤 operator+로 전달한다. string_view나 int도 마찬가지다.

이렇게 묵시적 변환을 활용하면 편리할 때가 많다. 하지만 상식적으로 string_view를 더하는 것은 맞지 않다. string_view를 묵시적으로 변환하지 않게 하려면 생성자 앞에 `explicit` 키워드를 붙인다.

```cpp
class SpreadsheetCell
{
    public:
        SpreadsheetCell() = default;
        SpreadsheetCell(double initialValue);
        explicit SpreadsheetCell(std::string_view initialValue);
}
```

> explicit 키워드는 클래스를 정의하는 코드에서만 지정할 수 있으며, 인수를 하나만 지정해서 호출할 수 있는 생성자에만 적합하다.

### 1.3 operator+를 전역 함수로 구현

묵시적 변환은 다음과 같이 교환법칙이 성립하지 않는다.

```cpp
aThirdCell = myCell + 4; // 정상 작동
aThirdCell = 4 + myCell; // 컴파일 에러
```

하지만 클래스에 정의했던 operator+를 전역함수로 만들면 가능하다. 전역 함수는 특정 객체에 종속되지 않기 때문이다.

```cpp
SpreadsheetCell operator+(const SpreadsheetCell& lhs, const SpreadsheetCell& rhs)
{
    return SpreadsheetCell(lhs.getValue() + rhs.getValue());
}
```

전역 함수로 정의하려면 연산자를 헤더 파일에 선언해야 한다.

```cpp
class SpreadsheetCell
{
    // ...
};

SpreadsheetCell operator+(const SpreadsheetCell& lhs, const SpreadsheetCell& rhs);
```

## 2. 산술 연산자 오버로딩

다른 산술 연산자를 오버로딩하는 방법도 operator+와 비슷하다. 연산자를 다음과 같이 정의하면 `<op>` 자리에 +, -, *, / 연산자를 대입해서 각각의 연산을 수행할 수 있다.

> %도 오버로딩 할 수 있지만 SpreadsheetCell에 저장된 double 값에 적용하기에는 맞지 않다.

```cpp
class SpreadsheetCell
{
    // ...
};

SpreadsheetCell operator<op>(const SpreadsheetCell& lhs, const SpreadsheetCell& rhs);
```

구현 코드는 operator+와 같이 구현하면 되며, operator/ 의 경우 0으로 나누지 않도록 주의한다.

> operator*나 operator/에서 반드시 곱셈과 나눗셈을 구현해야한다는 법은 없지만, 가능하면 기존 연산자의 의미와 최대한 일치하도록 구현하는 것이 좋다.

### 2.1 축약형 산술 연산자의 오버로딩

축약형 연산자(+=, -= 등)도 연산자 오버로딩을 구현할 수 있다. 하지만 축약형 산술 연산자에 대한 오버로딩은 별도로 구현해야 한다.

축약형 연산자는 좌변의 객체를 새로 생성하지 않고 기존 객체를 변경한다는 점에서 기본 연산자 오버로딩과 다르다. 또한 대입 연산자처럼 수정된 객체에 대한 레퍼런스를 생성한다는 차이가 있다.

축약형 산술 연산자는 좌변에 반드시 객체가 나와야 한다. 따라서 전역 함수가 아닌 메서드로 구현해야 한다.

```cpp
class SpreadsheetCell
{
    public:
        SpreadsheetCell& operator+=(const SpreadsheetCell& rhs);
        SpreadsheetCell& operator-=(const SpreadsheetCell& rhs);
        SpreadsheetCell& operator*=(const SpreadsheetCell& rhs);
        SpreadsheetCell& operator/=(const SpreadsheetCell& rhs);
};

// operator+= 구현 코드 예시
SpreadsheetCell& operator+=(const SpreadsheetCell& rhs)
{
    set(getValue() + rhs.getValue());
    return *this;
}

// operator+= 메서드 사용 예시
SpreadsheetCell myCell(4), aThirdCell(2);
aThirdCell -= myCell;
aThirdCell += 5.4;
```

연산자에 대한 일반 버전과 축약 버전을 모두 정의할 때는 코드 중복을 피하도록 축약형 버전을 기준으로 일반 버전을 구현하는 것이 좋다.

```cpp
SpreadsheetCell operator+(const SpreadsheetCell& lhs, const SpreadsheetCell& rhs)
{
    auto result(lhs); // 로컬 복사본
    result += rhs; // op=() 버전으로 전달
    return result;
}
```

## 3. 비교 연산자 오버로딩

`>`, `<`, `==`와 같은 비교 연산자도 전역 함수로 구현해야 연산자의 좌변과 우변을 모두 묵시적으로 변환할 수 있다. 비교 연산자는 모두 bool 타입 값을 리턴한다.

다음과 같이 `<op>` 자리에 `==`, `<`, `>`, `!=`, `<=`, `>=` 이라는 여섯 개 연산자가 적용되도록 선언한다.

```cpp
class SpreadsheetCell
{
    // ...
};

bool operator<op>(const SpreadsheetCell& lhs, const SpreadsheetCell& rhs);

// operator== 구현 코드
bool operator==(const SpreadsheetCell& lhs, const SpreadsheetCell& rhs)
{
    return (lhs.getValue() == rhs.getValue());
}
```

`==`과 `<` 부터 구현한 뒤 이를 바탕으로 나머지 비교 연산자를 구현하면 한결 간편하다. 예를 들어 operator>= 을 operator<로 정의할 수 있다.

```cpp
bool operator>=(const SpreadsheetCell& lhs, const SpreadsheetCell& rhs)
{
    return !(lhs < rhs);
}
```

이렇게 정의한 비교 연산자로 다른 SpreadsheetCell 뿐만 아니라 double이나 int 값도 비교할 수 있다.

```cpp
if(myCell > aThirdCell || myCell < 10) {
    cout << myCell.getValue() << endl;
}
```