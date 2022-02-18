---
sort: 2
---

# Writing Classes

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 클래스 정의

다음은 SpreadsheetCell 클래스의 첫 버전으로, 각 셀마다 숫자 하나만 저장하도록 정의한다.

```cpp
class SpreadsheetCell
{
    public:
        void setValue(double inValue);
        double getValue() const;
    private:
        double mValue;
};
```

클래스 정의는 항상 `class` 키워드와 클래스 이름으로 시작한다. C++에서 클래스 정의는 문장으로, 반드시 세미콜론으로 끝나야 한다.

> 클래스 정의를 작성한 파일의 이름은 주로 클래스 이름과 똑같이 짓는다. 예를 들어 SpreadsheetCell 클래스 정의 파일은 SpreadsheetCell.h 파일에 저장한다.

### 1.1 클래스 멤버

클래스는 여러 개의 멤버를 가질 수 있다. 멤버는 메서드, 생성자, 소멸자와 같은 멤버 함수일 수도 있고, 열거형, 타입 앨리어스, 중첩 클래스와 같은 멤버 변수일 수도 있다. 멤버 변수는 데이터 멤버라고도 부른다.

SpreadsheetCell 클래스에서 메서드를 다음과 같이 선언했다. 여기서, 객체를 변경하지 않는 멤버 함수는 항상 const로 선언하는 것이 바람직하다.

```cpp
void setValue(double inValue);
double getValue() const;
```

SpreadsheetCell 클래스의 멤버 변수를 다음과 같이 선언했다.

```cpp
double mValue;
```

클래스는 멤버 함수와 이들이 사용할 데이터 멤버를 정의한다. 이러한 멤버는 그 클래스에 대한 인스턴스인 객체 단위로 적용된다. (단, 정적 멤버는 예외적으로 클래스 단위로 적용된다.)

멤버 함수의 구현 코드는 모든 객체가 공유하며, 클래스가 가질 수 있는 멤버 함수와 데이터 멤버수에는 제한이 없다. 또한 데이터 멤버의 이름과 멤버 함수의 이름이 같아도 된다.

### 1.2 접근 제어

클래스의 각 멤버는 세 가지 접근 제한자인 `public`, `protected`, `private` 중 하나로 지정한다. 한 번 지정된 접근 제한자는 다른 지정자로 변경하기 전까지 모든 멤버에 적용된다.

SpreadsheetCell 클래스에서 `setValue()`와 `getValue()` 메서드는 `public`으로 지정한 반면 데이터 멤버인 `mValue`는 private로 지정했다.

클래스에 접근 제한자를 따로 명시하지 않으면 `private`가 적용된다.

> C++에서는 struct도 class처럼 메서드를 가질 수 있다. 이는 struct의 디폴트 접근 제한자가 public이란 점을 제외하면 class와 같다. 관례상 메서드가 없거나 적으며, 데이터 멤버를 누구나 접근 가능한 구조라면, class보다 struct를 주로 사용한다.

접근 제한자의 의미와 용도를 정리하면 다음과 같다.

##### public
클래스 내부나 외부 등 어디에서나 객체의 멤버 함수나 데이터 멤버에 접근할 수 있다. 따라서 클라이언트가 사용할 동작에 사용되거나, private 또는 protected 데이터 멤버에 대한 접근 메서드에 사용된다.

##### protected
같은 클래스로 된 객체의 멤버 함수로 접근할 수 있으며, 파생 클래스의 멤버함수를 통해 베이스 클래스의 protected 멤버에 접근할 수 있다. 해당 접근 제한자는 외부 클라이언트가 사용하면 안 되는 헬퍼 메서드에 적용한다.

##### private
같은 클래스의 멤버 함수로만 접근할 수 있으며, 파생 클래스에서 베이스 클래스의 private 멤버에 접근할 수 없다. 일반적으로 데이터 멤버를 private로 지정한다.

### 1.3 선언 순서
C++ 에서는 멤버와 접근 제한자를 선언하는 순서를 지정하지 않으며, 접근 제한자를 반복해서 지정해도 문제없다.

### 1.4 클래스 내부의 멤버 이니셜라이저
클래스를 정의할 때 멤버 변수를 선언하는 동시에 초기화할 수 있다.

```cpp
class SpreadsheetCell
{
    // 클래스 정의의 나머지 부분은 생략
    private:
        double mValue = 0;
};
```

## 2. 메서드 정의 방법
앞선 SpreadsheetCell 클래스 정의는 메서드의 프로토타입만 선언했다. 프로토타입을 선언한 메서드의 정의 코드를 작성해야 하는데, 메서드 정의 코드보다 클래스 정의 코드가 먼저 나와야 한다. 클래스 정의는 주로 헤더 파일에 작성하고, 메서드 정의는 소스 파일에 작성한 뒤 소스 파일에서 `#include` 문으로 헤더 파일을 불러오는 방식으로 사용한다.

```cpp
#include "SpreadsheetCell.h"

void SpreadsheetCell::setValue(double inValue)
{
    mValue = inValue;
}

double SpreadsheetCell::getValue() const
{
    return mValue;
}
```

> `::`는 스코프 지정 연산자라 부른다. 컴파일러가 이 코드를 보면 `setValue()` 메서드는 SpreadsheetCell 클래스에 속한다는 것을 알 수 있다.

### 2.1 데이터 멤버 접근 방법
`setValue()`나 `getValue()` 메서드 같은 비정적 멤버는 항상 클래스가 아닌 객체에 대해 실행된다. 다시 말해 이런 메서드는 클래스에 정의된 데이터 멤버 중 현재 객체에 속한 멤버에 대해 접근한다.

예를 들어 어떤 객체든 앞에서 정의한 `setValue()` 메서드를 호출하면 자기 객체에 있는 mValue 변수의 값을 변경한다.

### 2.2 다른 메서드 호출하기
메서드는 같은 클래스에 정의된 다른 메서드도 호출할 수 있다. 

예를 들어 SpreadsheetCell 클래스를 확장하여, 스프레트시트 셀 안에 숫자 뿐만 아니라 텍스트 데이터도 넣을 수 있도록 확장한다. 텍스트 값을 가진 셀을 숫자로 해석하고 싶다면 텍스트를 숫자로 변환해준다. 이때 텍스트로 표현한 숫자가 올바르지 않다면 그 셀의 값을 무시한다. 또한 셀에 담긴 스트링이 정상적인 숫자를 표현하지 않으면 0으로 변환한다.

위와 내용을 SpreadsheetCell에 추가로 반영하면 다음과 같다.

```cpp
#include <string>
#include <string_view>

class SpreadsheetCell
{
    public:
        void setValue(double inValue);
        double getValue() const;

        void setString(std::string_view inString);
        std::string getString() const;
    
    private:
        std::string doubleToString(double inValue) const;
        double stringToDouble(std::string_view inString) const;
        double mValue;
};
```

위 코드는 `mValue`를 double로만 저장한다. 클라이언트가 이 데이터 멤버를 string 값으로 설정하면 double로 변환된다. 입력한 텍스트가 정상적인 숫자를 표현하지 않으면 0.0으로 설정된다.

코드는 텍스트 값을 설정하고 가져오는 메서드 두 개와 double과 string 값을 상호 변환하는 private 헬퍼 메서드 두 개를 추가로 정의했다.

정의 코드는 다음과 같다.

```cpp
#include "SpreadsheetCell.h"
using namespace std;

void SpreadsheetCell::setValue(double inValue)
{
    mValue = inValue;
}

double SpreadsheetCell::getValue() const
{
    return Value;
}

void SpreadsheetCell::setString(string_view inString)
{
    mValue = stringToDouble(inString);
}

string SpreadsheetCell::getString() const
{
    return doubleToString(mValue);
}

string SpreadsheetCell::doubleToString(double inValue) const
{
    return to_string(inValue);
}

double SpreadsheetCell::stringToDouble(string_view inString) const
{
    return strtod(inString.data(), nullptr);
}
```

### 2.3 this 포인터
일반 메서드를 호출하면 항상 메서드가 속한 객체의 포인터인 `this`가 숨겨진 매개변수 형태로 전달된다.

`this` 포인터로 해당 객체의 데이터 멤버나 메서드에 접근할 수 있으며, 다른 메서드나 함수에 매개변수로 전달할 수도 있다.

어떤 객체의 메서드 안에서 다른 메서드나 함수를 호출하는 과정에서 그 객체의 포인터를 전달할 때도 `this` 포인터를 사용한다. 예를 들어 다음과 같이 `printCell()` 이라는 함수를 별도로 만든 경우를 생각해보자.

```cpp
void printCell(const SpreadsheetCell& cell)
{
    cout << cell.getString() << endl;
}
```

`printCell()` 함수를 `setValue()` 메서드 안에서 호출하려면 반드시 `*this`를 인수로 전달해야 한다. 그래야 `printCell()` 안에서 호출할 메서드는 자신을 호출한 `setValue()`가 속한 것임을 알 수 있다.

```cpp
void SpreadsheetCell::setValue(double value)
{
    this.value = value;
    printCell(*this);
}
```

## 3. 객체 사용법
앞에서 작성한 SpreadsheetCell 클래스 정의에 따라 SpreadsheetCell 객체를 생성하려면 SpreadsheetCell 타입의 변수를 따로 선언해야 한다.

### 3.1 스택에 생성한 객체
SpreadsettCell 객체를 스택에 생성해서 사용하는 예시이다. 객체를 생성하는 방법은 변수를 선언하는 방법과 같다. `.` 연산자를 이용해 객체에 속한 메서드를 호출한다.

```cpp
SpreadsheetCell myCell, anotherCell;
myCell.setValue(6);
anotherCell.setString("3.2");
cout << "cell 1: " << myCell.getValue() << endl;
cout << "cell 2: " << anotherCell.getValue() << endl;
```

### 3.2 힙에 생성한 객체
new를 사용한 동적으로 객체를 생성한 예시이다. 힙에 생성한 객체는 `->` 연산자로 멤버에 접근한다. `->` 연산자는 역참조 연산자 `*` 와 `.` 연산자를 합친 것이다.

```cpp
SpreadsheetCell* myCellp = new SpreadsheetCell();
myCellp->setValue(3.7);
cout << "cell 1: " << myCellp->getValue() << " " << myCellp->getString() << endl;
delete myCellp;
myCellp = nullptr;
```

메모리 관련 문제가 발생하지 않게 하기 위해 다음과 같이 스마트 포인터를 사용한다.

```cpp
auto myCellp = make_unique<SpreadsheetCell>();
myCellp->setValue(3.7);
cout << "cell 1: " << myCellp->getValue() << " " << myCellp->getString() << endl;
```