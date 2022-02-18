---
sort: 3
---

# Object Life Cycles

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 객체 생성
스택에 생성되는 객체는 선언하는 시점에 생성되고, 스마트 포인터나 new, new[]를 사용할 때는 직접 공간을 할당해야 생성된다. 객체가 생성되면 그 안에 담긴 객체도 함께 생성된다.

### 1.1 생성자 작성 방법
생성자는 클래스 객체의 선언과 동시에 초깃값을 설정하는 특수한 메서드이다. 이 메서드에 객체를 초기화하는 코드를 작성하는 방식으로 초기화를 수행하게 된다. 그러면 객체가 생성될 때마다 클래스에 정의된 생성자 중에서 적합한 것이 실행된다.

생성자 이름은 클래스 이름과 똑같이 지정한다. 생성자는 리턴값이 없으며, 필요에 따라 매개변수를 받을 수 있다.

아무런 인수를 주지 않고 호출하는 생성자를 디폴트 생성자라 부른다. 매개변수를 하나도 받지 않게 작성하거나, 모든 배개변수가 디폴트 값으로 설정되도록 작성한다.

SpreadsheetCell 클래스의 생성자를 추가하면 다음과 같다.

```cpp
class SpreadsheetCell
{
    public:
        SpreadsheetCell(double initialValue);
    
    // 클래스 정의의 나머지 부분은 생략
};
```

SpreadsheetCell 생성자의 구현코드는 다음과 같다.

```cpp
SpreadsheetCell::SpreadsheetCell(double initialValue)
{
    setValue(initialValue);
}
```

생성자도 일종의 클래스 멤버이므로, 생성자 이름 앞에 반드시 스코프 지정 연산자를 붙여야 한다.

### 1.2 생성자 사용법

#### 1.2.1 스택 객체 생성자
스택에 할당한 SpreadsheetCell 객체의 생성자를 호출하는 방법은 다음과 같다.

```cpp
SpreadsheetCell myCell(5), anotherCell(4);
cout << "cell 1: " << myCell.getValue() << endl;
cout << "cell 2: " << anotherCell.getValue() << endl;
```

#### 1.2.2 힙 객체 생성자
SpreadsheetCell 객체를 동적으로 할당할 때 생성자를 호출하는 방법은 다음과 같다.

```cpp
// 스마트 포인터를 이용한 방법
auto smartCellp = make_unique<SpreadsheetCell>(4);

// 일반 포인터를 이용한 방법, 권장하지 않음
SpreadsheetCell* myCellp = new SpreadsheetCell(5);
spreadsheetCell* anotherCellp = nullptr;
anotherCellp = new SpreadsheetCell(4);
delete myCellp;
delete anotherCellp;
myCellp = nullptr;
anotherCellp = nullptr;
```

### 1.3 생성자 여러 개 제공하기
클래스에 생성자를 인수의 개수나 타입을 다르개 정의하여 여러 개 만들 수 있다(오버로딩).

다음은 SpreadsheetCell 클래스에 string 타입의 초깃값을 받는 생성자를 추가하였다.
```cpp
class SpreadsheetCell
{
    public:
        SpreadsheetCell(double iitialValue);
        SpreadsheetCell(std::string_view initialValue);
        // 나머지 코드 생략
};
```

구현 코드는 다음과 같다.
```cpp
SpreadsheetCell::SpreadsheetCell(string_view initialValue)
{
    setString(initialValue);
}
```

### 1.4 디폴트 생성자
디폴트 생성자는 아무런 인수도 받지 않는 생성자다.

#### 1.4.1 디폴트 생성자가 필요한 경우
객체 배열을 생성하는 경우, 각 객체마다 디폴트 생성자를 호출하게 된다. 예를 들어 SpreadsheetCell 클래스에 디폴트 생성자를 정의하지 않으면 다음의 코드에서 에러가 발생한다.

```cpp
SpreadsheetCell cells[3];   // 디폴트 생성자가 없어서 컴파일 오류 발생
SpreadsheetCell* myCellp = new SpreadsheetCell[10]; // 오류 발생
```

std::vector와 같은 표준 라이브러리 컨테이너에 저장하는 경우에도 반드시 디폴트 생성자를 정의해야 한다. 또한 어떤 클래스의 객체를 다른 클래스에서 생성할 때도 디폴트 생성자가 있으면 편하다.

#### 1.4.2 디폴트 생성자 작성 방법

SpreadsheetCell 클래스의 디폴트 생성자 정의는 다음과 같다.

```cpp
class SpreadsheetCell
{
    public:
        SpreadsheetCell();
        // 나머지 코드 생략
};
```

디폴트 생성자의 구현 코드는 다음과 같다.

```cpp
SpreadsheetCell::SpreadsheetCell()
{
    mValue = 0;
}
```

만약 멤버 이니셜라이저를 적용하는 경우, 생성자 구현 코드에 작성한 문장 또한 생략할 수 있다.

```cpp
SpreadsheetCell::SpreadsheetCell()
{
}
```

스택 객체의 디폴트 생성자 호출 방법은 다음과 같다.

```cpp
SpreadsheetCell myCell;
SpreadsheetCell myCell();   // 잘못된 문장
```

힙 객체의 디폴트 생성자 호출 방법은 다음과 같다.

```cpp
SpreadsheetCell* myCellp = new SpreadsheetCell();
SpreadsheetCell* myCellp = new SpreadsheetCell;     // 해당 문장도 가능
```

#### 1.4.3 컴파일러에서 생성한 디폴트 생성자

클래스의 생성자를 하나도 지정하지 않으면 인수를 받지 않는 디폴트 생성자를 컴파일러가 대신 만들어준다. 컴파일러에서 생성한 디폴트 생성자는 해당 클래스의 객체 멤버에 대해서도 디포르 생성자를 호출해준다. 하지만 int나 double과 같은 기본 타입에 대해서는 초기화하지 않는다.

#### 1.4.4 명시적 디폴트 생성자

빈 껍데기인 디폴트 생성자의 구현 코드를 적는 수고를 덜기 위해 명시적 디폴트 생성자를 제공한다. `default` 키워드를 사용하여 컴파일러가 디폴트 생성자를 자동으로 생성해준다.

```cpp
class SpreadsheetCell
{
    public:
        SpreadsheetCell() = default;
        SpreadsheetCell(double initialValue);
        SpreadsheetCell(std::string_view initialValue);
        // 나머지 코드 생략
};
```

#### 1.4.5 명시적으로 삭제된 생성자
정적 메서드로만 구성된 클래스를 정의하면 생성자를 작성할 필요가 없을 뿐만 아니라 컴파일러가 디폴트 생성자를 만들면 안 된다. 이렇게 `delete` 키워드를 이용하여 디폴트 생성자를 명시적으로 삭제하는 방법은 다음과 같다.

```cpp
class MyClass
{
    public:
        MyClass() = delete;
};
```

### 1.5 생성자 이니셜라이저
데이터 멤버를 초기화하기 위한 방법으로 생성자 이니셜라이저를 제공한다. SpreadsheetCell 클래스에 생성자 이니셜라이저를 적용하면 다음과 같다.

```cpp
SpreadsheetCell::SpreadsheetCell(double initialValue) : mValue(initialValue)
{
}
```

> 생성자 이니셜라이저를 이용하면 생성자 안에서 데이터 멤버를 초기화하는 것보다 더 효율적이다. 생성자 이니셜라이저를 이용하는 방법은 데이터 멤버를 생성하면서 초깃값을 설정하기 때문이다.

클래스에 있는 데이터 멤버 중 디폴트 생성자가 정의되어 있지 않다면 생성자 이니셜라이저를 사용해 그 객체를 적절히 초기화해야 한다. 예를 들어 std::string 타입의 데이터 멤버는 디폴트 생성자에 의해 초기화를 하지만, 디폴트 생성자가 정의되어 있지 않는 클래스를 데이터 멤버로 정의하는 클래스에서는 초기화할 방법이 없게되어 에러가 발생한다.

```cpp
class SpreadsheetCell
{
    public:
        SpreadsheetCell(double d); // 디폴트 생성자 정의 X
};

class SomeClass
{
    public:
        SomeClass();
    private:
        SpreadsheetCell mCell;
};
```

위와 같이 작성된 SomeClass 생성자를 구현하는 코드를 다음과 같이 작성하면 에러가 발생한다. mCell에 대하여 디폴트 생성자가 호출되어야 하는데, SpreadsheetCell 클래스의 디폴트 생성자가 정의되지 않았기 때문에 mCell을 초기화할 방법이 없다.

```cpp
SomeClass::SomeClass() { }
```

따라서 mCell을 명시적으로 생성자 이니셜라이저를 이용하여 초기화해주어야 한다.

```cpp
SomeClass::SomeClass() : mCell(1.0) { }
```

* 생성자 이니셜라이저 또는 클래스 내부 생성자 구문으로 초기화해야 하는 타입
    * const 데이터 멤버
    * 레퍼런스 데이터 멤버
    * 디폴트 생성자가 정의되지 않은 객체 데이터 멤버
    * 디폴트 생성자가 없는 베이스 클래스

생성자 이니셜라이저를 사용할 때 주의해야 할 점은, 생성자 이니셜라이저에서 나열한 순서가 아닌 클래스 정의에 작성한 순서대로 초기화된다는 것이다.

예를 들어 다음과 같이 정의된 Foo 클래스가 있고, 생성자에서는 단순히 double 값을 지정한 뒤 콘솔에 출력하도록 작성되었다.

```cpp
class Foo
{
    public:
        Foo(double value);
    private:
        double mValue;
};

Foo::Foo(double value) : mValue(value)
{
    cout << "Foo::mValue = " << mValue << endl;
}
```

이때 다음과 같이 Foo 객체를 데이터 멤버로 가지는 MyClass란 클래스를 정의하고, 생성자의 구현코드에서 생성자 이니셜라이저를 이용하여 double 데이터 멤버를 초기화한 후 해당 데이터 멤버를 Foo 생성자에 인수로 전달하여 초기화하였다. 생성자 내부에서는 double 데이터 멤버의 값을 출력하도록 작성되었다.

```cpp
class MyClass
{
    public:
        MyClass(double value);
    private:
        double mValue;
        Foo mFoo;
};

MyClass::MyClass(double value) : mValue(value), mFoo(mValue)
{
    cout << "MyClass::mValue = " << mValue << endl;
}
```

MyClass 인스턴스를 생성하고 출력되는 결과는 정상적으로 출력된다.

```cpp
MyClass instance(1.2);
```

```bash
Foo::mValue = 1.2
MyClass::mValue = 1.2
```

만약 MyClass 클래스 정의에서 mValue와 mFoo의 위치를 바꾸면 잘못된 결과가 나온다.

```cpp
class MyClass
{
        MyClass(double value);
    private:
        Foo mFoo;
        double mValue;    
};
```

생성자 이니셜라이저에서 mValue를 먼저 초기화 문장을 사용했지만, 실제 초기화는 클래스 정의에 나온 순서대로 초기화되기 때문에, mFoo를 초기화할 때 사용되는 mValue 값은 초기화되지 않은 값을 사용하게 된다.

### 1.6 복제 생성자
복제 생성자는 다른 객체와 똑같은 객체를 생성할 때 사용된다. 복제 생성자를 직접 작성하지 않으면 컴파일러가 대신 만들어준다. 컴파일러가 생성한 복제 생성자는 데이터 멤버가 기본 타입이라면 똑같이 복사하고, 객체 타입이라면 그 객체의 복제 생성자를 호출한다.

SpreadsheetCell 클래스에 복제 생성자를 추가하면 다음과 같다.

```cpp
class SpreadsheetCell
{
    public:
        SpreadsheetCell(const SpreadsheetCell& src);
        // 나머지 코드 생략
};
```

복제 생성자는 원본 객체에 대한 const 레퍼런스를 인수로 받는다. 생성자 안에서 원본 객체에 있는 데이터 멤버를 모두 복사한다.

복제 생성자의 구현 코드는 다음과 같다.

```cpp
SpreadsheetCell::SpreadsheetCell(const SpreadsheetCell& src) : mValue(src.mValue)
{
}
```

만약 데이터 멤버가 m1, m2, ... mn과 같이 선언돼 있다면 컴파일러는 다음과 같이 생성자를 만들어 준다. 즉, 대부분은 복제 생성자를 직접 작성할 필요가 없다.

```cpp
classname::classname(const classname& src)
    : m1(src.m1), m2(src.m2), ..., mn(src.mn)
{
}
```

#### 1.6.1 복제 생성자가 호출되는 경우
함수에 인수를 전달할 때 기본적으로 값으로 전달된다. 다시 말해 함수나 메서드는 값이나 객체의 복사본을 받는다. 따라서 함수나 메서드에 객체를 전달하면 컴파일러는 그 객체의 복제 생성자를 호출하는 방식으로 초기화한다.

예를 들어 string 매개변수를 값으로 받는 printfString() 함수가 있다.

```cpp
void printString(string inString)
{
    cout << inString << endl;
}
```

C++에서 string 타입은 일종의 클래스이기 때문에 printStinrg()에 string 매개변수를 전달해서 호출하면 string 매개변수인 inString은 이 클래스의 복제 생성자를 호출하는 방식으로 초기화된다. 

다음과 같이 printStinrg()에서 매개변수를 name으로 지정해서 호출하면 inString 객체를 초기화할 때 string의 복제 생성자가 실행된다.

```cpp
string name = "heading one";
printString(name);  // name을 복제
```

#### 1.6.2 복제 생성자 명시적으로 호출하기
다른 객체를 똑같이 복사하는 방식으로 객체를 만들 때 복제 생성자를 명시적으로 호출한다.

예를 들어 SpreadsheetCell 객체의 복사본을 만들려면 다음과 같이 작성한다.

```cpp
SpreadsheetCell myCell1(4);
SpreadsheetCell myCell2(myCell1);
```

#### 1.6.3 레퍼런스로 객체 전달하기
함수나 메서드에 객체를 레퍼런스로 전달하면 복제 연산으로 인한 오버헤드를 줄일 수 있다. 객체를 레퍼런스로 전달하는 방식이 값으로 전달하는 것보다 대체로 효율적이다. 객체에 있는 내용 전체가 아닌 객체의 주소만 복사하기 때문이다.

또한 레퍼런스 전달 방식을 사용하면 객체의 동적 메모리 할당에 관련된 문제도 피할 수 있다.

객체를 레퍼런스로 전달할 때 그 값을 사용하는 함수나 메서드는 원본 객체를 변경할 수 있다. 따라서 성능의 이유로 레퍼런스 전달 방식을 사용한다면 객체가 변경되지 않도록 객체 앞에 const를 붙여야 한다.

#### 1.6.4 명시적으로 디폴트로 만든 복제 생성자와 명시적으로 삭제된 복제 생성자
컴파일러가 생성한 복제 생성자를 명시적으로 디폴트로 만들거나 삭제할 수 있다.

```cpp
SpreadsheetCell(const SpreadsheetCell& src) = default;

SpreadsheetCell(const SpreadsheetCell& src) = delete;
```

복제 생성자를 삭제하면 객체를 더 이상 복제할 수 없다. 객체를 값으로 전달하지 않게 할 때 이렇게 설정한다.

### 1.7 이니셜라이저 리스트 생성자
이니셜라이저 리스트 생성자란 `std::initializer_list<T>`를 첫 번째 매개변수로 받고, 다른 매개변수는 없거나 디폴트 값을 가진 매개변수를 추가로 받는 생성자를 말한다.

`std::initializer_list<T>` 템플릿을 사용하려면 `<initializer_list>` 헤더를 인클루드 해야한다.

사용법은 다음과 같다. 이 클래스는 짝수 개의 원소를 가진 `initializer_list<T>`만 매개변수로 받는다. 짝수가 아니면 익셉션이 발생한다.

```cpp
class EvenSequence
{
    public:
        EvenSequence(initializer_list<double> args)
        {
            if(args.size() % != 0) {
                throw invalied_argument("initializer_list should "
                    "contain even number of elements.");
            }
            mSequence.reserve(args.size());
            for(const auto& value : args){
                mSequence.push_back(value);
            }
        }

        void dump() const
        {
            for(const auto& value : mSequence) {
                cout << value << ", ";
            }
            cout << endl;
        }
    private:
        vector<double> mSequence;
}
```

이니셜라이저 리스트 생성자 안에서 각 원소에 접근하는 부분을 범위 기반 for 문으로 구현할 수 있다. 또는 vector의 `assign()` 메서드를 사용해도 된다.

EvenSequence 객체는 다음과 같이 생성한다.

```cpp
EvenSequence p1 = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
p1.dump();

try {
    EvenSequence p2 = {1.0, 2.0, 3.0};
} catch (const invalied_argument& e) {
    cout << e.what() << endl;
}
```

p1 생성자는 다음과 같이 등호를 생략해도 된다.

```cpp
EvenSequence p1{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
```

p2 생성자자에서는 원소 수가 홀수이기 때문에 익셉션이 발생한다.

> 표준 라이브러리에 나온 클래스는 모두 이니셜라이저 리스트 생성자를 지원한다.

### 1.8 위임 생성자
위임 생성자를 사용하면 같은 클래스의 다른 생성자를 생성자 안에서 호출할 수 있다. 하지만 생성자 안에서 다른 생성자를 직접 호출할 수는 없다. 반드시 생성자 이니셜라이저에서 호출해야 하며, 멤버 이니셜라이저 리스트에 이것만 적어야 한다.

```cpp
SpreadsheetCell::SpreadsheetCell(string_view initialValue) : SpreadsheetCell(stringToDouble(initialValue))
{
}
```

해당 예제에서 string_view 타입 생성자(위임 생성자)가 호출되면 타깃 생성자(double 타입 생성자)에 위임한다. 타깃 생성자가 리턴하면 위임 생성자의 코드가 실행된다.

### 1.9 컴파일러가 생성하는 생성자에 대한 정리
디폴트 생성자와 복제 생성자 사이에 일정한 패턴이 없다. 복제 생성자를 명시적으로 정의하지 않는 한 컴파일러는 무조건 복제 생성자를 만든다. 반면 어떤 생성자라도 정의했다면 컴파일러는 디폴트 생성자를 만들지 않는다.

디폴트 생성자와 디폴트 복제 생성자는 명시적으로 디폴트로 만들거나 삭제하는가에 따라 자동 생성 여부가 결정된다.

## 2. 객체 소멸
객체가 제거되는 경우 객체의 소멸자를 호출한 다음 할당받은 메모리를 반환한다. 객체를 정리하는 작업은 소멸자에서 구체적으로 지정할 수 있다.

소멸자를 선언하지 않으면 컴파일러가 만들어주는데, 이를 이용해 멤버를 따라 재귀적으로 소멸자를 호출하면서 객체를 삭제할 수 있다.

스택 객체는 스코프를 벗어날 때 자동으로 삭제된다. 스택 객체가 삭제되는 순서는 선언 및 생성 순서와 반대다.

스마트 포인터를 사용하지 않은 힙 객체는 자동으로 삭제되지 않는다. 객체 포인터에 대해 delete를 명시적으로 호출해서 그 객체의 소멸자를 호출하고 메모리를 해제해야 한다.

## 3. 객체에 대입하기
객체의 값을 다른 객체에 대입할 수 있다.

```cpp
SpreadsheetCell myCell(5), anotherCell;
anotherCell = myCell;
```

myCell이 anotherCell에 대입하였다.

> C++에서 복제(copy)는 객체를 초기화 할 때만 적용되는 표현이며, 이미 값이 할당된 객체를 덮어쓸 때는 대입(assign)이라고 표현한다. 복제 기능은 복제 생성자에서 제공하며, 일종의 생성자이기 때문에 객체를 생성하는 데만 사용할 수 있고, 생성된 객체에 다른 값을 대입하는 데는 쓸 수 없다.

C++은 클래스마다 대입을 수행하는 대입 연산자를 제공한다. 클래스에 있는 `=` 연산자를 오버로딩한 것이기 때문에 이름이 `operator=`이다. anotherCell의 대입 연산자는 myCell이란 인수를 전달해서 호출한 것이다.

보통의 경우와 마찬가지로 대입 연산자를 직접 정의하지 않아도 된다. C++에서 객체끼리 서로 대입할 수 있도록 자동으로 만들어주기 때문이다.

### 3.1 대입 연산자 선언 방법

대입 연산자는 복제 생성자처럼 원본 객체에 대한 const 레퍼런스를 받을 때가 많다.

```cpp
class SpreadsheetCell
{
    public:
        SpreadsheetCell& operator=(const SpreadsheetCell& rhs);
}
```

### 3.2 대입 연산자 정의 방법

대입 연산자와 복제 생성자의 구현 방법의 몇 가지 중요한 차이점이 있다. 

복제 생성자는 초기화할 때 단 한 번만 호출된다. 그 시점에는 타깃 객체가 유효한 값을 갖고 있지 않다. 반면 대입 연산자는 객체에 이미 할당된 값을 덮어쓴다. 그래서 객체에서 메모리를 동적으로 할당하지 않는 한 이 차이점은 크게 드러나지 않는다.

C++는 객체에 자기 자신을 대입할 수 있다. 따라서 대입 연산자를 구현할 때 자기 자신을 대입하는 경우도 반드시 고려해야 한다.
```cpp
SpreadsheetCell cell(4);
cell = cell; // 자기 자신을 대입
```

위의 예시에서 사용된 SpreadsheetCell 클래스는 데이터 멤버가 double이란 기본 타입으로 지정됐기 때문에 자기 자신을 대입하는 경우를 고려하지 않아도 된다. 하지만 클래스에 동적으로 할당한 메모리나 다른 리소스가 있다면 자기 자신을 대입하는 작업을 처리하기 쉽지 않다. 이런 경우 대입 연산자를 시작하는 부분에서 자기 자신을 대입하는지 확인해서 그렇다면 곧바로 리턴하게 만든다.

```cpp
SpreadsheetCell& SpreadsheetCell::operator=(const SpreadsheetCell& rhs)
{
    if(this == &rhs) {
        return *this;
    } else {
        mValue = rhs.mValue;
        return *this;
    }
}
```

> 만약 자기 대입이 아닌 경우에는 모든 멤버에 대입연산을 수행해야 한다.

### 3.3 명시적으로 대폴트로 만들거나 삭제한 대입 연산자

컴파일러가 자동으로 생성한 대입 연산자를 다음과 같이 명시적으로 디폴트로 만들거나 삭제할 수 있다.

```cpp
SpreadsheetCell& operator=(const SpreadsheetCell& rhs) = default;

SpreadsheetCell& operator=(const SpreadsheetCell& rhs) = delete;
```

## 4. 복제와 대입 구분하기

때로는 객체를 복제 생성자로 초기화할지 아니면 대입 연산자로 대입할지 구분하기 힘들 때가 있다. 기본적으로 선언처럼 생겼다면 복제 생성자를 사용하고, 대입문처럼 생겼다면 대입 연산자로 처리한다.

복제 생성자를 이용하는 경우

```cpp
SpreadsheetCell myCell(5);
SpreadsheetCell anotherCell(myCell);
SpreadsheetCell aThirdCell = myCell;
```

위에서 aThirdCell도 복제 생성자를 이용하는 경우이다. 선언문이기 때문으로, 이 문장은 `SpreadsheetCell aThridCell(myCell);`의 또다른 표현에 불과하다.

대입 연산자를 생성하는 경우

```cpp
anotherCell = myCell; // anotherCell의 operator=을 호출
```