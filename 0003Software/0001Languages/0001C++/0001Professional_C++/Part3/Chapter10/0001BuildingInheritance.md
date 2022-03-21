---
sort: 1
---

# Building Classes With Inheritance

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 클래스 확장하기

클래스를 정의할 때 컴파일러에 기존 클래스를 상속, 파생, 확장 한다고 선언할 수 있다. 이는 새로 만들 클래스에 기존 클래스의 데이터 멤버와 메서드를 자동으로 가져올 수 있다. 여기서 원본 클래스를 부모 클래스라고 부르며, 이를 이용하여 만들어진 자식 클래스라고 부른다.

### 1.1 상속 예제

부모 클래스인 Base 클래스에 대한 선언이다.

```cpp
class Base
{
    public:
        void someMethod();
    protected:
        int mProtectedInt;
    private:
        int mPrivateInt();
};
```

다음의 Derived 클래스는 Base 클래스를 상속한다.

```cpp
class Derived : public Base
{
    public:
        void someOtherMethod();
};
```

Derived는 Base가 클래스가 가진 특성을 그대로 물려받은 완전한 형태의 클래스다.

### 1.2 클라이언트 입장에서 본 상속

다른 코드에서 볼 때 Derived 타입의 객체는 Base 타입의 객체이기도 하다. 따라서 Base에 있는 public 메서드나 데이터 멤버뿐만 아니라 Derived의 public 메서드와 데이터 멤버도 사용할 수 있다.

파생 클래스의 메서드를 호출할 때 같은 상속 계층에 있는 클래스 중에서 그 메서드가 속한 클래스를 구체적으로 지정하지 않아도 된다. 예를 들어 다음 코드는 Derived 객체에 있는 두 메서드를 호출하는데, 그 중 하나는 Base 클래스에 정의된 것이다.

```cpp
Derived myDerived;
myDerived.someMethod();
myDerived.someOtherMethod();
```

반대로 Base 타입 객체는 Derived 객체의 메서드나 데이터 멤버를 사용할 수 없다. Derived는 Base 타입이지만 Base는 Derived 타입이 아니기 때문이다.

어떤 객체를 포인터나 레퍼런스로 가리킬 때 그 객체를 선언한 클래스의 객체뿐만 아니라 그 클래스의 파생 클래스 객체도 가리킬 수 있다. 예를 들어 Base에 대한 포인터로 Derived 객체를 가리킬 수 있다. 타입이 맞지 않은 것처럼 보이지만 정상적으로 컴파일된다.

```cpp
Base* base = new Derived(); // Derived 객체를 생성해서 Base 포인터에 저장한다.
```

하지만 Base 포인터로 Derived 클래스의 메서드를 호출할 수는 없다.

```cpp
base->someOtherMethod(); // 컴파일 에러
```

### 2.2 파생 클래스 입장에서 본 상속

파생 클래스는 베이스 클래스에 선언된 public 및 protected 메서드나 데이터 멤버를 사용할 수 있다. 예를 들어 Derived의 someOtherMethod()를 구현하는 코드에서 Base에 선언된 mProtectedInt라는 데이터 멤버를 사용할 수 있다.

```cpp
void Derived::someOtherMethod()
{
    cout << "I can access base class data member mProtectedInt." << endl;
    cout << "Its value is " << mProtectedInt << endl;
}
```

단, 파생 클래스는 베이스 클래스의 private 데이터 멤버에 접근할 수 없다.

### 2.3 상속 방지

C++에서 클래스를 정의할 때 final 키워드를 붙이면 다른 클래스가 이 클래스를 상속할 수 없다.

```cpp
class Base final
{
    // ...
};

// 컴파일 에러 발생
class Derived : public Base
{
    // ...
};
```

## 2. 메서드 오버라이딩

클래스를 상속하는 주된 이유는 기능을 추가하거나 바꾸기 위해서이다. 앞서 Derived 클래스는 Base 클래스에 someOtherMethod()라는 메서드를 추가하는 방식으로 새로운 기능을 정의했다. 또 다른 메서드인 someMethod()는 Base 클래스에 정의된 그대로 동작한다.

베이스 클래스에 정의된 메서드의 동작을 변경은 메서드 오버라이딩을 이용한다.

### 2.1 메서드 오버라이딩과 virtual 속성

베이스 클래스에 virtual 키워드로 선언된 메서드만 파생 클래스에서 오버라이드할 수 있다.

```cpp
class Base
{
    public:
        virtual void someMethod();
    protected:
        int mProtectedInt;
    private:
        int mPrivateInt;
};
```

파생 클래스도 마찬가지로 모든 메서드를 virtual로 선언한다.

```cpp
class Derived : public Base
{
    public:
        virtual void someOtherMethod();
};
```

### 2.2 메서드 오버라이딩 문법

파생 클래스에서 베이스 클래스의 메서드를 오버라이드하려면 그 메서드를 베이스 클래스에 나온 것과 똑같이 선언하고 맨 뒤에 override 키워드를 붙인다. 그러고 나서 메서드 본문에 파생 클래스에서 구현하려는 방식으로 코드를 작성한다.

예를 들어 Base 클래스에 선언된 someMethod()란 메서드가 Base.cpp 에 정의되어 있다고 하자. 참고로 메서드를 구현할 때는 virtual 키워드를 생략한다.

```cpp
void Base::someMethod()
{
    cout << "This is Base's version of someMethod()." << endl;
}
```

Derived 클래스에서 someMethod()를 새로 정의하려면 Derived 클래스를 정의하는 코드에서 이 메서드의 선언문을 다음과 같이 고쳐야 하낟.

```cpp
class Derived : public Base
{
    public:
        virtual void someMethod() ovveride; // Base의 someMethod() 오버라이딩
        virtual void someOtherMethod();
};

// 구현 코드
void Derived::someMethod()
{
    cout << "This is Derived's version of someMethod()." << endl;
}
```

override 키워드를 반드시 적지 않아도 되지만 가능하면 적는 것이 좋다.

### 2.3 클라이언트 관점에서 본 오버라이드한 메서드

Base나 Derived 객체에 대해 someMethod()를 호출할 수 있으며, 객체가 속한 클래스에 따라 동작이 달라진다.

```cpp
Base myBase;
myBase.someMethod(); // Base에 정의된 someMethod() 호출

Derived myDerived;
myDerived.someMethod(); // Derived 버전의 someMethod() 호출
```

포인터나 레퍼런스는 해당 클래스뿐만 아니라 파생 클래스 객체까지 가리킬 수 있으며, 객체 자신이 속한 클래스에 선언된 메서드를 호출한다. 즉 virtual로 선언됐다면 파생 클래스에서는 오버라이드한 메서드를 호출할 것이다.

```cpp
Derived myDerived;
Base& ref = myDerived;
ref.someMethod(); // Derived 버전의 someMethod() 호출
```

하지만 베이스 클래스에 정의되지 않은 파생 클래스의 데이터 멤버나 메서드는 접근할 수 없다.

```cpp
Derived myDerived;
Base& ref = myDerived;
myDerived.someOtherMethod(); // 정상 작동
ref.someOtherMethod(); // 에러
```

파생 클래스를 베이스 클래스로 캐스팅하거나 변수에 대입할 수 있으나, 그 순간 파생 클래스의 정보가 사라지낟.

```cpp
Derived myDerived;
Base assignedObject = myDerived; // Base 변수에 Derived 객체 대입
assignedObject.someMethod(); // Base 버전의 someMethod() 호출
```

> 슬라이싱 : 파생 클래스의 데이터 멤버나 오버라이드된 메서드가 삭제되는 것

#### 2.4 override 키워드

override 키워드를 적지 않는 경우 잘못된 메서드를 호출할 수 있다.

```cpp
class Base
{
    public:
        virtual void someMethod(double d);
};

class Derived : public Base
{
    public:
        virtual void someMethod(double d); // override 생략
};

// someMethod 호출 코드
Derived myDerived;
Base& ref = myDerived;
ref.someMethod(1.1); // Derived 버전의 someMethod() 호출
```

만약 Derived 클래스의 someMethod의 매개변수 타입을 int로 지정한 경우

```cpp
class Base
{
    public:
        virtual void someMethod(double d);
};

class Derived : public Base
{
    public:
        virtual void someMethod(int d); // override 생략
};

// someMethod 호출 코드
Derived myDerived;
Base& ref = myDerived;
ref.someMethod(1.1); // Base 버전의 someMethod() 호출
```

Derived 클래스에서 override 키워드를 지정을 하면 Base 클래스의 메서드를 업데이트 했을 때 Derived 클래스가 업데이트 되지 않았을 때 컴파일 에러를 발생시킨다. 이는 잘못된 메서드 호출을 막을 수 있다.

따라서 베이스 클래스의 메서드를 오버라이드 할 때는 항상 override 키워드를 붙여야 한다.

### 2.5 virtual 메서드

virtual로 선언하지 않은 메서드를 오버라이드하면 몇 가지 미묘한 문제가 발생한다. 따라서 오버라이드 할 때는 항상 virtual로 선언하는 것이 좋다.

##### 오버라이드 하지 않고 숨기기

```cpp
class Base
{
    public:
        void go() { cout << "go() called on Base" << endl; }
};

class Derived : public Base
{
    public:
        void go() { cout << "go() called on Derived" << endl; }
};

// go() 메서드 호출
Derived myDerived;
myDerived.go(); // go() called on Derived 출력

Base& ref = myDerived;
ref.go(); // go() called on Base 출력
```

Derived 객체의 go() 메서드를 호출하면 Derived 클래스에 선언된 메서드를 호출하지만, 이는 실제로 오버라이드 된 것이 아니라 Derived 클래스에 go()란 이름을 갖는 메서드가 새로 생성된 것이다.

ref 변수는 Base 타입 레퍼런스인데 Base 클래스 안에서 virtual 키워드를 지정하지 않았기 때문에 이 메서드가 파생 클래스에 있는지 찾아보지 않는다. 따라서 Base의 go() 메서드가 호출된다.

##### virtual 소멸자의 필요성

소멸자를 virtual로 선언하지 않으면 객체가 소멸할 때 메모리가 해제되지 않을 수 있다. 클래스를 final로 선언할 때를 제외한 나머지 경우는 항상 소멸자를 virtual로 선언하는 것이 좋다.

예를 들어 파생 클래스의 생성자에서 동적으로 할당된 메모리를 사용하다가 소멸자에서 삭제하도록 작성했을 때 소멸자가 호출되지 않으면 메모리가 해제되지 않는다. 마찬가지로 std::unique_ptr처럼 파생 클래스에 자동으로 삭제되는 멤버가 있을 때 그 클래스의 인스턴스가 삭제될 때 소멸자가 호출되지 않으면 이런 멤버가 삭제되지 않고 남게 된다.

```cpp
class Base
{
    public:
        Base() {}
        ~Base() {}
};

class Derived : public Base
{
    public:
        Derived()
        {
            mString = new char[30];
            cout << "mString allocated" << endl;
        }

        ~Derived()
        {
            delete [] mString;
            cout << "mString deallocated" << endl;
        }
    private:
        char* mString;
};

int main()
{
    Base* ptr = new Derived(); // mString이 여기서 할당
    delete ptr; // ~Base는 호출되지만 ~Derived는 호출되지 않음

    return 0;
}
```

위 예제는 소멸자를 virtual로 선언하지 않았을 때 소멸자가 제대로 호출되지 않음을 보여준다. 대부분의 컴파일러는 파생 클래스의 소멸자가 아닌 베이스 클래스의 소멸자를 호출하도록 처리한다. 따라서 소멸자에서 virtual로 지정하고 다음과 같이 default로 지정한다.

```cpp
class Base
{
    public:
        virtual ~Base() = default;
}
```

> 특별한 이유가 없거나 클래스를 final로 선언하지 않았다면 소멸자를 포함한 모든 메서드를 virtual로 선언한다. 단, 생성자는 virtual로 선언할 수 없고 그럴 필요도 없다. 객체를 생성할 때 항상 정확한 클래스를 지정하기 때문이다.

### 2.6 오버라이딩 방지하기

메서드를 final로 지정하면 파생 클래스에서 오버라이드할 수 없다.

```cpp
class Base
{
    public:
        virtual ~Base() = default;
        virtual void someMethod() final;
};

class Derived : public Base
{
    public:
        virtual void someMethod() override; // 컴파일 에러
};
```