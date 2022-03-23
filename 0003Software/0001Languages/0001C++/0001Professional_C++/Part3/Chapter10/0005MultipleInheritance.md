---
sort: 5
---

# Multiple Inheritance

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 여러 클래스 상속하기

다중 상속의 정의는 클래스 이름 옆에 상속할 베이스 클래스를 나열하면 된다.

```cpp
class Baz : public Foo, public Bar
{
    // 클래스 선언 코드
};
```

Baz 클래스는 다음과 같은 속성을 갖게 된다.

* Baz 객체는 Foo와 Bar에 있는 데이터 멤버와 public 메서드를 갖는다.
* Baz 객체는 Foo와 Bar에 있는 protected 데이터 멤버와 메서드에 접근할 수 있다.
* Baz 객체를 Foo나 Bar로 업캐스팅할 수 있다.
* Baz 객체를 생성하면 Foo와 Bar의 디폴트 생성자가 호출된다. 이때 호출 순서는 첫 줄에 정의한 순서를 따른다.
* Baz 객체를 삭제하면 Foo와 Bar의 소멸자가 자동으로 호출된다. 이때 호출 순서는 클래스 정의에 나열한 클래스 순서와 반대다.

다음 코드에 나온 DogBird 클래스는 Dog 클래스와 Bird 클래스를 동시에 상속한다.

```cpp
class Dog
{
    public:
        virtual void bark() { cout << "Woof!" << endl; }
};

class Bird
{
    public:
        virtual void chirp() { cout << "Chirp!" << endl; }
};

class DogBird : public Dog, public Bird
{
};
```

이 예제에서 DogBird 객체는 Dog와 Bird에 있는 public 메서드를 모두 제공한다.

```cpp
DogBird myConfusedAnimal;
myConfusedAnimal.bark();
myConfusedAnimal.chirp();
```

## 2. 이름 충돌과 모호한 베이스 클래스

### 2.1 모호한 이름

Dog 클래스와 Bird 클래스 둘 다 eat() 메서드를 가지고 있다고 하자. Dog와 Bird는 서로 관련이 없기 때문에 어느 한쪽이 다른 쪽의 메서드를 오버라이딩할 수 없다. 따라서 DogBird라는 파생 클래스에서도 그대로 유지된다.

DogBird의 eat() 메서드를 호출하면 컴파일러는 eat() 메서드를 호출하는 부분이 모호하다는 에러를 발생한다. 어느 버전의 eat()을 호출해야 하는지 판단할 수 없기 때문이다.

```cpp
class Dog
{
    public:
        virtual void bark() { cout << "Woof!" << endl; }
        virtual void eat() { cout << "The dog ate." << endl; }
};

class Bird
{
    public:
        virtual void chirp() { cout << "Chirp!" << endl; }
        virtual void eat() { cout << "The bird ate." << endl; }
};

class DogBird : public Dog, public Bird
{
};

int main()
{
    DogBird myConfusedAnimal;
    myConfusedAnimal.eat(); // 에러 발생
    return 0;
}
```

이러한 상황이 발생하지 않게 하려면 dynamic_cast()로 객체를 명시적으로 업캐스팅해서 원하지 않는 버전을 컴파일러가 볼 수 없게 가리거나 스코프 지정 연산자로 원하는 버전을 구체적으로 지정한다.

```cpp
dynamic_cast<Dog&>(myConfusedAnimal).eat();
myConfusedAnimal.Dog::eat();
```

파생 클래스 사이에 이름이 같은 메서드가 있을 때도 스코프 지정 연산자로 원하는 메서드를 명확히 지정해야 한다.

```cpp
class DogBird : public Dog, public Bird
{
    public:
        void eat() override;
};

void DogBird::eat()
{
    Dog::eat();
}
```

### 2.2 모호한 베이스 클래스

같은 클래스를 두 번 상속할 때 모호한 상황이 발생한다. 예를 들어 Bird 클래스가 Dog을 상속하면 DogBird 코드에서 컴파일 에러가 발생한다. 베이스 클래스가 모호하기 때문이다.

```cpp
class Dog {};
class Bird : public Dog {};
class DogBird : public Bird, publiv Dog {}; // 에러 발생
```

베이스 클래스가 모호한 경우는 상속 관계가 이상하거나 클래스 계층이 정리되지 않았을 때 주로 발생한다.

데이터 멤버에 대해서도 모호함이 발생할 수 있다. Dog와 Bird의 데이터 멤버 중 이름이 같은 것이 있을 때 이 멤버에 접근하면 에러가 발생한다. 가장 흔한 사례는 부모가 겹칠 때이다.

다음 예는 Bird와 Dog가 모두 Animal 클래스를 상속한 경우이다. Animal 클래스에 sleep() 이란 public 메서드가 있을 때 DogBird 객체로 이 메서드를 호출할 수 없다. 컴파일러는 Dog와 Bird 중 어디에 있는 sleep()을 호출할지 판단할 수 없기 때문이다.

이러한 경우에는 최상단의 클래스를 순수 가상 메서드로만 구성된 추상 클래스로 만들면 된다.

다음 코드는 eat()를 순수 가상 메서드로 선언해서 Animal을 추상 베이스 클래스로 만든 예제이다. 이 클래스를 상속하는 모든 파생 클래스는 반드시 eat() 메서드를 구현해야 하며, DogBird 클래스도 어느 부모의 eat() 메서드를 사용할지 명확히 밝혀야 한다.

> Dog와 Bird 사이에서 모호함이 발생하는 근본 원인은 같은 클래스를 상속하기 때문이 아니라 메서드 이름이 같기 때문이다.

```cpp
class Animal
{
    public:
        virtual void eat() = 0;
};

class Dog : public Animal
{
    public:
        virtual void bark() { cout << "Woof!" << endl; }
        virtual void eat() { cout << "The dog ate." << endl; }
};

class Bird : public Animal
{
    public:
        virtual void chirp() { cout << "Chirp!" << endl; }
        virtual void eat() { cout << "The bird ate." << endl; }
};

class DogBird : public Dog, public Bird
{
    public:
        using Dog::eat;
};
```

### 2.3 다중 상속 활용법

다중 상속을 활용하는 가장 간단한 예는 is-a 관계를 맺는 대상이 하나 이상인 객체에 대한 클래스를 정의하기 위해서다.

다중 상속의 가장 적합하면서 간단한 예는 믹스인 클래스를 구현할 때다.

컴포넌트 기반으로 클래스를 모델링할 때도 다중 상속을 사용한다. 