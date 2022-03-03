---
sort: 1
---

# friend

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

C++는 클래스 안에서 다른 클래스나 다른 클래스의 멤버 함수 또는 비멤버 함수를 `friend`로 선언하는 기능을 제공한다.

`friend`로 지정한 대상은 이 클래스의 protected나 private 데이터 멤버와 메서드에 접근할 수 있다.

다음 예제는 Foo 클래스에서 Bar 클래스를 프랜드로 지정한 예제이다.

```cpp
class Foo
{
    friend class Bar;
    // ...
};
```

Bar에 있는 모든 메서드는 Foo의 private나 protected 데이터 멤버 및 메서드에 접근할 수 있다.

다음과 같이 Bar에 있는 메서드 중 특정한 메서드만 프랜드로 지정할 수 있다.

```cpp
class Foo
{
    friend void Bar::processFoo(const Foo& foo);
    // ...
};
```

일반 함수도 클래스의 프랜드가 될 수 있다.

```cpp
class Foo
{
    friend void dumpFoo(const Foo& foo);
    // ...
};

void dumpFoo(const Foo& foo)
{
    // ...
}
```

dumpFoo() 함수는 Foo 클래스의 외부에 정의되어 있지만, Foo 클래스의 private와 protected 데이터 멤버에 직접 접근할 수 있다. Foo 클래스에 friend로 선언된 함수는 함수 프로토타입의 역할을 하므로, 다른 곳에 따로 선언하지 않아도 된다.

> 클래스나 메서드를 프랜드로 지정하는 기능을 너무 많이 사용하면 클래스의 내부가 외부 클래스나 함수에 드러나서 캡슐화 원칙이 깨진다. 따라서 꼭 필요할 때만 사용한다.