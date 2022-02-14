---
sort: 4
---

# Smart Pointers

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 스마트 포인터
스마트 포인터는 동적으로 할당한 메모리를 관리하기 쉽게 만들어준다. 이는 메모리 누수를 방지하는데 도움된다.

기본적으로 스마트 포인터는 메모리뿐만 아니라 동적으로 할당한 모든 리소스를 가리킨다. 

스마트 포인터가 스코프를 벗어나거나 리셋되면 거기에 할당된 리소스가 자동으로 해제된다. 함수 스코프 안에서 동적으로 할당된 리소스를 관리하는 데 사용할 수도 있고, 클래스의 데이터 멤버로 사용할 수도 있다. 동적으로 할당된 리소스의 소유권을 함수의 인수로 넘겨줄 때도 스마트 포인터를 활용한다.

##### 스마트 포인터의 활용
1. 템플릿을 이용하면 모든 포인터 타입에 대해 타입에 안전한 스마트 포인터 클래스를 작성할 수 있다.
2. 연산자 오버로딩을 이용하여 스마트 포인터 객체에 대한 인터페이스를 제공해서 스마트 포인터 객체를 일반 포인터처럼 활용할 수 있다.

##### 스마트 포인터의 종류 
1. unique_ptr : 리소스에 대한 고유 소유권을 가지며, 스코프를 벗어나거나 리셋되면 참조하던 리소스를 해제한다.
2. shared_ptr : 리소스의 소유자를 추적하도록 레퍼런스 카운팅을 구현한 스마트 포인터로, 스마트 포인터를 복사해서 리소스를 가리키는 인스턴스를 새로 생성할 수 있으며, 이때 레퍼런스 카운트가 증가한다. 또한 스코프를 벗어나거나 리셋되면 레퍼런스 카운트가 감소한다. 레퍼런스 카운트가 0이 되면 자동으로 해제된다.

> unique_ptr과 shared_ptr과 같은 스마트 포인터를 사용하려면 `<memory>` 헤더 파일을 인클루드해야 한다.

## 2. unique_ptr
동적으로 할당한 리소스는 항상 unique_ptr와 인스턴스에 저장하는 것이 바람직하다.

### 2.1 unique_ptr 생성 방법

```cpp
void notleaky()
{
    auto mySimpleSmartPtr = make_unique<Simple>();
    mySimplePtr->go();
}
```
위 예제는 `make_unique()`와 `auto` 키워드를 이용하여 unique_ptr을 생성했다. `make_unique()`에 `Simple`이라는 포인터 타입을 지정했다. 만약 Simple 생성자에서 매개변수를 받는다면 `make_unique()`에 인자로 지정하면 된다. 예를 들어 `Simple(int, int)`라면 `make_unique<Simple>(1,2)`와 같이 생성자 인수를 전달할 수 있다.

> make_unique()는 C++ 14부터 제공하는데, 만약 make_unique()를 지원하지 않는 컴파일러를 사용한다면 `unique_ptr<Simple> mySimpleSmartPtr(new Simple());`과 같이 생성해야 한다.

### 2.2 unique_ptr 사용 방법
스마트 포인터는 일반 포인터와 같이 `*`나 `->`로 역참조한다. 예를 들어 앞에서 본 예제에서 go()메서드를 호출할 때 `->` 연산자를 사용했다.

```cpp
mySimpleSmartPtr->go();
```

또한 일반 포인터처럼 작성해도 된다.

```cpp
(*mySimpleSmartPtr).go();
```

##### get()
`get()` 메서드를 이용하면 내부 포인터에 직접 접근할 수 있다. 일반 포인터만 전달할 수 있는 함수에 스마트 포인터를 전달할 때 유용하다. 예를 들어 다음과 같은 함수가 있다고 하자.

```cpp
void processData(Simple* simple) { /* 스마트 포인터를 사용하는 코드 */ }
```

그러면 이 함수를 다음과 같이 호출할 수 있다.

```cpp
auto mySimpleSmartPtr = make_unique<Simple>();
processData(mySimpleSmartPtr.get());
```

##### reset()
`reset()`을 이용하면 unique_ptr의 내부 포인터를 해제하고, 필요하다면 이를 다른 포인터로 변경할 수 있다.

```cpp
mySimpleSmartPtr.reset();               // 리소스 해제 후 nullptr로 초기화
mySimpleSmartPtr.reset(new Simple());   // 리소스 해제 후 새로운 Simple 인스턴스로 설정
```

##### release()
`release()`를 이용하면 unique_ptr와 내부 포인터의 관계를 끊을 수 있다. `release()` 메서드는 리소스에 대한 내부 포인터를 리턴한 뒤 스마트 포인터를 nullptr로 설정한다. 그러면 스마트 포인터는 그 리소스에 대한 소유권을 잃으며, 리소스를 다 쓴 뒤 반드시 직접 해제해야 한다.

```cpp
Simple* simple = mySimpleSmartPtr.release();    // 소유권을 해제한다.
// simple 포인터를 사용하는 코드

delete simple;
simple = nullptr;
```

##### std::move()
unique_ptr는 단독 소유권을 표현하기 때문에 복사할 수 없다. `std::move()` 유틸리티를 사용하면 하나의 unique_ptr를 다른 곳으로 이동할 수 있는데, 복사라기 보다는 이동의 개념이다. 즉 명시적으로 소유권을 이전하는 용도로 사용된다.

```cpp
class Foo
{
    public:
        Foo(unique_ptr<int> data) : mData(move(data)) { }
    private:
        unique_ptr<int> mData;
};

auto myIntSmartPtr = make_unique<int>(42);
Foo f(move(myIntSmartPtr));
```

### 2.3 unique_ptr와 C 스타일 배열
unique_ptr는 기존 C 스타일의 동적 할당 배열을 저장하는 데 적합하다. 예를 들어 정수 10개를 가진 C 스타일의 동적 할당 배열을 다음과 같이 표현할 수 있다.

```cpp
auto myVariableSizedArray = make_unique<int[]>(10);
```

> 이렇게 unique_ptr로 C 스타일의 동적 할당 배열을 저장할 수는 있지만, 이보다는 std::array나 std::vector와 같은 표준 라이브러리 컨테이너를 사용하는 것이 바랍직하다.

## 3. shared_ptr
share_ptr의 사용법은 unique_ptr과 비슷하다. shared_ptr는 `make_shared()`로 생성한다.

```cpp
auto mySimpleSmartPtr = make_shared<Simple>();
```

C++17 부터 shared_ptr도 unique_ptr와 마찬가지로 기존 C 스타일 동적 할당 배열에 대한 포인터를 저장할 수 있다. 하지만 역시 표준 라이브러리 컨테이너를 사용하는 것이 바람직하다.

shared_ptr도 `get()`과 `reset()` 메서드를 제공한다. 다른 점은 `reset()`을 호출하면 레퍼런스 카운팅 메커니즘에 따라 마지막 shared_ptr가 제거되거나 리셋될 때 리소스가 해제된다.

참고로 shared_ptr는 `release()`를 지원하지 않는다. 현재 동일한 리소스를 공유하는 shared_ptr의 개수는 `use_count()`로 알아낼 수 있다.

### 3.1 shared_ptr 캐스팅하기
shared_ptr를 캐스팅하는 함수로 `const_pointer_cast()`, `dynamic_pointer_cast()`, `static_pointer_cast()`가 제공된다. C++17 부터는 `reinterpret_pointer_cast()도 추가됐다.

### 3.2 레퍼런스 카운팅이 필요한 이유
레퍼런스 카운팅은 어떤 클래스의 인스턴스 수나 현재 사용 중인 특정한 객체를 추적하는 메커니즘이다. 레퍼런스 카운팅을 지원하는 스마트 포인터는 실제 포인터를 참조하는 스마트 포인터 수를 추적한다. 그래서 스마트 포인터가 중복 삭제되는 것을 방지한다.

##### 중복 삭제 문제

다음은 shared_ptr 두 개를 만들고 각각 하나의 Simple 객체를 가리키도록 작성하면 두 포인터가 제거될 때 서로 Simple 객체를 삭제하려 시도한다. 이는 컴파일러에 따라 프로그램이 죽어버릴 수 있다.

```cpp
void doubleDelete()
{
    Simple* mySimple = new Simple();
    shared_ptr<Simple> smartPtr1(mySimple);
    shared_ptr<Simple> smartPtr2(mySimple);
}
```

따라서 다음과 같이 shared_ptr 두 개로 가리키는 것이 아닌 복사본을 만들어 사용해야 한다.

```cpp
void noDoubleDelete()
{
    auto smartPtr1 = make_shared<Simple>();
    shared_ptr<Simple> smartPrt2(martPtr1);
}
```

### 3.3 앨리어싱
앨리어싱(aliasing)이란 어떤 포인터의 복사본을 여러 객체나 코드에서 갖고 있는 경우를 말한다.

shared_ptr는 앨리어싱을 지원한다. 그래서 한 포인터를 다른 shared_ptr와 공유하면서 다른 객체를 가리킬 수 있다. 예를 들어 shared_ptr가 객체를 가리키는 동시에 그 객체의 멤버도 가리키게 할 수 있다.

```cpp
class Foo
{
    public:
        Foo(int value) : mData(value) {}
        int mData;
};

auto foo = make_shared<Foo>(42);
auto aliasing = shared_ptr<int>(foo, &foo->mData);
```

여기서 두 shared_ptr가 모두 삭제될 때만 Foo 객체가 삭제된다.

소유한 포인터는 레퍼런스 카운팅에 사용되는 반면, 저장된 포인터는 포인터를 역참조하거나 그 포인터에 대해 `get()`을 호출할 때 리턴된다. 저장된 포인터는 비교 연산을 비롯한 대부분의 연산에 적용할 수 있다.

이렇게 하지 않고 `owner_before()` 메서드나 `std::owner_less` 클래스를 사용하여 소유한 포인터에 대해 비교 연산을 수행해도 된다. 이러한 기능은 shared_ptr를 `std::set`에 저장할 때와 같이 특정한 상황에 유용하다.

## 4. weak_ptr
weak_ptr는 shared_ptr가 가리키는 리소스의 레퍼런스를 관리하는 데 사용된다. weak_ptr는 리소스를 직접 소유하지 않기 때문에 shared_ptr가 해당 리소스를 해제하는 데 아무런 영향을 미치지 않는다.

weak_ptr는 삭제될 때 가리키던 리소스를 삭제하지 않고, shared_ptr가 그 리소스를 해제했는지 알아낼 수 있다.

weak_ptr의 생성자는 shared_ptr나 다른 weak_ptr를 인수로 받는다. weak_ptr에 저장된 포인터에 접근하려면 shared_ptr로 변환해야 한다. 변환 방법은 다음 두 가지가 있다.

* weak_ptr 인스턴스의 `lock()` 메서드를 이용하여 shared_ptr를 리턴받는다. 이때 shared_ptr에 연결된 weak_ptr가 해제되면 shared_ptr의 값은 nullptr가 된다.
* shared_ptr의 생성자에 weak_ptr를 인수로 전달해서 shared_ptr를 새로 생성한다. 이때 shared_ptr에 연결된 weak_ptr가 해제되면 `std::bad_weak_ptr` 익셉션이 발생한다.

```cpp
void useResource(weak_ptr<Simple>& weakSimple)
{
    auto resource = weakSimple.lock();
    if(resource) {
        cout << "Resource still alive." << endl;
    } else {
        cout << "Resource has been freed!" << endl;
    }
}

int main()
{
    auto sharedSimple = make_shared<Simple>();
    weak_ptr<Simple> weakSimple(sharedSimple);

    // weak_ptr를 사용한다.
    useResource(weakSimple);

    // shared_ptr 리셋
    // Simple 리소스에 대한 shared_ptr는 하나뿐이므로,
    // weak_ptr가 살아 있더라도 리소스가 해제된다.
    sharedSimple.reset();

    // weak_ptr를 한 번 더 사용한다.
    useResource(weakSimple);

    return 0;
}
```

이 코드를 실행한 결과는 다음과 같다.
```bash
Simple constructor called!
Resource still alive.
Simple destructor called!
Resource has been freed!
```

## 5. 이동 의미론
표준 스마트 포인터인 shared_ptr와 unique_ptr, weak_ptr는 모두 성능 향상을 위해 이동 의미론을 지원한다. 이동 의미론을 이용하면 함수에서 스마트 포인터를 리턴하는 과정을 효율적으로 처리할 수 있다. 예를 들어 다음과 같이 `create()` 함수를 작성해서 `main()` 함수에서 호출할 수 있다.

```cpp
unique_ptr<Simple> create()
{
    auto ptr = make_unique<Simple>();
    // ptr를 사용하는 코드를 작성

    return ptr;
}

int main()
{
    unique_ptr<Simple> mySmartPtr1 = create();
    auto mySmartPtr2 = create();
    return 0;
}
```

## 6. enable_shared_from_this
믹스인 클래스인 `std::enable_shared_from_this`를 이용하면 객체의 메서드에서 shared_ptr나 weak_ptr를 안전하게 리턴할 수 있다.

enable_shared_from_this 믹스인 클래스는 다음 두 개의 메서드를 클래스에 제공한다.
* `shared_from_this()`: 객체의 소유권을 공유하는 shared_ptr를 리턴한다.
* `weak_from_this()`: 객체의 소유권을 추적하는 weak_ptr를 리턴한다.

간단한 사용 방법을 소개하면 다음과 같다.

```cpp
class Foo : public enable_shared_from_this<Foo>
{
    public:
        shared_ptr<Foo> getPointer() {
            return shared_from_this();
        }
};

int main()
{
    auto ptr1 = make_shared<Foo>();
    auto ptr2 = ptr1->getPointer();
}
```

여기서 객체의 포인터가 shared_ptr에 이미 저장된 상태에서만 객체에 `shared_from_this()`를 사용할 수 있다는 점에 주의한다.

`getPointer()` 메서드를 다음과 같이 구현하면 안 된다. Foo 클래스를 이렇게 구현한 상태에서 앞에 나온 `main()` 처럼 작성하면 중복 삭제가 발생한다. 두 개의 shared_ptr가 동일한 객체를 가리키고 있어서 스코프를 벗어나면 서로 이 객체를 삭제하려 하기 때문이다.

```cpp
class Foo
{
    public:
        shared_ptr<Foo> getPointer() {
            return shared_ptr<Foo>(this);
        }
};
```

## 7. auto_ptr
C++11 이전에는 표준 라이버르리에서 스마트 포인터를 간단히 구현한 auto_ptr를 제공했는데, 몇 가지 심각한 단점이 존재하여 C++17 부터는 완전히 삭제되었다.