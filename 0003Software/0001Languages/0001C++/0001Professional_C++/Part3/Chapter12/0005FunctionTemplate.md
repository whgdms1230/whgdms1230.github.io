---
sort: 5
---

# Function Templates

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 함수 템플릿

일반 함수도 템플릿화할 수 있다. 예를 들어 배열에서 값을 하나 찾아서 그 인덱스를 리턴하는 제네릭 함수를 작성해보자.

```cpp
static const size_t NOT_FOUND = static_cast<size_t>(-1);

template<typename T>
size_t Find(const T& value, const T* arr, size_t size)
{
    for(size_t i = 0; i < size; i++){
        if(arr[i] == vale){
            return i; // 값을 찾으면 인덱스를 리턴한다.
        }
    }

    return NOT_FOUND; // 값을 찾지 못하면 NOT_FOUND를 리턴한다.
}
```

> 위에서 NOT_FOUND와 같이 특수한 size_t 값을 리턴하지 않고 std::optional<size_t> 타입의 값을 리턴하도록 작성해도 된다.

이렇게 작성한 `Find()` 함수 템플릿은 모든 타입의 배열에 적용할 수 있다.

이 함수는 두 가지 방식으로 호출할 수 있다. 하나는 꺾쇠괄호 안에 타입 매개변수를 명시적으로 지정하는 것이고, 다른 하나는 주어진 인수를 바탕으로 컴파일러가 타입 매개변수를 알아서 추론하도록 타입을 생략하는 것이다.

```cpp
int myInt = 3, intArry[] = {1, 2, 3, 4};
const size_t sizeIntArray = std::size(intArray);

size_t res;
res = Find(myInt, intArray, sizeIntArray); // 타입 추론을 통해 Find<int>를 호출한다.
res = Find<int>(myInt, intArray, sizeIntArray); // Find<int>를 명시적으로 호출한다.

if(res != NOT_FOUND)
    cout << res << endl;
else
    cout << "Not found" << endl;

// double이나 SpreadsheetCell 등과 같은 타입도 같은 방식으로 적용할 수 있다.
```

간혹 컴파일러가 배열의 크기를 정확히 아는 경우가 있다. 대표적인 예로 스택 기반의 배열을 사용할 때다. 이러한 배열에 대해 `Find()`를 호출할 때 배열의 크기에 대한 인수를 생략할 수 있다면 편할 것이다.

이럴 때는 다음과 같이 함수 템플릿을 이용하면 된다. 이 코드는 `Finㅇ()`에 대한 호출을 단순히 이전 `Find()` 함수 템플릿으로 포워딩하기만 하면 된다. 또한 코드에 나온 것처럼 함수 템플릿도 클래스 템플릿처럼 비타입 매개변수를 받게 만들 수 있다.

```cpp
template <typename T, size_t N>
size_t Find(const T&, value, const T(&arr)[N])
{
    return Find(value, arr, N);
}
```

구현한 `Find()`를 사용하는 방법은 다음과 같다.

```cpp
int myInt = 3, intArray[] = {1, 2, 3, 4};
size_t res = Find(myInt, intArray);
```

클래스 템플릿의 메서드 정의와 마찬가지로 함수 템플릿을 사용하는 코드는 이 템플릿의 프로토 타입뿐만 아니라 정의 코드도 접근할 수 있어야 한다. 따라서 함수 템플릿을 여러 소스 파일에서 사용한다면 함수 템플릿을 정의하는 코드를 헤더 파일에 넣어두거나 명시적으로 인스턴스화하는 것이 좋다.

> 함수 템플릿의 템플릿 매개변수도 클래스 템플릿처럼 디폴트 값을 지정할 수 있다.

## 2. 함수 템플릿의 특수화

클래스 템플릿과 마찬가지로 함수 템플릿도 특수화할 수 있다. 예를 들어 `operator==` 대신 `strcmp()`로 값을 비교하도록 C 스타일 스트링인 `const char*`에 특화된 `Find()` 함수를 만들 수 있다.

```cpp
template<>
size_t Find<const char*>(const char* const& value, const char* const* arr, size_t size)
{
    for(size_t i = 0; i < size; i++){
        if(strcmp(arr[i], value) == 0){
            return i; // 원소를 찾으면 인덱스를 리턴한다.
        }
    }
    return NOT_FOUND; // 찾지 못하면 NOT_FOUND를 리턴한다.
}
```

이때 매개변수 타입을 인수로부터 추론할 수 있다면 함수 이름에서 `<const char*>`를 생략해도 된다.

```cpp
template<>
size_t Find(const char* const& value, const char* const* arr, size_t size);
```

그런데 오버로딩을 함께 적용하면 타입 추론 규칙이 복잡해질 수 있으므로 실수를 방지하기 위해서 타입을 명시적으로 지정하는 것이 좋다.

특수화한 `Find()` 함수도 첫 번째 매개변수로 `const char* const&` 대신 `const char*`만 받을 수 있지만, 특수화되지 않은 버전의 `Find()`와 인수를 서로 일치시키는 것이 타입을 정확히 유추하는 데 도움 된다.

이렇게 특수화한 버전의 `Find()`를 사용하는 방법은 다음과 같다.

```cpp
const char* word = "two";
const char* words[] = {"one", "two", "three", "four"};
const size_t sizeWords = std::size(words);
size_t res;

// const char*에 대해 특수화된 버전을 호출한다.
res = Find<const char*>(word, words, sizeWords);
// const char*에 대해 특수화된 버전을 호출한다.
res = Find(word, words, sizeWords);
```

## 3. 함수 템플릿 오버로딩

함수 템플릿도 일반 함수처럼 오버로딩할 수 있다. 예를 들어 `Find()`를 `const char*`에 대해 특수화되지 않고, `const char*` 스트링을 처리하는 일반 함수로 만들어도 된다.

```cpp
size_t Find(const char* const& value, const char* const* arr, size_t size)
{
    for(size_t i = 0; i < size i++){
        if(strcmp(arr[i], value) == 0){
            return i;
        }
    }
    return NOT_FOUND;
}
```

이렇게 해도 앞 절에서 구현한 템플릿 특수화 버전과 똑같이 실행되나, 이 함수가 호출될 때 규칙은 좀 다르다.

```cpp
const char* word = "two";
const char* words[] = {"one", "two", "three", "four"};
const size_t sizeWords = std::size(words);
size_t res;

// T=const char*인 템플릿 호출
res = Find<const char*>(word, words, sizeWords);
// 비 템플릿 Find() 호출
res = Find(word, words, sizeWords);
```

이처럼 `const char*` 타입이 명시적으로 지정될 때 뿐만 아니라 타입 추론으로 결정될 때도 함수가 제대로 호출되게 하려면 일반 함수로 오버로딩하지 말고 템플릿 특수화를 이용해야 한다.

> `const char*`에 대해 템플릿 특수화를 적용한 `Find()`와 `const char*`에 대해 일반 함수로 작성한 `Find()`를 동시에 사용할 수도 있다. 기본적으로 컴파일러는 항상 템플릿 버전보다 일반 함수 버전을 우선시한다. 하지만 템플릿 인스턴스화를 명시적으로 지정하면 컴파일러는 무조건 템플릿 버전을 선택한다.

## 4. 클래스 템플릿의 friend 함수 템플릿

함수 템플릿은 클래스 템플릿에서 연산자를 오버로딩할 때 유용하다.

예를 들어 Grid 클래스 템플릿에 덧엠 연산자를 오버로딩해서 두 그리드를 더하는 기능을 추가하고 싶을 수 있다. 덧셈의 결과로 나오는 Grid의 크기는 두 피연산자 중 작은 Grid의 크기에 맞춘다. 그리고 두 셀 모두 실제로 값이 들어 있을 때만 더한다.

그럼 이런 기능을 제공하는 `operator+`를 독립 함수 템플릿으로 만드는 경우를 생각해보자. 정의 코드는 다음과 같으며 Grid.h에 추가해야 한다.

```cpp
template<typename T>
Grid<T> operator+(const Grid<T>& lhs, const Grid<T>& rhs)
{
    size_t minWidth = std::min(lhs.getWidth(), rhs.getWidth());
    size_t minHeight = std::min(lhs.getHeight(), rhs.getHeight());

    Grid<T> result(minWidth, minHeight);
    for(size_t y = 0; y < minHeight; ++y){
        for(size_t x = 0; x < minWidth; ++x){
            const auto& leftElement = lhs.mCells[x][y];
            const auto& rightElement = rhs.mCells[x][y];
            if(leftElement.has_value() && rightElement.has_value())
                result.at(x, y) = leftElement.value() + rightElement.value();
        }
    }
    return result;
}
```

이 함수 템플릿은 모든 타입의 Grid에 적용할 수 있다. 단, 그리드에 지정할 원소의 타입이 덧셈 연산을 지원해야 한다.

이렇게 구현하면 Grid 클래스의 private 멤버인 mCells에 접근한다는 문제가 있다. 물론 public 메서드인 `at()`을 사용해도 되지만, 여기서는 함수 템플릿을 클래스 템플릿의 friend로 만드는 방법을 소개한다. 

이를 위해 덧셈 연산자를 Grid 클래스의 friend로 만든다. 그런데 Grid 클래스와 `operator+`가 모두 템플릿이다. 실제로 원하는 바는 `operator+`를 특정한 타입 T에 대해 인스턴스화한 것이 T 타입에 대한 Grid 템플릿 인스턴스의 friend가 되게 만드는 것이다.

```cpp
// Grid 템플릿에 대한 전방 선언
template <typename T> class Grid;

// 템플릿화한 operator+에 대한 프로토 타입
template <typename T>
Grid<T> operator+(const Grid<T>& lhs, const Grid<T>& rhs);

template <typename T>
class Grid
{
    public:
        // 코드 생략
        friend Grid<T> operator+ <T>(const Grid<T>& lhs, const Grid<T>& rhs);
        // 코드 생략
};
```

이 템플릿을 T 타입으로 인스턴스화한 것에 대해 `operator+`를 T 타입으로 인스턴스화한 것이 friend가 돼야 한다. 다시 말해 클래스 인스턴스와 함수 인스턴스 사이의 friend 관계가 1:1 대응되게 해야 한다. 이때 `operator+`에 명시적으로 `<T>`를 지정한 부분이 중요하다. 이렇게 하면 컴파일러는 `operator+`를 템플릿으로 취급한다.

## 5. 템플릿 매개변수 추론에 대한 보충 설명

컴파일러는 함수 템플릿에 전달된 인수를 보고 템플릿 매개변수의 타입을 추론한다. 추론할 수 없는 템플릿 매개변수는 반드시 명시적으로 지정해야 한다.

예를 들어 다음처럼 `add()` 함수 템플릿은 템플릿 매개변수를 세 개 받는다.

```cpp
template <typename RetType, typename T1, typename T2>
RetType add(const T1& t1, const T2& t2) { return t1 + t2; }
```

이렇게 작성한 함수 템플릿에 매개변수 세 개를 모두 지정하는 예는 다음과 같다.

```cpp
auto result = add<long long, int, int>(1, 2);
```

그런데 템플릿 매개변수인 T1과 T2는 이 함수의 매개변수이기 때문에 컴파일러는 T1과 T2의 타입을 추론한다. 그래서 `add()`를 호출할 때 리턴값에 대한 타입만 지정해도 된다.

```cpp
auto result = add<long long>(1, 2);
```

물론 추론할 매개변수가 매개변수 목록의 마지막에 있을 때만 이렇게 할 수 있다. 그렇지 않다면 해당 템플릿 매개변수의 타입을 명시해주어야 한다.

리턴 타입에 대한 템플릿 매개변수도 디폴트 값을 지정할 수 있다. 그러면 `add()`를 호출할 때 타입을 하나도 지정하지 않아도 된다.

```cpp
template<typename RetType = long long, typename T1, typename T2>
RetType add(const T1 & t1, const T2& t2) { return t1 + t2; }

auto result = add(1, 2);
```

## 6. 함수 템플릿의 리턴 타입

`add()` 함수 템플릿에서 리턴값의 타입도 컴파일러가 추론하게 할 수 있다. 하지만 리턴 타입은 템플릿 타입 매개변수에 따라 결정된다.

```cpp
template<typename T1, typename T2>
RetType add(const T1& t1, const T2& t2) { return t1 + t2; }
```

여기서 RetType은 반드시 t1 + t2 표현식의 타입으로 지정해야 한다. 그런데 T1과 T2를 모르기 때문에 이 표현식의 타입도 모른다.

auto를 이용하여 리턴 타입을 자동으로 추론하면 다음과 같이 구현할 수 있다.

```cpp
template<typename T1, typename T2>
auto add(const T1& t1, const T2& t2) { return t1 + t2; }
```

여기서 auto로 표현식의 타입을 추론하면 레퍼런스와 const 지정자가 사라진다. 반면 decltype은 이를 제거하지 않는다.

```cpp
template<typename T1, typename T2>
decltype(auto) add(const T1& t1, const T2, t2)
{
    return t1 + t2;
}
```