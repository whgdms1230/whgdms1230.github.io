---
sort: 2
---

# Exception Mechanics

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 익셉션을 위한 파일 입출력 함수 예제

익셉션은 파일 입출력 과정에서 발생하기 쉽다. 다음 코드는 파일을 열고, 그 파일에 담긴 정수 목록을 읽어서 std::vector에 담아 리턴하는 함수를 구현한 것이다.

```cpp
vector<int> readIntegerFile(string_view fileName)
{
    ifstream inputStream(fileName.data());
    // 파일에 담긴 정숫값을 하나씩 읽어서 벡터에 추가한다.
    vector<int> integers;
    int temp;
    while(inputStream >> temp){
        integers.push_back(temp);
    }
    return integers;
}
```

여기서 다음 코드는 파일 끝에 도달하거나 에러가 발생하기 전까지 ifstream에서 읽은 값을 저장한다.

```cpp
while(inputStream >> temp)
```

`>>` 연산을 수행할 때 에러가 발생하면 ifstream 객체에 에러 글래그가 설정된다. 그러면 `bool()` 변환 연산자가 false를 리턴하면서 while 루프가 종료된다.

앞서 정의한 readIntegerFile() 함수를 사용하는 방법은 다음과 같다.

```cpp
const string fileName = "IntegerFile.txt";
vector<int> myInts = readIntegerFile(fileName);
for(const auto& element : myInts){
    cout << element << " ";
}
cout << endl;
```

## 2. 익셉션 던지고 받기

프로그램에 익셉션을 구현하는 코드는 두 부분으로 나뉜다. 하나는 발생한 익셉션을 처리하는 `try/catch` 문이고, 다른 하나는 익셉션을 던지는 `throw` 문이다. 둘 다 반드시 지정된 형식에 맞게 작성해야 한다. 하지만 `throw` 문이 실행되는 지점은 대부분 C++ 런타임과 같이 어떤 라이브러리의 깊숙한 곳에 있어서 프로그래머가 직접 볼 수 없을 때가 많다. 그렇다 하더라도 `try/catch` 구문으로 반드시 처리해줘야 한다.

```cpp
try{
    // 익셉션이 발생할 수 있는 코드
} catch (익셉션_타입1 익셉션_이름){
    // 익셉션_타입1 익셉션을 처리하는 코드
} catch (익셉션_타입2 익셉션_이름){
    // 익셉션_타입2 익셉션을 처리하는 코드
}
```

예외 상황이 발생할 수 있는 코드에 `throw` 문으로 익셉션을 직접 던져도 된다. 또한 `throw` 문으로 익셉션을 직접 던지거나 익셉션을 던지는 함수를 호출하는 문장이 담긴 함수를 호출할 수도 있다. 후자의 경우 여러 단계의 호출 과정을 거칠 수도 있다.

익셉션이 발생하지 않으면 `catch` 블록은 실행되지 않고, `try` 문의 마지막 문장을 실행하고 나서 `try/catch` 문을 빠져나와 바로 다음 문장을 실행한다.

반면 익셉션이 발생하면 `throw` 또는 `throw` 문이 담긴 함수를 호출하는 문장의 바로 뒤에 있는 코드는 실행되지 않고, 발생한 익셉션의 타입에 맞는 `catch` 블록으로 실행 흐름이 바뀐다.

`catch` 블록에서 더 이상 실행 흐름이 바뀌지 않는다면, 다시 말해 어떤 값을 리턴하거나, 다른 익셉션을 던지거나, 발생한 익셉션을 그대로 다시 던지는 등의 작업을 수행하지 않으면 방금 실행한 `catch` 블록의 마지막 문장을 끝낸 후 `try/catch` 문을 빠져나와 그다음 코드를 실행한다.

##### 0으로 나누는 상황을 감시하는 함수

이 코드는 `<stdexcept>` 헤더에 정의된 `std::invalid_argument`라는 익셉션을 던진다.

```cpp
double SafeDivide(double num, double den)
{
    if(den == 0)
        throw invalid_argument("Divide by zero");
    return num / den;
}

int main()
{
    try{
        cout << SafeDivide(5, 2) << endl;
        cout << SafeDivide(10, 0) << endl;
        cout << SafeDivide(3, 3) << endl;
    } catch (const invalid_argument& e){
        cout << "Caught exception: " << e.what() << endl;
    }
    return 0;
}
```

이 코드를 실행한 결과는 다음과 같다.

```bash
2.5
Caught exception: Divide by zero
```

여기서 `throw`는 C++에 정의된 키워드로서, 익셉션을 던지려면 반드시 이 키워드를 써야 한다.

`throw` 문에 나온 `invalid_argument()`는 던질 `invalid_argument` 타입의 익셉션 객체를 생성한다. `invalid_argument`는 C++ 표준 라이브러리에서 제공하는 표준 익셉션 중 하나다. 표준 라이브러리에 정의된 익셉션은 일정한 계층을 형성하고 있다. 이 계층 구조에 속한 클래스마드 `what()` 메서드가 있는데, 이 메서드는 익셉션을 표현하는 `const char*` 스트링을 리턴한다. 이 값은 익셉션 생성자의 인수로 전달하는 방식으로 설정한다.

##### readIntegerFile() 함수에서의 익셉션

`readIntegerFile()` 함수에서 발생할 수 있는 가장 심각한 문제는 파일을 열 때 에러가 발생할 수 있다는 것이다. 따라서 이 과정에서 익셉션을 던질 수 있도록 수정한다. 이때 `<exception>` 헤더에 정의된 `std::exception` 타입으로 익셉션을 생성한다.

```cpp
vector<int> readIntegerFile(string_view fileName)
{
    ifstream inputStream(fileName.data());
    if(inputStream.fail()){
        // 파일 열기 실패: 익셉션을 던진다.
        throw exception();
    }
    
    // 파일에 담긴 정숫값을 하나씩 읽어서 벡터에 추가한다.
    vector<int> integers;
    int temp;
    while(inputStream >> temp){
        integers.push_back(temp);
    }
    return integers;
}
```

이 함수에서 파일 열기에 실패하면 `throw exception()` 문장이 실행되면서 함수의 나머지 코드를 건너뛰고 가장 가까운 핸들러 코드로 실행 흐름이 바뀐다.

익셉션을 던지는 코드와 이를 처리하는 코드는 항상 나란히 작성하는 것이 좋다. 익셉션 처리 과정을 다르게 표현하면 어떤 코드 블록을 실행하다가 문제가 발생하면 다른 코드 블록으로 대처하는 것이다.

아래 코드에서 `main()` 함수는 `try` 블록에서 던진 `exception` 타입의 익셉션에 대해 `catch` 문을 에러 메시지로 출력하는 방식으로 처리한다.

```cpp
int main()
{
    const string fileName = "IntegerFile.txt";
    vector<int> myInts;
    try{
        myInts = readIntegerFile(fileName);
    } catch (const exception& e){
        cerr << "Unable to open file " << fileName << endl;
        return 1;
    }

    for(const auto& element : myInts){
        cout << element << " ";
    }
    cout << endl;
    return 0;
}
```

## 3. 익셉션 타입

던질 수 있는 익셉션의 타입에는 제한이 없다. 다음과 같이 간단히 int 타입 객체를 던져도 된다.

```cpp
vector<int> readIntegerFile(string_view fileName)
{
    ifstream inputStream(fileName.data());
    if(inputStream.fail()){
        // 파일 열기 실패: 익셉션을 던진다.
        throw 5;
    }
    
    // 나머지 코드 생략
}
```

그러면 `catch` 문도 다음과 같이 변경한다.

```cpp
try{
    myInts = readIntegerFile(fileName);
} catch (int e){
    cerr << "Unable to open file " << fileName << " (" << e << ")" << endl;
    return 1;
}
```

또는 다음과 같이 C 스타일 스트링인 `const char*` 타입으로 던져도 된다. 스트링에 예외 상황에 대한 정보를 담을 때 유용한 기법이다.

```cpp
vector<int> readIntegerFile(string_view fileName)
{
    ifstream inputStream(fileName.data());
    if(inputStream.fail()){
        // 파일 열기 실패: 익셉션을 던진다.
        throw "Unable to open file";
    }
    
    // 나머지 코드 생략
}
```

`const char*` 타입의 익셉션을 받는 부분은 다음과 같이 그 값을 출력할 수 있다.

```cpp
try{
    myInts = readIntegerFile(fileName);
} catch (const char* e){
    cerr << e << endl;
    return 1;
}
```

하지만 방금 본 두 예제처럼 기본 타입을 사용하기보다는 타입을 새로 정의하는 것이 바람직한데 그 이유는 다음과 같다.

* 객체의 클래스 이름에 예외 상황에 대한 정보를 드러낼 수 있다.
* 예외 상황의 종류뿐만 아니라 다른 정보도 담을 수 있다.

C++ 표준 라이브러리에 미리 정의돼 있는 익셉션 클래스를 활용할 수도 있고, 익셉션 클래스를 직접 정의할 수도 있다.

> 익셉션 객체는 항상 const 레퍼런스로 받는 것이 좋다. 익셉션 객체를 값으로 받으면 객체 슬라이싱이 발생한다.

## 4. 여러 가지 익셉션 던지고 받기

### 4.1 익셉션 타입 매칭과 const

### 4.2 모든 익셉션 매칭하기

## 5. 처리하지 못한 익셉션

## 6. noexcept
