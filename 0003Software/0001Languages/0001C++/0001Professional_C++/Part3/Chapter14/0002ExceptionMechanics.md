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

readIntegerFile()에서 발생할 수 있는 다른 오류(파일 열기 실패, 데이터 읽기 오류)가 발생할 때 익셉션을 던지도록 다음과 같이 수정할 수 있다.

> 이번에는 exception을 상속한 runtime_error로 구현한다. 이 타입은 생성자를 호출할 때 예외에 대한 설명을 지정할 수 있다. runtime_error 익셉션 클래스는 `<stdexcept>` 헤더에 정의되어 있다.

```cpp
vector<int> readIntegerFile(string_view fileName)
{
    ifstream inputSream(fileName.data());
    if(inputSream.fail()){
        // 파일 열기에 실패한 경우 : 익셉션을 던진다.
        throw runtime_error("Unable to open the file.");
    }

    // 파일에서 정수를 하나씩 읽어 벡터에 추가한다.
    vector<int> integers;
    int tmp;
    while(inputSream >> temp) {
        integers.push_back(temp);
    }

    if(!inputSream.eof()){
        // 파일 끝(EOF)에 도달하지 않았다.
        // 다시 발해 파일을 읽는 도중 에러가 발생했다.
        // 따라서 익셉션을 던진다.
        throw runtime_error("Error reading the file.");
    }

    return integers;
}
```

앞에서 main() 함수를 작성할 때 catch 구문이 runtime_error의 베이스 클래스인 exception 타입을 받도록 지정해뒀기 때문에 여기서는 변경할 필요 없다. 이렇게 하면 catch 문은 두 가지 상황을 모두 처리하게 된다.

```cpp
try {
    myInts = readIntegerFile(fileName);
} catch (const exception& e) {
    cerr < e.what() << endl;
    return 1;
}
```

이렇게 하지 않고 readIntegerFile()에서 익셥선을 두 가지 타입으로 따로 나눠서 던져도 된다. 파일을 열 수 없으면 invalid_argument 익셉션을 던지고, 정수를 읽을 수 없으면 runtime_error 익셉션을 던진다.

```cpp
vector<int> readIntegerFile(string_view fileName)
{
    ifstream inputSream(fileName.data());
    if(inputSream.fail()){
        // 파일 열기에 실패한 경우 : 익셉션을 던진다.
        throw invalid_argument("Unable to open the file.");
    }

    // 파일에서 정수를 하나씩 읽어 벡터에 추가한다.
    vector<int> integers;
    int tmp;
    while(inputSream >> temp) {
        integers.push_back(temp);
    }

    if(!inputSream.eof()){
        // 파일 끝(EOF)에 도달하지 않았다.
        // 다시 발해 파일을 읽는 도중 에러가 발생했다.
        // 따라서 익셉션을 던진다.
        throw runtime_error("Error reading the file.");
    }

    return integers;
}
```

> invalid_argument와 runtime_error에는 public 디폴트 생성자가 없고 string 인수를 받는 생성자만 있다.

main()에서는 invalid_argument와 runtime_error를 받는 catch 문을 각각 작성한다.

```cpp
try {
    myInts = readIntegerFile(fileName);
} catch (const invalid_argument& e) {
    cerr < e.what() << endl;
    return 1;
} catch (const runtime_error& e) {
    cerr < e.what() << endl;
    return 2;
}
```

try 블록에서 익셉션이 발생하면 컴파일러는 그 익셉션 타입과 일치하는 catch 문을 선택한다.

### 4.1 익셉션 타입 매칭과 const

처리하려는 익셉션 타입에 const가 지정됐는지 여부는 매칭 과정에 영향을 미치지 않는다. 다시 말해 다음 문장은 runtime_error 타입에 속하는 모든 익셉션을 매칭한다.

```cpp
} catch (const runtime_error& e) {
```

다음 문장도 마찬가지로 runtime_error 타입에 속하는 모든 익셉션을 매칭한다.

```cpp
} catch (runtime_error& e) {
```

### 4.2 모든 익셉션 매칭하기

catch 문에서 모든 종류의 익셉션에 매칭하려면 다음과 같이 특수한 문법으로 작성한다.

```cpp
try {
    myInts = readIntegerFile(fileName);
} catch (...) {
    cerr < "Error reading or opening file " << fileName << endl;
    return 1;
}
```

> `...` 은 모든 익셉션 타입에 매칭하라는 와일드카드다. 문서에 익셉션 타입이 정확히 나와 있지 않아서 모든 익셉션을 받게 만들 때 유용하다. 하지만 이는 필요 없는 익셉션까지 처리하기 때문에 발생 가능한 익셉션을 확실히 알 수 있다면 이렇게 구현하지 않는 것이 좋다.

`catch (...)` 구문은 디폴트 catch 핸들러를 구현할 때도 유용하다. 익셉션이 발생하면 catch 핸들러가 코드에 나열된 순서대로 검색하면서 조건에 맞는 것을 실행한다.

```cpp
try{
    // 익셉션이 발생할 수 있는 코드
} catch (const invalid_argument& e) {
    // invalid_argument 익셉션을 처리하는 핸들러 코드   
} catch (const runtime_error& e) {
    // runtime_error 익셉션을 처리하는 핸들러 코드   
} catch (...) {
    // 나머지 모든 익셉션을 처리하는 핸들러 코드   
}
```

## 5. 처리하지 못한 익셉션

프로그램에서 발생한 익셉션을 처리하는 곳이 하나도 없으면 프로그램이 종료돼버린다. 그래서 미처 처리하지 못한 익셉션을 모두 잡도록 main() 함수 전체를 try/catch 구문으로 감싸는 패턴을 많이 사용한다.

```cpp
try {
    main(argc, argv);
} catch (...) {
    // 에러 메시지를 출력한 뒤 프로그램을 종료한다.
}
```

> 반드시 프로그램에서 발생할 수 있는 익셉션을 모두 잡아서 처리하도록 작성한다.

catch 구문으로 처리하지 못한 익셉션이 남아 있다면 프로그램을 다르게 실행하도록 구현하는 방법도 있다. 예를 들어 프로그램이 잡지 못한 익셉션을 만나면 `terminate()` 함수를 호출하게 만들 수 있다. 이 함수는 C++에서 기본으로 제공하며, 내부적으로 `<cstdlib>` 헤더에 정의된 abort() 함수를 호출해서 프로그램을 죽인다. 또는 `set_terminate()`에 인수를 받지 않고 리턴값도 없는 콜백 함수를 포인터로 지정하는 방식으로 `terminate_handler`를 직접 구현해도 된다. 이들 함수 모드 `<exception>` 헤더에 선언돼 있다.

```cpp
try {
    main(argc, argv);
} catch (...) {
    if (terminate_handler != nullptr) {
        terimnate_handler();
    } else {
        terminate();
    }
}
// 정상 종료 코드
```

종료 직전에 유용한 정보를 담은 에러 메시지를 출력하기 위해 커스텀 콜백 함수인 myTerminate()를 terminate_handler로 지정하도록 한다. 이 핸들러는 readIntegerFile()이 던지는 익셉션을 제대로 처리하지 않고 그냥 에러 메시지만 출력한 뒤 exit()를 호출해서 프로그램을 종료시킨다. exit() 함수는 프로세스를 종료하는 방식을 표현하는 정숫값을 인수로 받는다.

```cpp
void myTerminate()
{
    cout << "Uncaught exception!" << endl;
    exit(1);
}

int main()
{
    set_terminate(myTerminate);

    const string fileName = "IntegerFile.txt";
    vector<int> myInts = readIntegerFile(fileName);

    for (const auto& element : myInts){
        cout << element << " ";
    }
    cout << endl;
    return 0;
}
```

> set_terminate() 함수로 새로운 terminate_handler를 지정하면 기존에 설정된 핸들러를 리턴한다. terminate_handler는 프로그램 전체에서 접근할 수 있기 때문에 처리할 일이 끝나면 이를 리셋하는 것이 바람직다. 이 예제에서는 terminate_handler를 사용하는 다른 코드가 없기 때문에 리셋하지 않았다.

> set_terminate()는 반드시 알아야 할 기능 중 하나지만, 에러 처리에 가장 효과적인 수단은 아니다. 그보다는 처리할 익셉션을 try/catch 구문에 구체적으로 지정해서 꼭 필요한 익셉션만 제대로 처리하는 것이 바람직하다.

## 6. noexcept

함수에 noexcept 키워드를 지정해서 어떠한 익셉션도 던지지 않는다고 지정할 수 있다. 예를 들어 앞에서 본 readIntegerFile() 함수에 noexcept를 지정하면 익셉션을 하나도 던지지 않는다.

```cpp
vector<int> readIntegerFile(string_view fileName) noexcept;
```

noexcept 키워드가 지정된 함수에 익셉션을 던지는 코드가 있으면 terminate()를 호출해서 프로그램을 종료시킨다.

파생 클래스에서 virtual 메서드를 오버라이드할 때 베이스 클래스에 정의된 메서드에 noexcept가 지정되지 않았더라도 오버라이드하는 메서드에 noexcept를 지정할 수 있다. 하지만 그 반대로는 할 수 없다.