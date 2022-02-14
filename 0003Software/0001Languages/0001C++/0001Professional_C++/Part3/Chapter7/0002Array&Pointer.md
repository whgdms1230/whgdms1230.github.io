---
sort: 2
---

# Array-Pointer Duality

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 배열
배열은 서로 타입이 같은 원소들을 하나의 변수에 담아서 각 원소를 인덱스로 접근하게 한다.

### 1.1 기본 타입 배열
배열에 대한 메모리를 할당하면 실제 메모리도 연속된 공간을 할당한다. 이 때 메모리 한 칸은 배열의 한 원소를 담을 수 있는 크기로 할당된다.

다음은 다섯 개의 int 값으로 구성된 배열을 로컬 변수로 선언한 코드이다. 이 로컬 변수는 스택에 메모리가 할당된다. 이때 스택에 생성한 배열의 크기가 컴파일 시간에 결정되도록 상숫값으로 지정해야 한다.

```cpp
int myArray[5];
```

배열을 힙에 선언할 때도 비슷하다. 배열의 위치를 가리키는 포인터를 사용한다는 점만 다르다. 다음 코드는 int 값 다섯 개를 담는 배열에 메모리를 할당해서 그 공간을 가리키는 포인터를 myArrayPtr란 변수에 저장한다. 이때 myArrayPtr 변수는 배열의 0번째 원소를 가리킨다.

```cpp
int* myArrayPtr = new int[5];
```

new[]를 호출한 횟수만큼 delete[]를 호출해서 배열에서 할당했던 메모리를 해제하도록 코드를 작성해야 한다.

```cpp
delete [] myArrayPtr;
myArrayPtr = nullptr;
```

배열을 힙에 할당하는 방법의 장점은 실행 시간에 크기를 정할 수 있다는 것이다. 예를 들어 다음 코드는 askUserForNumberOfDocuments()란 함수로부터 받은 문서 수만큼의 크기를 가진 Document 객체 배열을 생성한다.

```cpp
Document* creatDocArray()
{
    size_t numDocs = askUserForNumberOfDocuments();
    Document* docArray = new Document[numDocs];
    return docArray;
}
```

### 1.2 객체 배열
객체에 대한 배열도 기본 타입 배열과 비슷하다. N개의 객체로 구성된 배열을 new[N]으로 할당하면 객체를 담기에 충분한 크기의 N개 블록이 연속된 공간에 할당된다. new[]를 호출하면 각 객체마다 제로 인수 생성자(zero-argument constructor)가 호출된다.

```cpp
class Simple
{
    public:
        Simple() { cout << "Simple constructor called!" << endl; }
        ~Simple() { cout << "Simple destructor called!" << endl; }
}

Simple* mySimpleArray = new Simple[4];
```

위 예제는 Simple 객체 4개로 구성된 배열을 할당하는 코드로, Simple 생성자가 네 번 호출된다.

### 1.3 배열 삭제하기
new[]로 할당하면 반드시 그 수 만큼 delete[]를 호출해서 메모리를 해제해야 한다. delete[]는 할당된 메모리를 해제함과 동시에 각 원소의 객체마다 소멸자를 호출한다.

```cpp
Simple* mySimpleArray = new Simple[4];
// mySimpleArray 사용
delete [] mySimpleArray;
mySimpleArray = nullptr;
```

> 배열 버전의 delete인 delete[]를 사용하지 않으면 프로그램이 이상하게 동작할 수 있다.

배열의 원소가 객체일 때만 모든 원소에 대해 소멸자가 호출된다. 포인터 배열에 대해 delete[]를 호출할 때는 각 원소가 가리키는 객체를 일일이 해제해야 한다.

```cpp
const size_t size = 4;
Simple** mySimplePtrArray = new Simple*[size];

// 포인터마다 객체를 할당한다.
for(size_t i = 0; i < size; i++) { mySimplePtrArray[i] = new Simple(); }

// mySimplePtrArray 사용

// 할당된 객체를 삭제한다.
for(size_t i = 0; i < size; i++) { delete mySimplePtrArray[i]; }

// 배열을 삭제한다.
delete [] mySimplePtrArray;
mySimplePtrArray = nullptr;
```

### 1.4 다차원 배열
다차원 배열이란 여러 개의 인덱스 값을 사용하도록 일차원 배열을 확장한 것이다. 다음은 3X3 크기의 이차원 배열을 스택에 생성하고 0으로 초기화한 뒤 테스트 코드로 접근하는 예시이다.

```cpp
char board[3][3] = {};
// 테스트 코드
board[0][0] = 'X';  // (0,0) 지점에 X를 둔다.
board[2][1] = 'O';  // (2,1) 지점에 O를 둔다.
```

다차원 배열에서 차원 수를 실행 시간에 결정하고 싶다면 힙 배열로 생성한다. 동적으로 할당하는 일차원 배열을 포인터로 접근하듯이 동적으로 할당하는 다차원 배열도 포인터로 접근한다. 단지 이차원 배열의 경우 포인터에 대한 포인터로 원소에 접근하는 반면 N차원 배열은 N단계의 포인터로 접근한다는 점만 다르다.

다음과 같이 다차원 배열을 동적으로 할당하면 컴파일 오류가 발생한다.

```cpp
char** board = new char[i][j];  // 컴파일 오류 발생
```

그 이유는 힙에서는 메모리 공간이 연속적으로 할당되지 않기 때문에, 스택 방식의 다차원 배열처럼 메모리를 할당하면 안 된다.

힙 배열의 첫 번째 인덱스에 해당하는 차원의 배열을 연속적인 공간에 먼저 할당한다. 그런 다음 이 배열의 각 원소에 두 번째 인덱스에 해당하는 차원의 배열을 가리키는 포인터를 저장한다.

다음은 이차원 배열을 동적으로 할당하는 예시이다.

```cpp
char** allocateCharacterBoard(size_t xDimension, size_t yDimension)
{
    char** myArray = new char*[xDimension]; // 첫 번째 차원의 배열 할당
    for(size_t i = 0; i < xDimension; i++) {
        myArray[i] = new char[yDimension]; // i번째 하위 배열을 할당
    }
    return myArray;
}
```

다차원 힙 배열에 할당된 메모리를 해제할 때도 마찬가지로 일일이 해제해야 한다.

```cpp
void releaseCharacterBoard(char** myArray, size_t xDimension)
{
    for(size_t i = 0; i < xDimension; i++) {
        delete [] myArray[i]; // i번째 하위 배열을 해제
    }
    delete [] myArray; // 첫 번째 차원의 배열 해제
}
```

## 2. 포인터

### 2.1 포인터의 작동 방식
포인터는 메모리의 한 지점을 가리키는 숫자에 불과하다. 

`*` 연산자로 포인터를 역참조하면 메모리에서 한 단계 더 들어가 볼 수 있다. 포인터를 주소 관점에서 보면 역참조는 포인터가 가리키는 주소로 점프하는 것과 같다.

`&` 연산자를 사용하면 특정 지점의 주소를 알 수 있다. 이렇게 하면 메모리에 대한 참조 단계가 하나 더 늘어난다. 이 연산자를 주소 관점에서 보면 프로그램은 특정 메모리 지점을 숫자로 표현한 주소로 본다.

### 2.2 포인터에 대한 타입 캐스팅
포인터는 단지 메모리 주소에 불과해서 타입을 엄격히 따지지 않는다. 포인터의 타입은 C 스타일 캐스팅을 이용해서 얼마든지 바꿀 수 있다.

```cpp
Document* documentPtr = getDocument();
char* myCharPtr = (char*)documentPtr;
```

정적 캐스팅을 사용하면 더 안전하다. 관련 없는 데이터 타입으로 포인터를 캐스팅하면 컴파일 에러가 발생한다.

```cpp
Document* documentPtr = getDocument();
char* myCharPtr = static_cast<char*>(documentPtr); // 컴파일 에러 발생
```

> 정적 캐스팅하려는 포인터와 캐스팅 결과에 대한 포인터가 가리키는 객체가 서로 상속관계에 있다면 컴파일 에러가 발생하지 않는다. 하지만 상속 관계에 있는 대상끼리 캐스팅 할 때는 동적 캐스팅을 사용하는 것이 더 안전하다.

## 3. 배열과 포인터

스택 배열에 접근할 때 포인터를 사용할 수 있다. 배열의 주소는 사실 첫 번째 원소에 대한 주소다. 그래서 힙 배열과 똑같은 방식으로 포인터를 사용할 수 있다.

```cpp
int myIntArray[10];
int* myIntPtr = myIntArray;
// 포인터로 배열 접근하기
myIntPtr[4] = 5;
```

스택 배열을 포인터로 접근하는 기능은 배열을 함수에 넘길 때 유용하다. 다음 함수는 정수 배열을 포인터로 받는다. 여기서 포인터만으로는 크기를 알 수 없기 때문에 함수를 호출할 때 배열의 크기를 지정해야한다.

```cpp
void doubleInts(int* theArray, size_t size)
{
    for(size_t i = 0; i < size; i++) {
        theArray[i] *= 2;
    }
}
```

위 함수를 호출할 때 스택 배열을 전달해도 되고 힙 배열을 전달해도 된다. 힙 배열을 전달하면 이미 포인터가 담겨 있어서 함수에 값으로 전달된다. 스택 배열을 전달하면 배열 변수를 전달하기 때문에 컴파일러가 이를 배열에 대한 포인터로 변환한다. 이때 프로그래머가 직접 첫 번째 원소의 주소를 넘겨도 된다.

```cpp
size_t arrSize = 4;
int* heapArray = new int[arrSize]{1, 5, 3, 4};
doubleInts(heapArray, arrSize); // 힙 배열 전달
delete [] heapArray;
heapArray = nullptr;

int stackArray[] = {5, 7, 9, 11};
arrSize = std::size(stackArray);
doubleInts(stackArray, arrSize); // 스택 배열 전달
doubleInts(&stackArray[0], arrSize); // 스택 배열의 첫 번째 원소의 주소 전달
```

> 컴파일러는 배열을 함수로 전달하는 부분을 포인터로 취급한다. 배열을 인수로 받아서 그 안에 담긴 값을 변경하는 함수는 복사본이 아닌 원본을 직접 수정한다. 포인터와 마찬가지로 배열을 전달하면 실제로 레퍼런스 전달 방식의 효과가 나타난다.