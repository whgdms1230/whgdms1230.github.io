---
sort: 5
---

# Stack Unwinding and Cleanup

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 스택 풀기와 청소

어떤 코드가 익셉션을 던지면 이를 받아서 처리할 `catch` 핸들러르 스택에서 찾는다. 이때 `catch` 핸들러는 현재 스택 프레임에 바로 있을 수도 있고, 몇 단계의 함수 호출 스택을 거슬러 올라가야 나타날 수도 있다. 어떻게든 `catch` 핸들러를 발견하면 그 핸들러가 정의된 스택 단계로 되돌아가는데, 이 과정에서 중간 단계에 있던 스택 프레임을 모두 풀어버린다. 이를 스택 풀기라 부르며, 스코프가 로컬인 소멸자를 모두 호출하고, 각 함수에서 미처 실행하지 못한 코드는 건너뛴다.

그런데 스택 풀기가 발생할 때 포인터 변수를 해제하고 리소스를 정리하는 작업은 실행되지 않는다. 그래서 다음과 같은 문제가 발생할 수 있다.

```cpp
void funcOne();
void funcTwo();

int main()
{
    try{
        funcOne();
    }catch(const exception& e){
        cerr << "Exception caught!" << endl;
        return 1;
    }
    return 0;
}

void funcOne()
{
    string str1;
    string* str2 = new string();
    funcTwo();
    delete str2;
}

void funcTwo()
{
    ifstream fileStream;
    fileStream.open("filename");
    throw exception();
    fileStream.close();
}
```

`funcTwo()`에서 익셉션을 던질 때 가장 가까운 핸들러는 `main()`에 있다. 그래서 실행 흐름은 즉시 `funcTwo()`에 있던 `throw exception();` 문장에서 `main()`의 `cerr << "Exception caught!" <<endl;`로 건너뛴다.

`funcTwo()`의 실행 지점은 익셉션을 던진 문장에 여전히 머물러 있다. 따라서 그 뒤에 나온 `fileStream.close();` 문장은 실행되지 않는다.

다행히 `ifstream` 소멸자는 호출된다. `filestream`이 스택에 있는 로컬 변수이기 때문이다. 만약 `fileStream`을 동적으로 할당했다면 제거되지 않기 때문에 파일은 닫히지 않고 그대로 남게 된다.

`funcOne()`에서 실행 지점이 `funcTwo()` 호출에 있으므로 그 뒤에 나온 `delete str2;` 문장은 실행되지 않는다. 따라서 메모리 누수가 발생한다.

이러한 문제 때문에 C 언어에서 사용하던 할당 모델과 익센션 프로그래밍 기법을 섞어 쓰면 안 된다. C++로 코드를 작성할 때는 반드시 스택 기반 할당 방식을 적용해야 한다.

다음에 나오는 두 가지 기법은 스택 기반의 할당 방식을 사용하지 않는 경우에 메모리 누수를 막기 위한 방법을 제시한다.

### 1.1 스마트 포인터 활용

스택 기반 할당 기법을 사용할 수 없다면 스마트 포인터를 활용한다.

```cpp
void funcOne()
{
    string str1;
    auto str2 = make_unique<string>("hello");
    funcTwo();
}
```

여기서 `str2` 포인터는 `funcOne()`을 호출한 후 리턴될 때 또는 그 안에서 익셉션이 발생할 때 자동으로 제거된다.

물론 특별히 이유가 있을 때만 동적으로 할당해야 한다. 예를 들어 앞에 나온 `funcOne`에서 굳이 `str2`를 동적으로 할당할 필요가 없다. 스택 기반 `string` 변수로도 충분하다. 여기서 동적으로 할당한 이유는 단지 익셉션을 던진 후 일어나는 일을 간단히 보여주기 위해서다.

### 1.2 익셉션 잡고, 리소스 정리한 뒤, 익셉션 다시 던지기

다음은 각 함수마다 발생 가능한 익셉션을 모두 잡아서 리소스를 제대로 정리한 뒤 그 익셉션을 다시 스택의 상위 핸들러로 던지는 방법이다.

예를 들어 `funcOne()`을 이렇게 처리하도록 수정하면 다음과 같다.

```cpp
void funcOne()
{
    string str1;
    string* str2 = new string();
    try{
        funcTwo();
    }catch(...){
        delete str2;
        throw; // 같은 익셉션을 다시 위로 던진다.
    }
    delete str2;
}
```

이 함수는 `funcTwo()`를 호출하는 문장과 여기서 발생하는 익셉션을 처리하는 핸들러를 정의하고 있다. 이 핸들러는 리소스를 정리한 뒤 잡은 익셉션을 다시 던진다. 이렇게 현재 잡은 익셉션을 다시 던질 때는 `throw`란 키워드만 적어도 된다.

> 익셉션을 다시 던지는 방식보다는 스마트 포인터나 RAII 클래스를 사용하는 방법이 더 좋다.