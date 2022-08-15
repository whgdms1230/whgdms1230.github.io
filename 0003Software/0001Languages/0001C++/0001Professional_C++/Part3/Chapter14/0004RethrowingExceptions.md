---
sort: 4
---

# Rethrowing Exceptions

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 익셉션 다시 던지기

`throw` 키워드는 현재 발생한 익셉션을 다시 던질 때 사용한다.

```cpp
void g() { throw invalid_argument("Some exception"); }

void f()
{
    try{
        g();
    }catch(const invalid_arguemtn& e){
        cout << "caught in f: " << e.what() << endl;
        throw; // 다시 던지기
    }
}

int main()
{
    try {
        f();
    }catch(const invalid_argument& e){
        cout << "caught in main: " << e.what() << endl;
    }
    return 0;
}
```

이 코드를 실행한 결과는 다음과 같다.

```bash
caught in f: Some exception
caught in main: Some exception
```

여기서 `throw e;`와 같은 문장으로 익셉션을 다시 던지면 된다고 생각하기 쉽지만 그러면 안 된다. 익셉션 객체에 대한 슬라이싱이 발생하기 때문이다. 예를 들어 `f()`에서 `std::exception`을 잡고, `main()`에서 `exception`과 `invalid_arguemtn` 익셉션을 모두 잡으려면 다음과 같이 수정한다.

```cpp
void g() { throw invalid_argument("Some exception"); }

void f()
{
    try{
        g();
    }catch(const exception& e){
        cout << "caught in f: " << e.what() << endl;
        throw; // 다시 던지기
    }
}

int main()
{
    try {
        f();
    }catch(const invalid_argument& e){
        cout << "invalid_argument caught in main: " << e.what() << endl;
    }catch(const exception& e){
        cout << "exception caught in main: " << e.what() << endl;
    }
    return 0;
}
```

이 코드를 실행한 결과는 다음과 같다.

```bash
caught in f: Some exception
invalid_argument caught in main: Some exception
```

만약 `f()`에서 `throw;` 문장을 `throw e;` 로 바꾸면 다음과 같은 결과가 나온다.

```bash
caught in f: Some exception
exception caught in main: Some exception
```

`throw e;` 문장에서 슬라이싱이 발생하여 `invalid_argument`가 `exception`으로 돼버렸기 때문이다. 따라서 익셉션을 다시 던질 때는 항상 `throw;`으로 적어야 한다.