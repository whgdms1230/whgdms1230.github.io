---
sort: 2
---

# String Literal

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## String Literal
스트링 리터럴이란 다음과 같이 변수에 담지 않고 곧바로 값으로 표현한 스트링을 말한다. 다음은 hello란 스트링을 변수에 담지 않고 화면 출력에 사용된 스트링 리터럴 값이다.

```cpp
cout << "hello" << endl;
```

## Literal Pooling
스트링 리터럴은 내부적으로 메모리의 읽기 전용 영역에 저장된다. 그래서 컴피일러는 같은 스트링 리터럴이 반복되면 그 중 한 스트링에 대한 레퍼런스를 재사용하는 방식으로 메모리를 절약한다. 이를 리터럴 풀링이라 한다.

## 스트링 리터럴의 대입
스트링 리터럴을 변수에 대입할 수 있지만, 메모리 읽기 전용 영역에 있게 되거나 동일한 리터럴을 여러 곳에 공유할 수 있기 때문에 변수에 저장하면 위험하다. 따라서 C++ 표준에서는 스트링 리터럴을 `const char이 n개인 배열` 타입으로 정의한다. 하지만 `const`가 없던 시절에 작성된 레거시 코드의 하위 호환성을 보장하도록 `const char*`이 아닌 타입으로 저장하는 컴파일러도 있기 때문에 컴파일러에 따라 동작이 다를 수 있다.

따라서 다음과 같이 코드를 작성하게 되면 결과를 예측할 수 없다.

```cpp
char* ptr = "hello";
ptr[1] = 'a';
```

따라서 스트링 리터럴을 참조할 때는 `const` 문자에 대한 포인터를 사용하는 것이 훨씬 안전하다.

```cpp
const char* ptr = "hello";
ptr[1] = 'a'                // 에러 발생
```

문자 배열(`char[]`)에 초깃값을 설정할 때도 스트링 리터럴을 사용하는데, 이 때 컴파일러는 주어진 스트링을 충분히 담을 정도로 큰 배열을 생성하여 스트링 값을 복사하며, 이 값은 읽기 전용 메모리에 넣지 않으며 재사용하지 않는다. 따라서 다음과 같이 스트링을 수정할 수 있게 된다.

```cpp
char arr[] = "hello";
arr[1] = 'a';               // 스트링 수정 가능
```

## Raw String Literal
로 스트링 리터럴이란 여러 줄에 걸쳐 작성한 스트링 리터럴로서, 스트링 안에 인용 부호를 이스케이프 시퀀스로 표현할 필요가 없으며, 오히려 이스케이프 시퀀스는 일반 텍스트로 취급하는 스트링 리터럴이다. 로 스트링 리터럴은 `R"(`로 시작해서 `)"`로 끝난다.

먼저 다음은 이스케이프 시퀀스로 표현된 스트링 리터럴이다.

```cpp
const char* str = "Hello \"World\"!";

const char* str = "Line 1\nLine 2";
```

이를 로 스트링 리터럴을 사용하여 표현하면 다음과 같다.

```cpp
const char* str = R"(Hello "World"!)";

const char* str = R"(Line 1
Line 2)";
```

로 스트링 리터럴에 이스케이프 시퀀스가 나오면 다음과 같은 결과가 나온다.

```cpp
const char* str = R"(Is the following a tab character? \t)";
```

```bash
Is the following a tab character? \t
```

만약 로 스트링 리터럴 중간에 `)"`가 나오면 에러가 발생하게 된다.

```cpp
const char* str = R"(Embedded )" characters)"; // 에러 발생
```

이러한 경우에는 확장 로 스트링 리터럴 구문으로 표현해야 한다. 확장 로 스트링 리터럴은 다음과 같다.

```cpp
R"d-char-sequence(r-char-sequence)d-char-sequence"
```

여기서 `r-char-sequence`에 해당하는 부분이 실제 로 스트링이며, `d-char-sequence` 부분이 구분자 시퀀스로 반드시 로 스트링 리터럴의 시작과 끝에 똑같이 나와야 한다. 위의 예제를 확장 로 스트링 리터럴 구문으로 다시 표현하면 다음과 같다.

```cpp
const char* str = R"-(Embedded )" characters)-";
```

로 스트링 리터럴은 데이터베이스 쿼리 스트링이나, 정규 표현식, 파일 경로 등을 표현하는데 사용된다.