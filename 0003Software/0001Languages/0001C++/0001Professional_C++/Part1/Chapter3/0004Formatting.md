---
sort: 4
---

# Formatting

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. Formatting
포매팅(코드 서식)에 대한 가이드라인을 정하고 같은 팀 내에서 모두 같은 명명 규칙과 서식을 따르게 함으로써 코드의 통일성과 가독성을 높일 수 있다.

### 1.1 중괄호 정렬에 관한 논쟁
코드 블록을 표시하는 중괄호를 적는 위치에 대한 것이다.

예를 들면 다음 예는 함수, 클래스, 메서드 이름을 제외한 나머지 모든 경우 첫 문장과 같은 줄에 적는 경우이다.
```cpp
void someFunction()
{
    if (condition()) {
        cout << "condition was true" << endl;
    } else {
        cout << "condition was false" << endl;
    }
}
```

만약 이 예제를 모두 늘어뜨리면 다음과 같을 수 있다.
```cpp
void someFunction()
{
    if (condition())
    {
        cout << "condition was true" << endl;
    }
    else
    {
        cout << "condition was false" << endl;
    }
}
```

만약 한 문장으로 구성된 블록에 중괄호를 적지 않는다면 다음과 같을 수 있다.
```cpp
void someFunction()
{
    if (condition())
        cout << "condition was true" << endl;
    else
        cout << "condition was false" << endl;
}
```

이렇듯 중괄호에 대한 논쟁은 많을 수 밖에 없다.

### 1.2 스페이스와 소괄호에 대한 논쟁

문장 단위에 적용되는 포매팅에 대한 논쟁으로, 키워드 뒤에 항상 한 칸을 뛰우고, 연산자 앞 뒤에도 한 칸씩 띄우는 것, 매개변수 리스트나 함수 호출에 나온 콤마 뒤에 띄우는 것, 연산 순서가 명확히 드러나도록 소괄호를 사용하는 것 등에 대한 논쟁이다.

예를 들어, 위의 조건을 만족하는 포매팅 예시를 보면 다음과 같다.
```cpp
if (i == 2) {
    j = i + (k / m);
}
```

또는 if 문에 함수 서식을 적용해서 키워드와 소괄호 사이를 띄우지 않거나 연산자의 우선순위를 구분할 필요 없어 소괄호를 생략한다면 다음과 같다.
```cpp
if( i == 2 ) {
    j = i + k / m;
}
```

### 1.3 스페이스와 탭
스페이스와 탭은 단순히 스타일에 대한 취향 문제가 아닌, 팀에서 합의가 이루어져야 할 사항이다. 만약 합의가 이루어지지 않는다면 코드가 지저분해지고, 관리가 어려워진다.

