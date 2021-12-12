---
sort: 1
---

# C style String

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## NUL
C 스트링은 스트링의 마지막에 널(NUL) 문자 `\0`를 붙여서 스트링이 끝났음을 표한한다. 예를 들어, 'hello'란 스트링을 구성하는 문자는 다섯 개이지만, 메모리에 저장할 때는 문자 여섯 개의 공간이 필요하다.

## cstring
C++에서 C언어세서 사용하는 스트링 연산 함수를 사용하기 위해서 `<cstring>` 헤더를 사용한다.

C 언어에서 사용되는 스트링 함수 몇 가지를 살펴보자

## strcpy()
`strcpy()` 함수는 스트링 타입 매개변수 두 개를 받아서 두 번째 스트링을 첫 번째 스트링에 복사한다. 이 때 두 스트링의 길이가 같은지 확인하지 않는다.

## strlen()
`strlen()` 함수는 스트링의 길이를 리턴한다. 이 때, 널 값을 포함하지 않은 길이만큼 리턴한다.

다음은 `strcpy()`와 `strlen()`함수를 이용하여 스트링을 복사하여 리턴하는 `copyString()` 함수의 구현 예제이다.
```c
char* copyString(const char* str)
{
  char* result = new char[strlen(str) + 1];
  strcpy(result, str);
  return result;
}
```

* `sizeof()` 함수는 데이터 타입이나 변수의 크기를 구하는데 사용되므로, `strlen()` 함수와 다른 결과가 나온다. `sizeof()`의 경우 널 값을 포함하여 리턴하는 반면, `strlen()`는 널 값을 포함하지 않기 때문에 `char[]` 변수에 대한 리턴값이 다르게 된다. 또한, `char*`로 저장된 변수에 대한 리턴값은 `sizeof()`의 경우 포인터의 크기를 리턴하고, `strlen`의 경우에는 널 값을 제외한 길이를 리턴하므로 그 값에 차이가 발생한다.

## strcat()
`strcat()` 함수는 스트링 매개변수 두 개를 받아서 첫 번째 스트링에 두 번째 스트링을 이어붙이는 함수이다. 다음은 `strcat()` 함수를 이용하여 여러 개의 스트링 매개변수들을 이어 붙여 리턴하는 `appendStrings()` 함수의 구현 예제이다.

```c
char* appendStrings(const char* str1, const shar* str2, const char* str3)
{
  char* result = new char[strlen(str1) + strlen(str2) + strlen(str3) + 1];
  strcpy(result, str1);
  strcat(result, str2);
  strcat(result, str3);
  return result;
}
```