---
sort: 4
---

# initializer_list

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

이니셜라이저 리스트(`initializer_list`)는 `<initializer_list>` 헤더 파일에 정의되어 있으며, 이를 활용하여 여러 인수를 받는 함수를 작성할 수 있음.

다음 예제는 `makeSum()` 함수가 정수에 대한 이니셜라이저 리스트를 인수로 받는다.
```cpp
#include <initializer_list>

int makeSum(initializer_list<int> lst)
{
  int total = 0;
  for (int value : lst) {
    total += value;
  }
  return total;
}
```

위에서 정의한 `makeSum()` 함수를 호출하는 방법은 다음과 같다.
```cpp
int a = makeSum({1, 2, 3});
```

* 이니셜라이저 리스트는 타입에 안전하다.
* 이니셜라이저 리스트를 정의할 때는 지정한 타입만 허용한다.