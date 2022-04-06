---
sort: 5
---

# 범위 기반 for 문

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

범위 기반 `for`문(range-based for statement)은 컨테이너에 담긴 원소에 대해 반복문을 실행하는데 주로 사용된다. `C` 스타일의 루프, [이니셜라이저 리스트(`initializer_list`)](/00003Software/0001Languages/0001Professional_C++/Part1/Chapter1/0002Switch-initializer), `std::array`, `std::vector`, 표준 라이브러리에서 제공하는 컨테이너처럼 반복자를 리턴하는 `begin()`/`end()` 메서드가 정의된 모든 타입에 적용할 수 있다.

다음 예제는 범위 기반 for문을 돌면서 배열의 모든 원소에 대한 복제본을 출력한다.
```cpp
std::array<int, 4> arr = {1, 2, 3, 4};
for (int i : arr) {
  std::cout << i << std::endl;
}
```