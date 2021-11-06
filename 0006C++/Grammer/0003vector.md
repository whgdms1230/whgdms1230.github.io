---
sort: 3 
---

# vector 기초

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

`std::vector`는 `C++` 표준 라이브러리로, `C` 스타일의 배열을 대체하며, 메모리 관리를 신경 쓸 필요가 없기 때문에 배열보다 훨씬 유연하고 안전하다.

```cpp
vector<int> myVector = { 11, 22 };

myVector.push_back(33);
myVector.push_back(44);

cout << "1st element: " << myVector[0] << endl;
```

* `vector`는 제네릭 컨테이너로, 거의 모든 종류의 객체를 담을 수 있다.