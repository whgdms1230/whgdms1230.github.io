---
sort: 6
---

# Variable Templates

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 변수 템플릿

C++14부터 변수 템플릿도 제공한다.

```cpp
template<typename T>
constexpr T pi = T(3.141592653);
```

이 코드는 파이 값에 대한 변수 템플릿으로서, 특정한 타입의 변수를 생성하려면 다음과 같이 작성한다.

```cpp
float piFloat = pi<float>;
long double piLongDouble = pi<long double>;
```

그러면 지정한 타입으로 표현할 수 있는 범위에 맞게 파이값을 구할 수 있다.

다른 템플릿과 마찬가지로 변수 템플릿도 특수화할 수 있다.