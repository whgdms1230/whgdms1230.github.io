---
sort: 4
---

# Scope Resolution

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 스코프

변수, 함수, 클래스 이름을 비롯하여 프로그램에서 사용하는 모든 이름은 일정한 스코프에 속한다.

스코프는 네임스페이스, 함수 정의, 중괄호 블록, 클래스 정의 등으로 생성한다.

간혹 스코프 안에 다른 스코프와 같은 이름이 있으면 다른 스코프의 이름을 가린다. 또 어떤 경우는 현재 코드에서 사용하는 대상의 스코프가 디폴트 스코프 범위에 속하지 않을 수도 있다. 이런 경우 스코프 지정자인 `::`로 해당 스코프를 지정할 수 있다.

다음 예제는 클래스에 선언된 static 메서드가 전역 스코프와 네임스페이스에도 각각 선언된 경우이다.

```cpp
class Demo
{
    public:
        static int get() { return 5; }
};

int get() { return 10; }

namespace NS
{
    int get() { return 20; }
}
```

전역 스코프는 이름이 따로 없지만 스코프 이름 없이 스코프 지정 연산자만 붙여서 접근할 수 있다.

```cpp
int main()
{
    auto pd = std::make_unique<Demo>();
    Demo d;
    std::cout << pd->get() << std::endl; // 5
    std::cout << d.get() << std::endl; // 5
    std::cout << NS::get() << std::endl; // 20
    std::cout << Demo::get() << std::endl; // 5
    std::cout << ::get() << std::endl; // 10
    std::cout << get() << std::endl; // 10
    return 0;
}
```