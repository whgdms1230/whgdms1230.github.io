---
sort: 3
---

# C++ String

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## C++ std::string
C++ string은 연산자 오버로딩을 활용하여 C 스타일 스트링보다 훨씬 사용성이 높다.

* `+` 연산자
```cpp
string A("12");
string B("34");
string C;
C = A + B; // C == "1234"
```

* `+=` 연산자
```cpp
string A("12");
string B("34");
A += B; // A == "1234"
```

* `==` 연산자 : C는 `==`연산자를 이용해 두 문장을 비교할 수 없다. C string은 스트링의 내용이 아닌 포인터 값을 비교하기 때문에 항상 `false`를 반환한다.

* 그 이외에도 `!=`, `<` 등과 같은 연산자를 사용할 수 있다.

* 또한 `[]`를 이용하여 문자에 접근할 수 있다.

* 연산자 오버로딩으로 string을 확장해도 메모리 관련 작업은 string 클래스에서 자동으로 처리되기 때문에 메모리 오버런(memeory overrun)이 발생하지 않는다.

* c_str() 메서드를 이용하여 C++ string을 C string으로 반환시킬 수 있다. 단, string에 대한 메모리를 다시 할당하거나 해당 string 객체를 제거하면 이 메서드가 리턴한 const 포인터를 더 이상 사용할 수 없게 된다.

## std::string 리터럴
소스 코드에 나온 스트링 리터럴은 주로 const char*로 처리한다. 표준 사용자 정의 리터럴 `s`를 사용하면 스트링 리터럴을 std::string으로 만들 수 있다.
```cpp
auto string1 = "Hello World";   // string1의 타입은 const char*
auto string2 = "Hello World"s;  // string2의 타입은 std::string
```

## 하이레벨 숫자 변환
std 네임스페이스는 숫자와 string을 쉽게 변환할 수 있도록 다양한 편의 함수를 제공한다. 숫자 타입을 string으로 변환하는 함수는 다음과 같다.
```cpp
string to_string(int val);
string to_string(unsigned val);
string to_string(long val):
string to_string(unsigned long val);
string to_string(long long val);
string to_string(unsigned long long val);
string to_string(float val);
string to_string(double val);
string to_string(long double val);
`

또한 반대로 변환하는 함수도 다음과 같이 std 네임스페이스에 정의되어 있다. 여기서 str은 변환하려는 원본 string 값을 의미하고, idx는 아직 변환되지 않은 부분의 맨 앞에 있는 문자의 인덱스를 가리키는 포인터고, base는 변환할 수의 밑이다. 이 변환 함수들은 제일 앞에 나온 공백 문자를 무시하고, 변환에 실패하면 `invalid_argument` 익셉션을 던지고, 변환된 값이 리턴 타입의 ㅂ멈위를 벗어나면 `out_of_range` 익셉션을 던진다.
```cpp
int stoi(const string& str, size_t *idx=0, int base=10);
long stol(const string& str, size_t *idx=0, int base=10);
unsigned long stoul(const string& str, size_t *idx=0, int base=10);
long long stoll(const string& str, size_t *idx=0, int base=10);
unsigned long long stoull(const string& str, size_t *idx=0, int base=10);
float stof(const string& str, size_t *idx=0, int base=10);
double stod(const string& str, size_t *idx=0, int base=10);
long double stold(const string& str, size_t *idx=0, int base=10);
```

## 로우레벨 숫자 변환


## std::string_view
std::string_view 클래스는 `<string_view>`헤더에 정의되어 있으며, const string& 대신 사용할 수 있다. string_view의 인터페이스는 c_str()이 없다는 점을 제외하면 std::string과 같다. std::string으로 스트링을 표현할 때 함수의 매개변수로 전달할 읽기 전용 스트링은 std::string_view로 지정한다.