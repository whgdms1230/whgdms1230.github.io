---
sort: 7
---

# Exception

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

`exception`은 예상하지 못한 상황을 표현하는 클랙스/객체이다. 익셉션을 활용하면 문제가 발생했을 때 융통성 있게 대처할 수 있다.

* `throw` : 익셉션을 발생시킴
* `catch` : 익셉션을 처리함

다음 예제는 분모의 인수가 0이면 익셉션을 발생시킨다. `std::invalid_argument` 익셉션을 사용하였는데, 이는 `<stdexcept>` 헤더파일을 불러와야 한다.
> `exception` 타입은 발생할 수 있는 상황에 맞게 직접 정의해서 사용하는 것이 좋다.

```cpp
double divideNumbers(double numerator, double denominator)
{
  if(denominator == 0) {
    throw invalid_argument("Denoominator cannot be 0.");
  }
  return numerator / denominator;
}
```

`throw` 문장이 실행되면 함수에서 값을 리턴하지 않고 실행을 즉시 중단하며, `try/catch` 블록으로 감싸서 `exception`을 처리할 수 있다.
> `exception`이 발생하게 되면, 바로 `catch`블록으로 넘어가게 되기 때문에, 예제에서 세 번째 호출 문장은 실행되지 않는다.

```cpp
try{
  cout << divideNumbers(2.5, 0.5) << endl;
  cout << divideNumbers(2.3, 0) << endl;
  cout << divideNumbers(4.5, 2.5) << endl;
} catch(const invalid_argument& exception) {
  cout << "Exception caught: " << exception.what() << endl;
}
```