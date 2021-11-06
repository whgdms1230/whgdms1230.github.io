---
sort: 6
---

# 스마트 포인터 기본

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

기존 `C` 스타일의 포인터 대신 `C++`에서는 스마트 포인터(`smart pointer`) 사용하여 메모리와 관련된 문제들을 방지할 수 있다. 스마트 포인터로 지정한 객체가 스코프를 벗어나면 메모리가 자동으로 해제되기 때문이다.

C++에서 가장 중요하게 사용되는 스마트 포인터 타입은 `std::unique_ptr`과 `std::shared_ptr`이다.

## 1. unique_ptr

### 1.1 unique_ptr 개념
`unique_ptr`은 포인터로 가리키는 대상이 스코프를 벗어나거나 삭제될 때 할당된 메모리나 리소스도 자동으로 삭제된다는 점을 제외하면 일반 포인터와 같다. 또한, `unique_ptr`는 `return`문이 실행되거나 익셉션이 발생하더라도 항상 할당된 메모리나 리소스를 해제할 수 있으며, 이는 함수를 간결하게 한다.(리소스 해제 코드를 작성하지 않아도 되기 때문)

### 1.2 unique_ptr 생성
`unique_ptr`를 생성할 때는 반드시 `std::make_unique<>()`를 사용해야 한다.
```cpp
auto anEmployee = make_unique<Empolyee>();
```

`make_unique()`는 `C++14`부터 추가되었는데, 만약 컴파일러가 `C++14`를 지원하지 않는다면 다음의 방법으로 `unique_ptr`를 만든다.
```cpp
unique_ptr<Employee> anEmployee(new Employee);
```

스마트 포인터로 지정한 `anEmployee`의 사용법은 일반 포인터와 같다.
```cpp
if (anEmployee) {
  cout << "Salary: " << anEmployee->salary << endl;
}
```

`unique_ptr`은 C 스타일의 배열을 저장하는 데도 활용할 수 있다.
```cpp
auto employees = make_unique<Employee[]>(10);
cout << "Salary: " << employees[0].salary << endl;
```

## 2. shared_ptr

### 2.1 shared_ptr 개념
`shared_ptr`를 사용하면 데이터를 공유할 수 있다. `shared_ptr`에 대한 대입 연산이 발생할 때마다 레퍼런스 카운트가 증가하며, 이는 `shared_ptr`이 가리키는 데이터를 레퍼런스 카운트 만큼 소유하고 있다는 것이다.

`shared_ptr` 역시 스코프를 벗어나면 레퍼런스 카운트가 감소하고, 레퍼런스 카운트가 0이 되면 포인터로 가리키던 객체를 해제한다.

### 2.2 shared_ptr 생성
`shared_ptr`는 `std::make_shared<>()`로 생성한다.
```cpp
auto anEmployee = make_shared<Employee>();
if (anEmployee) {
  cout << "Salary: " << anEmployee->salary << endl;
}
```

`C++17`부터 shared_ptr에 배열도 저장할 수 있다. 단, 이 경우에는 `make_shared<>()`를 사용할 수 없고, 다음과 같이 작성해야 한다.
```cpp
shared_ptr<Employee[]> employees(new Employee[10]);
cout << "Salary: " << employees[0].salary << endl;
```

## 3. unique_ptr과 shared_ptr

일반적으로 `unique_ptr`을 기본적으로 사용하며, 소유권을 공유할 필요가 있다면 `shared_ptr`을 사용한다.

* `auto_ptr`은 `C++17`부터 완전 삭제 되었다.