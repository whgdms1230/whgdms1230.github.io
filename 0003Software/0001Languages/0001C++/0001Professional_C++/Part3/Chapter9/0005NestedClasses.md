---
sort: 5
---

# Nested Classes

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 중첩 클래스

클래스 안에서 다른 클래스를 정의할 수 있다. 예를 들어 SpreadsheetCell 클래스를 Spreadsheet 클래스 안에서 정의할 수 있다. 그러면 Spreadsheet 클래스의 일부분이 되기 때문에 이름을 간단히 Cell 이라고 붙여도 된다.

```cpp
class Spreadsheet
{
    public:
        class Cell
        {
            public:
                Cell() = default;
                Cell(double initialValue);
            // ...
        };
        Spreadsheet(size_t width, size_t height, const SpreadsheetApllication& theApp);
    // ...
};
```

Spreadsheet 클래스 밖에서 Cell 클래스를 참조하기 위해 Spreadsheet:: 스코프 지정자를 사용해야 한다. 

```cpp
Spreadsheet::Cell::Cell(double initialValue)
    : mValue(initialValue)
{    
}
```

이렇게 스코프 지정 연산자를 붙이는 규칙은 Spreadsheet 클래스 안에 있는 메서드의 리턴 타입에도 적용된다. 단, 매개변수에는 적용되지 않는다.

```cpp
Spreadsheet::Cell& Spreadsheet::getCellAt(size_t x, size_t y)
{
    verifyCoordinate(x, y);
    return mCells[x][y];
}
```

Spreadsheet 클래스에서는 Cell을 선언만 하고 정의코드는 따로 작성할 수 있다.

```cpp
class Spreadsheet
{
    public:
        class Cell
        
        Spreadsheet(size_t width, size_t height, const SpreadsheetApllication& theApp);
    // ...
};

class Spreadsheet::Cell
{
    public:
        Cell() = default;
        Cell(double initialValue);
    // ...
};
```

> 중첩 클래스도 접근 제어 규칙이 똑같이 적용된다.