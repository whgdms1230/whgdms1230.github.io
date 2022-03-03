---
sort: 6
---

# Enumerated Types Inside Classes

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 클래스에 열거 타입 정의하기

```cpp
class SpreadsheetCell
{
    public:
        enum class Color { Red = 1, Green, Blue, Yellow };
        void setColor(Color color);
        Color getColor() const;
    
    private:
        Color mColor = Color::Red;
};

// 구현 코드
void SpreadsheetCell::setColor(Color color) { mColor = color; }

SpreadsheetCell::Color SpreadsheetCell::getColor() const { return mColor; }

// 메서드 사용법
SpreadsheetCell myCell(5);
myCell.setColor(SpreadsheetCell::Color::Blue);
auto color = myCell.getColor();
```