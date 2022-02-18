---
sort: 1
---

# Introducin The Spreadsheet Example

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 스프레트시트 예제

해당 예제를 기반으로 이어질 내용을 설명한다.

스프레트시트의 가장 기본적인 컨셉은 다음과 같다.
* 스프레트시트는 셀이란 단위로 구성된 2차원 격자로서, 각 셀은 숫자나 스트링을 담을 수 있다.
* 스프레트시트 프로그램은 일정한 영역의 셀에 담긴 값을 이용하여 여러 가지 수학 연산을 제공한다.

스프레트시트 예제 애플리케이션은 `Spreadsheet` 클래스와 `SpreadsheetCell`이란 기본 클래스를 사용한다. `Spreadsheet` 객체마다 `SpreadsheetCell` 객체를 가진다.

`Spreadsheet` 객체들을 관리하는 `SpreadsheetApplication`이란 클래스도 정의한다.