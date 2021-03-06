---
sort: 1
---

# The Object-Oriented Philosophy

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

절차형 접근 방식이 '이 프로그램이 무슨 일을 하는가?' 라는 질문을 토대로 접근하는데 반해, 객체지향 접근 방식은 '현실세계의 어떤 대상을 모델링하는가?'라는 관점에서 접근한다.

다소 추상적일 수 있는 개념을 물리적인 대상을 클래스, 컴포넌트, 프로퍼티(속성), 동작의 관점에서 분석을 한다.

## 1. 클래스

클래스란 개념을 적용하면 구체적인 대상과 그 대상에 대한 정의를 구분할 수 있다.

예를 들어 누가 '오렌지가 뭐냐'고 물을 때 '오렌지는 나무에서 자라고 주황색을 띄고 독특한 향과 맛을 내는 과일의 한종류'라고 답할 수 있다. 이처럼 클래스는 **어떤 대상의 유형을 정의하는 속성을 정리한 것**이다.

반면 어떤 특정한 오렌지를 가리킬 때는 구체적인 대상(객체)을 의미한다. 모든 객체마다 속하는 클래스가 있다.

내 책상에 있는 주황색 과일이란 객체는 오렌지의 한 종류이므로 오렌지 클래스에 속한다고 말할 수 있다.

객체란 어떤 클래스에 속하는 구체적인 인스턴스(사례)다. 다시 말해 같은 클래스에 속한 객체라도 서로 뚜렷이 구분되는 특징을 가진다.

## 2. 컴포넌트

절차형 프로그래밍에서 복잡한 작업을 프로시저(절차) 단위로 쪼개는 것이 중요한 것처럼 OOP에서는 객체를 작은 컴포넌트 단위로 구분하는 사고방식이 굉장히 중요하다.

컴포넌트는 본질적으로 클래스와 같다. 클래스보다 작고 구체적이라는 점만 다르다. 클래스는 여러개의 작고 관리하기 쉬운 컴포넌트 단위로 나눈다. 각각의 컴포넌트는 좀 더 세분화해서 하위 컴포넌트를 구성할 수 있다.

## 3. 프로퍼티

객체는 프로퍼티(속성)로 구분한다. 예를 들어 오렌지 클래스를 설명할 때 주황색을 띠고 독특한 맛을 내는 과일이라고 정의했을 때 주황색과 독특한 맛이란 두 가지 특징이 바로 프로퍼티다. 오렌지를 구성하는 프로퍼티는 같지만 구체적인 값은 다를 수 있다.

프로퍼티를 클래스 관점에서도 볼 수 있다. 오렌지는 일종의 과일이고 모두 나무에서 자란다. 이러한 프로퍼티는 과일이란 클래스도 가지지만 오렌지가 가지는 독특한 주황색이란 속성을 통해 다른 과일 객체와 구분된다.

이처럼 프로퍼티는 객체의 특성을 표현하며, 객체를 구분짓는 특성이 된다.

## 4. 동작

동작은 객체가 하는 일 또는 그 객체로 할 수 있는 일을 표현한다. OOP에서는 여러 가지 동작을 수행하는 클래스를 만들고 서로 상호작용하는 방식을 정의함으로써 데이터를 조작하는 코드를 훨씬 다양하게 제공할 수 있다. 이러한 클래스의 동작은 클레스 메서드로 구현한다.

## 5. OOP 디자인 예시

주식종목 추천 프로그램에 대한 OOP 방식의 디자인

1. '주식시세'에 대한 클래스 정의 : 시세 정보를 수집하려면 각각의 주식시세를 그룹 단위로 묶어야 하므로 '주식시세 컬렉션'을 표현하는 클래스를 먼저 정의하고, 그 안에 한 종목의 '주식 시세'를 표현하는 작은 컴포넌트를 담는다.

2. 프로퍼티 정의 : 주식시세 컬렉션 클래스는 최소한 수집한 시세 목록이란 프로퍼티를 가져야 한다. 가장 최근에 조회한 날짜와 시각에 대한 프로퍼티를 추가한다.

3. 동작 정의 : '시세 정보 가져오기'라는 동작을 정의하며, 이는 '주식시세 컬렉션'이 서버와 통신해서 시세 정보를 가져오고 이를 정렬된 리스트로 제공하도록 정의한다.

4. 그 이외의 프로퍼티/동작 정의 : 회사 이름, 종목 번호, 현재 가격과 같은 속성을 가질 수 있고, 분석, 주식 매도/매수와 같은 동작을 정의할 수도 있다.

UML 클래스 다이어그램으로 표현하면 다음과 같을 것이다. `StockQuoteCollection`은 0개 이상(0..*)의 `StockQuote` 객체를 가지고, `StockQuote` 객체는 단 하나(1)의 `StockQuoteCollection`에 속한다.

<img src="OOPExample.png"  width="280" height="500">
