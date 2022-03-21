---
sort: 2
---

# Inheritance For Reuse

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. WeatherPrediction 클래스

상속의 이해를 위한 일기예보 프로그램 예제이다.

* 온도를 나타내며, 온도 단위로 섭씨와 화씨를 모두 사용한다.
* 날씨 예측을 위해 현재 온도와 목성과 화성 사이의 현재 거리 정보를 기반으로 날씨를 예측하는 서드파티 클래스 라이브러리를 이용한다.

```cpp
class WeatherPrediction
{
    public:
        // virtual 소멸자
        virtual ~WeatherPrediction();
        // 현재 온도를 화씨 단위로 설정한다.
        virtual void setCurrentTempFahrenheit(int temp);
        // 목성과 화성 사이의 현재 거리를 설정한다.
        virtual void setPositionOfJupiter(int distanceFromMars);
        // 내일 온도에 대한 예측값을 가져온다.
        virtual int getTomorrowTempFahrenheit() const;
        // 내일 비가 올 확률을 가져온다.
        // 값이 1이면 확실히 비가 오고, 0이면 비가 오지 않는다는 것을 의미한다.
        virtual double getChanceOfRain() const;
        // 사용자에게 다음과 같은 포맷으로 결과를 출력한다.
        // Result: x.xx chnce. Temp. xx
        virtual void showResult() const;
        // 현재 온도를 스트링값으로 리턴한다.
        virtual std::string getTemperature() const;
    private:
        int mCurrentTempFahrenheit;
        int mDistanceFromMars;
};
```

WeahterPrediction 클래스의 메서드는 모두 virtual로 선언했다. 이 클래스의 메서드를 파생 클래스에서 오버라이드한다고 가정했기 때문이다.

일기예보 프로그램에 필요한 작업은 대부분 이 클래스로 처리하나, 작성할 프로그램은 섭씨로도 표현해야 하며, showResult() 메서드에서 출력하는 결과의 형식이 맞지 않다.

## 2. 파생 클래스에 기능 추가하기

요구 정의사항에 맞게 프로그램을 개발하기 위해 WeatherPrdiction 클래스를 상속하는 MyWeatherPrediction 클래스를 정의한다.

```cpp
#include "WeatherPrediction.h"

class MyWeatherPrediction : public WeatherPrediction
{
};
```

이렇게 정의하면 WeatherPrediction 클래스와 MyWeatherPrediction 클래스는 이름만 다를 뿐 동일하게 사용가능하다.

가장 먼저 MyWeatherPrediction 클래스에 섭씨 단위를 추가할 것이다. 하지만 WeatherPrediction 클래스가 내부적으로 어떻게 작동하는지 모르기 때문에 MyWeatherPrediction 클래스와 WeatherPrediction 클래스를 중계하는 인터페이스를 추가할 것이다.

클라이언트가 현재 온도를 섭씨 단위로 설정하는 메서드와 내일 온도 예측값을 섭씨 단위로 받는 메서드를 추가하며, 섭씨와 화씨를 양방향으로 변환하는 private 헬퍼 메서드도 정의한다.

```cpp
#include "WeatherPrediction.h"

class MyWeatherPrediction : public WeatherPrediction
{
    public:
        virtual void setCurrentTempCelsius(int temp);
        virtual int getTomorrowTempCelsius() const;
    private:
        static int convertCelsiusToFahrenheit(int celsius);
        static int convertFahrenheitToCelsius(int fahrenheit);
};
```

여기서 변환 메서드의 구현 코드는 이 클래스 객체에서 동일하기 때문에 이 메서드를 static으로 지정한다.

> 새로 정의한 메서드를 부모 클래스의 명명 규칙을 그대로 따르게 함으로써 인터페이스를 일관성 있게 유지할 수 있게 할 수 있다.

현재 온도를 섭씨 단위로 설정하려면 현재 온도를 부모 클래스가 이애할 수 있는 단위로 변환해서 전달해야 한다.

```cpp
void MyWeahterPrediction::setCurrentTempCelsius(int temp)
{
    int fahrenheitTemp = convertCelsiusToFahrenheit(temp);
    setCurrentTempFahrenheit(fahrenheitTemp);
}
```

> 베이스 클래스의 메서드를 그대로 호출할 수 있음을 보여준다.

마찬가지로 getTomorrowTempCelsius() 구현 코드에서도 부모 클래스의 기능을 이용하여 현재 온도를 화씨 단위로 가져와서 섭씨로 변환하여 리턴한다.

```cpp
int MyWeatherPrediction::getTomorrowTempCelsius() const
{
    int fahrenheitTemp = getTomorrowTempFahrenheit();
    return convertFahrenheitToCelsius(fahrenheitTemp);
}
```

위 두 메서드는 부모 클래스의 코드를 재사용하고 있다. 실제 동작은 기존 메서드로 처리하고, 새 인터페이스는 기존 메서드를 감싸기만 하였다.

## 3. 파생 클래스에서 기존 기능 변경하기

상속의 또 다른 목적은 기존 기능을 변경하는 데 있다. showResult() 메서드를 통한 출력 결과를 다음과 같이 재정의하여 수정하고자 한다.

```cpp
class MyWeatherPrediction : public WeatherPrediction
{
    public:
        virtual void setCurrentTempCelsius(int temp);
        virtual int getTomorrowTempCelsius() const;
        virtual void showResult() const override;
    private:
        static int convertCelsiusToFahrenheit(int celsius);
        static int convertFahrenheitToCelsius(int fahrenheit);
};

void MyWeatherPrediction::showResult() const
{
    cout << "Tomorrow's temperature will be " <<
            getTomorrowTempCelsius() << " degrees Celsius (" <<
            getTomorrowTempFahrenheit() << " degrees Fahrenheit)" << endl;
    cout << "Chance of rain is " << (getChanceOfRain() * 100) << " percent" << endl;
    if(getChanceOfRain() > 0.5) {
        cout << "Bring an umbrella!" << endl;
    }
}
```

> showResult() 메서드를 재 정의함으로써 MyWeatherPrediction 객체는 재정의된 메서드를 호출하게 된다.