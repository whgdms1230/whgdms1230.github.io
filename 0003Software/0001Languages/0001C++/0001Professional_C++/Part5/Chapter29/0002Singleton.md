---
sort: 1
---

# The Singleton Pattern

* 참고문헌 : 전문가를 위한 C++(마크 그레고리 / 한빛미디어)

## 1. 싱글턴 패턴

싱글턴이라는 단어는 '어떤 개체 하나'를 의미한다. 프로그래밍에서 싱글턴 패턴은 프로그램에 클래스의 인스턴스가 딱 하나만 존재한다는 것을 표현한다.

클래스에 싱글턴 패턴을 적용하면 그 클래스의 인스턴스를 한 개만 만들 수 있다. 또한 객체 하나를 프로그램 전체에서 전역적으로 접근할 수 있다.

싱글턴 패턴의 주의사항은 다음과 같다. 싱글턴이 여러 개라면 프로그램을 구동할 때 원하는 순서대로 초기화하도록 보장하기 힘들다. 또한 프로그램을 종료할 때도 싱글턴이 반드시 있도록 보장하는 것도 힘들다.

또한 싱글턴 클래스는 의존 관계를 가리기 때문에 결합도가 높아지고 단위 테스트가 복잡해진다.

## 2. 예제: 로깅 메커니즘

싱글턴 패턴은 유틸리티 클래스 구현에 적합하다. 따라서 많이 사용되는 클래스인 로거 클래스를 싱글턴으로 구현하고자 한다.

로거 클래스를 싱글턴으로 구현하기에 적합한 이유는 다음과 같다.

* 언제 어디서나 사용할 수 있어야 한다.
* 사용하기 쉬워야 한다.
* 인스턴스가 하나뿐이다.

## 3. 싱글턴 구현 방법

C++로 싱글턴을 구현하는 방법은 두 가지다.

첫 번째 구현 방법은 static 메서드만 가지는 클래스로 구현하는 것이다. 이렇게 구현한 클래스는 인스턴스를 만들 수 없고 어디서나 사용할 수 있다. 이 방법의 단점은 생성과 삭제 기능이 부족하다는 것이다. 하지만 static 메서드로만 구성된 클래스는 엄밀히 말해서 싱글턴이 아니라 static 클래스다. 이는 완전한 싱글턴이라고 표현하기 어렵다.

두 번째 구현 방법은 접근 제어 메커니즘을 이용하여 클래스의 인스턴스를 하나만 생성하고 접근하게 만드는 것이다. 이는 싱글턴의 의미를 제대로 살린 방식이다.

Logger 클래스의 기능을 정의하면 다음과 같다.

* 스트링 하나 또는 여러 스트링으로 구성된 벡터 하나를 로그에 남긴다.
* 로그 메시지마다 로그 수준을 정해서 메시지 앞에 붙인다.
* 특정한 로그 수준에 대해서만 메시지를 남기도록 로거를 설정할 수 있다.
* 로그 메시지가 곧바로 파일에 저장되도록 메시지를 모두 디스크로 보낸다.

정의된 기능을 바탕으로 작성한 Logger 클래스의 정의 코드는 다음과 같다.

```cpp
// 싱글턴 Logger 클래스의 정의
class Logger final
{
    public:
        enum class LogLevel{
            Error,
            Info,
            Debug
        };

        // 싱글턴 Logger 객체에 대한 레퍼런스를 리턴한다.
        static Logger& instance();

        // 복제와 이동 생성을 막는다.
        Logger(const Logger&) = delete;
        Logger(Logger&&) = delete;

        // 복제 대입과 이동 대입 연산을 막는다.
        Logger& operator=(const Logger&) = delete;
        Logger& operator=(Logger&&) = delete;

        // 로그 수준을 설정한다.
        void setLogLevel(LogLevel level);

        // 지정된 로그 수준으로 스트링으로 표현한 메시지 하나를 로그에 남긴다.
        void log(std::string_view message, LogLevel logLevel);

        // 지정된 로그 수준으로 벡터에 담긴 여러 메시지를 로그에 남긴다.
        void log(const std::vector<std::string>& messages, LogLevel logLevel);

    private:
        // private 생성자와 소멸자
        Logger();
        ~Logger();

        // 로그 수준을 사람이 읽을 수 있는 스트링 형태로 변환한다.
        std::string_view getLogLevelString(LogLevel level) const;

        static const char* const kLogFileName;
        std::ofstream mOutputStream;
        LogLevel mLogLevel = LogLevel::Error;
};
```

여기서는 스콧 메이어가 제시한 싱글턴 패턴 방식에 따라 구현한다. 다시 말해 instance() 메서드 안에 Logger 클래스의 로컬 static 인스턴스를 둔다. C++는 이렇게 로컬에 둔 static 인스턴스가 스레드에 안전하게 초기화되도록 보장해준다. 그래서 이렇게 구현하면 스레드 동기화 메커니즘을 따로 구현하지 않아도 된다. 이를 흔히 매직 스태틱이라 부른다. 단, 초기화할때만 스레드에 안전하다는 점에 주의한다. Logger 클래스의 메서드를 여러 스레드가 호출할 때는 각 메서드를 스레드에 안전하게 구현하기 위해 동기화 메커니즘을 적용해야 한다.

Logger 클래스를 구현하는 방법은 간단하다. 로그 파일을 열고 지정된 로그 수준에 따라 로그 메시지들을 파일에 쓰면 된다. 생성자와 소멸자는 instance() 메서드에 있는 Logger 클래스의 static 인스턴스가 생성되고 소멸될 때 자동으로 호출된다. 생성자와 소멸자를 private로 지정했기 때문에 외부에서 Logger 인스턴스를 생성하거나 제거할 수 없다.

구현 코드는 다음과 같다.

```cpp
#include "Logger.h"
#include <stdexcept>

using namespace std;

const char& const Logger::kLogFileName = "log.out"; //*

Logger& Logger::instance()
{
    static Logger instance;
    return instance;
}

Logger::Logger()
{
    mOutputStream.open(kLogFileName, ios_base::app);
    if(!mOutputStream.good()) {
        throw runtime_error("Unable to initialize the Logger!");
    }
}

Logger::~Logger()
{
    mOutputStream << "Logger shutting down." << endl;
    mOutputStream.close();
}

void Logger::setLogLevel(LogLevel level)
{
    mLogLevel = level;
}

string_view Logger::getLogLevelString(LogLevel level) const
{
    switch(level){
        case LogLevel::Error:
            return "ERROR";
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Debug;
            return "DEBUG";
    }
    throw runtime_error("Invalid log level.");
}

void Logger::log(string_view message, LogLevel logLevel)
{
    if(mLogLevel < logLevel) {
        return;
    }

    mOutputStream << getLogLevelString(logLevel).data() << ": " << message << endl;
}

void Logger::log(const vector<string>& messages, LogLevel)
{
    if(mLogLevel < logLevel){
        return;
    }

    for(const auto& message : messages){
        log(message, logLevel);
    }
}
```

## 4. 싱글턴 사용 방법

싱글턴으로 정의한 Logger 클래스를 사용하는 코드는 다음과 같다.

```cpp
// 로그 수준을 디버그로 설정한다.
Logger::instance().setLogLevel(Logger::LogLevel::Debug);

// 로그 메시지를 남긴다.
Logger::instance().log("test message", Logger::LogLevel::Debug);
vector<string> items = {"item1", "item2"};
Logger::instance().log(items, Logger::LogLevel::Error);

// 로그 수준을 에러로 변경한다.
Logger::instance().setLogLevel(Logger::LogLevel::Error);
// 여기서부터 로그 수준이 Error로 바뀌므로 Debug 수준으로 지정된 메시지는 무시한다.
Logger::instance().log("A debug message", Logger::LogLevel::Debug);
```

