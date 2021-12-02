---
sort: 1
---

# Install

## 0. 참고 문헌
*- [CycloneDDS Githug pages](https://github.com/eclipse-cyclonedds/cyclonedds)*

## 1. Install

CycloneDDS를 설치하기위해 [CMake](https://cmake.org/download/) 및 [Bison](https://www.gnu.org/software/bison/)이 설치되어야 한다.

다음으로 cyclone dds git repository에서 clone 한다.

```bash
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
```

## 2. Build

다음으로 clone 받아온 패키지를 cmake를 이용하여 build하는 과정이다.

```bash
cd cyclonedds

mkdir build
cd build

cmake -DBUILD_EXAMPLES=ON ..
cmake --build .
```

위의 과정을 거치면 `~/cyclonedds/build` 안에 빌드되어있는 것을 알 수 있다.