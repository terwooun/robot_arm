# robot_arm — 조원 수정본 V10 통합

2026-10-21에 사용자가 제공한 다섯 Arduino 소스 파일을 **원본 그대로** 반영했습니다.
이번 통합에서는 추가 증속이나 위치 변경을 하지 않았습니다.
조원 원본 설명은 [README.txt](README.txt), 이전 버전 기록은
[이전 README](docs/README-before-teammate.md)에 있습니다. 이전 문서의 속도와 정지 정책은 현재 버전에 적용하지 마세요.

## 업로드와 실행

`robot_arm.ino`, `xy_control.h`, `xy_motion.h`, `z_homing.h`, `battery_approach.h`를
같은 `robot_arm` 폴더에 두고 Arduino Mega 2560으로 업로드합니다.
Wire와 Servo 라이브러리를 사용합니다. 시리얼 모니터는 **115200bps, 줄바꿈**입니다.

사진의 수동 시작 자세에서 `s`로 기준 등록 후 `run`을 입력합니다.
`run`은 처음 Z 호밍을 실행하며, 기본값은 `!` 입력 전까지 반복입니다.
`!` 또는 경로/센서/추종 오류로 멈추면 임의 위치에서 자동 재개하지 않습니다.
재시작할 때는 팔과 건전지를 초기 배치로 복구하고 기준을 다시 등록합니다.

## 현재 설정과 명령

인수 없이 입력하면 조회만 합니다. 설정은 재부팅하면 기본값으로 돌아갑니다.

| 명령 | 부팅 기본값 | 설정 가능한 값 / 의미 |
|---|---:|---|
| `speed` | 5 | `speed 1 / 2 / 3 / 4 / 5 / 10`; XY와 일반 Z 이동 배율 |
| `loadspeed` | 1.5 | `loadspeed 1 / 1.5 / 2`; 집게 닫힘 상태 이동의 배율 상한 |
| `lastlift` | 0.75 | `lastlift 0.5 / 0.75 / 1`; 마지막 첫 건전지 반환 전 상승만 별도 제한 |
| `diagboost` | 1 | `diagboost 1 / 2 / 3 / 5`; 첫 above 동작만 추가 배율, 합산 최대 15 |
| `cycles` | 0 | 0은 연속 반복, 1..99는 지정 횟수 후 빈 팔 시작 자세 복귀 |

기준 XY 속도는 20mm/s, 가속도는 12mm/s²입니다. 속도는 배율에 비례,
가속도는 배율의 제곱에 비례합니다. 따라서 기본 speed 5에서 빈 팔 목표는
100mm/s·300mm/s²입니다. 일반 부하 이동은 최대 30mm/s,
마지막 상승은 최대 15mm/s이며 실제 속도는 펄스·계산·기구 조건에 따라 달라집니다.
Z 호밍 속도는 그대로지만 일반 Z/XYZ 속도에는 배율이 적용됩니다.
`cycles 1` 완료는 팔의 복귀이며 건전지의 최초/반환 좌표는 약간 다릅니다.

기존 `pos`, `e`, `h`, `homez`, `o`, `c`, `angle`, `xm`, `ym`, `xy`,
`xycheck`, `xycal`, `xyhelp`, `above`, `pick`, `place`, `pickinfo`도 유지됩니다.
X/Y/Z 기어비는 확정값 32/9이며 Z AS5600은 모터축을 측정합니다.

## 변경된 검사와 대기

자동 직선 XY는 이동 중 축 추종 오차가 1°를 초과하면 정지합니다.
직선 구간 종료 후 손목 위치 오차가 1mm를 초과해도 다음 단계로 진행하지 않습니다.
XYZ 동시 이동의 XY 오차 검사는 이동 후 수행되며 속도에 따라 정지 조건이 다릅니다.
집게 열기/닫기 대기는 각각 120/160ms입니다. 실제 파지 센서는 없습니다.

## 검증

호스트 테스트는 모의 엔코더/시리얼/펄스를 이용합니다. 실물 성능 보증이 아닙니다.

```sh
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/xy_motion_test.cpp -o /tmp/xy_math && /tmp/xy_math
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/xy_control_test.cpp -o /tmp/xy_test && /tmp/xy_test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/battery_approach_test.cpp -o /tmp/battery_test && /tmp/battery_test
g++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/z_homing_test.cpp -o /tmp/z_test && /tmp/z_test
```

통합 시 실제 Mega 2560 컴파일도 확인합니다. 처리량 목표는 3분에 10개에서 20개이며,
현재 작동 설정을 먼저 확인한 뒤 별도 변경으로 단계적으로 증속할 예정입니다.
