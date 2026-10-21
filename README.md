// Host simulation of switch edges, pulse output and failure paths.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <string>
#include <iostream>
#define F(x) x
#define HIGH 1
#define LOW 0
const int Z_PUL=8, Z_DIR=9, Z_HOME=24;
const bool Z_DIR_INVERT=false;
const double PULSE_PER_DEGREE=3200.0*(32.0/9.0)/360.0;
long zEncoderCount=99;
uint16_t zLastRaw=123;
bool zEncoderOK=true;
unsigned long nowUs=0;
int position=20, pulses=0, dir=LOW, pul=LOW, mode=0, readsAtHome=0;
bool sensorOK=true, finalReadOK=true, failSensorOnPulse=false, stopOnPulse=false, forceTimeout=false;
int firstDirection=-1;
bool positiveSeen=false, negativeAfterPositive=false, stopDuringBackoff=false;
struct SerialMock {
  bool stop=false;
  std::string output;
  int available(){return stop ? 1:0;}
  int read(){stop=false;return '!';}
  void println(const char *s){output+=s;output+='\n';}
} Serial;
unsigned long micros(){nowUs+=100;return nowUs;}
unsigned long millis(){nowUs+=100;return nowUs/1000;}
int digitalRead(int){
  if(mode==6 && dir==HIGH && position<100)return HIGH;
  if(mode==1)return HIGH;
  if(mode==2)return LOW;
  if(mode==3 && position<=0)return (++readsAtHome%2) ? HIGH:LOW;
  if(mode==4 && position<=0 && ++readsAtHome<=10)return readsAtHome%2 ? HIGH:LOW;
  if(mode==5 && position==1 && dir==HIGH && ++readsAtHome<=10)return readsAtHome%2 ? LOW:HIGH;
  return position<=0 ? HIGH:LOW;
}
void digitalWrite(int pin,int value){
  if(pin==Z_DIR){dir=value;return;}
  if(value==HIGH && pul==LOW){
    if(firstDirection<0)firstDirection=dir;
    if(dir==HIGH)positiveSeen=true;
    else if(positiveSeen)negativeAfterPositive=true;
    ++pulses;position+=dir==HIGH?1:-1;
    if(stopDuringBackoff && dir==HIGH)Serial.stop=true;
    if(stopOnPulse)Serial.stop=true;
    if(failSensorOnPulse)sensorOK=false;
    if(forceTimeout)nowUs+=21000000UL;
  }
  pul=value;
}
void updateZEncoder(){zEncoderOK=sensorOK;}
bool readAS5600Raw(uint16_t &raw){raw=321;return sensorOK && finalReadOK;}
#include "../z_homing.h"
void reset(int initial=20){
  position=initial;pulses=0;dir=pul=LOW;mode=0;readsAtHome=0;
  nowUs=0;sensorOK=finalReadOK=true;failSensorOnPulse=stopOnPulse=forceTimeout=false;firstDirection=-1;
  zEncoderCount=99;zLastRaw=123;zEncoderOK=true;
  positiveSeen=negativeAfterPositive=stopDuringBackoff=false;
  Serial.stop=false;Serial.output.clear();
}
void expectFailure(){
  zHomed=true; // A failed new home must invalidate a previous successful home.
  homeZ();assert(pul==LOW);assert(zEncoderCount==99);assert(!zHomed);
  assert(Serial.output.find("Z HOME ERROR")!=std::string::npos);
  assert(Serial.output.find("Z HOME DONE")==std::string::npos);
}
int main(){
  assert(Z_HOME_SEEK_US==2000); // Both seek and backoff use the new 2x pulse rate.
  const long backoff=lround(5*PULSE_PER_DEGREE);
  reset();homeZ();assert(zEncoderCount==0 && zLastRaw==321 && zHomed);
  assert(position==backoff && pul==LOW && firstDirection==LOW);
  assert(!negativeAfterPositive);
  assert(Serial.output.find("SLOW SEEK")==std::string::npos);
  assert(Serial.output.find("Z HOME DONE")!=std::string::npos);
  reset(-5);homeZ();assert(firstDirection==HIGH && position==backoff-5 && zEncoderCount==0);
  assert(!negativeAfterPositive);
  reset();mode=1;expectFailure();assert(pulses==backoff);
  assert(Serial.output.find("still HIGH after +5 DEG")!=std::string::npos);
  reset();mode=2;expectFailure();assert(pulses==lround(30*PULSE_PER_DEGREE));
  // Contact chatter must no longer prevent retreat, including HIGH during it.
  for(int contactMode=3;contactMode<=6;++contactMode){
    reset();mode=contactMode;homeZ();
    assert(position==backoff && zEncoderCount==0 && pul==LOW);
    assert(!negativeAfterPositive);
    assert(Serial.output.find("Z HOME DONE")!=std::string::npos);
  }
  reset(-200);expectFailure();assert(pulses==backoff);
  reset();sensorOK=false;expectFailure();assert(pulses==0);
  assert(Serial.output.find("AS5600 communication failed")!=std::string::npos);
  reset();failSensorOnPulse=true;expectFailure();assert(pulses==1);
  reset();finalReadOK=false;expectFailure();assert(position==backoff);
  reset();stopOnPulse=true;expectFailure();assert(pulses==1);
  reset();stopDuringBackoff=true;expectFailure();assert(position==1);
  reset();forceTimeout=true;expectFailure();assert(pulses==1);
  reset();Serial.stop=true;expectFailure();assert(pulses==0);
  std::cout<<"Z single-touch homing: retreat, chatter, limits, sensor, stop, timeout passed\n";
}


## 최신 속도 설정

직전 버전 대비 자동 동작의 부하 상태별 속도를 변경했습니다.

| 구간 | 직전 | 현재 목표 |
|---|---:|---:|
| 건전지 잡은 상태 XY | 40mm/s | 80mm/s (2배) |
| 빈 집게 XY | 80mm/s | 240mm/s (3배) |
| 잡은 상태 Z 펄스 상한 | 125펄스/s | 250펄스/s (2배) |
| 빈 집게 Z 펄스 상한 | 125펄스/s | 약 375펄스/s (3배) |
| Z 호밍 탐색/후퇴 | 125펄스/s | 250펄스/s (2배) |

기존 가감속 계산 방식을 유지해 XY 가속도는 잡은 상태 96mm/s²,
빈 집게 864mm/s²입니다. 열린 각도 100°만 빈 집게로 분류하고 중간 각도는
잡은 상태 속도를 사용합니다. 이는 파지 센서 판정이 아닌 명령 상태입니다.
수동 조그와 저속 끝점 보정은 그대로이며 자동 운반 및 homez에 적용합니다.
실제 속도는 Mega 계산시간, I2C, 펄스 제한과 가감속 때문에 목표보다 낮을 수 있습니다.
기계의 보장된 최대 속도는 아니므로 첫 실물 실행에서 추종 오차와 진동을 확인합니다.
호밍 탐색 범위·후퇴 거리·타임아웃·스위치 감지와 ! 중지, 작업 위치 보정은 유지합니다.
