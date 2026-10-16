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
    ++pulses;position+=dir==HIGH?1:-1;
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
  Serial.stop=false;Serial.output.clear();
}
void expectFailure(){
  homeZ();assert(pul==LOW);assert(zEncoderCount==99);
  assert(Serial.output.find("Z HOME ERROR")!=std::string::npos);
  assert(Serial.output.find("Z HOME DONE")==std::string::npos);
}
int main(){
  reset();homeZ();assert(zEncoderCount==0 && zLastRaw==321);
  assert(position==0 && pul==LOW && firstDirection==LOW);
  assert(Serial.output.find("SLOW SEEK")!=std::string::npos);
  assert(Serial.output.find("Z HOME DONE")!=std::string::npos);
  reset(-5);homeZ();assert(firstDirection==HIGH && position==0 && zEncoderCount==0);
  reset();mode=1;expectFailure();assert(pulses==lround(5*PULSE_PER_DEGREE));
  reset();mode=2;expectFailure();assert(pulses==lround(30*PULSE_PER_DEGREE));
  reset();mode=3;expectFailure();assert(position==0);
  assert(Serial.output.find("switch did not stay stable")!=std::string::npos);
  reset();mode=4;homeZ();assert(position==0 && zEncoderCount==0 && pul==LOW);
  assert(Serial.output.find("Z HOME DONE")!=std::string::npos);
  reset();mode=5;homeZ();assert(position==0 && zEncoderCount==0 && pul==LOW);
  assert(Serial.output.find("Z HOME DONE")!=std::string::npos);
  reset();sensorOK=false;expectFailure();assert(pulses==0);
  assert(Serial.output.find("AS5600 communication failed")!=std::string::npos);
  reset();failSensorOnPulse=true;expectFailure();assert(pulses==1);
  reset();finalReadOK=false;expectFailure();assert(position==0);
  reset();stopOnPulse=true;expectFailure();assert(pulses==1);
  reset();forceTimeout=true;expectFailure();assert(pulses==1);
  reset();Serial.stop=true;expectFailure();assert(pulses==0);
  std::cout<<"Z homing: normal, initially pressed, stuck contacts, bounce, sensor, stop, timeout passed\n";
}
