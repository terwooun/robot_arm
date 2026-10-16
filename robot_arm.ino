#include <Wire.h>
#include <Servo.h>

Servo gripper;
const int GRIPPER_PIN = 10;
const int OPEN_ANGLE = 100;
const int CLOSE_ANGLE = 20;

// ======================================================
// 3축 로봇팔 + X/Y Encoder + Z AS5600
//
// Y축(1축)
// D4  -> PUL+
// D5  -> DIR+
// D18 -> Encoder A
// D19 -> Encoder B
//
// X축(2축)
// D6 -> PUL+
// D7 -> DIR+
// D2 -> Encoder A
// D3 -> Encoder B
//
// Z축(3축)
// D8 -> PUL+
// D9 -> DIR+
//
// AS5600
// VCC -> 5V
// GND -> GND
// SDA -> D20
// SCL -> D21
// DIR -> GND
//
// 입력:
// x 30
// y -10
// z 20
//
// e    : X/Y/Z 현재 엔코더 위치
// zero : 현재 위치를 0점으로 설정
// h    : X/Y/Z 홈 스위치 입력 확인 (모터 이동 없음)
// o / open : 그리퍼 열기 (D10, 100도)
// c / close : 그리퍼 닫기 (20도)
// angle 60 : 그리퍼 각도 지정 (0~180도)
// ======================================================


// ======================================================
// Motor Pin
// ======================================================

const int Y_PUL = 4;
const int Y_DIR = 5;

const int X_PUL = 6;
const int X_DIR = 7;

const int Z_PUL = 8;
const int Z_DIR = 9;

// NC 배선: COM -> GND, NC -> 입력핀
// LOW = 눌리지 않음, HIGH = 눌림 또는 배선 단선
const int X_HOME = 22;
const int Y_HOME = 23;
const int Z_HOME = 24;


// ======================================================
// X / Y Encoder Pin
// ======================================================

const int Y_ENC_A = 18;
const int Y_ENC_B = 19;

// AS5600이 D20/D21을 사용하므로
// X Encoder는 D2/D3 사용
const int X_ENC_A = 2;
const int X_ENC_B = 3;


// ======================================================
// AS5600
// ======================================================

const byte AS5600_ADDR = 0x36;

const long AS5600_CPR = 4096;

// Z축 누적 엔코더 카운트
long zEncoderCount = 0;

uint16_t zLastRaw = 0;

bool zEncoderOK = false;


// ======================================================
// Motor Setting
// ======================================================

const long PULSES_PER_REV = 3200;

// X/Y Encoder
// 1000 PPR × 4 = 4000 CPR
const long ENCODER_CPR = 4000;

// X/Y/Z 공통 확정 기어비 (실물 확인)
// Motor 9T -> Robot 32T
const double GEAR_RATIO = 32.0 / 9.0;


// 관절 출력축 1도당 명령 Pulse
const double PULSE_PER_DEGREE =
  PULSES_PER_REV *
  GEAR_RATIO /
  360.0;


// X/Y 관절 1도당 Encoder Count
const double ENC_COUNT_PER_DEGREE =
  ENCODER_CPR *
  GEAR_RATIO /
  360.0;


// ======================================================
// Direction
// ======================================================

const bool Y_DIR_INVERT = false;
const bool X_DIR_INVERT = false;
const bool Z_DIR_INVERT = false;


// X/Y Encoder 방향
const int Y_ENC_SIGN = -1;
const int X_ENC_SIGN = -1;


// Z AS5600 방향
// z 10 명령 시 실제각이 -10 정도 나오면
// 1 -> -1 로 변경
const int Z_ENC_SIGN = 1;


// ======================================================
// X/Y Encoder Count
// ======================================================

volatile long yEncoderCount = 0;
volatile long xEncoderCount = 0;

volatile uint8_t yPrevState = 0;
volatile uint8_t xPrevState = 0;


// Quadrature Encoder 상태표
const int8_t QUAD_TABLE[16] =
{
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};


// ======================================================
// Speed
// ======================================================

const unsigned int START_DELAY_US = 1500;
const unsigned int FAST_DELAY_US  = 900;

const long ACCEL_PULSES = 300;


// ======================================================
// Y Encoder ISR
// ======================================================

void updateYEncoder()
{
  uint8_t currentState =
    (digitalRead(Y_ENC_A) << 1)
    |
    digitalRead(Y_ENC_B);

  uint8_t index =
    (yPrevState << 2)
    |
    currentState;

  yEncoderCount += QUAD_TABLE[index];

  yPrevState = currentState;
}


// ======================================================
// X Encoder ISR
// ======================================================

void updateXEncoder()
{
  uint8_t currentState =
    (digitalRead(X_ENC_A) << 1)
    |
    digitalRead(X_ENC_B);

  uint8_t index =
    (xPrevState << 2)
    |
    currentState;

  xEncoderCount += QUAD_TABLE[index];

  xPrevState = currentState;
}


// ======================================================
// AS5600 RAW 값 읽기
// ======================================================

bool readAS5600Raw(uint16_t &raw)
{
  Wire.beginTransmission(AS5600_ADDR);

  // RAW ANGLE High Byte
  Wire.write(0x0C);

  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  if (
    Wire.requestFrom(
      AS5600_ADDR,
      (byte)2
    ) != 2
  )
  {
    return false;
  }

  uint16_t highByte = Wire.read();
  uint16_t lowByte  = Wire.read();

  raw =
    ((highByte << 8) | lowByte)
    &
    0x0FFF;

  return true;
}


// ======================================================
// Z Encoder 누적값 갱신
//
// AS5600은 0~4095 후 다시 0으로 돌아가므로
// 회전 방향을 판단하여 누적값으로 변환
// ======================================================

void updateZEncoder()
{
  uint16_t raw;

  if (!readAS5600Raw(raw))
  {
    zEncoderOK = false;
    return;
  }


  // 처음 읽었을 때
  if (!zEncoderOK)
  {
    zLastRaw = raw;
    zEncoderOK = true;
    return;
  }


  int difference =
    (int)raw -
    (int)zLastRaw;


  // 4095 -> 0 넘어갈 때
  if (difference > 2048)
  {
    difference -= 4096;
  }

  // 0 -> 4095 넘어갈 때
  else if (difference < -2048)
  {
    difference += 4096;
  }


  zEncoderCount += difference;

  zLastRaw = raw;
}


// ======================================================
// SETUP
// ======================================================

void setup()
{
  gripper.attach(GRIPPER_PIN);
  gripper.write(OPEN_ANGLE);

  pinMode(X_HOME, INPUT_PULLUP);
  pinMode(Y_HOME, INPUT_PULLUP);
  pinMode(Z_HOME, INPUT_PULLUP);

  // ----------------------
  // Motor
  // ----------------------

  pinMode(Y_PUL, OUTPUT);
  pinMode(Y_DIR, OUTPUT);

  pinMode(X_PUL, OUTPUT);
  pinMode(X_DIR, OUTPUT);

  pinMode(Z_PUL, OUTPUT);
  pinMode(Z_DIR, OUTPUT);


  digitalWrite(Y_PUL, LOW);
  digitalWrite(X_PUL, LOW);
  digitalWrite(Z_PUL, LOW);


  // ----------------------
  // X/Y Encoder
  // ----------------------

  pinMode(Y_ENC_A, INPUT);
  pinMode(Y_ENC_B, INPUT);

  pinMode(X_ENC_A, INPUT);
  pinMode(X_ENC_B, INPUT);


  yPrevState =
    (digitalRead(Y_ENC_A) << 1)
    |
    digitalRead(Y_ENC_B);


  xPrevState =
    (digitalRead(X_ENC_A) << 1)
    |
    digitalRead(X_ENC_B);


  attachInterrupt(
    digitalPinToInterrupt(Y_ENC_A),
    updateYEncoder,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(Y_ENC_B),
    updateYEncoder,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(X_ENC_A),
    updateXEncoder,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(X_ENC_B),
    updateXEncoder,
    CHANGE
  );


  // ----------------------
  // AS5600
  // ----------------------

  Wire.begin();

  delay(100);


  uint16_t raw;

  if (readAS5600Raw(raw))
  {
    zLastRaw = raw;
    zEncoderOK = true;
  }


  // ----------------------
  // Serial
  // ----------------------

  Serial.begin(9600);
  Serial.setTimeout(100);

  delay(1000);


  Serial.println();
  Serial.println("==============================");
  Serial.println("3 AXIS + X/Y/Z ENCODER");
  Serial.println("==============================");

  Serial.println();
  Serial.println("x 30");
  Serial.println("y -15");
  Serial.println("z 20");

  Serial.println();
  Serial.println("e    = Encoder Check");
  Serial.println("zero = Encoder Zero");
  Serial.println("h    = Home Switch Check (NO MOTION)");
  Serial.println("o / open  = Gripper Open (100 DEG)");
  Serial.println("c / close = Gripper Close (20 DEG)");
  Serial.println("angle 60  = Gripper Angle (0 ~ 180 DEG)");


  if (zEncoderOK)
  {
    Serial.println();
    Serial.println("AS5600 : CONNECTED");
  }

  else
  {
    Serial.println();
    Serial.println("AS5600 : NOT FOUND");
  }


  Serial.println();
  Serial.println("Enter command:");
}


// ======================================================
// LOOP
// ======================================================

void loop()
{
  // 정지 상태에서도 Z Encoder 계속 확인
  updateZEncoder();


  if (Serial.available() > 0)
  {
    String input =
      Serial.readStringUntil('\n');

    input.trim();
    input.toLowerCase();

    if (input == "o" || input == "open")
    {
      gripper.write(OPEN_ANGLE);
      Serial.print("GRIPPER OPEN : ");
      Serial.print(OPEN_ANGLE);
      Serial.println(" DEG");
      Serial.println("Enter command:");
      return;
    }

    if (input == "c" || input == "close")
    {
      gripper.write(CLOSE_ANGLE);
      Serial.print("GRIPPER CLOSE : ");
      Serial.print(CLOSE_ANGLE);
      Serial.println(" DEG");
      Serial.println("Enter command:");
      return;
    }

    if (input.startsWith("angle "))
    {
      String angleText = input.substring(6);
      angleText.trim();
      bool valid = angleText.length() > 0;
      // 숫자가 아닌 입력을 0도로 해석하여 움직이지 않도록 검사
      for (unsigned int i = 0; i < angleText.length(); i++)
      {
        if (angleText.charAt(i) < '0' || angleText.charAt(i) > '9')
        {
          valid = false;
        }
      }

      long angle = angleText.toInt();
      if (valid && angleText.length() <= 3 && angle <= 180)
      {
        gripper.write((int)angle);
        Serial.print("GRIPPER ANGLE : ");
        Serial.print(angle);
        Serial.println(" DEG");
      }
      else
      {
        Serial.println("ERROR : angle must be an integer from 0 to 180 DEG");
      }
      Serial.println("Enter command:");
      return;
    }

    if (input == "h")
    {
      printHomeSwitches();
      Serial.println();
      Serial.println("Enter command:");
      return;
    }


    // ==================================================
    // Encoder Check
    // ==================================================

    if (input == "e")
    {
      printAllEncoders();

      Serial.println();
      Serial.println("Enter command:");

      return;
    }


    // ==================================================
    // Encoder Zero
    // ==================================================

    if (input == "zero")
    {
      noInterrupts();

      xEncoderCount = 0;
      yEncoderCount = 0;

      interrupts();


      uint16_t raw;

      if (readAS5600Raw(raw))
      {
        zLastRaw = raw;
        zEncoderCount = 0;
        zEncoderOK = true;
      }


      Serial.println();
      Serial.println("X/Y/Z ENCODER ZERO SET");

      Serial.println();
      Serial.println("Enter command:");

      return;
    }


    // ==================================================
    // Command
    // ==================================================

    int spaceIndex =
      input.indexOf(' ');


    if (spaceIndex == -1)
    {
      Serial.println("ERROR");
      Serial.println("Example : z 30");

      return;
    }


    char axis =
      input.charAt(0);


    float angle =
      input.substring(
        spaceIndex + 1
      ).toFloat();


    // 초기 테스트 안전범위
    if (angle < -90 || angle > 90)
    {
      Serial.println("ERROR");
      Serial.println("Range : -90 ~ +90 DEG");

      return;
    }


    // ==================================================
    // X Axis
    // ==================================================

    if (axis == 'x')
    {
      long before =
        readXEncoder();


      moveAxis(
        X_PUL,
        X_DIR,
        angle,
        X_DIR_INVERT,
        false
      );


      delay(200);


      long after =
        readXEncoder();


      printEncoderResult(
        'X',
        angle,
        before,
        after,
        X_ENC_SIGN
      );
    }


    // ==================================================
    // Y Axis
    // ==================================================

    else if (axis == 'y')
    {
      long before =
        readYEncoder();


      moveAxis(
        Y_PUL,
        Y_DIR,
        angle,
        Y_DIR_INVERT,
        false
      );


      delay(200);


      long after =
        readYEncoder();


      printEncoderResult(
        'Y',
        angle,
        before,
        after,
        Y_ENC_SIGN
      );
    }


    // ==================================================
    // Z Axis
    // ==================================================

    else if (axis == 'z')
    {
      updateZEncoder();

      long before =
        zEncoderCount;


      Serial.println();
      Serial.print("Z COMMAND : ");
      Serial.print(angle);
      Serial.println(" DEG");


      moveAxis(
        Z_PUL,
        Z_DIR,
        angle,
        Z_DIR_INVERT,
        true
      );


      delay(200);

      updateZEncoder();


      long after =
        zEncoderCount;


      printZEncoderResult(
        angle,
        before,
        after
      );
    }


    else
    {
      Serial.println("ERROR");
      Serial.println("Axis = X / Y / Z");

      return;
    }


    Serial.println();
    Serial.println("Enter command:");
  }
}


// ======================================================
// Motor Move
// ======================================================

void moveAxis(
  int pulPin,
  int dirPin,
  float angle,
  bool invertDirection,
  bool trackZ
)
{
  if (angle == 0)
  {
    return;
  }


  bool direction;


  if (angle > 0)
  {
    direction = HIGH;
  }

  else
  {
    direction = LOW;
  }


  if (invertDirection)
  {
    direction = !direction;
  }


  digitalWrite(
    dirPin,
    direction
  );


  delay(20);


  long totalPulses =
    round(
      abs(angle)
      *
      PULSE_PER_DEGREE
    );


  Serial.println();

  Serial.print("COMMAND PULSE : ");
  Serial.println(totalPulses);


  long accelPulses =
    ACCEL_PULSES;


  if (
    totalPulses <
    ACCEL_PULSES * 2
  )
  {
    accelPulses =
      totalPulses / 2;
  }


  for (
    long i = 0;
    i < totalPulses;
    i++
  )
  {
    unsigned int pulseDelay;


    // 가속
    if (
      accelPulses > 0
      &&
      i < accelPulses
    )
    {
      pulseDelay =
        START_DELAY_US
        -
        (
          (
            START_DELAY_US
            -
            FAST_DELAY_US
          )
          *
          i
          /
          accelPulses
        );
    }


    // 감속
    else if (
      accelPulses > 0
      &&
      (totalPulses - i)
      <=
      accelPulses
    )
    {
      long remaining =
        totalPulses - i;


      pulseDelay =
        FAST_DELAY_US
        +
        (
          (
            START_DELAY_US
            -
            FAST_DELAY_US
          )
          *
          (
            accelPulses
            -
            remaining
          )
          /
          accelPulses
        );
    }


    // 일정속도
    else
    {
      pulseDelay =
        FAST_DELAY_US;
    }


    digitalWrite(
      pulPin,
      HIGH
    );

    delayMicroseconds(
      pulseDelay
    );


    digitalWrite(
      pulPin,
      LOW
    );

    delayMicroseconds(
      pulseDelay
    );


    // Z축 이동 중 AS5600 값 지속 확인
    if (
      trackZ
      &&
      (i % 10 == 0)
    )
    {
      updateZEncoder();
    }
  }


  if (trackZ)
  {
    updateZEncoder();
  }
}


// ======================================================
// X Encoder
// ======================================================

long readXEncoder()
{
  noInterrupts();

  long value =
    xEncoderCount;

  interrupts();

  return value;
}


// ======================================================
// Y Encoder
// ======================================================

long readYEncoder()
{
  noInterrupts();

  long value =
    yEncoderCount;

  interrupts();

  return value;
}


// ======================================================
// X/Y Encoder Result
// ======================================================

void printEncoderResult(
  char axis,
  float commandAngle,
  long before,
  long after,
  int encoderSign
)
{
  long rawDelta =
    after - before;


  long encoderDelta =
    rawDelta *
    encoderSign;


  double actualAngle =
    encoderDelta
    *
    360.0
    /
    ENCODER_CPR
    /
    GEAR_RATIO;


  double expectedCount =
    commandAngle
    *
    ENC_COUNT_PER_DEGREE;


  double error =
    actualAngle -
    commandAngle;


  Serial.println();
  Serial.println("==============================");

  Serial.print(axis);
  Serial.println(" ENCODER RESULT");

  Serial.println("==============================");


  Serial.print("Command Angle   : ");
  Serial.print(commandAngle, 3);
  Serial.println(" DEG");


  Serial.print("Expected Count  : ");
  Serial.println(expectedCount, 1);


  Serial.print("Encoder Count   : ");
  Serial.println(encoderDelta);


  Serial.print("Actual Angle    : ");
  Serial.print(actualAngle, 3);
  Serial.println(" DEG");


  Serial.print("Error           : ");
  Serial.print(error, 3);
  Serial.println(" DEG");


  Serial.println("==============================");
}


// ======================================================
// Z AS5600 Result
// ======================================================

void printZEncoderResult(
  float commandAngle,
  long before,
  long after
)
{
  long rawDelta =
    after - before;


  long encoderDelta =
    rawDelta *
    Z_ENC_SIGN;


  // AS5600은 모터축에 설치
  // Motor Angle / Gear Ratio = Z Output Angle
  double motorAngle =
    encoderDelta
    *
    360.0
    /
    AS5600_CPR;


  double actualZAngle =
    motorAngle
    /
    GEAR_RATIO;


  double expectedCount =
    commandAngle
    *
    GEAR_RATIO
    *
    AS5600_CPR
    /
    360.0;


  double error =
    actualZAngle -
    commandAngle;


  Serial.println();
  Serial.println("==============================");
  Serial.println("Z AS5600 ENCODER RESULT");
  Serial.println("==============================");


  Serial.print("Command Angle   : ");
  Serial.print(commandAngle, 3);
  Serial.println(" DEG");


  Serial.print("Expected Count  : ");
  Serial.println(expectedCount, 1);


  Serial.print("AS5600 Count    : ");
  Serial.println(encoderDelta);


  Serial.print("Motor Angle     : ");
  Serial.print(motorAngle, 3);
  Serial.println(" DEG");


  Serial.print("Actual Z Angle  : ");
  Serial.print(actualZAngle, 3);
  Serial.println(" DEG");


  Serial.print("Error           : ");
  Serial.print(error, 3);
  Serial.println(" DEG");


  Serial.println("==============================");
}


// ======================================================
// X/Y/Z Home Switch Input Check (no motor movement)
// ======================================================

void printHomeSwitches()
{
  const int xState = digitalRead(X_HOME);
  const int yState = digitalRead(Y_HOME);
  const int zState = digitalRead(Z_HOME);

  Serial.println();
  Serial.println("HOME SWITCH STATUS (NC, NO MOTION)");
  Serial.print("X HOME (D22): ");
  Serial.println(xState == LOW ? "LOW - RELEASED" : "HIGH - PRESSED OR OPEN CIRCUIT");
  Serial.print("Y HOME (D23): ");
  Serial.println(yState == LOW ? "LOW - RELEASED" : "HIGH - PRESSED OR OPEN CIRCUIT");
  Serial.print("Z HOME (D24): ");
  Serial.println(zState == LOW ? "LOW - RELEASED" : "HIGH - PRESSED OR OPEN CIRCUIT");
}


// ======================================================
// X/Y/Z Current Position
// ======================================================

void printAllEncoders()
{
  updateZEncoder();


  long xCount =
    readXEncoder();

  long yCount =
    readYEncoder();


  double xAngle =
    xCount
    *
    X_ENC_SIGN
    *
    360.0
    /
    ENCODER_CPR
    /
    GEAR_RATIO;


  double yAngle =
    yCount
    *
    Y_ENC_SIGN
    *
    360.0
    /
    ENCODER_CPR
    /
    GEAR_RATIO;


  double zMotorAngle =
    zEncoderCount
    *
    Z_ENC_SIGN
    *
    360.0
    /
    AS5600_CPR;


  double zAngle =
    zMotorAngle
    /
    GEAR_RATIO;


  Serial.println();
  Serial.println("==============================");
  Serial.println("ENCODER CURRENT POSITION");
  Serial.println("==============================");


  Serial.print("X Count : ");
  Serial.println(
    xCount * X_ENC_SIGN
  );

  Serial.print("X Angle : ");
  Serial.print(
    xAngle,
    3
  );
  Serial.println(" DEG");


  Serial.println();


  Serial.print("Y Count : ");
  Serial.println(
    yCount * Y_ENC_SIGN
  );

  Serial.print("Y Angle : ");
  Serial.print(
    yAngle,
    3
  );
  Serial.println(" DEG");


  Serial.println();


  Serial.print("Z Count : ");
  Serial.println(
    zEncoderCount * Z_ENC_SIGN
  );

  Serial.print("Z Motor Angle : ");
  Serial.print(
    zMotorAngle,
    3
  );
  Serial.println(" DEG");

  Serial.print("Z Output Angle : ");
  Serial.print(
    zAngle,
    3
  );
  Serial.println(" DEG");


  Serial.println("==============================");
}
