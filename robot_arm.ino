#include <Wire.h>
#include <Servo.h>

Servo gripper;
const int GRIPPER_PIN = 10;
const int OPEN_ANGLE = 100;
const int CLOSE_ANGLE = 20;

// ======================================================
// 3-axis robotic arm with X/Y encoders and Z AS5600
//
// Y axis (first joint)
// D4  -> PUL+
// D5  -> DIR+
// D18 -> Encoder A
// D19 -> Encoder B
//
// X axis (second joint)
// D6 -> PUL+
// D7 -> DIR+
// D2 -> Encoder A
// D3 -> Encoder B
//
// Z axis (third joint)
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
// Serial commands:
// x 30
// y -10
// z 20
//
// e: show X/Y/Z encoder positions
// zero: reset the current encoder references
// h: show X/Y/Z home switch inputs without motion
// homez: seek Z switch, back off by 5 degrees; ! to interrupt
// o / open: open gripper (D10, 100 degrees)
// c / close: close gripper (20 degrees)
// angle 60: set gripper angle (0 to 180 degrees)
// xyhelp: help for XY motion and pose calibration
// xm 1 / ym 1: move forward / upward by 1 mm after XY calibration
// xy 1 1: simultaneous 1-mm forward and upward move
// Z axis uses the existing z angle command
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

// NC wiring: COM to GND, NC to input
// LOW = released, HIGH = pressed or disconnected
const int X_HOME = 22;
const int Y_HOME = 23;
const int Z_HOME = 24;


// ======================================================
// X / Y Encoder Pin
// ======================================================

const int Y_ENC_A = 18;
const int Y_ENC_B = 19;

// AS5600 uses pins D20/D21
// X encoder uses pins D2/D3
const int X_ENC_A = 2;
const int X_ENC_B = 3;


// ======================================================
// AS5600
// ======================================================

const byte AS5600_ADDR = 0x36;

const long AS5600_CPR = 4096;

// Accumulated Z encoder counts
long zEncoderCount = 0;

uint16_t zLastRaw = 0;

bool zEncoderOK = false;


// ======================================================
// Motor Setting
// ======================================================

const long PULSES_PER_REV = 3200;

// X/Y Encoder
// 1000 PPR x 4 = 4000 CPR
const long ENCODER_CPR = 4000;

// Confirmed X/Y/Z gear ratio from physical teeth count
// Motor 9T -> Robot 32T
const double GEAR_RATIO = 32.0 / 9.0;


// Command pulses per degree at joint output
const double PULSE_PER_DEGREE =
  PULSES_PER_REV *
  GEAR_RATIO /
  360.0;


// X/Y encoder counts per output degree
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


// X/Y encoder direction signs
const int Y_ENC_SIGN = -1;
const int X_ENC_SIGN = -1;


// Z AS5600 encoder sign
// Measured z +5 produces raw change of -202 counts; align the signs.
// Do not modify motor direction or homing direction.
const int Z_ENC_SIGN = -1;


// ======================================================
// X/Y Encoder Count
// ======================================================

volatile long yEncoderCount = 0;
volatile long xEncoderCount = 0;

volatile uint8_t yPrevState = 0;
volatile uint8_t xPrevState = 0;


// Quadrature encoder state transition table
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

#include "xy_control.h"


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
// Read AS5600 raw angle
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
// Update accumulated Z encoder count
//
// AS5600 wraps from 4095 to 0.
// Convert the raw reading into a signed accumulated count.
// ======================================================

void updateZEncoder()
{
  uint16_t raw;

  if (!readAS5600Raw(raw))
  {
    zEncoderOK = false;
    return;
  }


  // First valid reading
  if (!zEncoderOK)
  {
    zLastRaw = raw;
    zEncoderOK = true;
    return;
  }


  int difference =
    (int)raw -
    (int)zLastRaw;


  // Handle 4095 to 0 wraparound
  if (difference > 2048)
  {
    difference -= 4096;
  }

  // Handle 0 to 4095 wraparound
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

#include "z_homing.h"
#include "battery_approach.h"

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
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000UL, true);
#endif

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

  // Faster 4x command scale (previous default v4 was 3x).
  // Return to 'speed 1' if there is vibration, missed steps or grip slip.
  robotSpeedScale = 5.0f;
  Serial.begin(115200);
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
  Serial.println(F("homez = Z homing toward negative direction; ! = stop during homing"));
  Serial.println("o / open  = Gripper Open (100 DEG)");
  Serial.println("c / close = Gripper Close (20 DEG)");
  Serial.println("angle 60  = Gripper Angle (0 ~ 180 DEG)");
  Serial.println(F("xyhelp = XY mm motion (requires measured pose calibration)"));


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
  Serial.println(F("speed / speed 1 / speed 2 / speed 3 / speed 4 / speed 5 / speed 10"));
  Serial.println(F("diagboost / diagboost 1 / diagboost 2 / diagboost 3 / diagboost 5 (OPT-IN)"));
  Serial.println(F("loadspeed / loadspeed 1 / loadspeed 1.5 / loadspeed 2 (loaded motion only)"));
  Serial.println(F("cycles 0=repeat forever, cycles 1=single test, cycles 2=two tests"));
  Serial.println(F("s then run; ! stops motion (reset pose after stop)"));
  Serial.println("Enter command:");
}


// ======================================================
// LOOP
// ======================================================

void loop()
{
  // Keep checking Z encoder when idle
  updateZEncoder();


  if (Serial.available() > 0)
  {
    String input =
      Serial.readStringUntil('\n');

    input.trim();
    input.toLowerCase();

    // Speed only: no changes to the taught battery positions or path order.
    if (input == "speed") {
      Serial.print(F("SPEED MULTIPLIER: "));
      Serial.print(robotSpeedScale, 1);
      Serial.println(F("x (commanded, not measured)"));
      return;
    }
    if (input.startsWith("speed ")) {
      String factor = input.substring(6);
      factor.trim();
      if (factor == "1" || factor == "2" || factor == "3" || factor == "4" ||
          factor == "5" || factor == "10") {
        robotSpeedScale = factor.toFloat();
        Serial.print(F("SPEED SET TO "));
        Serial.print(robotSpeedScale, 0);
        Serial.println(F("x; test with NO batteries first"));
      } else {
        Serial.println(F("ERROR: speed 1 / 2 / 3 / 4 / 5 / 10"));
      }
      return;
    }

    // Extra boost only for the first XYZ diagonal after homing, not the homing switch.
    if (input == "diagboost") {
      Serial.print(F("DIAGONAL BOOST: "));
      Serial.print(batteryDiagonalBoost, 0);
      Serial.println(F("x (diagonal only, effective capped at 15x)"));
      return;
    }
    if (input.startsWith("diagboost ")) {
      String factor=input.substring(10);
      factor.trim();
      if (factor == "1" || factor == "2" || factor == "3" || factor == "5") {
        batteryDiagonalBoost=factor.toFloat();
        Serial.print(F("DIAGONAL BOOST SET: "));
        Serial.print(batteryDiagonalBoost, 0);
        Serial.println(F("x; TEST EMPTY AT 1x FIRST"));
      } else Serial.println(F("ERROR: diagboost 1 / 2 / 3 / 5"));
      return;
    }

    if (batteryHandleCommand(input))
    {
      Serial.println("Enter command:");
      return;
    }

    if (input == "homez")
    {
      homeZ();
      Serial.println("Enter command:");
      return;
    }

    if (xyHandleCommand(input))
    {
      Serial.println("Enter command:");
      return;
    }

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
      // Reject nonnumeric input instead of interpreting it as zero degrees
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
      zHomed = false;
      xyInvalidateReference();
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


    // Initial manual-test safety range
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
      xyInvalidateReference();
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
      xyInvalidateReference();
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

    // Negative Z is the confirmed home direction. Positive retreat is allowed.
    if (pulPin == Z_PUL && angle < 0 && digitalRead(Z_HOME) == HIGH)
    {
      digitalWrite(Z_PUL, LOW);
      Serial.println(F("Z HOME LIMIT: negative motion stopped"));
      break;
    }


    // Accelerate
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


    // Decelerate
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


    // Cruise at constant pulse period
    else
    {
      pulseDelay =
        FAST_DELAY_US;
    }


    // Speed multiplier applies to manual X/Y/Z pulse profiles as well.
    pulseDelay = (unsigned int)(pulseDelay / robotSpeedScale);

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


    // Continue reading AS5600 during Z motion
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


  // Original documentation comment (English-only source)
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
