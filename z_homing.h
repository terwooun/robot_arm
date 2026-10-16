#ifndef ROBOT_ARM_Z_HOMING_H
#define ROBOT_ARM_Z_HOMING_H

// Confirmed: negative Z approaches the NC switch on D24; positive releases it.
const float Z_HOME_SEARCH_DEG = 30.0f;
const float Z_HOME_RELEASE_DEG = 5.0f;
const float Z_HOME_CLEARANCE_DEG = 1.0f;
const unsigned long Z_HOME_TIMEOUT_MS = 20000UL;
const unsigned long Z_HOME_STABLE_MS = 20UL;
const unsigned long Z_HOME_SETTLE_MS = 250UL;
const unsigned int Z_HOME_SEEK_US = 4000;
const unsigned int Z_HOME_FINE_US = 8000;
unsigned long zHomeStarted = 0;
bool zHomeCancelled = false;
enum ZHomeFailure {
  Z_HOME_OK, Z_HOME_UNSTABLE, Z_HOME_REPRESSED,
  Z_HOME_NOT_FOUND, Z_HOME_NOT_RELEASED, Z_HOME_SENSOR_ERROR
};
ZHomeFailure zHomeFailure = Z_HOME_OK;

bool zHomeCheck()
{
  while (Serial.available())
    if (Serial.read() == '!') zHomeCancelled = true;
  return !zHomeCancelled && (unsigned long)(millis() - zHomeStarted) < Z_HOME_TIMEOUT_MS;
}

// Poll the switch even during pulse delays. A detected edge stops pulses first;
// contact stability is checked afterwards with the motor stationary.
bool zHomeWait(unsigned long duration, int targetState)
{
  const unsigned long start = micros();
  do
  {
    if (!zHomeCheck()) return false;
    if (targetState >= 0 && digitalRead(Z_HOME) == targetState) return false;
  } while ((unsigned long)(micros() - start) < duration);
  return true;
}

bool zHomeStable(int state)
{
  digitalWrite(Z_PUL, LOW);
  const unsigned long start = millis();
  unsigned long stableSince = start;
  bool tracking = false;
  while ((unsigned long)(millis() - start) < Z_HOME_SETTLE_MS)
  {
    if (!zHomeCheck()) return false;
    if (digitalRead(Z_HOME) != state)
    {
      tracking = false;
      continue;
    }
    if (!tracking) { stableSince = millis(); tracking = true; }
    if ((unsigned long)(millis() - stableSince) >= Z_HOME_STABLE_MS) return true;
  }
  zHomeFailure = Z_HOME_UNSTABLE;
  return false;
}

// targetState=-1 means a fixed clearance move, which must remain released.
bool zHomeTravel(bool positive, float maxDegrees, unsigned int halfPeriod, int targetState)
{
  digitalWrite(Z_PUL, LOW);
  digitalWrite(Z_DIR, positive != Z_DIR_INVERT ? HIGH : LOW);
  if (!zHomeWait(20000UL, -1)) return false;
  const long maxPulses = lround(maxDegrees * PULSE_PER_DEGREE);
  for (long i = 0; i <= maxPulses; ++i)
  {
    if (!zHomeCheck()) return false;
    const int state = digitalRead(Z_HOME);
    if (targetState >= 0 && state == targetState) return zHomeStable(targetState);
    if (targetState < 0 && state != LOW) { zHomeFailure = Z_HOME_REPRESSED; return false; }
    if (i == maxPulses)
    {
      if (targetState < 0) return true;
      zHomeFailure = targetState == HIGH ? Z_HOME_NOT_FOUND : Z_HOME_NOT_RELEASED;
      return false;
    }
    digitalWrite(Z_PUL, HIGH);
    const bool keepMoving = zHomeWait(halfPeriod, targetState < 0 ? HIGH : targetState);
    digitalWrite(Z_PUL, LOW);
    if (!keepMoving)
    {
      if (targetState < 0) zHomeFailure = Z_HOME_REPRESSED;
      return targetState >= 0 && zHomeCheck() && zHomeStable(targetState);
    }
    if (!zHomeWait(halfPeriod, targetState < 0 ? HIGH : targetState))
    {
      if (targetState < 0) zHomeFailure = Z_HOME_REPRESSED;
      return targetState >= 0 && zHomeCheck() && zHomeStable(targetState);
    }
    if (i % 10 == 0)
    {
      updateZEncoder();
      if (!zEncoderOK) { zHomeFailure = Z_HOME_SENSOR_ERROR; return false; }
    }
  }
  return false;
}

bool zHomeRelease()
{
  Serial.println(F("Z HOME: RELEASE + (max 5 DEG), then clearance +1 DEG"));
  return zHomeTravel(true, Z_HOME_RELEASE_DEG, Z_HOME_SEEK_US, LOW)
    && zHomeTravel(true, Z_HOME_CLEARANCE_DEG, Z_HOME_SEEK_US, -1);
}

void homeZ()
{
  zHomeStarted = millis();
  zHomeCancelled = false;
  zHomeFailure = Z_HOME_OK;
  digitalWrite(Z_PUL, LOW);
  Serial.println(F("Z HOME: START; ! stops; search max 30 DEG; total timeout 20 s"));
  updateZEncoder();
  if (!zEncoderOK) zHomeFailure = Z_HOME_SENSOR_ERROR;
  bool ok = zEncoderOK && zHomeCheck();
  if (ok && digitalRead(Z_HOME) == HIGH) ok = zHomeRelease();
  if (ok)
  {
    Serial.println(F("Z HOME: SEEK -"));
    ok = zHomeTravel(false, Z_HOME_SEARCH_DEG, Z_HOME_SEEK_US, HIGH);
  }
  if (ok) ok = zHomeRelease();
  if (ok)
  {
    Serial.println(F("Z HOME: SLOW SEEK -"));
    ok = zHomeTravel(false, Z_HOME_RELEASE_DEG + Z_HOME_CLEARANCE_DEG + 1.0f, Z_HOME_FINE_US, HIGH);
  }
  uint16_t raw;
  if (ok)
  {
    ok = readAS5600Raw(raw);
    if (!ok) { zEncoderOK = false; zHomeFailure = Z_HOME_SENSOR_ERROR; }
    else ok = zHomeStable(HIGH);
  }
  digitalWrite(Z_PUL, LOW);
  if (!ok)
  {
    Serial.println(F("Z HOME ERROR: stopped; zero NOT set"));
    if (zHomeCancelled) Serial.println(F("Reason: ! stop"));
    else if ((unsigned long)(millis() - zHomeStarted) >= Z_HOME_TIMEOUT_MS)
      Serial.println(F("Reason: timeout"));
    else
    {
      switch (zHomeFailure)
      {
        case Z_HOME_UNSTABLE: Serial.println(F("Reason: switch did not stay stable for 20 ms within 250 ms")); break;
        case Z_HOME_REPRESSED: Serial.println(F("Reason: switch HIGH again during positive clearance")); break;
        case Z_HOME_NOT_FOUND: Serial.println(F("Reason: switch HIGH not found within search distance")); break;
        case Z_HOME_NOT_RELEASED: Serial.println(F("Reason: switch LOW not found within +5 DEG release; check backlash/wiring")); break;
        case Z_HOME_SENSOR_ERROR: Serial.println(F("Reason: AS5600 communication failed")); break;
        default: Serial.println(F("Reason: homing interrupted before completion")); break;
      }
    }
    return;
  }
  zLastRaw = raw;
  zEncoderCount = 0;
  zEncoderOK = true;
  Serial.println(F("Z HOME DONE: Z=0 DEG; switch HIGH; move z positive to leave home"));
}

#endif
