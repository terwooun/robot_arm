#ifndef ROBOT_ARM_Z_HOMING_H
#define ROBOT_ARM_Z_HOMING_H

// Confirmed: negative Z approaches the NC switch on D24; positive releases it.
const float Z_HOME_SEARCH_DEG = 30.0f;
const float Z_HOME_RELEASE_DEG = 5.0f;
const float Z_HOME_CLEARANCE_DEG = 1.0f;
const unsigned long Z_HOME_TIMEOUT_MS = 20000UL;
const unsigned int Z_HOME_SEEK_US = 4000;
const unsigned int Z_HOME_FINE_US = 8000;
unsigned long zHomeStarted = 0;
bool zHomeCancelled = false;

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
  const unsigned long start = millis();
  do
  {
    if (!zHomeCheck() || digitalRead(Z_HOME) != state) return false;
  } while ((unsigned long)(millis() - start) < 20UL);
  return true;
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
    if (targetState < 0 && state != LOW) return false;
    if (i == maxPulses) return targetState < 0;
    digitalWrite(Z_PUL, HIGH);
    const bool keepMoving = zHomeWait(halfPeriod, targetState < 0 ? HIGH : targetState);
    digitalWrite(Z_PUL, LOW);
    if (!keepMoving)
      return targetState >= 0 && zHomeCheck() && zHomeStable(targetState);
    if (!zHomeWait(halfPeriod, targetState < 0 ? HIGH : targetState))
      return targetState >= 0 && zHomeCheck() && zHomeStable(targetState);
    if (i % 10 == 0)
    {
      updateZEncoder();
      if (!zEncoderOK) return false;
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
  digitalWrite(Z_PUL, LOW);
  Serial.println(F("Z HOME: START; ! stops; search max 30 DEG; total timeout 20 s"));
  updateZEncoder();
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
  if (ok) ok = readAS5600Raw(raw) && zHomeStable(HIGH);
  digitalWrite(Z_PUL, LOW);
  if (!ok)
  {
    Serial.println(F("Z HOME ERROR: stopped; zero NOT set"));
    if (zHomeCancelled) Serial.println(F("Reason: ! stop"));
    else if ((unsigned long)(millis() - zHomeStarted) >= Z_HOME_TIMEOUT_MS)
      Serial.println(F("Reason: timeout"));
    else Serial.println(F("Check switch travel/wiring/bounce and AS5600 communication"));
    return;
  }
  zLastRaw = raw;
  zEncoderCount = 0;
  zEncoderOK = true;
  Serial.println(F("Z HOME DONE: Z=0 DEG; switch HIGH; move z positive to leave home"));
}

#endif
