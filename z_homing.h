#ifndef ROBOT_ARM_Z_HOMING_H
#define ROBOT_ARM_Z_HOMING_H

// Confirmed: negative Z approaches the NC switch on D24; positive releases it.
const float Z_HOME_SEARCH_DEG = 30.0f;
const float Z_HOME_BACKOFF_DEG = 5.0f;
const unsigned long Z_HOME_TIMEOUT_MS = 20000UL;
const unsigned int Z_HOME_SEEK_US = 2000;
unsigned long zHomeStarted = 0;
bool zHomeCancelled = false;
bool zHomed = false;
enum ZHomeFailure {
  Z_HOME_OK, Z_HOME_NOT_FOUND, Z_HOME_NOT_RELEASED, Z_HOME_SENSOR_ERROR
};
ZHomeFailure zHomeFailure = Z_HOME_OK;

bool zHomeCheck()
{
  while (Serial.available())
    if (Serial.read() == '!') zHomeCancelled = true;
  return !zHomeCancelled && (unsigned long)(millis() - zHomeStarted) < Z_HOME_TIMEOUT_MS;
}

// First HIGH stops approach immediately. During fixed positive backoff,
// targetState=-1 ignores switch chatter; final LOW is checked after stopping.
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
    if (targetState >= 0 && state == targetState) return true;
    if (i == maxPulses)
    {
      if (targetState < 0) return true;
      zHomeFailure = targetState == HIGH ? Z_HOME_NOT_FOUND : Z_HOME_NOT_RELEASED;
      return false;
    }
    digitalWrite(Z_PUL, HIGH);
    const bool keepMoving = zHomeWait(halfPeriod, targetState);
    digitalWrite(Z_PUL, LOW);
    if (!keepMoving)
    {
      // A single target edge is sufficient; do not re-read a bouncing contact.
      return targetState >= 0 && zHomeCheck();
    }
    if (!zHomeWait(halfPeriod, targetState))
    {
      // A single target edge is sufficient; do not re-read a bouncing contact.
      return targetState >= 0 && zHomeCheck();
    }
    if (i % 10 == 0)
    {
      updateZEncoder();
      if (!zEncoderOK) { zHomeFailure = Z_HOME_SENSOR_ERROR; return false; }
    }
  }
  return false;
}

void homeZ()
{
  zHomed = false;
  zHomeStarted = millis();
  zHomeCancelled = false;
  zHomeFailure = Z_HOME_OK;
  digitalWrite(Z_PUL, LOW);
  Serial.println(F("Z HOME: SINGLE TOUCH (2x speed); ! stops; search max 30 DEG; timeout 20 s"));
  updateZEncoder();
  if (!zEncoderOK) zHomeFailure = Z_HOME_SENSOR_ERROR;
  bool ok = zEncoderOK && zHomeCheck();
  if (ok && digitalRead(Z_HOME) == LOW)
  {
    Serial.println(F("Z HOME: SEEK -"));
    ok = zHomeTravel(false, Z_HOME_SEARCH_DEG, Z_HOME_SEEK_US, HIGH);
  }
  if (ok)
  {
    Serial.println(F("Z HOME: CONTACT - stopped"));
    Serial.println(F("Z HOME: BACKOFF +5 DEG (no re-approach)"));
    ok = zHomeTravel(true, Z_HOME_BACKOFF_DEG, Z_HOME_SEEK_US, -1);
  }
  if (ok) ok = zHomeWait(200000UL, -1);
  uint16_t raw;
  if (ok)
  {
    ok = readAS5600Raw(raw);
    if (!ok) { zEncoderOK = false; zHomeFailure = Z_HOME_SENSOR_ERROR; }
  }
  if (ok)
  {
    ok = zHomeCheck();
    if (ok && digitalRead(Z_HOME) != LOW)
    {
      zHomeFailure = Z_HOME_NOT_RELEASED;
      ok = false;
    }
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
        case Z_HOME_NOT_FOUND: Serial.println(F("Reason: switch HIGH not found within search distance")); break;
        case Z_HOME_NOT_RELEASED: Serial.println(F("Reason: switch still HIGH after +5 DEG backoff; check backlash/wiring")); break;
        case Z_HOME_SENSOR_ERROR: Serial.println(F("Reason: AS5600 communication failed")); break;
        default: Serial.println(F("Reason: homing interrupted before completion")); break;
      }
    }
    return;
  }
  zLastRaw = raw;
  zEncoderCount = 0;
  zEncoderOK = true;
  zHomed = true;
  Serial.println(F("Z HOME DONE: Z=0 DEG at released position; switch LOW"));
}

#endif
