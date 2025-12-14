#include <Arduino.h>

// Stepper motor test for 28BYJ-48 + ULN2003 (4-wire IN1~IN4)
// Wiring (Arduino Mega): IN1->D30 IN2->D31 IN3->D32 IN4->D33
//
// Cycle:
// - Normal: 12 sec/rev, 2 rev
// - Slow:   24 sec/rev, 2 rev
// - Fast:    6 sec/rev, 2 rev
// Repeat forever.

static const int PIN_IN1 = 30;
static const int PIN_IN2 = 31;
static const int PIN_IN3 = 32;
static const int PIN_IN4 = 33;

// 28BYJ-48 half-step: 4096 steps/rev (commonly used approximation)
static const long STEPS_PER_REV = 4096L;
static const long STEPS_PER_SEGMENT = STEPS_PER_REV * 2L; // 2 revolutions

// Calibrated delays (microseconds/half-step) matching smart_clock.
// If your motor is slower/faster, tweak STEP_DELAY_FAST_US.
static const unsigned long STEP_DELAY_FAST_US = 1450;                 // ~6 sec/rev
static const unsigned long STEP_DELAY_NORMAL_US = STEP_DELAY_FAST_US * 2; // ~12 sec/rev
static const unsigned long STEP_DELAY_SLOW_US = STEP_DELAY_FAST_US * 4;   // ~24 sec/rev

// 8-step half-step sequence (IN1..IN4)
static const uint8_t SEQ[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1},
};

enum SpeedMode : uint8_t { MODE_NORMAL = 0, MODE_SLOW = 1, MODE_FAST = 2 };

static SpeedMode mode = MODE_NORMAL;
static long stepsRemaining = STEPS_PER_SEGMENT;
static uint8_t stepIndex = 0;
static unsigned long lastStepMicros = 0;

// Set to -1 if your motor spins the opposite direction.
static const int8_t DIRECTION = 1;

static unsigned long modeDelayUs(SpeedMode m) {
  switch (m) {
    case MODE_SLOW: return STEP_DELAY_SLOW_US;
    case MODE_FAST: return STEP_DELAY_FAST_US;
    case MODE_NORMAL:
    default: return STEP_DELAY_NORMAL_US;
  }
}

static const __FlashStringHelper* modeName(SpeedMode m) {
  switch (m) {
    case MODE_SLOW: return F("SLOW  (24s/rev)");
    case MODE_FAST: return F("FAST  ( 6s/rev)");
    case MODE_NORMAL:
    default: return F("NORMAL(12s/rev)");
  }
}

static void applyStep(uint8_t idx) {
  digitalWrite(PIN_IN1, SEQ[idx][0]);
  digitalWrite(PIN_IN2, SEQ[idx][1]);
  digitalWrite(PIN_IN3, SEQ[idx][2]);
  digitalWrite(PIN_IN4, SEQ[idx][3]);
}

static void nextMode() {
  mode = static_cast<SpeedMode>((static_cast<uint8_t>(mode) + 1) % 3);
  stepsRemaining = STEPS_PER_SEGMENT;

  Serial.print(F("Mode -> "));
  Serial.print(modeName(mode));
  Serial.println(F(" (2 rev)"));
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  applyStep(0);

  Serial.println(F("Stepper speed test start"));
  Serial.print(F("Steps/rev: "));
  Serial.println(STEPS_PER_REV);
  Serial.print(F("Delay(us): FAST="));
  Serial.print(STEP_DELAY_FAST_US);
  Serial.print(F(" NORMAL="));
  Serial.print(STEP_DELAY_NORMAL_US);
  Serial.print(F(" SLOW="));
  Serial.println(STEP_DELAY_SLOW_US);

  Serial.print(F("Mode -> "));
  Serial.print(modeName(mode));
  Serial.println(F(" (2 rev)"));
}

void loop() {
  const unsigned long interval = modeDelayUs(mode);
  const unsigned long now = micros();

  if (static_cast<unsigned long>(now - lastStepMicros) < interval) return;
  lastStepMicros = now;

  applyStep(stepIndex);
  int next = static_cast<int>(stepIndex) + static_cast<int>(DIRECTION);
  if (next < 0) next = 7;
  if (next > 7) next = 0;
  stepIndex = static_cast<uint8_t>(next);

  stepsRemaining--;
  if (stepsRemaining <= 0) {
    nextMode();
  }
}
