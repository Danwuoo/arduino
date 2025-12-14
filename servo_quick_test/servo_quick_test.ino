#include <Servo.h>

// Quick hardware sanity-check for servos on Arduino Mega:
// - Servo #1 signal -> D6
// - Servo #2 signal -> D7
// Power note: use an external 5V supply for servos if possible, and keep a common ground (servo GND tied to Arduino GND).

const int PIN_SERVO1 = 6;
const int PIN_SERVO2 = 7;

// Safer "moderate" commands:
// - For 180° positional servos: these are angles.
// - For continuous-rotation servos: 90 is stop, values away from 90 increase speed.
const int SERVO_STOP = 90;
const int SERVO_TEST_A = 60;
const int SERVO_TEST_B = 120;

const unsigned long HOLD_MS = 1500;

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println(F("=== Servo Quick Test ==="));
  Serial.println(F("Pins: Servo1=D6, Servo2=D7 (Arduino Mega)"));
  Serial.println(F("If servos do NOT move: check external 5V power + common GND."));
  Serial.println(F("This test alternates 60 <-> 120, with 90 as stop/center."));
  Serial.println();

  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);

  servo1.write(SERVO_STOP);
  servo2.write(SERVO_STOP);
  delay(HOLD_MS);
}

void loop() {
  Serial.println(F("CMD: 60"));
  servo1.write(SERVO_TEST_A);
  servo2.write(SERVO_TEST_A);
  delay(HOLD_MS);

  Serial.println(F("CMD: 120"));
  servo1.write(SERVO_TEST_B);
  servo2.write(SERVO_TEST_B);
  delay(HOLD_MS);

  Serial.println(F("CMD: 90 (stop/center)"));
  servo1.write(SERVO_STOP);
  servo2.write(SERVO_STOP);
  delay(HOLD_MS);
}

