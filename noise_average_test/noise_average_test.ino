#include <Servo.h>
#include <math.h>

const int PIN_SOUND = A0;
const int PIN_SERVO1 = 6;
const int PIN_SERVO2 = 7;

// 90 = stop on a continuous-rotation servo; push a bit above to keep it moving.
const int SERVO_CCW_SPEED = 100;

const unsigned long SAMPLE_WINDOW_MS = 5000;   // time per report
const unsigned long SAMPLE_INTERVAL_MS = 5;     // approx sample rate ~200 Hz
const int SOUND_MIDPOINT = 512;                 // expected 2.5 V center from MAX4466

unsigned long windowStart = 0;
unsigned long lastSampleTime = 0;
unsigned long sampleCount = 0;
unsigned long sumRaw = 0;
unsigned long sumAbsDiff = 0;
unsigned long sumSqDiff = 0;
int minReading = 1023;
int maxReading = 0;

Servo servo1;
Servo servo2;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  // Keep both servos spinning to mimic normal load during noise capture.
  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  servo1.write(SERVO_CCW_SPEED);
  servo2.write(SERVO_CCW_SPEED);

  pinMode(PIN_SOUND, INPUT);

  windowStart = millis();
  lastSampleTime = millis();

  Serial.println(F("Noise average test: sampling A0 while servos spin."));
  Serial.println(F("Outputs avg raw, mean abs diff, rms diff, min, max, samples per window."));
}

void loop() {
  unsigned long now = millis();

  // Sample the microphone front-end.
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;

    int reading = analogRead(PIN_SOUND);
    int diff = reading - SOUND_MIDPOINT;

    sumRaw += reading;
    sumAbsDiff += abs(diff);
    sumSqDiff += (unsigned long)diff * (unsigned long)diff;

    if (reading < minReading) minReading = reading;
    if (reading > maxReading) maxReading = reading;

    sampleCount++;
  }

  // Report once per window.
  if (now - windowStart >= SAMPLE_WINDOW_MS) {
    if (sampleCount > 0) {
      float avgRaw = (float)sumRaw / (float)sampleCount;
      float meanAbs = (float)sumAbsDiff / (float)sampleCount;
      float rms = sqrt((float)sumSqDiff / (float)sampleCount);

      Serial.print(F("Window "));
      Serial.print(SAMPLE_WINDOW_MS);
      Serial.print(F(" ms | samples="));
      Serial.print(sampleCount);
      Serial.print(F(" | avg_raw="));
      Serial.print(avgRaw, 2);
      Serial.print(F(" | mean_abs="));
      Serial.print(meanAbs, 2);
      Serial.print(F(" | rms="));
      Serial.print(rms, 2);
      Serial.print(F(" | min="));
      Serial.print(minReading);
      Serial.print(F(" | max="));
      Serial.println(maxReading);
    } else {
      Serial.println(F("No samples in window"));
    }

    // Reset accumulators for next window.
    windowStart = now;
    sumRaw = 0;
    sumAbsDiff = 0;
    sumSqDiff = 0;
    minReading = 1023;
    maxReading = 0;
    sampleCount = 0;

    // Refresh servo command in case the controller expects a recent pulse.
    servo1.write(SERVO_CCW_SPEED);
    servo2.write(SERVO_CCW_SPEED);
  }
}
