#include <Servo.h>

// =================================================================================
// Pin Definitions (Based on system_wiring_status.md)
// =================================================================================
const int PIN_RTC_SDA = 20;
const int PIN_RTC_SCL = 21;
const int PIN_SOUND = A0;
const int PIN_PIR1 = 22;
const int PIN_PIR2 = 23;
const int PIN_SERVO1 = 6;
const int PIN_SERVO2 = 7;
const int PIN_STEP_IN1 = 30;
const int PIN_STEP_IN2 = 31;
const int PIN_STEP_IN3 = 32;
const int PIN_STEP_IN4 = 33;

// =================================================================================
// Constants & Settings
// =================================================================================

// --- Stepper Settings ---
// Assumed 28BYJ-48 with 4096 steps per revolution (Half-step mode)
const int STEPS_PER_REV = 4096;

// Target Durations per Revolution (ms)
const unsigned long DURATION_FAST = 6000;
const unsigned long DURATION_NORMAL = 12000;
const unsigned long DURATION_SLOW = 24000;

// Calculated Delays (microseconds)
// Delay = (Duration * 1000) / STEPS_PER_REV
// Fast: 6000000 / 4096 ~= 1465 us
// Normal: 12000000 / 4096 ~= 2930 us
// Slow: 24000000 / 4096 ~= 5860 us

const unsigned long STEP_DELAY_FAST = 1465;
const unsigned long STEP_DELAY_NORMAL = 2930;
const unsigned long STEP_DELAY_SLOW = 5860;

// --- Servo Settings ---
// Continuous rotation servo (or modified).
// 0 = Full speed CW, 180 = Full speed CCW (or vice versa). 90 = Stop.
// Requirement: "Stable counter-clockwise rotation (medium-low speed)"
// Adjust SERVO_SPEED_CCW value to tune speed.
// Assuming 90 is stop, values > 90 are CCW (or < 90 depending on servo).
// Let's assume > 90 is CCW. "Medium-low" might be 95-100 or 85-80.
// If it's a standard servo used as continuous, 0 might be max speed one way.
// Let's set a placeholder we can tune.
const int SERVO_STOP = 90;
const int SERVO_CCW_SPEED = 100; // Example value, close to 90 for slow speed

// --- Sound Settings ---
const int SOUND_THRESHOLD = 300; // Threshold for analog read (0-1023). Tune this!
const unsigned long SOUND_ACTIVITY_WINDOW = 200; // ms. AC signal hold time.
const unsigned long SOUND_VALIDATION_TIME = 2000; // ms. 2 seconds required for state change.

// =================================================================================
// Global Objects
// =================================================================================
Servo servo1;
Servo servo2;

// =================================================================================
// Logic Classes
// =================================================================================

// Class to handle Stepper Motor (Non-blocking)
class StepperDriver {
  private:
    int pins[4];
    int stepIndex;
    unsigned long lastStepTime;
    unsigned long stepIntervalMicros;
    bool isMoving;
    int remainingStepsLocked; // For the "Locked" 2-turn action

    // 8-step sequence (Half-step)
    const int sequence[8][4] = {
      {1, 0, 0, 0}, // 8
      {1, 1, 0, 0}, // 1
      {0, 1, 0, 0}, // 2
      {0, 1, 1, 0}, // 3
      {0, 0, 1, 0}, // 4
      {0, 0, 1, 1}, // 5
      {0, 0, 0, 1}, // 6
      {1, 0, 0, 1}  // 7
    };

    // Note: To match specific direction, we might need to reverse sequence.
    // Assuming standard sequence is CW? Or CCW?
    // We can just reverse the index increment if needed.

  public:
    StepperDriver(int p1, int p2, int p3, int p4) {
      pins[0] = p1; pins[1] = p2; pins[2] = p3; pins[3] = p4;
      stepIndex = 0;
      lastStepTime = 0;
      stepIntervalMicros = STEP_DELAY_NORMAL;
      isMoving = true;
      remainingStepsLocked = 0;
    }

    void setup() {
      for (int i = 0; i < 4; i++) {
        pinMode(pins[i], OUTPUT);
      }
    }

    void setSpeedDelay(unsigned long delayMicros) {
      stepIntervalMicros = delayMicros;
    }

    void setMoving(bool moving) {
      isMoving = moving;
    }

    // Start the locked 2-turn sequence
    void startLockedSequence() {
      // Fast speed
      setSpeedDelay(STEP_DELAY_FAST);
      setMoving(true);
      // 2 Revolutions
      remainingStepsLocked = STEPS_PER_REV * 2;
    }

    bool isLocked() {
      return remainingStepsLocked > 0;
    }

    void update() {
      if (!isMoving && remainingStepsLocked <= 0) return;

      unsigned long currentMicros = micros();
      if (currentMicros - lastStepTime >= stepIntervalMicros) {
        lastStepTime = currentMicros;
        step();

        if (remainingStepsLocked > 0) {
          remainingStepsLocked--;
        }
      }
    }

  private:
    void step() {
      // Drive pins
      for (int i = 0; i < 4; i++) {
        digitalWrite(pins[i], sequence[stepIndex][i]);
      }

      // Increment index (Direction)
      // Assuming one direction for all operations based on specs "Pointer speed setting"
      stepIndex++;
      if (stepIndex >= 8) {
        stepIndex = 0;
      }
    }
};

// Class to handle Sound Detection with Activity Window and Debounce
class SoundManager {
  private:
    bool stableState;         // The validated output state
    bool lastSignalActive;    // The "activity" state (AC filtered)
    unsigned long lastNoiseTime; // Last time threshold was crossed
    unsigned long stateChangeTime; // Time when "activity" state last changed

  public:
    SoundManager() {
      stableState = false;
      lastSignalActive = false;
      lastNoiseTime = 0;
      stateChangeTime = 0;
    }

    void update() {
      unsigned long now = millis();

      // 1. Raw Detection (AC Signal)
      int val = analogRead(PIN_SOUND);
      int diff = abs(val - 512); // Assuming 2.5V bias (512)

      // If noise detected, update last heard time
      if (diff > SOUND_THRESHOLD) {
        lastNoiseTime = now;
      }

      // 2. Activity Signal (Short-term hold)
      // "Is there sound right now (or very recently)?"
      bool isSignalActive = (now - lastNoiseTime < SOUND_ACTIVITY_WINDOW);

      // 3. Validation Logic (Long-term debounce)
      // "Sound status must be maintained continuously for 2 seconds to be valid"

      if (isSignalActive != lastSignalActive) {
         // The activity state just changed (e.g. Quiet -> Noise, or Noise -> Quiet)
         stateChangeTime = now;
         lastSignalActive = isSignalActive;
      }

      // If the current activity state has persisted for > 2 seconds...
      if ((now - stateChangeTime) > SOUND_VALIDATION_TIME) {
         // ...then update the Stable State
         if (stableState != isSignalActive) {
            stableState = isSignalActive;
            // Transition happened
         }
      }
    }

    bool getStableState() {
      return stableState;
    }
};

// =================================================================================
// Main Logic
// =================================================================================

StepperDriver stepper(PIN_STEP_IN1, PIN_STEP_IN2, PIN_STEP_IN3, PIN_STEP_IN4);
SoundManager soundMgr;

// Application States
enum AppMode {
  MODE_IR_DETECTED,
  MODE_NO_IR,
  MODE_LOCKED_ANIMATION // Special state for the 2-turn fast spin
};

AppMode currentMode = MODE_NO_IR;
bool lastSoundState = false; // To detect transitions

// For Random Mode
unsigned long randomModeTimer = 0;
unsigned long nextRandomChange = 0;

void setup() {
  // Init Serial for debug
  Serial.begin(115200);
  Serial.println("System Starting...");

  // Init Pins
  pinMode(PIN_PIR1, INPUT);
  pinMode(PIN_PIR2, INPUT);

  // Init Servo
  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  // Start Servo: Stable CCW medium-low
  servo1.write(SERVO_CCW_SPEED);
  servo2.write(SERVO_CCW_SPEED);

  // Init Stepper
  stepper.setup();

  // Random seed
  randomSeed(analogRead(A5)); // Use unconnected pin
}

void loop() {
  unsigned long now = millis();

  // 1. Update Subsystems
  stepper.update();
  soundMgr.update();

  // 2. Check Inputs
  bool irDetected = (digitalRead(PIN_PIR1) == HIGH) || (digitalRead(PIN_PIR2) == HIGH);
  bool currentSoundState = soundMgr.getStableState();

  // 3. State Machine

  // High Priority: If Locked Animation is running, ignore everything else
  if (stepper.isLocked()) {
    currentMode = MODE_LOCKED_ANIMATION;
    // Keep checking until done
    // Once done, the StepperDriver.isLocked() will return false
    // We update lastSoundState to avoid immediate re-trigger if sound state changed during lock
    // "Re-evaluate current sound status"
    lastSoundState = currentSoundState;
    return; // Skip other logic
  } else {
    // If we just finished locking, or were in other modes
    if (currentMode == MODE_LOCKED_ANIMATION) {
       // Just finished. "After completion return to Normal speed".
       // Logic below will handle speed setting based on IR/Sound state.
       Serial.println("Locked sequence finished.");
    }
  }

  // Determine Mode based on IR
  if (irDetected) {
    currentMode = MODE_IR_DETECTED;
  } else {
    currentMode = MODE_NO_IR;
  }

  // Behavior based on Mode
  switch (currentMode) {
    case MODE_IR_DETECTED:
      // State 1 Logic

      // Detect Sound Transition: Sound -> No Sound
      if (lastSoundState == true && currentSoundState == false) {
        // Trigger: Switch to Fast, 2 turns locked
        Serial.println("Trigger: Sound -> No Sound. Starting Locked Sequence.");
        stepper.startLockedSequence();
        lastSoundState = currentSoundState;
        return; // Next loop will handle LOCKED_ANIMATION
      }

      // Normal State 1 Behavior
      if (currentSoundState) {
        // Have Sound: Slow
        stepper.setSpeedDelay(STEP_DELAY_SLOW);
        stepper.setMoving(true);
      } else {
        // No Sound: Normal
        stepper.setSpeedDelay(STEP_DELAY_NORMAL);
        stepper.setMoving(true);
      }
      break;

    case MODE_NO_IR:
      // State 2 Logic: Random
      // "Every X seconds regenerate random command"

      if (now > nextRandomChange) {
        // Generate new random command
        int randCmd = random(0, 4); // 0, 1, 2, 3
        int durationSec = random(2, 10); // Duration 2 to 10 seconds (Random)

        Serial.print("Random Mode: Cmd "); Serial.print(randCmd);
        Serial.print(" for "); Serial.print(durationSec); Serial.println("s");

        switch (randCmd) {
          case 0: // Fast
            stepper.setSpeedDelay(STEP_DELAY_FAST);
            stepper.setMoving(true);
            break;
          case 1: // Slow
            stepper.setSpeedDelay(STEP_DELAY_SLOW);
            stepper.setMoving(true);
            break;
          case 2: // Normal
            stepper.setSpeedDelay(STEP_DELAY_NORMAL);
            stepper.setMoving(true);
            break;
          case 3: // Stop
            stepper.setMoving(false);
            break;
        }

        nextRandomChange = now + (durationSec * 1000);
      }
      break;

    case MODE_LOCKED_ANIMATION:
      // Should not be reached here due to top check, but safe fallback
      break;
  }

  // Update history
  lastSoundState = currentSoundState;
}
