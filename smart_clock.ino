#include <Servo.h>

// =================================================================================
// 腳位定義（依 system_wiring_status.md）
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
// 常數與設定
// =================================================================================

// --- 步進馬達設定 ---
// 假設使用 28BYJ-48，半步驟模式一圈 4096 步
const int STEPS_PER_REV = 4096;

// 每圈目標時間（毫秒）
const unsigned long DURATION_FAST = 6000;
const unsigned long DURATION_NORMAL = 12000;
const unsigned long DURATION_SLOW = 24000;

// 依實測校正步進延遲：原本約 2ms/step 只達到約 8.3 秒／圈，
// 需求為 6 秒／圈，因此調整為約 1.45ms/step (≈ 2ms × 0.7229)。
// 延遲以微秒計算，並維持正常／慢速為快速的 2 倍與 4 倍。
const unsigned long STEP_DELAY_FAST = 1450;  // 約 6 秒／圈
const unsigned long STEP_DELAY_NORMAL = STEP_DELAY_FAST * 2; // 約 12 秒／圈
const unsigned long STEP_DELAY_SLOW = STEP_DELAY_FAST * 4;   // 約 24 秒／圈

// --- 伺服馬達設定 ---
// 連續旋轉伺服（或改裝後）。
// 0 = 順時針全速，180 = 逆時針全速（或相反），90 = 停止。
// 需求：穩定逆時針中低速轉動。
// 可透過調整 SERVO_SPEED_CCW 微調速度。
// 假設 90 為停止，> 90 代表逆時針（方向依伺服而定）。
// 先以接近 90 的數值作為可調基準。
const int SERVO_STOP = 90;
const int SERVO_CCW_SPEED = 100; // 例示值，接近 90 可維持中低速

// --- 聲音設定 ---
// 聲音類比讀值門檻（0-1023），需依現場調整。
// 聲音狀態需連續 2 秒才算成立。
const int SOUND_THRESHOLD = 300; // 聲音門檻，可依現場微調
const unsigned long SOUND_ACTIVITY_WINDOW = 200; // 毫秒，短期保持 AC 訊號
const unsigned long SOUND_VALIDATION_TIME = 2000; // 毫秒，連續 2 秒才視為狀態轉換

// =================================================================================
// 全域物件
// =================================================================================
Servo servo1;
Servo servo2;

// =================================================================================
// 邏輯類別
// =================================================================================

// 非阻塞步進馬達控制類別
class StepperDriver {
  private:
    int pins[4];
    int stepIndex;
    unsigned long lastStepTime;
    unsigned long stepIntervalMicros;
    bool isMoving;
    int remainingStepsLocked; // 用於「鎖定」的 2 圈動作

    // 8 步半步驟序列
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

    // 如需特定方向，可將序列反轉；若方向相反，改為遞減索引即可。

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

    // 啟動鎖定的 2 圈序列
    void startLockedSequence() {
      // 使用快速速度
      setSpeedDelay(STEP_DELAY_FAST);
      setMoving(true);
      // 2 圈
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
      // 驅動線圈腳位
      for (int i = 0; i < 4; i++) {
        digitalWrite(pins[i], sequence[stepIndex][i]);
      }

      // 依指定方向遞增索引（需求固定同一方向）
      stepIndex++;
      if (stepIndex >= 8) {
        stepIndex = 0;
      }
    }
};

// 聲音偵測與持續驗證類別
class SoundManager {
  private:
    bool stableState;         // 已驗證的穩定狀態
    bool lastSignalActive;    // 活動狀態（含 AC 緩衝）
    unsigned long lastNoiseTime; // 上次越過門檻的時間
    unsigned long stateChangeTime; // 活動狀態最近一次變化時間

  public:
    SoundManager() {
      stableState = false;
      lastSignalActive = false;
      lastNoiseTime = 0;
      stateChangeTime = 0;
    }

    void update() {
      unsigned long now = millis();

      // 1. 原始偵測（AC 訊號）
      int val = analogRead(PIN_SOUND);
      int diff = abs(val - 512); // 假設 2.5V 偏壓（512）

      // 偵測到噪音即更新最後聽到時間
      if (diff > SOUND_THRESHOLD) {
        lastNoiseTime = now;
      }

      // 2. 活動訊號（短期保持）
      // 判斷此刻或近期是否有聲音
      bool isSignalActive = (now - lastNoiseTime < SOUND_ACTIVITY_WINDOW);

      // 3. 驗證邏輯（長時間去抖動）
      // 聲音狀態需連續 2 秒才算成立

      if (isSignalActive != lastSignalActive) {
         // 活動狀態剛剛改變（例：安靜 -> 有聲或反之）
         stateChangeTime = now;
         lastSignalActive = isSignalActive;
      }

      // 若活動狀態已持續超過 2 秒...
      if ((now - stateChangeTime) > SOUND_VALIDATION_TIME) {
         // 更新穩定狀態
         if (stableState != isSignalActive) {
            stableState = isSignalActive;
            // 狀態轉換成立
         }
      }
    }

    bool getStableState() {
      return stableState;
    }
};

// =================================================================================
// 主邏輯
// =================================================================================

StepperDriver stepper(PIN_STEP_IN1, PIN_STEP_IN2, PIN_STEP_IN3, PIN_STEP_IN4);
SoundManager soundMgr;

// 應用狀態列舉
enum AppMode {
  MODE_IR_DETECTED,
  MODE_NO_IR,
  MODE_LOCKED_ANIMATION // 兩圈快速旋轉的特殊鎖定狀態
};

AppMode currentMode = MODE_NO_IR;
bool lastSoundState = false; // 用於偵測聲音狀態轉換

// 隨機模式用的計時
unsigned long randomModeTimer = 0;
unsigned long nextRandomChange = 0;

void setup() {
  // 初始化序列埠供除錯使用
  Serial.begin(115200);
  Serial.println("System Starting...");

  // 初始化輸入腳位
  pinMode(PIN_PIR1, INPUT);
  pinMode(PIN_PIR2, INPUT);

  // 初始化伺服馬達
  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  // 啟動伺服：穩定逆時針中低速
  servo1.write(SERVO_CCW_SPEED);
  servo2.write(SERVO_CCW_SPEED);

  // 初始化步進馬達
  stepper.setup();

  // 隨機種子（使用未連接的腳位）
  randomSeed(analogRead(A5)); // 未連接的腳位用來取亂數
}

void loop() {
  unsigned long now = millis();

  // 1. 更新子系統
  stepper.update();
  soundMgr.update();

  // 2. 讀取輸入
  bool irDetected = (digitalRead(PIN_PIR1) == HIGH) || (digitalRead(PIN_PIR2) == HIGH);
  bool currentSoundState = soundMgr.getStableState();

  // 3. 狀態機

  // 高優先：鎖定動畫執行時，忽略其他事件
  if (stepper.isLocked()) {
    currentMode = MODE_LOCKED_ANIMATION;
    // 持續檢查直到鎖定結束，解除後 isLocked() 會回傳 false
    // 同步最新聲音狀態，避免鎖定期間的變化立即觸發
    lastSoundState = currentSoundState;
    return; // 跳過其他邏輯
  } else {
    // 若剛解除鎖定或切換模式
    if (currentMode == MODE_LOCKED_ANIMATION) {
       // 剛完成兩圈加速，接下來依紅外線／聲音狀態設定速度
       Serial.println("Locked sequence finished.");
    }
  }

  // 依紅外線決定模式
  if (irDetected) {
    currentMode = MODE_IR_DETECTED;
  } else {
    currentMode = MODE_NO_IR;
  }

  // 依模式執行對應邏輯
  switch (currentMode) {
    case MODE_IR_DETECTED:
      // 狀態一邏輯

      // 偵測聲音轉換：有聲 -> 無聲
      if (lastSoundState == true && currentSoundState == false) {
        // 觸發：切換快速鎖定並轉動 2 圈
        Serial.println("Trigger: Sound -> No Sound. Starting Locked Sequence.");
        stepper.startLockedSequence();
        lastSoundState = currentSoundState;
        return; // 下一輪由鎖定模式接管
      }

      // 狀態一一般行為
      if (currentSoundState) {
        // 有聲音：慢速
        stepper.setSpeedDelay(STEP_DELAY_SLOW);
        stepper.setMoving(true);
      } else {
        // 無聲音：正常速度
        stepper.setSpeedDelay(STEP_DELAY_NORMAL);
        stepper.setMoving(true);
      }
      break;

    case MODE_NO_IR:
      // 狀態二邏輯：隨機
      // 每隔一段時間重新生成指令

      if (now > nextRandomChange) {
        // 產生新的隨機指令
        int randCmd = random(0, 4); // 0, 1, 2, 3
        int durationSec = random(2, 10); // 隨機維持 2~10 秒

        Serial.print("Random Mode: Cmd "); Serial.print(randCmd);
        Serial.print(" for "); Serial.print(durationSec); Serial.println("s");

        switch (randCmd) {
          case 0: // 加速
            stepper.setSpeedDelay(STEP_DELAY_FAST);
            stepper.setMoving(true);
            break;
          case 1: // 慢速
            stepper.setSpeedDelay(STEP_DELAY_SLOW);
            stepper.setMoving(true);
            break;
          case 2: // 正常
            stepper.setSpeedDelay(STEP_DELAY_NORMAL);
            stepper.setMoving(true);
            break;
          case 3: // 停止
            stepper.setMoving(false);
            break;
        }

        nextRandomChange = now + (durationSec * 1000);
      }
      break;

    case MODE_LOCKED_ANIMATION:
      // 理論上不會進入此處，僅為保護性處理
      break;
  }

  // 更新歷史狀態
  lastSoundState = currentSoundState;
}
