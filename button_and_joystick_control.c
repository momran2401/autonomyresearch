#include <Arduino.h>

#define BUTTON_SAVE_PIN    2    // D2  → Save / Continue
#define BUTTON_DELETE_PIN  3    // D3  → Delete last entry
#define JOY_BUTTON_PIN     4    // D4  → Joystick “history” button

#define JOY_X_PIN          A0   // ← Shutdown when held left/right
#define JOY_Y_PIN          A1   // VRy → A1, for UP/DOWN

// thresholds for detecting “UP” / “DOWN”
#define JOY_UP_THRESHOLD    400  // raw ADC < 400 → joystick tilted up
#define JOY_DOWN_THRESHOLD  600  // raw ADC > 600 → joystick tilted down

// thresholds for “left” / “right” shutdown hold
#define JOY_LEFT_THRESHOLD   400 // raw ADC < 400 → full left
#define JOY_RIGHT_THRESHOLD  600 // raw ADC > 600 → full right

// Debounce interval
const unsigned long DEBOUNCE_MS = 50;

// State for debouncing & last-sent flags
bool lastSaveState   = HIGH;
bool lastDeleteState = HIGH;
bool lastJoyBtnState = HIGH;
bool lastUpSent      = false;
bool lastDownSent    = false;

unsigned long lastSaveDebounce   = 0;
unsigned long lastDeleteDebounce = 0;
unsigned long lastJoyBtnDebounce = 0;
unsigned long lastUpDebounce     = 0;
unsigned long lastDownDebounce   = 0;

// ——— NEW: joystick-X shutdown hold state ———
unsigned long xHoldStart    = 0;
bool          xHoldActive   = false;
bool          shutdownSent  = false;
// ————————————————————————————————

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_SAVE_PIN,    INPUT_PULLUP);
  pinMode(BUTTON_DELETE_PIN,  INPUT_PULLUP);
  pinMode(JOY_BUTTON_PIN,     INPUT_PULLUP);
  // A0/A1 are inputs by default
}

void loop() {
  unsigned long now = millis();

  // 1) SAVE button (D2)
  bool saveState = digitalRead(BUTTON_SAVE_PIN);
  if (saveState != lastSaveState && now - lastSaveDebounce > DEBOUNCE_MS) {
    lastSaveDebounce = now;
    lastSaveState = saveState;
    if (saveState == LOW) {
      Serial.println("SAVE_PRESS");
    }
  }

  // 2) DELETE button (D3)
  bool deleteState = digitalRead(BUTTON_DELETE_PIN);
  if (deleteState != lastDeleteState && now - lastDeleteDebounce > DEBOUNCE_MS) {
    lastDeleteDebounce = now;
    lastDeleteState = deleteState;
    if (deleteState == LOW) {
      Serial.println("DELETE_PRESS");
    }
  }

  // 3) HISTORY button (D4)
  bool joyBtnState = digitalRead(JOY_BUTTON_PIN);
  if (joyBtnState != lastJoyBtnState && now - lastJoyBtnDebounce > DEBOUNCE_MS) {
    lastJoyBtnDebounce = now;
    lastJoyBtnState = joyBtnState;
    if (joyBtnState == LOW) {
      Serial.println("HISTORY");
    }
  }

  // 4) JOYSTICK Y → UP / DOWN
  int yRaw = analogRead(JOY_Y_PIN);
  if (yRaw > JOY_DOWN_THRESHOLD) {
    if (!lastDownSent && now - lastDownDebounce > DEBOUNCE_MS) {
      Serial.println("DOWN");
      lastDownSent = true;
      lastDownDebounce = now;
    }
  } else {
    lastDownSent = false;
  }
  if (yRaw < JOY_UP_THRESHOLD) {
    if (!lastUpSent && now - lastUpDebounce > DEBOUNCE_MS) {
      Serial.println("UP");
      lastUpSent = true;
      lastUpDebounce = now;
    }
  } else {
    lastUpSent = false;
  }

  // —— NEW: JOYSTICK X held left or right for 3 s → SHUTDOWN_PRESS ——  
  int xRaw = analogRead(JOY_X_PIN);
  bool xFulll = (xRaw < JOY_LEFT_THRESHOLD);
  bool xFullr = (xRaw > JOY_RIGHT_THRESHOLD);

  if (xFulll || xFullr) {
    if (!xHoldActive) {
      xHoldActive  = true;
      xHoldStart   = now;
      shutdownSent = false;
    }
    else if (!shutdownSent && now - xHoldStart >= 3000) {
      Serial.println("SHUTDOWN_PRESS");
      shutdownSent = true;
    }
  } else {
    // joystick returned to center
    xHoldActive  = false;
    shutdownSent = false;
  }
  // ————————————————————————————————

  delay(10);
}
