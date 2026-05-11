#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

AccelStepper stepper(AccelStepper::DRIVER, 26, 27);

#define HALL_SPEED_PIN  23
#define HALL_BRAKE_PIN  35
#define ENA_PIN         25

// ================= WHEEL CONFIG =================
#define MAGNETS         8           // 8 physical magnets on wheel
#define WHEEL_RADIUS_M  0.2032f     // tyre radius in metres (user specified)

#define PI_VAL          3.14159f

// ================= STEPPER / DISPLACEMENT CONFIG =================

#define STEPS_PER_REV        200      // NEMA17 full steps/rev
#define LEAD_MM              2.0f     // lead screw pitch in mm
#define TARGET_MM            2.0f     // ← TUNE: desired travel in mm
#define ACTUATOR_SPEED_KMPH  15.0f    // ← TUNE: speed threshold (km/h)
#define STEPPER_MAX_SPEED    4000     // ← TUNE: steps/sec
#define STEPPER_ACCEL        8000     // ← TUNE: steps/sec²


#define TARGET_STEPS  ((long)((TARGET_MM / LEAD_MM) * STEPS_PER_REV))

// ================= NOISE / FILTER TUNING =================
#define PULSE_DEBOUNCE_US      10000
#define PIN_CONFIRM_DELAY_US    2000
#define PIN_CONFIRM_DELAY2_US   4000
#define SPEED_WINDOW_MS          500
#define RPM_ZERO_THRESHOLD         2
#define RPM_MAX_SANITY           800
#define ENGAGE_STREAK              3
#define DISENGAGE_STREAK           4

// ================= BRAKING DISTANCE CONFIG =================
#define BRAKE_TRIGGER_KMPH   15.0f
#define BRAKE_DISPLAY_MS     10000

// ================= VARIABLES =================
volatile int           pulseCount     = 0;
volatile unsigned long lastPulseTime  = 0;
volatile unsigned long noiseRejected  = 0;

unsigned long          lastSpeedTime  = 0;
float                  rpm            = 0.0f;
float                  speed_kmph     = 0.0f;
float                  speed_ms       = 0.0f;

int                    lastRawPulses  = 0;
int                    lastNoise      = 0;

int                    engageCount    = 0;
int                    disengageCount = 0;
bool                   rpmActive      = false;

bool                   movedForward        = false;
bool                   brakeStateRaw       = false;
bool                   brakeStateDebounced = false;
unsigned long          brakeLastChangeTime = 0;

enum BrakeDistState { BD_IDLE, BD_MEASURING, BD_HOLDING };
BrakeDistState  bdState          = BD_IDLE;
float           brakeStartSpeed  = 0.0f;
float           brakeDist_m      = 0.0f;
unsigned long   brakeStartTime   = 0;
unsigned long   brakeStopTime    = 0;
unsigned long   holdStartTime    = 0;
float           lastBrakeDist_m  = 0.0f;
float           lastBrakeInitSpd = 0.0f;

// ================= INTERRUPT =================
void IRAM_ATTR countPulse() {
  unsigned long now = micros();
  if (now - lastPulseTime < PULSE_DEBOUNCE_US) return;

  delayMicroseconds(PIN_CONFIRM_DELAY_US);
  if (digitalRead(HALL_SPEED_PIN) != LOW) { noiseRejected++; return; }

  delayMicroseconds(PIN_CONFIRM_DELAY_US);
  if (digitalRead(HALL_SPEED_PIN) != LOW) { noiseRejected++; return; }

  pulseCount++;
  lastPulseTime = now;

  unsigned long lockoutStart = micros();
  while (digitalRead(HALL_SPEED_PIN) == LOW) {
    if (micros() - lockoutStart > 100000UL) break;
  }
}

// ================= BRAKE =================
void updateBrake() {
  bool rawPressed = (digitalRead(HALL_BRAKE_PIN) == LOW);
  if (rawPressed != brakeStateRaw) {
    brakeStateRaw = rawPressed;
    brakeLastChangeTime = millis();
  }
  if ((millis() - brakeLastChangeTime) >= 50) {
    brakeStateDebounced = brakeStateRaw;
  }
}
bool isBrakePressed() { return brakeStateDebounced; }

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  Serial.println("==============================================");
  Serial.println("  BRAKE DISTANCE MEASUREMENT SYSTEM");
  Serial.println("==============================================");
  Serial.print("Wheel radius : "); Serial.print(WHEEL_RADIUS_M, 4); Serial.println(" m");
  Serial.print("Circumference: ");
  Serial.print(2.0f * PI_VAL * WHEEL_RADIUS_M, 4); Serial.println(" m");
  Serial.print("Trigger speed: "); Serial.print(BRAKE_TRIGGER_KMPH); Serial.println(" km/h");
  Serial.println("----------------------------------------------");
  Serial.print("Target travel: "); Serial.print(TARGET_MM, 1);      Serial.println(" mm");
  Serial.print("Target steps : "); Serial.println(TARGET_STEPS);
  Serial.print("Accel trigger: "); Serial.print(ACTUATOR_SPEED_KMPH, 0); Serial.println(" km/h");
  Serial.print("Max speed    : "); Serial.print(STEPPER_MAX_SPEED);  Serial.println(" steps/s");
  Serial.print("Acceleration : "); Serial.print(STEPPER_ACCEL);      Serial.println(" steps/s²");
  Serial.println("==============================================");

  pinMode(HALL_SPEED_PIN, INPUT_PULLUP);
  pinMode(HALL_BRAKE_PIN, INPUT_PULLUP);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), countPulse, FALLING);

  // AccelStepper smooth motion — handles ramp up/down automatically
  stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper.setAcceleration(STEPPER_ACCEL);
  stepper.setCurrentPosition(0);   // home = 0

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("BRAKE DIST SYS  ");
  lcd.setCursor(0, 1); lcd.print("Ready...        ");
  delay(1500);
  lcd.clear();

  lastSpeedTime = millis();
}

// ================= LOOP =================
void loop() {
  updateBrake();
  calculateSpeed();
  updateBrakeDist();
  controlMotor();
  displayData();
  stepper.run();   // MUST be every loop — this is what drives AccelStepper
}

// ================= SPEED CALC =================
void calculateSpeed() {
  if (millis() - lastSpeedTime < SPEED_WINDOW_MS) return;

  detachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN));
  int  pulses   = pulseCount;
  int  noise    = (int)noiseRejected;
  pulseCount    = 0;
  noiseRejected = 0;
  unsigned long elapsed = millis() - lastSpeedTime;
  lastSpeedTime = millis();
  attachInterrupt(digitalPinToInterrupt(HALL_SPEED_PIN), countPulse, FALLING);

  lastRawPulses = pulses;
  lastNoise     = noise;
  float timeSec = elapsed / 1000.0f;

  float raw_rpm = 0.0f;
  if (pulses >= RPM_ZERO_THRESHOLD) {
    raw_rpm = ((float)pulses / (float)MAGNETS) * (60.0f / timeSec);
  }
  if (raw_rpm > RPM_MAX_SANITY) raw_rpm = 0.0f;

  if (!rpmActive) {
    if (raw_rpm > 0.0f) {
      engageCount++;
      disengageCount = 0;
      if (engageCount >= ENGAGE_STREAK) { rpmActive = true; engageCount = 0; }
    } else { engageCount = 0; }
  } else {
    if (raw_rpm == 0.0f) {
      disengageCount++;
      if (disengageCount >= DISENGAGE_STREAK) {
        rpmActive = false; disengageCount = 0; engageCount = 0;
      }
    } else { disengageCount = 0; }
  }

  if (rpmActive && raw_rpm > 0.0f) {
    rpm = 0.7f * rpm + 0.3f * raw_rpm;
  } else if (rpmActive && raw_rpm == 0.0f) {
    rpm = rpm * 0.6f;
    if (rpm < 1.0f) rpm = 0.0f;
  } else {
    rpm = rpm * 0.2f;
    if (rpm < 0.5f) rpm = 0.0f;
  }

  float circumference = 2.0f * PI_VAL * WHEEL_RADIUS_M;
  speed_kmph = rpm * circumference * 60.0f / 1000.0f;
  speed_ms   = speed_kmph / 3.6f;
}

// ================= BRAKING DISTANCE STATE MACHINE =================
void updateBrakeDist() {
  static unsigned long lastBDTime = 0;
  unsigned long now = millis();

  switch (bdState) {

    case BD_IDLE:
      if (speed_kmph >= BRAKE_TRIGGER_KMPH && isBrakePressed()) {
        bdState         = BD_MEASURING;
        brakeStartSpeed = speed_kmph;
        brakeDist_m     = 0.0f;
        brakeStartTime  = now;
        lastBDTime      = now;

        Serial.println();
        Serial.println(">>> BRAKING EVENT STARTED <<<");
        Serial.print("    Initial speed : "); Serial.print(brakeStartSpeed, 1);
        Serial.println(" km/h");
        Serial.println("    t(ms)   SPD(km/h)   DIST(m)");
        Serial.println("    -------+----------+---------");
      }
      break;

    case BD_MEASURING: {
      if (!isBrakePressed()) {
        Serial.println(">>> BRAKING ABORTED (brake released before stop) <<<");
        bdState = BD_IDLE;
        break;
      }

      float dtSec  = (now - lastBDTime) / 1000.0f;
      brakeDist_m += speed_ms * dtSec;
      lastBDTime   = now;

      static unsigned long lastPrint = 0;
      if (now - lastPrint >= 200) {
        lastPrint = now;
        Serial.print("    "); Serial.print(now - brakeStartTime);
        Serial.print("\t"); Serial.print(speed_kmph, 2);
        Serial.print("\t\t"); Serial.println(brakeDist_m, 3);
      }

      if (rpm == 0.0f && !rpmActive) {
        brakeStopTime    = now;
        lastBrakeDist_m  = brakeDist_m;
        lastBrakeInitSpd = brakeStartSpeed;
        holdStartTime    = now;
        bdState          = BD_HOLDING;

        Serial.println("    -------+----------+---------");
        Serial.println();
        Serial.println("╔══════════════════════════════════╗");
        Serial.println("║     BRAKING DISTANCE RESULT      ║");
        Serial.print("║  Initial speed : "); Serial.print(lastBrakeInitSpd, 1); Serial.println(" km/h         ║");
        Serial.print("║  Braking dist  : "); Serial.print(lastBrakeDist_m, 3);  Serial.println(" m            ║");
        Serial.print("║  Braking time  : "); Serial.print((brakeStopTime - brakeStartTime) / 1000.0f, 2); Serial.println(" s             ║");
        Serial.println("╚══════════════════════════════════╝");
        Serial.println("  (Displaying result for 10 seconds...)");
        Serial.println();
      }
      break;
    }

    case BD_HOLDING: {
      static unsigned long lastHoldPrint = 0;
      if (now - lastHoldPrint >= 1000) {
        lastHoldPrint = now;
        Serial.print("  RESULT >> Init: "); Serial.print(lastBrakeInitSpd, 1);
        Serial.print(" km/h | Dist: ");     Serial.print(lastBrakeDist_m, 3);
        Serial.print(" m | (");             Serial.print((BRAKE_DISPLAY_MS - (now - holdStartTime)) / 1000);
        Serial.println("s remaining)");
      }
      if (now - holdStartTime >= BRAKE_DISPLAY_MS) {
        bdState = BD_IDLE;
        Serial.println("  Result cleared. Ready for next measurement.");
        Serial.println();
      }
      break;
    }
  }
}

void controlMotor() {
  bool pedalPressed = isBrakePressed();

  if (speed_kmph >= ACTUATOR_SPEED_KMPH && pedalPressed && !movedForward) {
    stepper.moveTo(TARGET_STEPS);
    movedForward = true;
    Serial.print("ACTUATOR EXTEND  → "); Serial.print(TARGET_MM, 1);
    Serial.print(" mm ("); Serial.print(TARGET_STEPS); Serial.println(" steps)");
  }

  if ((speed_kmph < ACTUATOR_SPEED_KMPH || !pedalPressed) && movedForward) {
    stepper.moveTo(0);
    movedForward = false;
    Serial.println("ACTUATOR RETRACT → 0 mm");
  }
}

// ================= DISPLAY =================
void displayData() {
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay < 400) return;
  lastDisplay = millis();

  lcd.setCursor(0, 0);
  lcd.print("R:");
  String rpmStr = String((int)rpm);
  while (rpmStr.length() < 4) rpmStr += ' ';
  lcd.print(rpmStr);

  lcd.setCursor(8, 0);
  lcd.print("S:");
  String spdStr = String(speed_kmph, 1);
  while (spdStr.length() < 5) spdStr += ' ';
  lcd.print(spdStr);

  lcd.setCursor(0, 1);
  if (bdState == BD_MEASURING) {
    lcd.print("BD:");
    String distStr = String(brakeDist_m, 2);
    while (distStr.length() < 5) distStr += ' ';
    lcd.print(distStr);
    lcd.print("m      ");
  } else if (bdState == BD_HOLDING) {
    lcd.print("BD:");
    String distStr = String(lastBrakeDist_m, 2);
    while (distStr.length() < 5) distStr += ' ';
    lcd.print(distStr);
    lcd.print("m DONE ");
  } else {
    lcd.print(isBrakePressed() ? "BRK:ON  " : "BRK:OFF ");
    lcd.setCursor(9, 1);
    lcd.print(movedForward ? "ACT  " : "IDLE ");
  }

  if (bdState == BD_IDLE) {
    Serial.print("RPM:"); Serial.print(rpm, 1);
    Serial.print(" | SPD:"); Serial.print(speed_kmph, 1);
    Serial.print(" km/h | P:"); Serial.print(lastRawPulses);
    Serial.print(" N:"); Serial.print(lastNoise);
    Serial.print(" | ENG:"); Serial.print(engageCount);
    Serial.print(" DIS:"); Serial.print(disengageCount);
    Serial.print(" | "); Serial.print(rpmActive ? "ACTIVE" : "INACTIVE");
    Serial.print(" | BRK:"); Serial.print(isBrakePressed() ? "ON" : "OFF");
    Serial.print(" | ACT:"); Serial.println(movedForward ? "ON" : "OFF");
  }
}
