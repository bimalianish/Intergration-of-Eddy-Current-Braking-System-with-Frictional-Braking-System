#include <LiquidCrystal_I2C.h>

// Pin definitions
#define SPEED_SENSOR_PIN 2      // LM393 speed sensor (interrupt pin)
#define BRAKE_HALL_PIN 3        // Hall effect sensor for brake pedal
#define SOLENOID_PIN 8          // 12V solenoid control pin
#define LCD_SDA A4              // I2C SDA pin
#define LCD_SCL A5              // I2C SCL pin

// Constants
#define PULSES_PER_REVOLUTION 20 // Adjust based on your aluminum disk holes/teeth
#define WHEEL_CIRCUMFERENCE 2.0  // Wheel circumference in meters (adjust for your vehicle)
#define SPEED_THRESHOLD 40       // Speed threshold in km/hr

// Variables
volatile unsigned long pulseCount = 0;
unsigned long lastTime = 0;
float vehicleSpeed = 0;
bool solenoidActive = false;
bool brakePressed = false;

// LCD setup (I2C address 0x27, 16x2)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(9600);
  
  // Pin modes
  pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
  pinMode(BRAKE_HALL_PIN, INPUT_PULLUP);
  pinMode(SOLENOID_PIN, OUTPUT);
  
  // Initialize solenoid off
  digitalWrite(SOLENOID_PIN, LOW);
  
  // Attach interrupt for speed sensor
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), speedPulse, FALLING);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Speed Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();
  
  Serial.println("Speed monitoring system started");
}

void loop() {
  // Calculate speed every 500ms
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 500) {
    calculateSpeed();
    lastTime = currentTime;
  }
  
  // Read brake pedal status
  brakePressed = !digitalRead(BRAKE_HALL_PIN); // Assuming LOW when magnet is near
  
  // Control solenoid based on speed and brake status
  controlSolenoid();
  
  // Update LCD display
  updateDisplay();
  
  delay(100); // Small delay for stability
}

void speedPulse() {
  pulseCount++;
}

void calculateSpeed() {
  static unsigned long lastPulseCount = 0;
  unsigned long currentPulseCount = pulseCount;
  unsigned long pulseDiff = currentPulseCount - lastPulseCount;
  
  // Calculate RPM
  float rpm = (pulseDiff * 60.0 * 2.0) / PULSES_PER_REVOLUTION; // *2 because we calculate every 0.5 seconds
  
  // Calculate speed in km/hr
  vehicleSpeed = (rpm * WHEEL_CIRCUMFERENCE * 60) / 1000.0;
  
  // Ensure speed is not negative or unreasonably high
  if (vehicleSpeed < 0) vehicleSpeed = 0;
  if (vehicleSpeed > 200) vehicleSpeed = 0; // Reset if unrealistic speed
  
  lastPulseCount = currentPulseCount;
  
  // Debug output
  Serial.print("Pulses: ");
  Serial.print(pulseDiff);
  Serial.print(" | Speed: ");
  Serial.print(vehicleSpeed);
  Serial.println(" km/hr");
}

void controlSolenoid() {
  // Activate solenoid if speed > 40 km/hr and brake is NOT pressed
  if (vehicleSpeed > SPEED_THRESHOLD && !brakePressed) {
    if (!solenoidActive) {
      digitalWrite(SOLENOID_PIN, HIGH);
      solenoidActive = true;
      Serial.println("Solenoid ACTIVATED - Magnetic brake engaged");
    }
  } else {
    if (solenoidActive) {
      digitalWrite(SOLENOID_PIN, LOW);
      solenoidActive = false;
      Serial.println("Solenoid DEACTIVATED - Magnetic brake disengaged");
    }
  }
}

void updateDisplay() {
  // First line: Vehicle speed
  lcd.setCursor(0, 0);
  lcd.print("Speed: ");
  lcd.print(vehicleSpeed, 1);
  lcd.print(" km/h  ");
  
  // Second line: System status
  lcd.setCursor(0, 1);
  if (solenoidActive) {
    lcd.print("MAG BRAKE: ON   ");
  } else if (brakePressed) {
    lcd.print("BRAKE PRESSED   ");
  } else {
    lcd.print("NORMAL DRIVING  ");
  }
}
