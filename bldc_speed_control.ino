/*
 * IoT BLDC Motor Control with Hall Sensor Diagnostics
 * - Real-time hall sensor pulse monitoring
 * - Detailed debugging for motor and sensor status
 * - Blynk app control with live feedback
 * 
 * Wiring:
 * - ESC Signal: D6
 * - Hall Sensor: D5 (with pull-up)
 * - LCD: I2C (SDA=D2, SCL=D1)
 */

#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL3eGMQm88Z"
#define BLYNK_TEMPLATE_NAME "bldc motor"
#define BLYNK_AUTH_TOKEN "XJW-1SUJBxHpvB_aGMn4qvxWO6sjiT87"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include <Ticker.h>
#include <LiquidCrystal_I2C.h>

char ssid[] = "Infinix HOT 30i";
char pass[] = "11111118";

// ========== PIN DEFINITIONS ==========
const uint8_t escPin = D6;
const uint8_t hallPin = D5;

// ========== BLYNK VIRTUAL PINS ==========
#define VPIN_SETPOINT_RPM    V0  // Slider for desired RPM (0-1000)
#define VPIN_ACTUAL_RPM      V1  // Display actual RPM
#define VPIN_MOTOR_ON        V2  // Switch to enable motor
#define VPIN_THROTTLE        V3  // Display current throttle %
#define VPIN_HALL_STATUS     V4  // Hall sensor status (NEW)
#define VPIN_HALL_PULSES     V5  // Total pulse count (NEW)
#define VPIN_TEST_MODE       V6  // Manual PWM test (0-100%) (NEW)

// ========== OBJECTS ==========
Ticker escPWM;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ========== GLOBAL VARIABLES ==========
volatile unsigned long pwmPulseWidthUs = 1000;
volatile unsigned long hallPulseCount = 0;
volatile unsigned long lastHallTime = 0;
volatile unsigned long lastHallPulseTime = 0;

// Motor control variables
int desiredRPM = 0;
int throttlePercent = 0;
int actualRPM = 0;
bool motorEnabled = false;
bool hallSensorActive = false;

// Hall sensor diagnostics
unsigned long hallPulsesPerSecond = 0;
unsigned long lastPulseInterval = 0;  // Time between pulses in microseconds

// Timing
unsigned long lastLCDUpdate = 0;
unsigned long lastBlynkUpdate = 0;
unsigned long lastHallCheck = 0;

// ========== PWM GENERATION ==========
void sendESCPulse() {
  digitalWrite(escPin, HIGH);
  delayMicroseconds(pwmPulseWidthUs);
  digitalWrite(escPin, LOW);
}

// ========== HALL SENSOR INTERRUPT ==========
ICACHE_RAM_ATTR void hallSensorISR() {
  unsigned long currentMicros = micros();
  
  // Calculate pulse interval
  if (lastHallPulseTime > 0) {
    lastPulseInterval = currentMicros - lastHallPulseTime;
  }
  
  lastHallPulseTime = currentMicros;
  hallPulseCount++;
  lastHallTime = millis();
}

// ========== RPM CALCULATION ==========
void calculateRPM() {
  static unsigned long lastCalcTime = 0;
  static unsigned long lastPulseCount = 0;
  
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastCalcTime;
  
  if (deltaTime >= 500) {  // Calculate every 500ms
    unsigned long pulses = hallPulseCount - lastPulseCount;
    
    // Calculate pulses per second
    hallPulsesPerSecond = (pulses * 1000) / deltaTime;
    
    // RPM calculation - ADJUST pulsesPerRevolution for your motor
    // Common values: 1, 2, 3, or 6 pulses per revolution
    const int pulsesPerRevolution = 1;  // ← CHANGE THIS if needed!
    
    if (pulses > 0) {
      actualRPM = (pulses * 60000) / (deltaTime * pulsesPerRevolution);
      hallSensorActive = true;
    }
    
    lastPulseCount = hallPulseCount;
    lastCalcTime = currentTime;
  }
  
  // Timeout detection - motor stopped or sensor not working
  if (millis() - lastHallTime > 1000) {
    actualRPM = 0;
    hallPulsesPerSecond = 0;
    if (motorEnabled && throttlePercent > 0) {
      hallSensorActive = false;  // Sensor not detecting despite motor running
    }
  }
}

// ========== RPM TO THROTTLE CONVERSION ==========
int rpmToThrottle(int targetRPM) {
  const int MIN_RPM = 0;
  const int MAX_RPM = 8000;
  
  int throttle = map(targetRPM, MIN_RPM, MAX_RPM, 25, 100);
  return constrain(throttle, 25, 100);
}

// ========== SET THROTTLE TO ESC ==========
void setThrottle(int percent) {
  if (!motorEnabled) {
    pwmPulseWidthUs = 1000;  // Stop motor
    throttlePercent = 0;
    Serial.println("→ setThrottle: Motor disabled - PWM = 1000us");
    return;
  }
  
  // Constrain input first
  int requestedThrottle = constrain(percent, 0, 100);
  
  // Apply minimum throttle to ensure motor spins
  const int MIN_THROTTLE = 25;  // 25% minimum for reliable startup
  
  if (requestedThrottle > 0 && requestedThrottle < MIN_THROTTLE) {
    throttlePercent = MIN_THROTTLE;
  } else {
    throttlePercent = requestedThrottle;
  }
  
  unsigned long oldPWM = pwmPulseWidthUs;
  pwmPulseWidthUs = map(throttlePercent, 0, 100, 1000, 2000);
  
// ========== BLYNK CALLBACKS ==========
BLYNK_WRITE(VPIN_MOTOR_ON) {
  motorEnabled = param.asInt();

  if (!motorEnabled) {
    setThrottle(0);
    hallSensorActive = false;
    return;
  }

  // Motor is ON
  if (desiredRPM == 0) {
    setThrottle(0);  // Do NOT auto-boost to 25%
  }
  // kickstart
  else {
    throttlePercent = rpmToThrottle(desiredRPM);
    setThrottle(throttlePercent);
  }
}
BLYNK_WRITE(VPIN_SETPOINT_RPM) {
  desiredRPM = param.asInt();  // Read slider value

  // If motor ON, update throttle immediately
  if (motorEnabled) {
   if (desiredRPM == 0) {
    setThrottle(0);  // STOP motor at 0 RPM
   } else {
    throttlePercent = rpmToThrottle(desiredRPM);
    setThrottle(throttlePercent);
   }

  }
}

BLYNK_CONNECTED() {
  Serial.println("\n★★★ Blynk Connected - Syncing values ★★★");
  Blynk.syncVirtual(VPIN_SETPOINT_RPM);
  Blynk.syncVirtual(VPIN_MOTOR_ON);
}

// ========== LCD UPDATE ==========
void updateLCD() {
  // Line 1: Set RPM and Actual RPM
  lcd.setCursor(0, 0);
  lcd.print("S:");
  lcd.print(desiredRPM);
  
  if (desiredRPM < 10) lcd.print("   ");
  else if (desiredRPM < 100) lcd.print("  ");
  else if (desiredRPM < 1000) lcd.print(" ");
  
  lcd.print("A:");
  lcd.print(actualRPM);
  
  if (actualRPM < 10) lcd.print("   ");
  else if (actualRPM < 100) lcd.print("  ");
  else if (actualRPM < 1000) lcd.print(" ");
  
  // Line 2: Throttle, Status, and Hall sensor
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(throttlePercent);
  if (throttlePercent < 10) lcd.print("% ");
  else if (throttlePercent < 100) lcd.print("%");
  else lcd.print("");
  
  lcd.print(" ");
  
  if (motorEnabled) {
    lcd.print("ON ");
  } else {
    lcd.print("OFF");
  }
  
  // Hall sensor indicator
  lcd.print(" H:");
  lcd.print(hallPulsesPerSecond);
  lcd.print("  ");
}

// ========== SETUP ==========
void setup() {
  
  // Initialize LCD
  Wire.begin(D2, D1);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BLDC + Hall");
  lcd.setCursor(0, 1);
  lcd.print("Diagnostics v2.0");
  delay(2000);
  
  // Initialize pins
  pinMode(escPin, OUTPUT);
  pinMode(hallPin, INPUT_PULLUP);
  digitalWrite(escPin, LOW);
  
  // Start with MINIMUM throttle
  pwmPulseWidthUs = 1000;
  
  // Start PWM signal
  escPWM.attach_ms(20, sendESCPulse);
  delay(2000);
  
  // Attach hall sensor interrupt
  attachInterrupt(digitalPinToInterrupt(hallPin), hallSensorISR, FALLING);
  // Test hall sensor
  delay(1000);
  int hallState = digitalRead(hallPin);
  
  // Connect to WiFi
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi...");
  
  WiFi.begin(ssid, pass);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi OK");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(2000);
    
    // Connect to Blynk
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Blynk...");
    Blynk.config(BLYNK_AUTH_TOKEN);
    if (Blynk.connect()) {
      Serial.println("✓ Blynk Connected!");
      lcd.setCursor(0, 1);
      lcd.print("Connected!");
      delay(2000);
    } else {
      Serial.println("✗ Blynk Failed!");
      lcd.setCursor(0, 1);
      lcd.print("Failed!");
      delay(2000);
    }
  } else {
    Serial.println("\n✗ WiFi Failed!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed!");
    delay(3000);
  }
  
  // Final initialization
  pwmPulseWidthUs = 1000;
  throttlePercent = 0;
  desiredRPM = 0;
  motorEnabled = false;
  hallSensorActive = false;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  lcd.setCursor(0, 1);
  lcd.print("Motor: OFF");
  delay(2000);
  lcd.clear();
}

// ========== MAIN LOOP ==========
void loop() {
  Blynk.run();
  calculateRPM();
  checkHallSensor();
  
  // Update LCD (200ms)
  if (millis() - lastLCDUpdate >= 200) {
    lastLCDUpdate = millis();
    updateLCD();
  }
  
  // Send data to Blynk (500ms)
  if (millis() - lastBlynkUpdate >= 500) {
    lastBlynkUpdate = millis();
    Blynk.virtualWrite(VPIN_ACTUAL_RPM, actualRPM);
    Blynk.virtualWrite(VPIN_THROTTLE, throttlePercent);
    Blynk.virtualWrite(VPIN_HALL_STATUS, hallSensorActive ? "Active" : "Inactive");
    Blynk.virtualWrite(VPIN_HALL_PULSES, hallPulseCount);
  }
