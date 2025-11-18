#include <Wire.h>
#include <AccelStepper.h>
#include <AS5600.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <util/atomic.h>

#define STEPPER1_DIR_PIN 6
#define STEPPER1_STEP_PIN 5
#define STEPPER2_DIR_PIN 12
#define STEPPER2_STEP_PIN 13
#define FAN1_PWM_PIN 9
#define FAN1_IN1 10
#define FAN1_IN2 11
#define FAN2_PWM_PIN 4
#define FAN2_IN1 2
#define FAN2_IN2 3

#define STEPS_PER_REV 200
#define MICROSTEPS 32
#define TARGET_ANGLE 45.0f
#define TEMP_MIN 25.0f
#define TEMP_MAX 40.0f
#define TEMP2_MIN 30.0f
#define TEMP2_MAX 50.0f
#define SPEED_MIN_PERCENT 30
#define FAN_CHECK_INTERVAL 2000

const long STEPS_FULL_REV = (long)STEPS_PER_REV * MICROSTEPS;
const float STEPS_PER_DEG = (float)STEPS_FULL_REV / 360.0f;
const long MOVE_STEPS = (long)(TARGET_ANGLE * STEPS_PER_DEG);

AS5600 as5600;
AccelStepper stepper1(1, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(1, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
Adafruit_HTU21DF htu;
Adafruit_BMP280 bmp;

volatile float initialEncoderAngle = 0.0f;
volatile uint8_t stepper1State = 0;
volatile uint8_t stepper2State = 0;
volatile unsigned long state1PauseTime = 0;
volatile unsigned long state2PauseTime = 0;
unsigned long lastFanCheck = 0;
unsigned long lastFan2Check = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(FAN1_PWM_PIN, OUTPUT);
  pinMode(FAN1_IN1, OUTPUT);
  pinMode(FAN1_IN2, OUTPUT);
  digitalWrite(FAN1_IN1, HIGH);
  digitalWrite(FAN1_IN2, LOW);

  pinMode(FAN2_PWM_PIN, OUTPUT);
  pinMode(FAN2_IN1, OUTPUT);
  pinMode(FAN2_IN2, OUTPUT);
  digitalWrite(FAN2_IN1, HIGH);
  digitalWrite(FAN2_IN2, LOW);

  stepper1.setMaxSpeed(800.0f);
  stepper1.setAcceleration(400.0f);
  stepper1.setCurrentPosition(0);

  stepper2.setMaxSpeed(800.0f);
  stepper2.setAcceleration(400.0f);
  stepper2.setCurrentPosition(0);

  delay(100);

  initialEncoderAngle = as5600.readAngle() * (360.0f / 4096.0f);

  if (!htu.begin()) {
    while (1);
  }
  
  if (!bmp.begin(0x76)) {
    while (1);
  }

  stepper1.moveTo(MOVE_STEPS);
  stepper2.moveTo(MOVE_STEPS);
}

void loop() {
  stepper1.run();
  stepper2.run();

  if (stepper1.distanceToGo() == 0) handleStepper1FSM();
  if (stepper2.distanceToGo() == 0) handleStepper2FSM();

  unsigned long currentMillis = millis();
  
  if (currentMillis - lastFanCheck >= FAN_CHECK_INTERVAL) {
    handleFan1(currentMillis);
  }

  if (currentMillis - lastFan2Check >= (FAN_CHECK_INTERVAL + 50)) {
    handleFan2(currentMillis);
  }
}

void handleStepper1FSM() {
  unsigned long now = millis();
  uint8_t currentState;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currentState = stepper1State;
  }

  switch (currentState) {
    case 0:
      verifyAngle(TARGET_ANGLE);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state1PauseTime = now;
        stepper1State = 1;
      }
      break;
    case 1:
      if (now - state1PauseTime >= 1000) {
        stepper1.moveTo(0);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { stepper1State = 2; }
      }
      break;
    case 2:
      verifyAngle(0.0f);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state1PauseTime = now;
        stepper1State = 3;
      }
      break;
    case 3:
      if (now - state1PauseTime >= 1000) {
        stepper1.moveTo(-MOVE_STEPS);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { stepper1State = 4; }
      }
      break;
    case 4:
      verifyAngle(-TARGET_ANGLE);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state1PauseTime = now;
        stepper1State = 5;
      }
      break;
    case 5:
      if (now - state1PauseTime >= 1000) {
        stepper1.moveTo(0);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { stepper1State = 6; }
      }
      break;
    case 6:
      verifyAngle(0.0f);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state1PauseTime = now;
        stepper1State = 7;
      }
      break;
    case 7:
      if (now - state1PauseTime >= 2000) {
        stepper1.moveTo(MOVE_STEPS);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { stepper1State = 0; }
      }
      break;
  }
}

void handleStepper2FSM() {
  unsigned long now = millis();
  uint8_t currentState;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currentState = stepper2State;
  }

  switch (currentState) {
    case 0:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state2PauseTime = now;
        stepper2State = 1;
      }
      break;
    case 1:
      if (now - state2PauseTime >= 1000) {
        stepper2.moveTo(0);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { stepper2State = 2; }
      }
      break;
    case 2:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state2PauseTime = now;
        stepper2State = 3;
      }
      break;
    case 3:
      if (now - state2PauseTime >= 1000) {
        stepper2.moveTo(-MOVE_STEPS);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { stepper2State = 4; }
      }
      break;
    case 4:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state2PauseTime = now;
        stepper2State = 5;
      }
      break;
    case 5:
      if (now - state2PauseTime >= 1000) {
        stepper2.moveTo(0);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { stepper2State = 6; }
      }
      break;
    case 6:
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state2PauseTime = now;
        stepper2State = 7;
      }
      break;
    case 7:
      if (now - state2PauseTime >= 2000) {
        stepper2.moveTo(MOVE_STEPS);
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { stepper2State = 0; }
      }
      break;
  }
}

void verifyAngle(float target) {
  float raw = as5600.readAngle() * (360.0f / 4096.0f);
  if (raw == 0.0f) return;
  
  float rel = (raw - initialEncoderAngle) * -1.0f;
  if (rel > 180.0f) rel -= 360.0f;
  else if (rel < -180.0f) rel += 360.0f;
  
  Serial.print(rel); Serial.print(" / "); Serial.println(target);
}

void handleFan1(unsigned long now) {
  lastFanCheck = now;
  float t = htu.readTemperature();
  if (isnan(t)) {
    setFanSpeed(FAN1_PWM_PIN, 100);
    return;
  }
  uint8_t spd = calculateSpeed(t, TEMP_MIN, TEMP_MAX);
  setFanSpeed(FAN1_PWM_PIN, spd);
}

void handleFan2(unsigned long now) {
  lastFan2Check = now;
  float t = bmp.readTemperature();
  if (isnan(t)) {
    setFanSpeed(FAN2_PWM_PIN, 100);
    return;
  }
  uint8_t spd = calculateSpeed(t, TEMP2_MIN, TEMP2_MAX);
  setFanSpeed(FAN2_PWM_PIN, spd);
}

uint8_t calculateSpeed(float t, float tMin, float tMax) {
  if (t <= tMin) return 0;
  if (t >= tMax) return 100;
  return map((long)(t * 10), (long)(tMin * 10), (long)(tMax * 10), SPEED_MIN_PERCENT, 100);
}

void setFanSpeed(uint8_t pin, uint8_t pct) {
  if (pct > 100) pct = 100;
  analogWrite(pin, map(pct, 0, 100, 0, 255));
}