#include <Arduino.h>
#include <GyverStepper.h>
#include <ESP32Servo.h>

// ===================== ПИНЫ =====================

const int ROUTE_PIN = 26;
const int START_PIN = 27;

// Лидар
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN -1w
#define LIDAR_BAUDRATE 115200w

// До 6 серв
#define SERVO_UNUSED -1

const int SERVO_PINS[6] = {
  33,            // servo 0
  25,            // servo 1
  SERVO_UNUSED,  // servo 2
  SERVO_UNUSED,  // servo 3
  SERVO_UNUSED,  // servo 4
  SERVO_UNUSED   // servo 5
};

const int SERVO_COUNT = 6;

// Шаговые двигатели
GStepper<STEPPER2WIRE> stepper1(800, 19, 18, 17);
GStepper<STEPPER2WIRE> stepper2(800, 23, 22, 21);
int time_timer = 0;
// Сервы
Servo servos[SERVO_COUNT];

// ===================== СЕКТОРА ЛИДАРА =====================

#define NUM_SECTORS 12

#define SEC(n) (1 << (n))

#define SECTORS_NONE 0
#define SECTORS_ALL  0x0FFF

#define SECTORS_FRONT (SEC(7) | SEC(8) | SEC(9) | SEC(10) | SEC(11) | SEC(0))
#define SECTORS_BACK  (SEC(1) | SEC(2) | SEC(3) | SEC(4) | SEC(5) | SEC(6))

int sectorStatus[NUM_SECTORS] = { 0 };

// ===================== ПАРАМЕТРЫ ЛИДАРА =====================

static const uint8_t LIDAR_HEADER[] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t LIDAR_HEADER_LEN = 4;
static const uint8_t LIDAR_BODY_LEN = 32;

#define ALARM_DIST 270
#define WARNING_DIST 450
#define ALARM_HOLD_MS 300
#define SECTOR_OFFSET 1

static float sectorDistances[NUM_SECTORS] = { 0.0f };
static uint32_t sectorUpdateTime[NUM_SECTORS] = { 0 };
static uint32_t sectorAlarmUntil[NUM_SECTORS] = { 0 };

static const float NO_VALUE = 99999.0f;

// ===================== ТИПЫ ШАГОВ =====================

enum StepType {
  MOVE_STEPPERS,
  SERVO_MOVE,
  PAUSE_STEP,
  END_ROUTE
};

struct RouteStep {
  StepType type;

  long motor1Steps;
  long motor2Steps;

  int servoIndex;
  int servoAngle;

  unsigned long timeMs;

  uint16_t checkSectors;
};

// ===================== МАРШРУТЫ =====================
//
// Формат:
// {ТИП, motor1, motor2, servoIndex, servoAngle, timeMs, checkSectors}

RouteStep route1[] = {
  {MOVE_STEPPERS, 15000, 15000, -1,  0,    0, SECTORS_FRONT},
  {MOVE_STEPPERS, -600, -600, -1,  0,    0, SECTORS_BACK},
  {MOVE_STEPPERS, 1500, -1500, -1,  0,    0, SECTORS_ALL},
  {MOVE_STEPPERS, 2700, 2700, -1,  0,    0, SECTORS_FRONT},
  {MOVE_STEPPERS, 1500, -1500, -1,  0,    0, SECTORS_ALL},
  {MOVE_STEPPERS, -1000, -1000, -1,  0,    0, SECTORS_BACK},
  {MOVE_STEPPERS, 14000, 14000, -1,  0,    0, SECTORS_FRONT},
    {MOVE_STEPPERS, -13100, -13100, -1,  0,    0, SECTORS_BACK},
    {MOVE_STEPPERS, -1500, 1500, -1,  0,    0, SECTORS_ALL},
  {MOVE_STEPPERS, 2000, 2000, -1,  0,    0, SECTORS_FRONT},
  {SERVO_MOVE,    0,    0,     1,  0,  500, SECTORS_ALL},
  {MOVE_STEPPERS, -5100, -5100, -1,  0,    0, SECTORS_BACK},
  {SERVO_MOVE,    0,    0,     1, 43,  500, SECTORS_ALL},
  {MOVE_STEPPERS, 1900, 1900,  -1,  0,    0, SECTORS_FRONT},
  {MOVE_STEPPERS, 1500, -1500, -1,  0,    0, SECTORS_ALL},
  {MOVE_STEPPERS, -1500, -1500, -1,  0,    0, SECTORS_BACK},
  {MOVE_STEPPERS, 15000, 15000, -1,  0,    0, SECTORS_FRONT},
  {END_ROUTE,     0,    0,    -1,  0,    0, SECTORS_NONE}
};

// ИЗМЕНЁННЫЙ route2 – зеркальный к route1
RouteStep route2[] = {
  {MOVE_STEPPERS, 15300, 15300, -1,  0,    0, SECTORS_NONE},
  {MOVE_STEPPERS, -800, -800, -1,  0,    0, SECTORS_NONE},
  {MOVE_STEPPERS, 1500, -1500, -1,  0,    0, SECTORS_NONE},
  {MOVE_STEPPERS, 2700, 2700, -1,  0,    0,  SECTORS_NONE},
  {MOVE_STEPPERS, 1500, -1500, -1,  0,    0, SECTORS_NONE},
  {MOVE_STEPPERS, -1000, -1000, -1,  0,    0, SECTORS_NONE},
  {MOVE_STEPPERS, 14000, 14000, -1,  0,    0, SECTORS_FRONT},
  {MOVE_STEPPERS, -1700, -1700, -1,  0,    0, SECTORS_NONE},
  {MOVE_STEPPERS, -1500, 1500, -1,  0,    0, SECTORS_NONE},
  {MOVE_STEPPERS, -1500, -1500, -1,  0,    0, SECTORS_NONE},
  {MOVE_STEPPERS, -1500, 1500, -1,  0,    0, SECTORS_NONE},
  {MOVE_STEPPERS, -5000, -5000,  -1,  0,    0, SECTORS_NONE},
 //{MOVE_STEPPERS, 1500, -1500, -1,  0,    0, SECTORS_NONE},
 // {MOVE_STEPPERS, -1500, -1500, -1,  0,    0, SECTORS_FRONT},
  //{MOVE_STEPPERS, 15000, 15000, -1,  0,    0, SECTORS_FRONT},
  //{END_ROUTE,     0,    0,    -1,  0,    0, SECTORS_NONE}
};

// ===================== ПЕРЕМЕННЫЕ МАРШРУТА =====================

RouteStep* currentRoute;

int currentStepIndex = 0;

bool stepStarted = false;
bool motorsWereMoving = false;

unsigned long stepStartTime = 0;

int activeServoIndex = -1;

// ===================== ЛИДАР =====================

bool readBytesWithTimeout(HardwareSerial &ser, uint8_t *buffer, size_t length, uint32_t timeout_ms = 2) {
  uint32_t start = millis();
  size_t count = 0;

  while (count < length) {
    if (ser.available()) {
      buffer[count++] = ser.read();
    }

    if (millis() - start > timeout_ms) {
      return false;
    }
  }

  return true;
}

bool waitForHeader(HardwareSerial &ser) {
  uint8_t matchPos = 0;
  uint32_t start = millis();

  while (millis() - start < 1) {
    if (ser.available()) {
      uint8_t b = ser.read();

      if (b == LIDAR_HEADER[matchPos]) {
        matchPos++;

        if (matchPos == LIDAR_HEADER_LEN) {
          return true;
        }
      } else {
        matchPos = 0;
      }
    }
  }

  return false;
}

float decodeAngle(uint16_t rawAngle) {
  float angleDeg = (float)(rawAngle - 0xA000) / 64.0f;

  while (angleDeg < 0) angleDeg += 360.0f;
  while (angleDeg >= 360) angleDeg -= 360.0f;

  return angleDeg;
}

int angleToSector(float angleDeg) {
  float shifted = angleDeg + 15.0f;

  while (shifted < 0) shifted += 360.0f;
  while (shifted >= 360) shifted -= 360.0f;

  int sector = (int)(shifted / 30.0f) % NUM_SECTORS;
  sector = (sector + SECTOR_OFFSET) % NUM_SECTORS;

  return sector;
}

void updateSectorStatuses() {
  uint32_t now = millis();

  for (int s = 0; s < NUM_SECTORS; s++) {
    float dist = sectorDistances[s];

    if (dist == NO_VALUE) {
      sectorStatus[s] = 0;
    } else if (now < sectorAlarmUntil[s]) {
      sectorStatus[s] = 2;
    } else if (dist < WARNING_DIST) {
      sectorStatus[s] = 1;
    } else {
      sectorStatus[s] = 0;
    }
  }
}

bool parseAndProcessLidarPacket() {
  if (!waitForHeader(Serial1)) {
    updateSectorStatuses();
    return false;
  }

  uint8_t buffer[LIDAR_BODY_LEN];

  if (!readBytesWithTimeout(Serial1, buffer, LIDAR_BODY_LEN, 2)) {
    updateSectorStatuses();
    return false;
  }

  uint16_t startAngleTmp = buffer[2] | (buffer[3] << 8);
  float startAngleDeg = decodeAngle(startAngleTmp);

  uint8_t offset = 4;

  uint16_t distances[8];
  uint8_t intensities[8];

  for (int i = 0; i < 8; i++) {
    distances[i] = buffer[offset] | (buffer[offset + 1] << 8);
    intensities[i] = buffer[offset + 2];
    offset += 3;
  }

  uint16_t endAngleTmp = buffer[offset] | (buffer[offset + 1] << 8);
  float endAngleDeg = decodeAngle(endAngleTmp);

  if (endAngleDeg < startAngleDeg) {
    endAngleDeg += 360.0f;
  }

  float angleRange = endAngleDeg - startAngleDeg;
  float angleInc = angleRange / 8.0f;

  float tempSectorMin[NUM_SECTORS];

  for (int s = 0; s < NUM_SECTORS; s++) {
    tempSectorMin[s] = NO_VALUE;
  }

  for (int i = 0; i < 8; i++) {
    if (intensities[i] > 15) {
      float angle = startAngleDeg + i * angleInc;

      while (angle < 0) angle += 360.0f;
      while (angle >= 360) angle -= 360.0f;

      int sectorIndex = angleToSector(angle);
      float dist = (float)distances[i];

      if (dist < tempSectorMin[sectorIndex]) {
        tempSectorMin[sectorIndex] = dist;
      }
    }
  }

  uint32_t now = millis();

  for (int s = 0; s < NUM_SECTORS; s++) {
    if (tempSectorMin[s] != NO_VALUE) {
      sectorDistances[s] = tempSectorMin[s];
      sectorUpdateTime[s] = now;
    }
  }

  for (int s = 0; s < NUM_SECTORS; s++) {
    if ((now - sectorUpdateTime[s]) > 500) {
      sectorDistances[s] = NO_VALUE;
    }
  }

  for (int s = 0; s < NUM_SECTORS; s++) {
    float dist = sectorDistances[s];

    if (dist < ALARM_DIST && dist>100) {
      sectorAlarmUntil[s] = now + ALARM_HOLD_MS;
    }
  }

  updateSectorStatuses();
  return true;
}

bool isBlockedByLidar(uint16_t checkSectors) {
  if (checkSectors == SECTORS_NONE) {
    return false;
  }

  for (int s = 0; s < NUM_SECTORS; s++) {
    if (checkSectors & SEC(s)) {
      if (sectorStatus[s] == 2) {
        return true;
      }
    }
  }

  return false;
}

void printSectorStatuses() {
  Serial.print("SECTORS: ");

  for (int s = 0; s < NUM_SECTORS; s++) {
    Serial.print(sectorStatus[s]);

    if (s < NUM_SECTORS - 1) {
      Serial.print(",");
    }
  }

  Serial.println();
}

// ===================== МОТОРЫ =====================

void setupSteppers() {
  stepper1.setRunMode(FOLLOW_POS);
  stepper2.setRunMode(FOLLOW_POS);

  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(3000);

  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(3000);

  stepper1.autoPower(true);
  stepper2.autoPower(true);

  stepper1.enable();
  stepper2.enable();
}

// ===================== СЕРВЫ =====================

bool isServoAvailable(int index) {
  if (index < 0 || index >= SERVO_COUNT) return false;
  if (SERVO_PINS[index] == SERVO_UNUSED) return false;
  return true;
}

void attachAndWriteServo(int index, int angle) {
  if (!isServoAvailable(index)) {
    Serial.print("SERVO ");
    Serial.print(index);
    Serial.println(" is unused. Skip.");
    return;
  }

  angle = constrain(angle, 0, 180);

  servos[index].attach(SERVO_PINS[index], 500, 2500);
  servos[index].write(angle);

  activeServoIndex = index;

  Serial.print("SERVO ");
  Serial.print(index);
  Serial.print(" on pin ");
  Serial.print(SERVO_PINS[index]);
  Serial.print(" -> ");
  Serial.println(angle);
}

void detachActiveServo() {
  if (isServoAvailable(activeServoIndex)) {
    servos[activeServoIndex].detach();

    Serial.print("SERVO ");
    Serial.print(activeServoIndex);
    Serial.println(" DETACH");
  }

  activeServoIndex = -1;
}

// ===================== ЗАПУСК ШАГА =====================

void startRouteStep(RouteStep step) {
  stepStartTime = millis();

  if (step.type == MOVE_STEPPERS) {
    motorsWereMoving = false;

    Serial.print("MOVE: ");
    Serial.print(step.motor1Steps);
    Serial.print(" / ");
    Serial.println(step.motor2Steps);

    stepper1.setTarget(step.motor1Steps, RELATIVE);
    stepper2.setTarget(step.motor2Steps, RELATIVE);
  }

  else if (step.type == SERVO_MOVE) {
    attachAndWriteServo(step.servoIndex, step.servoAngle);
  }

  else if (step.type == PAUSE_STEP) {
    Serial.print("PAUSE: ");
    Serial.print(step.timeMs);
    Serial.println(" ms");
  }

  stepStarted = true;
}

// ===================== ПРОВЕРКА ЗАВЕРШЕНИЯ ШАГА =====================

bool isRouteStepFinished(RouteStep step, byte stepper1Move, byte stepper2Move) {
  if (step.type == MOVE_STEPPERS) {
    if (stepper1Move != 0 || stepper2Move != 0) {
      motorsWereMoving = true;
    }

    return motorsWereMoving && stepper1Move == 0 && stepper2Move == 0;
  }

  if (step.type == SERVO_MOVE) {
    if (millis() - stepStartTime >= step.timeMs) {
      detachActiveServo();
      return true;
    }

    return false;
  }

  if (step.type == PAUSE_STEP) {
    return millis() - stepStartTime >= step.timeMs;
  }

  return false;
}

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);

  pinMode(ROUTE_PIN, INPUT_PULLUP);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(14, OUTPUT);
  Serial1.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  setupSteppers();

  Serial.println("Robot route program with lidar safety");

  Serial.println("Servo configuration:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(": ");

    if (SERVO_PINS[i] == SERVO_UNUSED) {
      Serial.println("UNUSED");
    } else {
      Serial.print("GPIO ");
      Serial.println(SERVO_PINS[i]);
    }
  }

  if (digitalRead(ROUTE_PIN) == LOW) {
    currentRoute = route2;
    Serial.println("Selected route: 2 (mirrored)");
  } else {
    currentRoute = route1;
    Serial.println("Selected route: 1");
  }

  Serial.println("Waiting START_PIN press...");

  while (digitalRead(START_PIN) == HIGH) {
    parseAndProcessLidarPacket();
    delay(1);
  }
  time_timer = millis();
  Serial.println("START!");
}

// ===================== LOOP (исправленный, с защитой, без рывков) =====================

void loop() {

  if (time_timer + 90000< millis()){
  RouteStep currentStep = currentRoute[currentStepIndex];

  if (currentStep.type == END_ROUTE) {
    Serial.println("ROUTE FINISHED");

    while (true) {
      parseAndProcessLidarPacket();
      delay(10);
    }
  }

  bool blocked = isBlockedByLidar(currentStep.checkSectors);

  static bool wasBlocked = false;
  static long remaining1 = 0;
  static long remaining2 = 0;

  if (blocked && !wasBlocked) {
    // Вычисляем оставшиеся шаги (сколько ещё нужно проехать)
    remaining1 = stepper1.getTarget() - stepper1.getCurrent();
    remaining2 = stepper2.getTarget() - stepper2.getCurrent();
    // Останавливаем моторы
    stepper1.setTarget(0, RELATIVE);
    stepper2.setTarget(0, RELATIVE);
    Serial.println("LIDAR BLOCKED - motors stopped");
    digitalWrite(14,1);
    wasBlocked = true;
  }
  else if (!blocked && wasBlocked) {
    // Возобновляем движение с оставшимися шагами
    if (remaining1 != 0) stepper1.setTarget(remaining1, RELATIVE);
    if (remaining2 != 0) stepper2.setTarget(remaining2, RELATIVE);
    motorsWereMoving = false;
    Serial.println("LIDAR CLEAR - resuming");
    digitalWrite(14,0);
    wasBlocked = false;
  }

  // ВСЕГДА вызываем tick() – это устраняет рывки
  byte stepper1Move = stepper1.tick();
  byte stepper2Move = stepper2.tick();

  parseAndProcessLidarPacket();

  if (blocked) {
    delay(1);
    return;
  }

  if (!stepStarted) {
    startRouteStep(currentStep);
  }

  if (isRouteStepFinished(currentStep, stepper1Move, stepper2Move)) {
    Serial.print("Step finished: ");
    Serial.println(currentStepIndex);

    currentStepIndex++;
    stepStarted = false;
    motorsWereMoving = false;
    remaining1 = remaining2 = 0;
    delay(10);
  }
  }
}
