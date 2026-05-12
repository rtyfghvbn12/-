#include <Arduino.h>

/********************************************************
 *  КОНСТАНТЫ И ПАРАМЕТРЫ
 ********************************************************/

#define NUM_SECTORS 12

// UART лидара
#define LIDAR_RX_PIN 16   // TX лидара -> GPIO16 ESP32
#define LIDAR_TX_PIN -1   // не используется
#define BAUDRATE 115200

const unsigned long STATUS_PRINT_INTERVAL = 100;

// Пакет лидара
static const uint8_t LIDAR_HEADER[] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t LIDAR_HEADER_LEN = 4;
static const uint8_t LIDAR_BODY_LEN = 32;

// Пороги
#define ALARM_DIST 400
#define WARNING_DIST 650
#define ALARM_HOLD_MS 300
#define SECTOR_OFFSET 1

/********************************************************
 *  ГЛОБАЛЬНЫЕ МАССИВЫ
 ********************************************************/

static float sectorDistances[NUM_SECTORS] = { 0.0f };
static uint32_t sectorUpdateTime[NUM_SECTORS] = { 0 };
static uint32_t sectorAlarmUntil[NUM_SECTORS] = { 0 };

static const float NO_VALUE = 99999.0f;

/********************************************************
 *  ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
 ********************************************************/

bool readBytesWithTimeout(HardwareSerial &ser, uint8_t *buffer, size_t length, uint32_t timeout_ms = 500) {
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

  while (true) {
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

    if (millis() - start > 100) {
      return false;
    }
  }
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

/********************************************************
 *  ПАРСИНГ ПАКЕТА ЛИДАРА
 ********************************************************/

bool parseAndProcessPacket() {
  if (!waitForHeader(Serial1)) {
    return false;
  }

  uint8_t buffer[LIDAR_BODY_LEN];

  if (!readBytesWithTimeout(Serial1, buffer, LIDAR_BODY_LEN, 500)) {
    Serial.println("Failed to read 32 bytes");
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

    if (dist != NO_VALUE && dist < ALARM_DIST) {
      sectorAlarmUntil[s] = now + ALARM_HOLD_MS;
    }
  }

  return true;
}

/********************************************************
 *  ВЫВОД СТАТУСОВ СЕКТОРОВ
 ********************************************************/

void printSectorStatuses() {
  Serial.print("SECTORS: ");

  for (int s = 0; s < NUM_SECTORS; s++) {
    int sectorStatus;
    float dist = sectorDistances[s];

    if (dist == NO_VALUE) {
      sectorStatus = 0;
    } else if (millis() < sectorAlarmUntil[s]) {
      sectorStatus = 2;
    } else if (dist < WARNING_DIST) {
      sectorStatus = 1;
    } else {
      sectorStatus = 0;
    }

    Serial.print(sectorStatus);

    if (s < NUM_SECTORS - 1) {
      Serial.print(",");
    }
  }

  Serial.println();
}

/********************************************************
 *  SETUP / LOOP
 ********************************************************/

void setup() {
  Serial.begin(BAUDRATE);

  // Лидар работает только на приём:
  // TX лидара -> GPIO16 ESP32
  Serial1.begin(BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  Serial.println("ESP32 Lidar parser. No LEDs, no UART2.");
}

void loop() {
  parseAndProcessPacket();

  static unsigned long lastPrintMillis = 0;

  if (millis() - lastPrintMillis >= STATUS_PRINT_INTERVAL) {
    lastPrintMillis = millis();
    printSectorStatuses();
  }
}