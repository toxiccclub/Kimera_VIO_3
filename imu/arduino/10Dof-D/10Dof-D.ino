#include "Waveshare_10Dof-D.h"
#include "imudata.h"

#define legPPS 2

// int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

volatile uint8_t acqWaiting = 0;
volatile uint8_t stampReady = 0;

volatile uint8_t flag = 0;

uint8_t i;

volatile uint32_t oldTime = 0;
volatile uint32_t newTime = 0;
volatile uint32_t innerTimeSpan = 0;

volatile uint32_t timestamp = 0;
volatile uint32_t timeSpan = 0;

IMU_ST_ANGLES_DATA stAngles;
IMU_ST_SENSOR_DATA stGyroRawData;
IMU_ST_SENSOR_DATA stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;

uint16_t command;

ImuData datum;

bool sensorOk = true;
// bool waitingData = true;  // false;
// int pps;

char buffer[40];


void setup() {
  IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
  Serial.begin(115200);

  imuInit(&enMotionSensorType, &enPressureType);
  if (IMU_EN_SENSOR_TYPE_ICM20948 != enMotionSensorType) {
    Serial.println(
        "Motion sensor ICM-20948 was not detected. No IMU data will be sent");
    sensorOk = false;
    pinMode(LED_BUILTIN, OUTPUT);
  } else {
    sensorOk = true;
    pinMode(legPPS, INPUT);
  }

  datum.startA = 0xF0;
  datum.startB = 0xF1;

  datum.stopA = 0xFA;
  datum.stopB = 0xFB;
  attachInterrupt(0, pulse, RISING);
  Serial.flush();
  delay(1000);
  Serial.flush();
  delay(500);
}

void pulse() {
  newTime = micros();
  if (oldTime == 0) {
    oldTime = newTime;
    return;
  }
  innerTimeSpan = (newTime - oldTime) / 10;
  oldTime = newTime;

  if (acqWaiting == 1 && stampReady == 0 && flag == 0) {
    timestamp = newTime;
    timeSpan = innerTimeSpan;
    stampReady = 1;
  }
}

void loop() {
  if (!sensorOk) {
    digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(1500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    return;
  }

  if (acqWaiting == 0) {
    auto read = Serial.readBytes((uint8_t*)&command, sizeof(uint16_t));
    if (read == sizeof(uint16_t) && command == 0xFFFF) {
      acqWaiting = 1;
      return;
    } else {
      acqWaiting = 0;
      return;
    }
  }

  if (stampReady == 1) {
    flag = 1;
    for (i = 0; i < 10; ++i) {
      imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);

      datum.accelerometerRawX = stAccelRawData.s16X;
      datum.accelerometerRawY = stAccelRawData.s16Y;
      datum.accelerometerRawZ = stAccelRawData.s16Z;

      datum.gyroscopeRawX = stGyroRawData.s16X;
      datum.gyroscopeRawY = stGyroRawData.s16Y;
      datum.gyroscopeRawZ = stGyroRawData.s16Z;

      datum.timestamp = timestamp + timeSpan * i;

      sprintf(buffer,
              "%lu,%d,%d,%d,%d,%d,%d",
              datum.timestamp,
              datum.gyroscopeRawX,
              datum.gyroscopeRawY,
              datum.gyroscopeRawZ,
              datum.accelerometerRawX,
              datum.accelerometerRawY,
              datum.accelerometerRawZ);

      Serial.println(buffer);

      Serial.flush();
      delayMicroseconds(timeSpan);
    }

    acqWaiting = 0;
    stampReady = 0;
    flag = 0;
  }
}
