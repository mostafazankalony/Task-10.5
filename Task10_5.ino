#include "Wire.h"
#define MPU6050_IMU_ADDRESS 0x68
#define GYRO_FULL_SCALE_1000_DPS 0x10
unsigned long lastPrintMillis = 0;
#define INTERVAL_MS_PRINT 1000

struct gyroscope_raw {
  int16_t z;
} gyroscope;

float yaw = 0; 
float gyro_z_calibration = 0;  

void setup() {
  Wire.begin();
  Serial.begin(115200);
  I2CwriteByte(MPU6050_IMU_ADDRESS, 0x6B, 0x00);
  I2CwriteByte(MPU6050_IMU_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);
  calibrateGyro();
}

void loop() {
  unsigned long currentMillis = millis();
  readGyroZ();
  float gyroZ_dps = (gyroscope.z / 32.8) - gyro_z_calibration;
  yaw += gyroZ_dps * (INTERVAL_MS_PRINT / 1000.0);
  if (currentMillis - lastPrintMillis > INTERVAL_MS_PRINT) {
    Serial.print("Yaw (");
    Serial.print("\xC2\xB0");  
    Serial.print("):\t");
    Serial.println(yaw, 2);

    lastPrintMillis = currentMillis;
  }
}

void I2CwriteByte(uint8_t address, uint8_t registerAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();
}


void readGyroZ() {
  Wire.beginTransmission(MPU6050_IMU_ADDRESS);
  Wire.write(0x43);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_IMU_ADDRESS, 6, true);

  Wire.read();  // Discard X-axis high byte
  Wire.read();  // Discard X-axis low byte
  Wire.read();  // Discard Y-axis high byte
  Wire.read();  // Discard Y-axis low byte

  gyroscope.z = (Wire.read() << 8 | Wire.read());  // Z-axis data
}


void calibrateGyro() {
  long sum = 0;
  const int numReadings = 1000;

  for (int i = 0; i < numReadings; i++) {
    readGyroZ();
    sum += gyroscope.z;
    delay(3);
  }

  // Calculate average offset
  gyro_z_calibration = sum / (float)numReadings / 32.8;
}
