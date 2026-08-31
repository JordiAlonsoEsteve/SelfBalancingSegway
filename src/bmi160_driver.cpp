#include "bmi160_driver.h"

BMI160Driver::BMI160Driver(uint8_t address) : _address(address) {}

bool BMI160Driver::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool BMI160Driver::begin(int sdaPin, int sclPin, uint32_t frequency) {
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(frequency);

  // Soft reset
  if (!writeRegister(0x7E, 0xB6)) return false;
  delay(100);

  // Power up Accelerometer (Normal Mode)
  if (!writeRegister(0x7E, 0x11)) return false;
  delay(100);

  // Power up Gyroscope (Normal Mode)
  if (!writeRegister(0x7E, 0x15)) return false;
  delay(100);

  // Verify Chip ID (0xD1 expected for BMI160)
  Wire.beginTransmission(_address);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  
  if (Wire.requestFrom(_address, (uint8_t)1) == 1) {
    return (Wire.read() == 0xD1);
  }

  return false;
}

void BMI160Driver::setOffsets(float ax, float ay, float az, float gx, float gy, float gz) {
  _ax_offset = ax; _ay_offset = ay; _az_offset = az;
  _gx_offset = gx; _gy_offset = gy; _gz_offset = gz;
}

bool BMI160Driver::readSensor(BMI160Data &data) {
  Wire.beginTransmission(_address);
  Wire.write(0x0C); // Start reading at Gyro Data registers
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(_address, (uint8_t)12) == 12) {
    data.gx = (int16_t)(Wire.read() | (Wire.read() << 8));
    data.gy = (int16_t)(Wire.read() | (Wire.read() << 8));
    data.gz = (int16_t)(Wire.read() | (Wire.read() << 8));
    data.ax = (int16_t)(Wire.read() | (Wire.read() << 8));
    data.ay = (int16_t)(Wire.read() | (Wire.read() << 8));
    data.az = (int16_t)(Wire.read() | (Wire.read() << 8));

    // Calculate scaled physical values
    data.ax_g = (data.ax / ACCEL_SCALE) - _ax_offset;
    data.ay_g = (data.ay / ACCEL_SCALE) - _ay_offset;
    data.az_g = (data.az / ACCEL_SCALE) - _az_offset;

    data.gx_dps = (data.gx / GYRO_SCALE) - _gx_offset;
    data.gy_dps = (data.gy / GYRO_SCALE) - _gy_offset;
    data.gz_dps = (data.gz / GYRO_SCALE) - _gz_offset;

    return true;
  }

  return false;
}