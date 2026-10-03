#pragma once

#include <Arduino.h>
#include <Wire.h>

// https://www.mouser.com/datasheet/2/783/BST-BMI160-DS000-1509569.pdf?srsltid=AfmBOorlDqGw3mSnC4matrRgUmdm_NKhPUnTDEewLf--AGLuGkLYcshX
// Scalers based on default BMI160 ranges (±2g and ±2000dps/s)
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 16.384f // Matches default ±2000 dps range

struct BMI160Data {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  float ax_g, ay_g, az_g;
  float gx_dps, gy_dps, gz_dps;
};

class BMI160Driver {
public:
  BMI160Driver(uint8_t address = 0x68);
  
  bool begin(int sdaPin = 21, int sclPin = 22, uint32_t frequency = 100000);
  bool readSensor(BMI160Data &data);
  void setOffsets(float ax, float ay, float az, float gx, float gy, float gz);

private:
  uint8_t _address;
  float _ax_offset = -0.0419f, _ay_offset = 0.0301f, _az_offset = 0.0074f;
  float _gx_offset = 0.0441f, _gy_offset = 0.3343f, _gz_offset = 0.3221f;

  bool writeRegister(uint8_t reg, uint8_t value);
};

