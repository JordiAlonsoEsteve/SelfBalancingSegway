#pragma once
#include <Arduino.h>

struct EncoderData {
  int32_t pos1;
  int16_t speed1; // counts per second
  int32_t pos2;
  int16_t speed2; // counts per second
};

// Initializes hardware PCNT counters and launches background task on Core 0
void initEncoders();

// Thread-safe getter to read current positions and speeds from main loop or telemetry
EncoderData getEncoderData();
