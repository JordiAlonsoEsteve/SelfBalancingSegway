#include "encoder_driver.h"
#include <ESP32Encoder.h>

// Pin Definitions
static const int MOTOR1_ENC_A = 16;
static const int MOTOR1_ENC_B = 23;

static const int MOTOR2_ENC_A = 26;
static const int MOTOR2_ENC_B = 27;

// Hardware Encoder Objects (PCNT)
static ESP32Encoder encoder1;
static ESP32Encoder encoder2;

// Shared data state and spinlock for dual-core thread safety
static EncoderData g_encoderData = {0, 0, 0, 0};
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

// FreeRTOS task running on Core 0
static void encoderTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(25); // 25 ms interval (40 Hz)

  int32_t prevPos1 = 0;
  int32_t prevPos2 = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    int32_t currentPos1 = (int32_t)encoder1.getCount();
    int32_t currentPos2 = (int32_t)encoder2.getCount();

    // Speed calculation in counts/sec over 25ms window (1000ms / 25ms = multiplier of 40)
    int16_t speed1 = (int16_t)((currentPos1 - prevPos1) * 40);
    int16_t speed2 = (int16_t)((currentPos2 - prevPos2) * 40);

    prevPos1 = currentPos1;
    prevPos2 = currentPos2;

    // Safely update global state across cores
    portENTER_CRITICAL(&encoderMux);
    g_encoderData.pos1 = currentPos1;
    g_encoderData.speed1 = speed1;
    g_encoderData.pos2 = currentPos2;
    g_encoderData.speed2 = speed2;
    portEXIT_CRITICAL(&encoderMux);
  }
}

void initEncoders() {
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encoder1.attachFullQuad(MOTOR1_ENC_A, MOTOR1_ENC_B);
  encoder2.attachFullQuad(MOTOR2_ENC_A, MOTOR2_ENC_B);

  encoder1.clearCount();
  encoder2.clearCount();

  // Create task on Core 0 (alongside Bluetooth)
  xTaskCreatePinnedToCore(
    encoderTask,
    "EncoderTask",
    2048,
    NULL,
    1,
    NULL,
    0 // Core 0
  );
}

EncoderData getEncoderData() {
  EncoderData dataCopy;
  portENTER_CRITICAL(&encoderMux);
  dataCopy = g_encoderData;
  portEXIT_CRITICAL(&encoderMux);
  return dataCopy;
}