#include <Wire.h>
#include <Arduino.h>
#include "kalman.h"
#include "BluetoothSerial.h"
#include "wheelControl.h"
#include "bmi160_driver.h"
#include "encoder_driver.h"
#include "PID.h"
#include "LQR.h"
// Check if Bluetooth is enabled in the ESP32 core
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


// ==========================================================
// CONSTANTS
// ==========================================================
constexpr float TICK_IN_CM = 15/1040.0f; // 15cm per 1040 ticks
constexpr float TICK_TO_METERS = TICK_IN_CM / 100.0f; // 0.000136 meters per tick
constexpr float DEG_TO_RAD_FACTOR = PI / 180.0f;

constexpr uint32_t CONTROL_PERIOD_US = 2500 * 1.5;                
constexpr uint32_t SAMPLE_PERIOD_US  = CONTROL_PERIOD_US / 2; // 400 Hz sensing/telemetry
constexpr float CONTROL_DT = CONTROL_PERIOD_US / 1000000.0f;  // dt for PID (control-rate)
constexpr float SAMPLE_DT  = SAMPLE_PERIOD_US  / 1000000.0f;  // dt for Kalman (sample-rate

const int bmi160_addr = 0x68;
const int sda_pin     = 21;     // ESP32 Hardware Default SDA
const int scl_pin     = 22;     // ESP32 Hardware Default SCL
// Balance Loop Tunings
const float balKp = 9.7 / DEG_TO_RAD_FACTOR;
const float balKi = 0.0;
const float balKd = 0.45 / DEG_TO_RAD_FACTOR;
// Position Loop Tunings
// Position Loop Tunings (Outputs radians instead of degrees)
const float posKp = (0.0001 * DEG_TO_RAD_FACTOR) / TICK_TO_METERS;
const float posKi = 0.0;
const float posKd = (0.05 * DEG_TO_RAD_FACTOR) / TICK_TO_METERS;
// Correcting robot's balance
const float MECHANICAL_ZERO = -1.4;
// ==========================================================
// DATA STRUCTURES
// ==========================================================

// Structure to hold standard SI units (Meters, Radians, Seconds)
struct IMUstate {
    float angle_rad;
    float target_angle_rad;
    float rate_rad_s;
};
  
struct EncoderState {
    float pos_left_m;
    float speed_left_m_s;
    float pos_right_m;
    float speed_right_m_s;
};


struct SystemInput
{
    float u_left_normalized;
    float u_right_normalized;
};

// Structure to hold all data needed for printing/transmitting
struct TelemetryPacket {
  bool imuSuccess;
  float ax, ay, az;
  float gx, gy, gz;
  float filtered_angle;
  float filtered_rate;
  float enc_pos1;
  float enc_speed1;
  float enc_pos2;
  float enc_speed2;
  float target_angle;
  float u_left;
  float u_right;
  float time_taken;
  uint32_t current_time;};

BMI160Data imuData;

// Compact struct strictly for System ID over Bluetooth
struct __attribute__((packed)) SystemIdWirePacket {
  uint16_t header = 0xABCD; // Magic marker to prevent desync
  float   filtered_angle;
  float   target_angle;
  float   filtered_rate;
  float enc_pos1;
  float enc_speed1;
  float enc_pos2;
  float enc_speed2;
  float   u_left;
  float   u_right;
  float   time_taken;
  uint32_t current_time;
};

// ==========================================================
// GLOBAL OBJECTS
// ==========================================================
BMI160Driver bmi160(bmi160_addr);
PID balancePID(balKp, balKi, balKd, -255.0, 255.0);
PID positionPID(posKp, posKi, posKd, -3.0 * DEG_TO_RAD_FACTOR, 3.0 * DEG_TO_RAD_FACTOR);// FreeRTOS Queue Handle
QueueHandle_t telemetryQueue;
BluetoothSerial SerialBT;
LQR lqrController;


// ==========================================================
// UTILITY FUNCTIONS
// ==========================================================
// Conversion function for IMU data (Degrees -> Radians)
IMUstate convertIMUtoSensibleUnits(float angle_deg, float target_angle, float rate_dps) {
    IMUstate state;
    
    // Degrees -> Radians
    state.angle_rad = angle_deg * DEG_TO_RAD_FACTOR;
    state.target_angle_rad = target_angle * DEG_TO_RAD_FACTOR;
    state.rate_rad_s = rate_dps * DEG_TO_RAD_FACTOR;
    
    return state;
}

// Conversion function for Encoder data (Ticks -> Meters)
EncoderState convertEncoderToSensibleUnits(int32_t pos_left_ticks, int16_t speed_left_tps, 
                                           int32_t pos_right_ticks, int16_t speed_right_tps) {
    EncoderState state;
    
    // Ticks -> Meters
    state.pos_left_m = static_cast<float>(pos_left_ticks) * TICK_TO_METERS;
    state.speed_left_m_s = static_cast<float>(speed_left_tps) * TICK_TO_METERS;

    state.pos_right_m = static_cast<float>(pos_right_ticks) * TICK_TO_METERS;
    state.speed_right_m_s = static_cast<float>(speed_right_tps) * TICK_TO_METERS;
    
    return state;
}

// Conversion function for System Input (PWM -> Normalized)
SystemInput convertSystemInputToSensibleUnits(int16_t u_left_pwm, int16_t u_right_pwm) {
    SystemInput state;
    
    // Normalized PWM for matrix B scaling
    state.u_left_normalized = static_cast<float>(u_left_pwm) / 255.0f;
    state.u_right_normalized = static_cast<float>(u_right_pwm) / 255.0f;
    
    return state;
}

void Batched_telemetryTask(void *pvParameters) {
  TelemetryPacket packet;

  // Batch 40 packets (100ms of data at 400Hz)
  const uint8_t BATCH_SIZE = 40; 
  SystemIdWirePacket wireBuffer[BATCH_SIZE];
  uint8_t bufferIndex = 0;

  for (;;) {
    // Wait for a new packet from Core 1
    if (xQueueReceive(telemetryQueue, &packet, portMAX_DELAY) == pdPASS) {
      if (!packet.imuSuccess) {
        continue; // Skip failed IMU reads
      }

      // Pack only the essential variables into the buffer
      wireBuffer[bufferIndex].header         = 0xABCD;
      wireBuffer[bufferIndex].filtered_angle = packet.filtered_angle;
      wireBuffer[bufferIndex].target_angle   = packet.target_angle;
      wireBuffer[bufferIndex].filtered_rate  = packet.filtered_rate;
      wireBuffer[bufferIndex].enc_pos1       = packet.enc_pos1;
      wireBuffer[bufferIndex].enc_speed1     = packet.enc_speed1;
      wireBuffer[bufferIndex].enc_pos2       = packet.enc_pos2;
      wireBuffer[bufferIndex].enc_speed2     = packet.enc_speed2;
      wireBuffer[bufferIndex].u_left         = packet.u_left;
      wireBuffer[bufferIndex].u_right        = packet.u_right;
      wireBuffer[bufferIndex].time_taken     = packet.time_taken;
      wireBuffer[bufferIndex].current_time   = packet.current_time;
      
      bufferIndex++;

      // Once the buffer hits 40, blast the 800-byte chunk over Bluetooth
      if (bufferIndex >= BATCH_SIZE) {
        SerialBT.write(
          reinterpret_cast<const uint8_t*>(wireBuffer),
          sizeof(wireBuffer)
        );
        bufferIndex = 0; // Reset for the next batch
      }
    }
  }
}

void Serial_telemetryTask(void *pvParameters) {
  static int counter = 0;
  TelemetryPacket packet;

  for (;;) {
    // Wait for a new packet from Core 1
    if (xQueueReceive(telemetryQueue, &packet, portMAX_DELAY) == pdPASS) {
      if (!packet.imuSuccess) {
        continue; // Skip failed IMU reads
      }

      if (++counter >= 10) { // Print every 10th packet
        counter = 0;

        // Print the telemetry data to Bluetooth
        SerialBT.printf(
          "Angle: %.2f rad, Target: %.2f rad, Rate: %.2f rad/s | Left: %.2f m, %.2f m/s | Right: %.2f m, %.2f m/s | u: %.2f, %.2f | Time taken: %.3f ms\n",
          packet.filtered_angle,
          packet.target_angle,
          packet.filtered_rate,
          packet.enc_pos1,
          packet.enc_speed1,
          packet.enc_pos2,
          packet.enc_speed2,
          packet.u_left,
          packet.u_right,
          packet.time_taken
        );
      }
    }
  }
}

// ==========================================================
// VARIABLES
// ==========================================================
int noiseCounter = 0;
float currentNoiseLeft = 0.0f;
float currentNoiseRight = 0.0f;
uint32_t nextControlTime = 0;
String inputBuffer = "";
unsigned long lastTime = 0;

uint32_t nextSampleTime = 0;
bool controlTick = false; // flips every sample; true on samples where control law runs

// Held between control updates (zero-order hold), also used for telemetry in between
float u_left = 0.0f, u_right = 0.0f;
int32_t pwmLeft = 0, pwmRight = 0;
float telemetryPwmLeft = 0.0f, telemetryPwmRight = 0.0f;



// ==========================================================
void setup() {
  Serial.begin(500000);
  while (!Serial);
  
  Wire.begin(sda_pin, scl_pin);
  Wire.setClock(400000);
  
  if (!bmi160.begin(sda_pin, scl_pin, 400000)) {
    Serial.println("BMI160 initialization failed! Check wiring/power.");
    while (1);
  }

  Serial.println("BMI160 initialized successfully!");

  initEncoders();

  telemetryQueue = xQueueCreate(10, sizeof(TelemetryPacket));

  xTaskCreatePinnedToCore(
    Batched_telemetryTask, "TelemetryTask", 4096, NULL, 1, NULL, 0
  );

  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  digitalWrite(PIN_STBY, HIGH);
  lastTime = micros();
  nextControlTime = lastTime + CONTROL_PERIOD_US;
  nextSampleTime = lastTime + SAMPLE_PERIOD_US;

  SerialBT.begin("ESP32_Robot_BT"); // Name of your Bluetooth device
  Serial.println("Bluetooth started! Ready to pair as 'ESP32_Robot_BT'");
  
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - nextSampleTime) < 0) {
    return;
  }
  nextSampleTime += SAMPLE_PERIOD_US;

  // ========================================================
  // SENSING (runs every sample, 800 Hz)
  // ========================================================
  float &raw_gx = imuData.gx_dps;
  float &raw_gy = imuData.gy_dps;
  float &raw_gz = imuData.gz_dps;
  float &raw_ax = imuData.ax_g;
  float &raw_ay = imuData.ay_g;
  float &raw_az = imuData.az_g;

  bool imuSuccess = bmi160.readSensor(imuData);
  EncoderData encData = getEncoderData();

  float filtered_angle = 0.0f;
  float filtered_rate = 0.0f;
  float _ = 0.0f;

  updateKalman(
    raw_ay, raw_az, raw_gx,
    SAMPLE_DT,                 
    filtered_angle,
    filtered_rate,
    _
  );

  IMUstate imuState = convertIMUtoSensibleUnits(
    filtered_angle, MECHANICAL_ZERO, filtered_rate);

  EncoderState encoderState = convertEncoderToSensibleUnits(
    encData.pos1, encData.speed1,
    encData.pos2, encData.speed2);

  // ========================================================
  // CONTROL (runs every other sample)
  // ========================================================
  controlTick = !controlTick;

  if (controlTick) {
    float avg_pos = (encoderState.pos_left_m + encoderState.pos_right_m) / 2.0f;
    float avg_speed = (encoderState.speed_left_m_s + encoderState.speed_right_m_s) / 2.0f;

    float angle_adjustment = positionPID.compute(
      0.0f, avg_pos, avg_speed, CONTROL_DT, false);

    float dynamic_target_angle = imuState.target_angle_rad + angle_adjustment;

    float u = balancePID.compute(
      dynamic_target_angle,
      imuState.angle_rad,
      imuState.rate_rad_s,
      CONTROL_DT);

    u_left = u;
    u_right = u;

    setMotorOutputs(u_left, u_right, pwmLeft, pwmRight);

    if (fabs(u_left) > 0.001f) {
      telemetryPwmLeft = (u_left < 0) ? pwmLeft : -pwmLeft;
    }
    if (fabs(u_right) > 0.001f) {
      telemetryPwmRight = (u_right < 0) ? pwmRight : -pwmRight;
    }
  }
  // else: no new control action this sample — u_left/u_right/pwm*/telemetryPwm*
  // stay at their last-computed (held) values, matching what the H-bridge is
  // physically doing between control updates.

  SystemInput systemInput = convertSystemInputToSensibleUnits(
    telemetryPwmLeft, telemetryPwmRight);

  // ========================================================
  // TELEMETRY (runs every sample)
  // ========================================================
  float time_taken = (int32_t)(micros() - now) / 1000.0f;

  TelemetryPacket currentData = {
    imuSuccess,
    raw_ax, raw_ay, raw_az,
    raw_gx, raw_gy, raw_gz,
    imuState.angle_rad,
    imuState.rate_rad_s,
    encoderState.pos_left_m,
    encoderState.speed_left_m_s,
    encoderState.pos_right_m,
    encoderState.speed_right_m_s,
    imuState.target_angle_rad,
    systemInput.u_left_normalized,
    systemInput.u_right_normalized,
    time_taken,
    micros() // current_time
    
  };
  xQueueSend(telemetryQueue, &currentData, 0);
}