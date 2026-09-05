#include <Wire.h>
#include <Arduino.h>
#include "kalman.h"
//#include "complementary_filter.h"
#include "BluetoothSerial.h"
#include "wheelControl.h"
#include "bmi160_driver.h"
#include "encoder_driver.h"
# include "PID.h"
//# include "LQR.h"


// LQR stuff
//LQR lqrController;
//float u_left = 0.0f;
//float u_right = 0.0f;

constexpr float TICK_IN_CM = 0.0136f;
constexpr float TICK_TO_METERS = TICK_IN_CM / 100.0f; // 0.000136 meters per tick
constexpr float DEG_TO_RAD_FACTOR = PI / 180.0f;

// Structure to hold standard SI units (Meters, Radians, Seconds)
struct PhysicalState {
    float angle_rad;
    float rate_rad_s;
    float pos_left_m;
    float speed_left_m_s;
    float pos_right_m;
    float speed_right_m_s;
    float avg_pos_m;
    float avg_speed_m_s;
    float u_left_normalized;  // Optional: -1.0 to 1.0 instead of -255 to 255
    float u_right_normalized;
};

// Conversion function
PhysicalState convertToSensibleUnits(float angle_deg, float rate_dps, 
                                     int32_t pos_left_ticks, int16_t speed_left_tps, 
                                     int32_t pos_right_ticks, int16_t speed_right_tps,
                                     float u_left_pwm, float u_right_pwm) {
    PhysicalState state;
    
    // Degrees -> Radians
    state.angle_rad = angle_deg * DEG_TO_RAD_FACTOR;
    state.rate_rad_s = rate_dps * DEG_TO_RAD_FACTOR;
    
    // Ticks -> Meters (Standard SI is strongly preferred over cm for state-space math)
    state.pos_left_m = static_cast<float>(pos_left_ticks) * TICK_TO_METERS;
    state.speed_left_m_s = static_cast<float>(speed_left_tps) * TICK_TO_METERS;
    
    state.pos_right_m = static_cast<float>(pos_right_ticks) * TICK_TO_METERS;
    state.speed_right_m_s = static_cast<float>(speed_right_tps) * TICK_TO_METERS;
    
    // Convenience averages for the robot center
    state.avg_pos_m = (state.pos_left_m + state.pos_right_m) / 2.0f;
    state.avg_speed_m_s = (state.speed_left_m_s + state.speed_right_m_s) / 2.0f;

    // Map PWM to a percentage [-1.0, 1.0]. 
    // Multiply this by max battery voltage if you want actual Volts for matrix B.
    state.u_left_normalized = u_left_pwm / 255.0f;
    state.u_right_normalized = u_right_pwm / 255.0f;
    
    return state;
}

// ==========================================================
// SYSTEM IDENTIFICATION PARAMETERS
// ==========================================================
bool enableSysId = true;         // Toggle noise injection
float noiseAmplitude = 35.0f;    // Max PWM noise amplitude 
int noiseHoldCycles = 5;         // Hold noise for 5 cycles (12.5ms at 400Hz)
int noiseCounter = 0;
float currentNoiseLeft = 0.0f;
float currentNoiseRight = 0.0f;

// ==========================================================
// CONTROL LOOP TIMING
// ==========================================================
constexpr uint32_t CONTROL_PERIOD_US = 2500; // 2.5 ms in microseconds (400 Hz)
constexpr float CONTROL_DT = CONTROL_PERIOD_US / 1000000.0f; // 2.5 ms in seconds

uint32_t nextControlTime = 0;


// Check if Bluetooth is enabled in the ESP32 core
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

const int bmi160_addr = 0x68;
const int sda_pin     = 21;     // ESP32 Hardware Default SDA
const int scl_pin     = 22;     // ESP32 Hardware Default SCL
BMI160Driver bmi160(bmi160_addr);
BMI160Data imuData;

// ==========================================================
// TUNING VARIABLES
// ==========================================================
// Balance Loop Tunings
float balKp = 12;
float balKi = 0;
float balKd = 0.4;
PID balancePID(balKp, balKi, balKd, -255.0, 255.0);

// Position Loop Tunings
// Limit output to +/- 10 degrees so the robot doesn't try to faceplant 
// when pushed hard.
float posKp = 0.0006;
float posKi = 0.0;
float posKd = 0.01;
PID positionPID(posKp, posKi, posKd, -2.0, 2.0);

float MECHANICAL_ZERO = -1.4;

String inputBuffer = "";
unsigned long lastTime = 0;

// ==========================================================
// MULTI-CORE TELEMETRY SETUP
// ==========================================================

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
  uint32_t current_time;
};

// FreeRTOS Queue Handle
QueueHandle_t telemetryQueue;
BluetoothSerial SerialBT;

// Compact 20-byte struct strictly for System ID over Bluetooth
struct __attribute__((packed)) SystemIdWirePacket {
  float   filtered_angle;
  float   filtered_rate;
  float enc_pos1;
  float enc_speed1;
  float enc_pos2;
  float enc_speed2;
  float   u_left;
  float   u_right;
  uint32_t current_time;
};

// ==========================================================
// CORE 0: BATCHING TELEMETRY TASK
// ==========================================================
void telemetryTask(void *pvParameters) {
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
      wireBuffer[bufferIndex].filtered_angle = packet.filtered_angle;
      wireBuffer[bufferIndex].filtered_rate  = packet.filtered_rate;
      wireBuffer[bufferIndex].enc_pos1       = packet.enc_pos1;
      wireBuffer[bufferIndex].enc_speed1     = packet.enc_speed1;
      wireBuffer[bufferIndex].enc_pos2       = packet.enc_pos2;
      wireBuffer[bufferIndex].enc_speed2     = packet.enc_speed2;
      wireBuffer[bufferIndex].u_left         = packet.u_left;
      wireBuffer[bufferIndex].u_right        = packet.u_right;
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

//void processCommand(String cmd) {
//  cmd.trim(); 
//  if (cmd.length() < 2) return;
//  
//  char type = cmd.charAt(0);
//  float val = cmd.substring(1).toFloat();
//  bool balChanged = false;
//  bool posChanged = false;
//
//  // Uppercase for BALANCE loop
//  if      (type == 'P') { balKp = val; balChanged = true; }
//  else if (type == 'I') { balKi = val; balChanged = true; }
//  else if (type == 'D') { balKd = val; balChanged = true; }
//  // Lowercase for POSITION loop
//  else if (type == 'p') { posKp = val; posChanged = true; }
//  else if (type == 'i') { posKi = val; posChanged = true; }
//  else if (type == 'd') { posKd = val; posChanged = true; }
//  else if (type == 'Z') { MECHANICAL_ZERO = val; Serial.printf(">>> MECHANICAL ZERO SET TO: %.2f <<<\n", MECHANICAL_ZERO); SerialBT.printf(">>> MECHANICAL ZERO SET TO: %.2f <<<\n", MECHANICAL_ZERO); }
//  else { Serial.printf("Unknown command: %s\n", cmd.c_str()); SerialBT.printf("Unknown command: %s\n", cmd.c_str()); return; }
//
//  if (balChanged) {
//    balancePID.setTunings(balKp, balKi, balKd);
//    Serial.printf("\n>>> BALANCE TUNED: P:%.2f I:%.2f D:%.2f <<<\n\n", balKp, balKi, balKd);
//    SerialBT.printf("\n>>> BALANCE TUNED: P:%.2f I:%.2f D:%.2f <<<\n\n", balKp, balKi, balKd);
//  }
//  
//  if (posChanged) {
//    positionPID.setTunings(posKp, posKi, posKd);
//    Serial.printf("\n>>> POSITION TUNED: p:%.4f i:%.4f d:%.4f <<<\n\n", posKp, posKi, posKd);
//    SerialBT.printf("\n>>> POSITION TUNED: p:%.4f i:%.4f d:%.4f <<<\n\n", posKp, posKi, posKd);
//  }
//}

//void checkTuning() {
//  //while (Serial.available()) {
//  //  char c = Serial.read();
//  //  if (c == '\n') { processCommand(inputBuffer); inputBuffer = ""; } 
//  //  else { inputBuffer += c; }
//  //}
//  while (SerialBT.available()) {
//    char c = SerialBT.read();
//    if (c == '\n') { processCommand(inputBuffer); inputBuffer = ""; } 
//    else { inputBuffer += c; }
//  }
//}

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
    telemetryTask, "TelemetryTask", 4096, NULL, 1, NULL, 0
  );

  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  digitalWrite(PIN_STBY, HIGH);
  lastTime = micros();
  nextControlTime = lastTime + CONTROL_PERIOD_US;

  SerialBT.begin("ESP32_Robot_BT"); // Name of your Bluetooth device
  Serial.println("Bluetooth started! Ready to pair as 'ESP32_Robot_BT'");
  
}

void loop() {
  // Handle tuning commands whenever they arrive.
  //checkTuning();

  uint32_t now = micros();

  // Not time for the next control iteration yet.
  if ((int32_t)(now - nextControlTime) < 0) {
    //SerialBT.printf("Waiting for next control iteration. Time left: %.3f ms\n", (nextControlTime - now) / 1000.0);
    return;
  }

  // Are we running late?
  if ((int32_t)(now - nextControlTime) > CONTROL_PERIOD_US) {
    //SerialBT.printf("Warning: Control loop is running late! by %.3f ms\n", (now - nextControlTime) / 1000.0);
    nextControlTime = now; // Reset the next control time to now to avoid accumulating delay
  }
  // Schedule the next iteration.
  nextControlTime += CONTROL_PERIOD_US;

  
  // ========================================================
  // IMU DATA
  // ========================================================

  float &raw_gx = imuData.gx_dps;
  float &raw_gy = imuData.gy_dps;
  float &raw_gz = imuData.gz_dps;

  float &raw_ax = imuData.ax_g;
  float &raw_ay = imuData.ay_g;
  float &raw_az = imuData.az_g;

  float filtered_angle = 0.0f;
  float filtered_rate = 0.0f;
  float smoothed_rate = 0.0f;

  // ========================================================
  // 1. READ IMU + ENCODERS
  // ========================================================
  bool imuSuccess = bmi160.readSensor(imuData);
  EncoderData encData = getEncoderData();

  // ========================================================
  // 2. KALMAN FILTER
  // ========================================================
  updateKalman(
    raw_ay,
    raw_az,
    raw_gx,
    CONTROL_DT,
    filtered_angle, 
    filtered_rate,
    smoothed_rate
  );

  // ========================================================
  // 4. POSITION LOOP
  // ========================================================
  float avg_pos =
    (encData.pos1 + encData.pos2) / 2.0f;

  float avg_speed =
    (encData.speed1 + encData.speed2) / 2.0f;
  float angle_adjustment = // Modifies the angle to force a deviation to
    // correct the position error. This is the outer loop of the cascaded PID.
    positionPID.compute(
      0.0f,
      avg_pos, // RAW ticks
      avg_speed, // RAW ticks/sec
      CONTROL_DT, 
      false

    );

  // ========================================================
  // 5. BALANCE LOOP
  // ========================================================
  float dynamic_target_angle =
  MECHANICAL_ZERO + angle_adjustment;

  float u =
    balancePID.compute(
      dynamic_target_angle,
      filtered_angle, // Current angle from Kalman filter
      smoothed_rate, // angular speed in degrees/sec from Kalman filter
      CONTROL_DT
    );
//
  float u_left = u;
  float u_right = u;

  // --- SYSTEM IDENTIFICATION INJECTION ---
  //if (enableSysId) {
  //  noiseCounter++;
  //  if (noiseCounter >= noiseHoldCycles) {
  //    // Generate independent random noise for left and right wheels
  //    currentNoiseLeft = (random(-100, 101) / 100.0f) * noiseAmplitude;
  //    currentNoiseRight = (random(-100, 101) / 100.0f) * noiseAmplitude;
  //    noiseCounter = 0;
  //  }
  //  
  //  // Add the decoupled excitation to the control effort
  //  u_left += currentNoiseLeft;
  //  u_right += currentNoiseRight;
  //  
  //  // Clamp to valid PWM limits independently
  //  if (u_left > 255.0f) u_left = 255.0f;
  //  if (u_left < -255.0f) u_left = -255.0f;
  //  
  //  if (u_right > 255.0f) u_right = 255.0f;
  //  if (u_right < -255.0f) u_right = -255.0f;
  //}

  // ========================================================
  // 6. TELEMETRY
  // ========================================================
  float time_taken = (int32_t)(micros() - now) / 1000.0f;

    // Transform into sensible units for telemetry and logging
  PhysicalState currentState = convertToSensibleUnits(
    filtered_angle - MECHANICAL_ZERO, // Adjusted for mechanical zero
    filtered_rate,
    encData.pos1,
    encData.speed1,
    encData.pos2,
    encData.speed2,
    u_left,
    u_right // For LQR this will be modify anyways
  );

  // CONTROL: LQR
  float stateVector[6] = {
    currentState.angle_rad, // Adjusted for mechanical zero
    currentState.rate_rad_s,
    currentState.pos_left_m,
    currentState.speed_left_m_s,
    currentState.pos_right_m,
    currentState.speed_right_m_s
  };
  //lqrController.computeControl(
  //  stateVector,
  //  u_left,
  //  u_right
  //);

  // DEBUG: PRINT THE STATE to serial USB
   //Serial.printf(
   //  "Angle: %.2f rad, Rate: %.2f rad/s | Left: %.2f m, %.2f m/s | Right: %.2f m, %.2f m/s | Target: %.2f | u: %.2f, %.2f | Time: %.3f ms\n",
   //  currentState.angle_rad,
   //  currentState.rate_rad_s,
   //  currentState.pos_left_m,
   //  currentState.speed_left_m_s,
   //  currentState.pos_right_m,
   //  currentState.speed_right_m_s,
   //   MECHANICAL_ZERO,
   //  u_left,
   //  u_right,
   //  time_taken
   //);

  TelemetryPacket currentData = {
    imuSuccess,
    raw_ax, raw_ay, raw_az,
    raw_gx, raw_gy, raw_gz,
    currentState.angle_rad,
    currentState.rate_rad_s,
    currentState.pos_left_m,
    currentState.speed_left_m_s,
    currentState.pos_right_m,
    currentState.speed_right_m_s,
    MECHANICAL_ZERO, // Target angle for telemetry
    currentState.u_left_normalized,
    currentState.u_right_normalized,
    time_taken,
    now
  };

  // Non-blocking. If the queue is full, skip this telemetry sample.
  xQueueSend(telemetryQueue, &currentData, 0);

  // ========================================================
  // 7. MOTORS
  // ========================================================
  setMotorOutputs(u, u);
}