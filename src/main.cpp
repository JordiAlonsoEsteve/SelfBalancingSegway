#include <Wire.h>
#include <Arduino.h>
#include "kalman.h"
//#include "complementary_filter.h"
#include "BluetoothSerial.h"
#include "wheelControl.h"
#include "bmi160_driver.h"
#include "encoder_driver.h"
#include "PID.h"

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
  int32_t enc_pos1;
  int16_t enc_speed1;
  int32_t enc_pos2;
  int16_t enc_speed2;
  float target_angle;
  float u;
  float time_taken;
};

// FreeRTOS Queue Handle
QueueHandle_t telemetryQueue;
BluetoothSerial SerialBT;

// This task will run exclusively on Core 0
void telemetryTask(void *pvParameters) {
  TelemetryPacket packet;

  int counter = 0;
  for (;;) {
    if (xQueueReceive(telemetryQueue, &packet, portMAX_DELAY) == pdPASS) {
      if (packet.imuSuccess) {
        // Always print fast to USB Serial
        //Serial.printf("%8.2f %8.2f %8.2f | %8d %6d | %8d %6d | %8.2f\n",  
        //              packet.filtered_angle, packet.filtered_rate, packet.target_angle,
        //              packet.enc_pos1, packet.enc_speed1,
        //              packet.enc_pos2, packet.enc_speed2,
        //              packet.u);
        //
        // Send over Bluetooth only 1 out of every 10 times
        if (++counter >= 10) {
          counter = 0;
          SerialBT.printf("%8.2f %8.2f %8.2f | %8d %6d | %8d %6d | %8.2f | %.2f\n", 
                        packet.filtered_angle, packet.filtered_rate, packet.target_angle,
                        packet.enc_pos1, packet.enc_speed1,
                        packet.enc_pos2, packet.enc_speed2,
                        packet.u,
                        packet.time_taken);
        }
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
//
  float avg_speed =
    (encData.speed1 + encData.speed2) / 2.0f;

  float angle_adjustment =
    positionPID.compute(
      0.0f,
      avg_pos,
      avg_speed,
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
      filtered_angle,
      smoothed_rate,
      CONTROL_DT
    );

  // ========================================================
  // 6. TELEMETRY
  // ========================================================
  float time_taken = (int32_t)(micros() - now) / 1000.0f;
  TelemetryPacket currentData = {
    imuSuccess,
    raw_ax,
    raw_ay,
    raw_az,
    raw_gx,
    raw_gy,
    raw_gz,
    filtered_angle,
    filtered_rate,
    encData.pos1,
    encData.speed1,
    encData.pos2,
    encData.speed2,
    dynamic_target_angle,
    u,
    time_taken
  };

  // Non-blocking. If the queue is full, skip this telemetry sample.
  xQueueSend(telemetryQueue, &currentData, 0);

  // ========================================================
  // 7. MOTORS
  // ========================================================
  setMotorOutputs(u, u);
}