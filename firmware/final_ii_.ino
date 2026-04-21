// ===================== IMU ======================
#include <Wire.h>
#include <MPU9250_WE.h>

#define MPU9250_ADDR 0x68
#define DEG_TO_RAD 0.01745329251

MPU9250_WE imu = MPU9250_WE(MPU9250_ADDR);

float gx, gy, gz;
float ax, ay, az;

float gx_bias = 7.368673;
float gy_bias = -5.246443;
float gz_bias = 0.654472;

unsigned long lastIMUMs = 0;
const unsigned long IMU_INTERVAL_MS = 10;

// ===== IMU AVERAGING =====
float sum_gx = 0, sum_gy = 0, sum_gz = 0;
float sum_ax = 0, sum_ay = 0, sum_az = 0;
int imu_count = 0;
// ===================================================


// ===================== ORIGINAL CODE ======================
#include <HardwareSerial.h>

HardwareSerial HoverSerial(1);

#define HOVER_RX 20
#define HOVER_TX 19

#define SERIAL_BAUD 115200
#define HOVER_BAUD  115200

#define START_FRAME 0xABCD
#define MAX_PWR     1000
#define SEND_HZ     50

struct __attribute__((packed)) SerialCommand {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
};
SerialCommand Command;

struct __attribute__((packed)) SerialFeedback {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
};
SerialFeedback Feedback;
SerialFeedback NewFeedback;

uint8_t rxIdx = 0;
uint16_t bufStartFrame = 0;
uint8_t *rxPtr;
uint8_t inByte = 0, prevByte = 0;

const unsigned long SEND_INTERVAL_MS = 1000UL / SEND_HZ;
unsigned long lastSendMs = 0;

int16_t tgtR = 0;
int16_t tgtL = 0;
int16_t curR = 0;
int16_t curL = 0;

const int16_t rampStep = 20;

// ===== ENCODERS =====
#define ENC_L_A 36
#define ENC_L_B 37
#define ENC_R_A 16
#define ENC_R_B 17

volatile long leftPos  = 0;
volatile long rightPos = 0;

unsigned long lastEncMs = 0;
const unsigned long ENC_PRINT_INTERVAL_MS = 100;
// ===================================================


// ===================== HELPERS ======================
static inline int16_t clampPwr(int32_t v) {
  if (v >  MAX_PWR) v =  MAX_PWR;
  if (v < -MAX_PWR) v = -MAX_PWR;
  return (int16_t)v;
}

void SendRaw(int16_t uSteer, int16_t uSpeed) {
  Command.start    = START_FRAME;
  Command.steer    = uSteer;
  Command.speed    = uSpeed;
  Command.checksum = Command.start ^ Command.steer ^ Command.speed;
  HoverSerial.write((uint8_t*)&Command, sizeof(Command));
}

void SEND_R_L(int16_t PWR_R, int16_t PWR_L) {
  int32_t speed = ((int32_t)PWR_L + (int32_t)PWR_R) / 2;
  int32_t steer = ((int32_t)PWR_R - (int32_t)PWR_L) / 2;

  SendRaw(clampPwr(steer), clampPwr(speed));
}
// ===================================================


// ===================== PC COMMAND ======================
void readPcCommand() {
  if (Serial.available() > 0) {
    int16_t cmdR = (int16_t)Serial.parseInt();
    int16_t cmdL = (int16_t)Serial.parseInt();

    if (Serial.available()) Serial.read();

    cmdR = constrain(cmdR, -250, 250);
    cmdL = constrain(cmdL, -250, 250);

    tgtR = clampPwr((long)cmdR * MAX_PWR / 250);
    tgtL = clampPwr((long)cmdL * MAX_PWR / 250);
  }
}
// ===================================================


// ===================== ENCODER ISR ======================
void IRAM_ATTR onLeftA() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) leftPos++;
  else leftPos--;
}

void IRAM_ATTR onLeftB() {
  if (digitalRead(ENC_L_A) != digitalRead(ENC_L_B)) leftPos++;
  else leftPos--;
}

void IRAM_ATTR onRightA() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) rightPos++;
  else rightPos--;
}

void IRAM_ATTR onRightB() {
  if (digitalRead(ENC_R_A) != digitalRead(ENC_R_B)) rightPos++;
  else rightPos--;
}
// ===================================================


// ===================== SETUP ======================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(10);

  HoverSerial.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX, HOVER_TX);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), onLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), onRightB, CHANGE);

  // ===== IMU INIT =====
  Wire.begin(8, 9);

  if (!imu.init()) {
    Serial.println("IMU NOT CONNECTED!");
    while (1);
  }

  imu.setAccRange(MPU9250_ACC_RANGE_2G);
  imu.setGyrRange(MPU9250_GYRO_RANGE_250);

  delay(1000);

  Serial.println("SYSTEM READY");
}
// ===================================================


// ===================== LOOP ======================
void loop() {
  readPcCommand();

  unsigned long now = millis();

  // ===== MOTOR =====
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;

    int16_t dR = constrain(tgtR - curR, -rampStep, rampStep);
    int16_t dL = constrain(tgtL - curL, -rampStep, rampStep);

    curR += dR;
    curL += dL;

    SEND_R_L(curR, curL);
  }

  // ===== IMU (100 Hz) =====
  if (now - lastIMUMs >= IMU_INTERVAL_MS) {
    lastIMUMs = now;

    xyzFloat g = imu.getGyrValues();
    xyzFloat a = imu.getAccRawValues();

    float _gx = (g.x - gx_bias) * DEG_TO_RAD;
    float _gy = (g.y - gy_bias) * DEG_TO_RAD;
    float _gz = (g.z - gz_bias) * DEG_TO_RAD;

    float _ax = (a.x / 16384.0) * 9.81;
    float _ay = (a.y / 16384.0) * 9.81;
    float _az = (a.z / 16384.0) * 9.81;

    sum_gx += _gx;
    sum_gy += _gy;
    sum_gz += _gz;

    sum_ax += _ax;
    sum_ay += _ay;
    sum_az += _az;

    imu_count++;
  }

  // ===== OUTPUT (10 Hz) =====
  if (now - lastEncMs >= ENC_PRINT_INTERVAL_MS) {
    lastEncMs = now;

    long L, R;
    noInterrupts();
    L = leftPos;
    R = rightPos;
    interrupts();

    float out_gx = gx, out_gy = gy, out_gz = gz;
    float out_ax = ax, out_ay = ay, out_az = az;

    if (imu_count > 0) {
      out_gx = sum_gx / imu_count;
      out_gy = sum_gy / imu_count;
      out_gz = sum_gz / imu_count;

      out_ax = sum_ax / imu_count;
      out_ay = sum_ay / imu_count;
      out_az = sum_az / imu_count;
    }

    // reset accumulators
    sum_gx = sum_gy = sum_gz = 0;
    sum_ax = sum_ay = sum_az = 0;
    imu_count = 0;

    Serial.print("ENC ");
    Serial.print(L);
    Serial.print(" ");
    Serial.print(R);

    Serial.print(" IMU ");
    Serial.print(out_gx, 4); Serial.print(" ");
    Serial.print(out_gy, 4); Serial.print(" ");
    Serial.print(out_gz, 4); Serial.print(" ");
    Serial.print(out_ax, 3); Serial.print(" ");
    Serial.print(out_ay, 3); Serial.print(" ");
    Serial.println(out_az, 3);
  }
}