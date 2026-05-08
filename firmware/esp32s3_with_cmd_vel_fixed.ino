#include <Wire.h>
#include <MPU9250_WE.h>
#include <HardwareSerial.h>

// ==========================================
// 1. تعريفات الـ IMU والبيانات
// ==========================================
#define MPU9250_ADDR 0x68
#define DEG_TO_RAD 0.01745329251
MPU9250_WE imu = MPU9250_WE(MPU9250_ADDR);

float gx_bias = 7.368673, gy_bias = -5.246443, gz_bias = 0.654472;
unsigned long lastIMUMs = 0, lastSendMs = 0, lastEncMs = 0;
float sum_gx = 0, sum_gy = 0, sum_gz = 0, sum_ax = 0, sum_ay = 0, sum_az = 0;
int imu_count = 0;

// ==========================================
// 2. الثوابت الفيزيائية والمعايرة (Odom Scaled)
// ==========================================
const float WHEEL_RADIUS = 0.08f; 
const float WHEEL_BASE   = 0.63f; 
const float TICKS_PER_REV_L = 875.0f; // ثابتة لدقة المسافة
const float TICKS_PER_REV_R = 800.0f; // ثابتة لدقة المسافة

const float METERS_PER_TICK_L = (2.0f * PI * WHEEL_RADIUS) / (TICKS_PER_REV_L * 4.0f);
const float METERS_PER_TICK_R = (2.0f * PI * WHEEL_RADIUS) / (TICKS_PER_REV_R * 4.0f);

// ==========================================
// 3. ثوابت التحكم والـ PID
// ==========================================
float target_VL = 0.0f, target_VR = 0.0f; 
float VL = 0.0f, VR = 0.0f;               
float Kp = 18.0f;                        // زدنا القوة لتحسين استجابة التصحيح
float Kb = 15.0f;                        // معامل التوازن الذكي
const int16_t MAX_PWR = 1000;            

// معامل تصحيح القوة (خنق الموتور اليمين لضبط المسار المستقيم)
float RIGHT_MOTOR_SCALER = 0.81f;        // القيمة المعدلة لتقليل الحودة شمال

// ==========================================
// 4. نظام الإنكودر (QEM + REG_READ)
// ==========================================
#define ENC_L_A 36 
#define ENC_L_B 37
#define ENC_R_A 16
#define ENC_R_B 17

volatile long leftPos = 0, rightPos = 0;
long prevL = 0, prevR = 0;
static const int8_t QEM[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
volatile uint8_t encL_prevState = 0, encR_prevState = 0;

void IRAM_ATTR onLeftEncoder() {
  uint32_t pins = REG_READ(GPIO_IN1_REG);
  uint8_t curr = ((pins >> (ENC_L_A - 32)) & 1) << 1 | ((pins >> (ENC_L_B - 32)) & 1);
  leftPos += QEM[(encL_prevState << 2) | curr];
  encL_prevState = curr;
}

void IRAM_ATTR onRightEncoder() {
  uint32_t pins = REG_READ(GPIO_IN_REG);
  uint8_t curr = ((pins >> ENC_R_A) & 1) << 1 | ((pins >> ENC_R_B) & 1);
  rightPos += QEM[(encR_prevState << 2) | curr];
  encR_prevState = curr;
}

// ==========================================
// 5. بروتوكول الهوفر بورد
// ==========================================
HardwareSerial HoverSerial(1);
struct __attribute__((packed)) SerialCommand {
  uint16_t start; int16_t steer; int16_t speed; uint16_t checksum;
};
SerialCommand Command;

void SEND_R_L(int16_t R, int16_t L) {
  int32_t speed = (L + R) / 2;
  int32_t steer = (R - L) / 2;
  Command.start = 0xABCD;
  Command.steer = (int16_t)steer;
  Command.speed = (int16_t)speed;
  Command.checksum = Command.start ^ Command.steer ^ Command.speed;
  HoverSerial.write((uint8_t*)&Command, sizeof(Command));
}

void readPcCommand() {
  static char pcBuf[64]; static uint8_t pcIdx = 0;
  static unsigned long lastCmdMs = 0;
  if (millis() - lastCmdMs > 500) { target_VL = 0; target_VR = 0; }
  
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (pcIdx > 0) {
        pcBuf[pcIdx] = '\0';
        float l, r; 
        if (sscanf(pcBuf, "%f %f", &l, &r) == 2) {
          target_VL = l; target_VR = r; lastCmdMs = millis();
        }
      }
      pcIdx = 0;
    } else if (pcIdx < 63) pcBuf[pcIdx++] = c;
  }
}

void setup() {
  Serial.begin(115200);
  HoverSerial.begin(115200, SERIAL_8N1, 20, 19);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), onLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), onRightEncoder, CHANGE);
  Wire.begin(8, 9); imu.init();
}

void loop() {
  readPcCommand(); 
  unsigned long now = millis();

  // 6. حلقة التحكم الأساسية (50Hz)
  if (now - lastSendMs >= 20) {
    float dt = (now - lastSendMs) / 1000.0f; lastSendMs = now;
    
    long L, R; noInterrupts(); L = leftPos; R = rightPos; interrupts();
    
    // عكس حساب السرعة الفعلية لتتطابق مع عكس اتجاه المواتير
    VL = -(((L - prevL) * METERS_PER_TICK_L) / dt);
    VR = -(((R - prevR) * METERS_PER_TICK_R) / dt);
    prevL = L; prevR = R;

    // معادلة التوازن الذكي (Smart Balance)
    float eL = target_VL - VL;
    float eR = target_VR - VR;
    float balance = (eL - eR) * Kb; 

    // حساب القوة مع الـ Feed-Forward والتوازن
    int16_t outL = (target_VL * 500) + (eL * Kp) + balance; 
    int16_t outR = (target_VR * 500) + (eR * Kp) - balance;
    
    // تطبيق الـ SCALER على الموتور اليمين فقط لضبط الاستقامة
    int16_t finalOutR = outR * RIGHT_MOTOR_SCALER;
    int16_t finalOutL = outL;

    // عكس القطبية عند الإرسال لتصحيح اتجاه "قدام وورا"
    SEND_R_L(-constrain(finalOutR, -MAX_PWR, MAX_PWR), -constrain(finalOutL, -MAX_PWR, MAX_PWR));
  }

  // 7. حلقة الـ IMU (تجميع البيانات)
  if (now - lastIMUMs >= 10) {
    lastIMUMs = now;
    xyzFloat g = imu.getGyrValues(); xyzFloat a = imu.getAccRawValues();
    sum_gx += (g.x - gx_bias) * DEG_TO_RAD; sum_gy += (g.y - gy_bias) * DEG_TO_RAD; sum_gz += (g.z - gz_bias) * DEG_TO_RAD;
    sum_ax += (a.x / 16384.0) * 9.81; sum_ay += (a.y / 16384.0) * 9.81; sum_az += (a.z / 16384.0) * 9.81;
    imu_count++;
  }

  // 8. الطباعة النهائية للـ ROS (10Hz)
  if (now - lastEncMs >= 100) {
    lastEncMs = now;
    // عكس قيم الـ Ticks لضمان أن التقدم للأمام يعطي قيم موجبة في ROS
    Serial.print("ENC "); Serial.print(-leftPos); Serial.print(" "); Serial.print(-rightPos);
    
    if (imu_count > 0) {
      Serial.print(" IMU ");
      Serial.print(sum_gx/imu_count, 4); Serial.print(" ");
      Serial.print(sum_gy/imu_count, 4); Serial.print(" ");
      Serial.print(sum_gz/imu_count, 4); Serial.print(" ");
      Serial.print(sum_ax/imu_count, 3); Serial.print(" ");
      Serial.print(sum_ay/imu_count, 3); Serial.print(" ");
      Serial.println(sum_az/imu_count, 3);
    } else { Serial.println(); }
    sum_gx = sum_gy = sum_gz = sum_ax = sum_ay = sum_az = 0; imu_count = 0;
  }
}