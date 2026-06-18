#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <driver/rmt.h>

// =============================================================================
// DSHOT CONFIGURATION
// =============================================================================
const int MOTOR_PINS[4] = {13, 14, 27, 26};

#define DSHOT_DISARM              0
#define DSHOT_MIN_THROTTLE        48
#define DSHOT_MAX_THROTTLE        2047
#define DSHOT_MAX_THROTTLE_90     ((uint16_t)(DSHOT_MAX_THROTTLE * 0.9f))  // 1842

#define DSHOT_CMD_SPIN_DIRECTION_1  20   // Normal direction (permanent)
#define DSHOT_CMD_SPIN_DIRECTION_2  21   // Reversed direction (permanent)
#define DSHOT_CMD_SAVE_SETTINGS    12   // Burn direction to ESC EEPROM

#define DSHOT_T1H 62
#define DSHOT_T1L 26
#define DSHOT_T0H 26
#define DSHOT_T0L 62

rmt_channel_t rmtChannels[4] = {
  RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3
};

rmt_item32_t global_dshot_items[4][16];

// =============================================================================
// SBUS CONFIGURATION
// Throttle  (Ch3): 1000 (bottom) to 1833 (top)
// Roll/Pitch/Yaw (Ch1/2/4): 167 (full left/down) to 1833 (full right/up), center = 1000
// =============================================================================
#define SBUS_BAUDRATE   100000
#define SBUS_RX_PIN     16
#define SBUS_FRAME_SIZE 25
#define SBUS_HEADER     0x0F
#define SBUS_FOOTER     0x00

// Throttle range
#define SBUS_THR_MIN             1000
#define SBUS_THR_MAX             1833
#define SBUS_THR_ARM_THRESHOLD   (SBUS_THR_MIN + 20)  // must be clearly at bottom to arm

// Roll/Pitch/Yaw range
#define SBUS_STICK_MIN  167
#define SBUS_STICK_MID  1000
#define SBUS_STICK_MAX  1833
#define SBUS_DEADZONE     20

uint16_t sbusChannels[16];

#define CH_ROLL     0   // Channel 1
#define CH_PITCH    1   // Channel 2
#define CH_THROTTLE 2   // Channel 3
#define CH_YAW      3   // Channel 4

bool readSBUS() {
  static uint8_t buf[SBUS_FRAME_SIZE];
  static int idx = 0;

  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    if (idx == 0 && b != SBUS_HEADER) continue;

    buf[idx++] = b;

    if (idx == SBUS_FRAME_SIZE) {
      idx = 0;
      if (buf[24] != SBUS_FOOTER) return false;

      // Failsafe — kill throttle instantly on signal loss
      bool frameLost = buf[23] & (1 << 2);
      bool failsafe  = buf[23] & (1 << 3);
      if (frameLost || failsafe) {
        sbusChannels[CH_THROTTLE] = 0;
        return true;
      }

      sbusChannels[0]  = ((uint16_t)buf[1]        | ((uint16_t)buf[2]  << 8))                              & 0x07FF;
      sbusChannels[1]  = (((uint16_t)buf[2]  >> 3) | ((uint16_t)buf[3]  << 5))                              & 0x07FF;
      sbusChannels[2]  = (((uint16_t)buf[3]  >> 6) | ((uint16_t)buf[4]  << 2) | ((uint16_t)buf[5]  << 10)) & 0x07FF;
      sbusChannels[3]  = (((uint16_t)buf[5]  >> 1) | ((uint16_t)buf[6]  << 7))                              & 0x07FF;
      sbusChannels[4]  = (((uint16_t)buf[6]  >> 4) | ((uint16_t)buf[7]  << 4))                              & 0x07FF;
      sbusChannels[5]  = (((uint16_t)buf[7]  >> 7) | ((uint16_t)buf[8]  << 1) | ((uint16_t)buf[9]  << 9))  & 0x07FF;
      sbusChannels[6]  = (((uint16_t)buf[9]  >> 2) | ((uint16_t)buf[10] << 6))                              & 0x07FF;
      sbusChannels[7]  = (((uint16_t)buf[10] >> 5) | ((uint16_t)buf[11] << 3))                              & 0x07FF;
      sbusChannels[8]  = ((uint16_t)buf[12]        | ((uint16_t)buf[13] << 8))                              & 0x07FF;
      sbusChannels[9]  = (((uint16_t)buf[13] >> 3) | ((uint16_t)buf[14] << 5))                              & 0x07FF;
      sbusChannels[10] = (((uint16_t)buf[14] >> 6) | ((uint16_t)buf[15] << 2) | ((uint16_t)buf[16] << 10)) & 0x07FF;
      sbusChannels[11] = (((uint16_t)buf[16] >> 1) | ((uint16_t)buf[17] << 7))                              & 0x07FF;
      sbusChannels[12] = (((uint16_t)buf[17] >> 4) | ((uint16_t)buf[18] << 4))                              & 0x07FF;
      sbusChannels[13] = (((uint16_t)buf[18] >> 7) | ((uint16_t)buf[19] << 1) | ((uint16_t)buf[20] << 9))  & 0x07FF;
      sbusChannels[14] = (((uint16_t)buf[20] >> 2) | ((uint16_t)buf[21] << 6))                              & 0x07FF;
      sbusChannels[15] = (((uint16_t)buf[21] >> 5) | ((uint16_t)buf[22] << 3))                              & 0x07FF;
      return true;
    }
  }
  return false;
}

// Throttle: 1000 (bottom) → DSHOT_MIN_THROTTLE, 1833 (top) → DSHOT_MAX_THROTTLE_90
uint16_t sbusThrottleToDshot(uint16_t sbusVal) {
  if (sbusVal == 0 || sbusVal <= SBUS_THR_ARM_THRESHOLD) return DSHOT_DISARM;
  uint16_t result = (uint16_t)map((long)sbusVal, SBUS_THR_MIN, SBUS_THR_MAX,
                                   DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE_90);
  return constrain(result, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE_90);
}

// Roll/Pitch/Yaw: center=1000 → 0.0, full low=167 → -1.0, full high=1833 → +1.0
float sbusStickToFloat(uint16_t sbusVal) {
  int centered = (int)sbusVal - SBUS_STICK_MID;
  if (abs(centered) < SBUS_DEADZONE) return 0.0f;
  float range = (centered > 0)
    ? (float)(SBUS_STICK_MAX - SBUS_STICK_MID - SBUS_DEADZONE)
    : (float)(SBUS_STICK_MID - SBUS_STICK_MIN - SBUS_DEADZONE);
  float scaled = (float)(abs(centered) - SBUS_DEADZONE) / range;
  return constrain(scaled * (centered > 0 ? 1.0f : -1.0f), -1.0f, 1.0f);
}

// =============================================================================
// PID CONTROLLER
// =============================================================================
class PID {
public:
  float Kp, Ki, Kd;
  float prev_error   = 0.0f;
  float integral     = 0.0f;
  float max_integral;

  PID(float p, float i, float d, float max_i)
    : Kp(p), Ki(i), Kd(d), max_integral(max_i) {}

  float compute(float target, float current, float dt) {
    if (dt <= 0.0f) return 0.0f;
    float error = target - current;
    integral += error * dt;
    integral = constrain(integral, -max_integral, max_integral);
    float derivative = (error - prev_error) / dt;
    prev_error = error;
    return (Kp * error) + (Ki * integral) + (Kd * derivative);
  }

  void reset() {
    prev_error = 0.0f;
    integral   = 0.0f;
  }
};

// Outer (Angle) Loops - Tells the drone to find 0 degrees (level)
PID roll_angle_pid (1.0f,  0.0f,  0.0f,  0.0f);
PID pitch_angle_pid(1.0f,  0.0f,  0.0f,  0.0f);

// Inner (Rate) Loops - The muscles that execute the leveling command
PID roll_rate_pid  (0.45f,  0.05f,  0.008f, 20.0f); // Lower Kp, tiny Kd to start
PID pitch_rate_pid (0.45f,  0.05f,  0.008f, 20.0f);
PID yaw_rate_pid   (0.3f,  0.0f,  0.0f,   20.0f); // Keep yaw simple

#define MAX_ANGLE_DEG  25.0f
#define MAX_YAW_RATE  120.0f

// =============================================================================
// HARDWARE & SENSOR STATE
// =============================================================================
Adafruit_MPU6050 mpu;

float dt;
unsigned long last_time;
unsigned long current_time;

float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float accX_offset = 0, accY_offset = 0;
float roll = 0, pitch = 0;
const float ALPHA = 0.97f;

float filtered_gyroX = 0.0f;
float filtered_gyroY = 0.0f;
float filtered_gyroZ = 0.0f;

// =============================================================================
// DSHOT FUNCTIONS
// =============================================================================
void dshotInit(int motorIndex) {
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(
    (gpio_num_t)MOTOR_PINS[motorIndex],
    rmtChannels[motorIndex]);
  config.clk_div       = 3;
  config.mem_block_num = 1;
  rmt_config(&config);
  rmt_driver_install(rmtChannels[motorIndex], 0, 0);
}

void dshotSend(int motorIndex, uint16_t value, bool telemetry) {
  value &= 0x7FF;
  uint16_t frame = (value << 1) | (telemetry ? 1 : 0);
  uint8_t  crc   = ((frame >> 0) ^ (frame >> 4) ^ (frame >> 8)) & 0x0F;
  frame = (frame << 4) | crc;

  for (int bit = 0; bit < 16; bit++) {
    bool one = frame & (1 << (15 - bit));
    global_dshot_items[motorIndex][bit].level0    = 1;
    global_dshot_items[motorIndex][bit].duration0 = one ? DSHOT_T1H : DSHOT_T0H;
    global_dshot_items[motorIndex][bit].level1    = 0;
    global_dshot_items[motorIndex][bit].duration1 = one ? DSHOT_T1L : DSHOT_T0L;
  }
  rmt_write_items(rmtChannels[motorIndex], global_dshot_items[motorIndex], 16, true);
}

void sendAll(uint16_t throttle) {
  for (int i = 0; i < 4; i++) dshotSend(i, throttle, false);
}

// Send a configuration command to one motor multiple times with telemetry=true
void sendCommand(int motorIndex, uint16_t cmd, int times) {
  for (int i = 0; i < times; i++) {
    dshotSend(motorIndex, cmd, true);
    delay(1);
  }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(SBUS_BAUDRATE, SERIAL_8E2, SBUS_RX_PIN, -1, true);
  while (!Serial) delay(10);

  // --- MPU6050 ---
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  delay(100);

  // --- Enhanced Gyro & Accel Calibration ---
  Serial.println("Calibrating sensors — keep drone perfectly level and still...");
  sensors_event_t a, g, temp;
  const int NUM_SAMPLES = 200;
  
  float accX_sum = 0, accY_sum = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    
    // Gyro sums
    gyroX_offset += g.gyro.x;
    gyroY_offset += g.gyro.y;
    gyroZ_offset += g.gyro.z;
    
    // Calculate raw acceleration angles during calibration
    accX_sum += -atan2(-a.acceleration.x, a.acceleration.z) * 57.2958f;
    accY_sum +=  atan2(-a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 57.2958f;
    
    delay(5);
  }
  
  // Average the gyro offsets
  gyroX_offset /= NUM_SAMPLES;
  gyroY_offset /= NUM_SAMPLES;
  gyroZ_offset /= NUM_SAMPLES;
  
  // Average the accelerometer structural mounting error
  accX_offset = accX_sum / NUM_SAMPLES;
  accY_offset = accY_sum / NUM_SAMPLES;
  
  Serial.println("Calibration complete.");

  // --- DShot RMT init ---
  Serial.println("Initializing DShot channels...");
  for (int i = 0; i < 4; i++) dshotInit(i);

  // -------------------------------------------------------------------------
  // PERMANENT ESC DIRECTION PROGRAMMING
  //
  // Physical layout:
  //   Motor 0 (pin 13) = front-left  — CCW = Direction 1 (default, no change)
  //   Motor 1 (pin 14) = front-right — CW  = Direction 2 (must be set)
  //   Motor 2 (pin 27) = back-left   — CW  = Direction 2 (must be set)
  //   Motor 3 (pin 26) = back-right  — CCW = Direction 1 (default, no change)
  //
  // Cmd 7  = DSHOT_CMD_SPIN_DIRECTION_1 → sets permanent baseline to normal
  // Cmd 8  = DSHOT_CMD_SPIN_DIRECTION_2 → sets permanent baseline to reversed
  // Cmd 12 = DSHOT_CMD_SAVE_SETTINGS    → burns the setting into ESC EEPROM
  //
  // NOTE: Motors must be spinning when commands 7/8 are sent.
  //       After the first successful flash, direction persists across reboots
  //       and this block just re-confirms the same setting each boot, which is safe.
  // -------------------------------------------------------------------------

  // Step 1: Clean disarm
  Serial.println("Step 1: Disarming ESCs (2 seconds)...");
  unsigned long start = millis();
  while (millis() - start < 2000) { sendAll(DSHOT_DISARM); delay(1); }


  // Step 3: Set Direction 1 (CCW/normal) for motors 0 and 3
  Serial.println("Step 3: Setting Direction 1 on motors 0 and 3 (front-left, back-right)...");
  delay(50);

  // Step 4: Set Direction 2 (CW/reversed) for motors 1 and 2
  Serial.println("Step 4: Setting Direction 2 on motors 1 and 2 (front-right, back-left)...");
  sendCommand(1, DSHOT_CMD_SPIN_DIRECTION_2, 10);
  sendCommand(2, DSHOT_CMD_SPIN_DIRECTION_2, 10);
  delay(10);

  // Step 5: Save all four motors to EEPROM
  Serial.println("Step 5: Saving settings to all ESCs...");
  sendCommand(0, DSHOT_CMD_SAVE_SETTINGS, 10);
  sendCommand(1, DSHOT_CMD_SAVE_SETTINGS, 10);
  sendCommand(2, DSHOT_CMD_SAVE_SETTINGS, 10);
  sendCommand(3, DSHOT_CMD_SAVE_SETTINGS, 10);
  delay(200);  // Give ESC EEPROM time to write

  // Step 6: Final disarm and settle
  Serial.println("Step 6: Settling after programming (1 second)...");
  start = millis();
  while (millis() - start < 1000) { sendAll(DSHOT_DISARM); delay(1); }

  Serial.println("Ready — push throttle stick to bottom to arm.");
  last_time = micros();
}

// =============================================================================
// MAIN CONTROL LOOP (200 Hz)
// =============================================================================
void loop() {
  current_time = micros();
  unsigned long elapsed = current_time - last_time;

  static bool prevArmed = false;

  if (elapsed < 5000) return;  // enforce 200 Hz

  dt = elapsed / 1000000.0f;
  last_time = current_time;
  if (dt > 0.05f) dt = 0.005f;

  // --- FIXED IMU AXIS SWAP (90-Deg Rotation) ---
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) return;

  // Swap X and Y because the sensor is rotated 90 degrees on the frame
  // Added negative signs based on your telemetry readings to correct direction
  float raw_gyroX = (g.gyro.y - gyroY_offset) * 57.2958f; 
  float raw_gyroY = -(g.gyro.x - gyroX_offset) * 57.2958f;
  float raw_gyroZ = (g.gyro.z - gyroZ_offset) * 57.2958f;

  filtered_gyroX = filtered_gyroX * 0.7f + raw_gyroX * 0.3f;
  filtered_gyroY = filtered_gyroY * 0.7f + raw_gyroY * 0.3f;
  filtered_gyroZ = filtered_gyroZ * 0.7f + raw_gyroZ * 0.3f;

  // Swap accelerometer inputs for the complementary filter angles
  float raw_roll_acc  = -atan2(-a.acceleration.x, a.acceleration.z) * 57.2958f;
  float raw_pitch_acc = atan2(-a.acceleration.y, 
   sqrt(a.acceleration.x * a.acceleration.x + 
         a.acceleration.z * a.acceleration.z)) * 57.2958f;

  // Subtract the calibration errors to target exactly 0.0 degrees
  float roll_acc  = raw_roll_acc - accX_offset;
  float pitch_acc = raw_pitch_acc - accY_offset;

  if (isnan(roll_acc))  roll_acc  = roll;
  if (isnan(pitch_acc)) pitch_acc = pitch;

  roll  = ALPHA * (roll  + filtered_gyroX * dt) + (1.0f - ALPHA) * roll_acc;
  pitch = ALPHA * (pitch + filtered_gyroY * dt) + (1.0f - ALPHA) * pitch_acc;

  // --- SBUS ---
  static uint16_t base_throttle_cmd = DSHOT_DISARM;
  static float    target_roll_angle  = 0.0f;
  static float    target_pitch_angle = 0.0f;
  static float    target_yaw_rate    = 0.0f;
  static bool     armed              = false;

  if (readSBUS()) {
    uint16_t ch_thr = sbusChannels[CH_THROTTLE];

    if (ch_thr == 0 || ch_thr <= SBUS_THR_ARM_THRESHOLD) {
      armed             = false;
      base_throttle_cmd = DSHOT_DISARM;
      roll_rate_pid.reset();
      pitch_rate_pid.reset();
      yaw_rate_pid.reset();
      roll_angle_pid.reset();
      pitch_angle_pid.reset();
    } else {
      armed             = true;
      base_throttle_cmd = sbusThrottleToDshot(ch_thr);
    }

    target_roll_angle  =  sbusStickToFloat(sbusChannels[CH_ROLL])  * MAX_ANGLE_DEG;
    target_pitch_angle = -sbusStickToFloat(sbusChannels[CH_PITCH]) * MAX_ANGLE_DEG;
    target_yaw_rate    =  sbusStickToFloat(sbusChannels[CH_YAW])   * MAX_YAW_RATE;
  }

  if (!armed) {
    sendAll(DSHOT_DISARM);

    roll_rate_pid.reset();
    pitch_rate_pid.reset();
    yaw_rate_pid.reset();

    roll_angle_pid.reset();
    pitch_angle_pid.reset();
    return;
  }
  
  prevArmed = true;

  // --- Cascade PID ---
  float desired_roll_rate  = roll_angle_pid.compute(target_roll_angle,  roll,  dt);
  float roll_output        = roll_rate_pid.compute(desired_roll_rate,   filtered_gyroX, dt);

  float desired_pitch_rate = pitch_angle_pid.compute(target_pitch_angle, -pitch, dt);
  float pitch_output       = pitch_rate_pid.compute(desired_pitch_rate,  filtered_gyroY, dt);

  float yaw_output         = yaw_rate_pid.compute(target_yaw_rate, filtered_gyroZ, dt);

  // --- Motor mixing ---
  // Motor 0 (front-left,  pin 13) CCW: - roll  - pitch  + yaw
  // Motor 1 (front-right, pin 14) CW:  + roll  - pitch  - yaw
  // Motor 2 (back-left,   pin 27) CW:  - roll  + pitch  - yaw
  // Motor 3 (back-right,  pin 26) CCW: + roll  + pitch  + yaw
// --- INVERTED PITCH SIGNS TO CORRECT BACKWARD FLIP ---
  // Changed (- pitch_output) to (+ pitch_output) for front motors
  // Changed (+ pitch_output) to (- pitch_output) for back motors
  float m0 = base_throttle_cmd - roll_output - pitch_output + yaw_output; // Front Left
  float m1 = base_throttle_cmd + roll_output - pitch_output - yaw_output; // Front Right
  float m2 = base_throttle_cmd - roll_output + pitch_output - yaw_output; // Back Left
  float m3 = base_throttle_cmd + roll_output + pitch_output + yaw_output; // Back Right

  uint16_t motor0_cmd = (uint16_t)constrain((int)m0, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE_90);
  uint16_t motor1_cmd = (uint16_t)constrain((int)m1, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE_90);
  uint16_t motor2_cmd = (uint16_t)constrain((int)m2, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE_90);
  uint16_t motor3_cmd = (uint16_t)constrain((int)m3, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE_90);

  dshotSend(0, motor0_cmd, false);
  dshotSend(1, motor1_cmd, false);
  dshotSend(2, motor2_cmd, false);
  dshotSend(3, motor3_cmd, false);

  // --- Telemetry (~10 Hz) ---
  static int print_divider = 0;
  if (print_divider++ % 20 == 0) {
    Serial.print("Roll: ");     Serial.print(roll, 1);
    Serial.print(" | Pitch: "); Serial.print(pitch, 1);
    Serial.print(" | TgtR: ");  Serial.print(target_roll_angle, 1);
    Serial.print(" | TgtP: ");  Serial.print(target_pitch_angle, 1);
    Serial.print(" | Thr: ");   Serial.print(base_throttle_cmd);
    Serial.print(" | M0: ");    Serial.print(motor0_cmd);
    Serial.print(" | M1: ");    Serial.print(motor1_cmd);
    Serial.print(" | M2: ");    Serial.print(motor2_cmd);
    Serial.print(" | M3: ");    Serial.println(motor3_cmd);
  }
}
