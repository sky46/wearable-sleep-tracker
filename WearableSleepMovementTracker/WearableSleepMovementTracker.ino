#include <Wire.h>

#define DEBUG_PRINT_ACCELS;

// Analog accelerometer pins
const int ANALOG_A_X_PIN = A0;
const int ANALOG_A_Y_PIN = A1;
const int ANALOG_A_Z_PIN = A2;

float
  analog_a_x,
  analog_a_y,
  analog_a_z,
  analog_a_mag
;

// MPU STUFF
// Based on ElegooTutorial by Ricardo Moreno Jr.
// https://github.com/rmorenojr/ElegooTutorial/blob/master/Lesson%2016%20-%20GY-521%20Module/MPU-6050_expanded/MPU-6050_expanded.ino

const int MPU_ADDR = 0x68; // I2C address from datasheet (AD0 should be logic low, wire to GND)
// x high 3B, x low 3C, y high 3D, y low 3E, z high 3F, z low 40
const int ACCEL_REG = 0x3B;
const int ACCEL_SCALE_REG = 0x1C;
const int POWER_REG = 0x6B;

// ACCEL_SCALE = 0      1    2    3
// Range is   +- 2g     4g   8g   16g
// sens (LSB/g)= 16384  8192 4096 2048
const float LSB_SENS_TABLE[4] = {16384, 8192, 4096, 2048};
const int ACCEL_SCALE = 3;
const float LSB_SENS = LSB_SENS_TABLE[ACCEL_SCALE];

// Offsets should be found and calibrated manually
const float MPU_A_X_OFFSET = 0;
const float MPU_A_Y_OFFSET = 0;
const float MPU_A_Z_OFFSET = 0;
float
  mpu_a_x,
  mpu_a_y,
  mpu_a_z,
  mpu_a_mag
;

// Movement detection
const int SAMPLE_INTERVAL = 150;  // slower sampling
const int WINDOW_SIZE = 20;       // larger buffer for smoothing
const float MOVEMENT_THRESHOLD = 0.45;  // higher = less sensitive
const float PERIODIC_SIMILARITY = 0.8;
unsigned long mpu_last_detection = 0; // millis() returns unsigned long, overflows at about 50 days
unsigned long analog_last_detection = 0; 

// Buffers
float analog_accel_buffer[WINDOW_SIZE];
float mpu_accel_buffer[WINDOW_SIZE];
int accel_buffer_index = 0;


float smooth(float *buffer, int size);
void setup();
void loop();
void read_from(const byte from_register, const int num_bytes, byte read_data[]);
void write_to(const byte to_register, const byte write_value);
void setup_mpu();
void record_mpu_accel();
void record_analog_accel();
bool detect_periodic_movement(float *buffer);
#ifdef DEBUG_PRINT_ACCELS
  void print_accels();
#endif


// Moving average helper
float smooth(float *buffer, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += buffer[i];
  return sum / size;
}

void setup() {
  Serial.begin(38400);
  Wire.begin();
  setup_mpu();

  Serial.println("Starting stable motion detection...");
}

void loop() {
  // Analog sensor
  record_analog_accel();
  analog_a_mag = sqrt(analog_a_x * analog_a_x + analog_a_y * analog_a_y + analog_a_z * analog_a_z);

  // MPU
  record_mpu_accel();
  mpu_a_mag = sqrt(mpu_a_x * mpu_a_x + mpu_a_y * mpu_a_y + mpu_a_z * mpu_a_z);

  #ifdef DEBUG_PRINT_ACCELS
    print_accels();
  #endif

  // Update buffers
  analog_accel_buffer[accel_buffer_index] = analog_a_mag;
  mpu_accel_buffer[accel_buffer_index] = mpu_a_mag;
  accel_buffer_index = (accel_buffer_index + 1) % WINDOW_SIZE;

  if (detect_periodic_movement(analog_accel_buffer)) {
    if (millis() - analog_last_detection > 2000) {
      Serial.println("Periodic movement detected on analog (arm)!");
      analog_last_detection = millis();
    }
  }

  if (detect_periodic_movement(mpu_accel_buffer)) {
    if (millis() - mpu_last_detection > 2000) {
      Serial.println("Periodic movement detected on MPU (leg)!");
      mpu_last_detection = millis();
    }
  }

  delay(SAMPLE_INTERVAL);
}

void setup_mpu() {
  byte buffer[1];
  read_from(0x6B, 1, buffer);
  // Serial.println(buffer[0]);
  write_to(POWER_REG, buffer[1] & 0b10001110); // Zero SLEEP, CYCLE, GYRO_STANDBY
  write_to(ACCEL_SCALE_REG, ACCEL_SCALE);
  // write_to(0x1D, 0b00001111); // Could calibrate digital low-pass filter, see register map
  return;
}

void read_from(const byte from_register, const int num_bytes, byte read_data[]) {
  // First send address of register from which to read
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(from_register); 
  // Keep control of bus to immediately read data
  Wire.endTransmission(false); 

  Wire.requestFrom(MPU_ADDR, num_bytes, true); // Releases bus after
  for (int i = 0; Wire.available(); ++i) {
    read_data[i] = Wire.read();
  }
  return;
}

void write_to(const byte to_register, const byte write_value) {
  Wire.beginTransmission(MPU_ADDR);
  // First send address of register to which to write
  Wire.write(to_register); 
  Wire.write(write_value);
  Wire.endTransmission(true);
  return;
}

void record_mpu_accel() {
  // x high byte 3B, x low 3C, y high 3D, y low 3E, z high 3F, z low 40
  // ACCEL_REG is 3B
  byte buffer[6];
  read_from(ACCEL_REG, 6, buffer);
  int16_t a_x_raw = (buffer[0] << 8) | buffer[1];
  int16_t a_y_raw = (buffer[2] << 8) | buffer[3];
  int16_t a_z_raw = (buffer[4] << 8) | buffer[5];
  // Combine high byte and low byte into 16-bit accel value, then divide by LSB sensitivity to get accel in g-forces
  // Also factor in offsets
  mpu_a_x = a_x_raw / LSB_SENS + MPU_A_X_OFFSET;
  mpu_a_y = a_y_raw / LSB_SENS + MPU_A_Y_OFFSET;
  mpu_a_z = a_z_raw / LSB_SENS + MPU_A_Z_OFFSET;
}

void record_analog_accel() {
  analog_a_x = (analogRead(ANALOG_A_X_PIN) - 512.0) / 100.0;
  analog_a_y = (analogRead(ANALOG_A_Y_PIN) - 512.0) / 100.0;
  analog_a_z = (analogRead(ANALOG_A_Z_PIN) - 512.0) / 100.0;
}

bool detect_periodic_movement(float *buffer) {
  // Smooth signal
  float avg = smooth(buffer, WINDOW_SIZE);

  // Compute standard deviation
  float var = 0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    float d = buffer[i] - avg;
    var += d * d;
  }
  var /= WINDOW_SIZE;
  float stdDev = sqrt(var);

  if (stdDev < MOVEMENT_THRESHOLD) return false;

  // Compare halves for periodicity
  int half = WINDOW_SIZE / 2;
  float dot = 0, n1 = 0, n2 = 0;
  for (int i = 0; i < half; i++) {
    dot += buffer[i] * buffer[i + half];
    n1 += buffer[i] * buffer[i];
    n2 += buffer[i + half] * buffer[i + half];
  }
  float sim = dot / (sqrt(n1 * n2) + 1e-6);
  return (sim > PERIODIC_SIMILARITY);
}
// AID Statement: Artificial Intelligence Tool: ChatGPT 5, used November 2025; Writing—Review & Editing: Used for writing code for the detection of periodic movement.

#ifdef DEBUG_PRINT_ACCELS
  void print_accels() {
    Serial.print("analog_a_x:");
    Serial.print(analog_a_x);
    Serial.print(' ');
    Serial.print("analog_a_y:");
    Serial.print(analog_a_y);
    Serial.print(' ');
    Serial.print("analog_a_z:");
    Serial.print(analog_a_z);
    Serial.print(' ');
    Serial.print("analog_a_mag:");
    Serial.print(analog_a_mag);
    Serial.print(' ');
    Serial.print("mpu_a_x:");
    Serial.print(mpu_a_x);
    Serial.print(' ');
    Serial.print("mpu_a_y:");
    Serial.print(mpu_a_y);
    Serial.print(' ');
    Serial.print("mpu_a_z:");
    Serial.print(mpu_a_z);
    Serial.print(' ');
    Serial.print("mpu_a_mag:");
    Serial.print(mpu_a_mag);
    Serial.println();
  }
#endif
