#include <Wire.h>

// Analog accelerometer pins
const int AX_PIN = A0;
const int AY_PIN = A1;
const int AZ_PIN = A2;

// MPU9250
#define MPU_ADDR 0x68
int16_t ax, ay, az;

// Movement detection
const int SAMPLE_INTERVAL = 150;  // slower sampling
const int WINDOW_SIZE = 20;       // larger buffer for smoothing
const float MOVEMENT_THRESHOLD = 0.45;  // higher = less sensitive
const float PERIODIC_SIMILARITY = 0.8;

// Buffers
float armBuffer[WINDOW_SIZE];
float legBuffer[WINDOW_SIZE];
int indexBuffer = 0;


// Moving average helper
float smooth(float *buffer, int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += buffer[i];
  return sum / size;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU9250
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("Starting stable motion detection...");
}

void loop() {
  // Analog sensor
  float axA = (analogRead(AX_PIN) - 512.0) / 100.0;
  float ayA = (analogRead(AY_PIN) - 512.0) / 100.0;
  float azA = (analogRead(AZ_PIN) - 512.0) / 100.0;
  float magA = sqrt(axA * axA + ayA * ayA + azA * azA);

  // MPU9250
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();

  float axM = ax / 16384.0;
  float ayM = ay / 16384.0;
  float azM = az / 16384.0;
  float magM = sqrt(axM * axM + ayM * ayM + azM * azM);

  // Update buffers
  armBuffer[indexBuffer] = magA;
  legBuffer[indexBuffer] = magM;
  indexBuffer = (indexBuffer + 1) % WINDOW_SIZE;

  static unsigned long lastDetection = 0;
  if (detectPeriodicMovement(armBuffer) || detectPeriodicMovement(legBuffer)) {
    if (millis() - lastDetection > 2000) {
      Serial.println("Periodic movement detected!");
      lastDetection = millis();
    }
  }

  delay(SAMPLE_INTERVAL);
}

bool detectPeriodicMovement(float *buffer) {
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
