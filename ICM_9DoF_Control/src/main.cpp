#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ================= ISM330DHCX =================
#define ISM330_ADDR   0x6B

#define WHO_AM_I      0x0F
#define CTRL1_XL      0x10
#define CTRL2_G       0x11

#define OUTX_L_G      0x22
#define OUTX_L_A      0x28

// Scale factors (datasheet)
#define GYRO_SENS     0.00875    // dps/LSB (±250 dps)
#define ACC_SENS      0.000061   // g/LSB   (±2g)

// Complementary filter
float roll = 0.0f;
float pitch = 0.0f;
float alpha = 0.98;

unsigned long lastT = 0;

// ---------- I2C helpers ----------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ISM330_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(ISM330_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ISM330_ADDR, len);
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.begin(21, 22);

  Serial.println("ISM330DHCX BRING-UP");

  // ---- Configure IMU ----
  // Gyro: 104 Hz, ±250 dps
  writeReg(CTRL2_G, 0x40);

  // Accel: 104 Hz, ±2g
  writeReg(CTRL1_XL, 0x40);

  lastT = millis();
  Serial.println("IMU CONFIGURED");
}

void loop() {
  uint8_t buf[6];

  // -------- READ GYRO --------
  readRegs(OUTX_L_G, buf, 6);
  int16_t gx_raw = buf[1] << 8 | buf[0];
  int16_t gy_raw = buf[3] << 8 | buf[2];
  int16_t gz_raw = buf[5] << 8 | buf[4];

  // -------- READ ACCEL --------
  readRegs(OUTX_L_A, buf, 6);
  int16_t ax_raw = buf[1] << 8 | buf[0];
  int16_t ay_raw = buf[3] << 8 | buf[2];
  int16_t az_raw = buf[5] << 8 | buf[4];

  // -------- SCALE DATA --------
  float gx = gx_raw * GYRO_SENS; // dps
  float gy = gy_raw * GYRO_SENS;

  float ax = ax_raw * ACC_SENS; // g
  float ay = ay_raw * ACC_SENS;
  float az = az_raw * ACC_SENS;

  // -------- TIME --------
  unsigned long now = millis();
  float dt = (now - lastT) / 1000.0f;
  lastT = now;

  // -------- ACCEL ANGLES --------
  float accelRoll  = atan2(ay, az) * 57.3;
  float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.3;

  // -------- COMPLEMENTARY FILTER --------
  roll  = alpha * (roll  + gx * dt) + (1.0f - alpha) * accelRoll;
  pitch = alpha * (pitch + gy * dt) + (1.0f - alpha) * accelPitch;

  // -------- OUTPUT --------
  Serial.print("ROLL:");
  Serial.print(roll, 2);
  Serial.print(",PITCH:");
  Serial.println(pitch, 2);

  delay(20); // ~100 Hz
}
