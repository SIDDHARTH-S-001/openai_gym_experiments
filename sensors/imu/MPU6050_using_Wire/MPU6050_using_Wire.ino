#include <Wire.h>

const int MPU6050_ADDRESS = 0x68; // MPU6050 I2C address
const int CONFIG = 0x1A;          // Configuration Register
const int PWR_MGMT_1 = 0x6B;      // Power Management 1 register
const int ACCEL_CONFIG = 0x1C;    // Accelerometer configuration register
const int GYRO_CONFIG = 0x1B;     // Gyroscope configuration register
const int ACCEL_XOUT_H = 0x3B;    // Accelerometer data registers
const int GYRO_XOUT_H = 0x43;     // Gyroscope data registers
const int AFS_SEL = 0x00;         // Accelerometer Full Scale Value set to +-2g
const int FS_SEL = 0x01;          // Gyroscope Full Scale Value set to +-500 degree/sec
// const int DLPF_CFG = 0x00;        // Digital Low Pass Filter disabled (Gyro output rate 8KHz, Accl Bandwidth 1KHz)
// const int DLPF_CFG = 0x01;     // Digital Low Pass Filter (Accl Bandwidth 184 Hz, delay 2.0 ms, Gyro Bandwidth 188 Hz, delay 1.9 ms)
const int DLPF_CFG = 0x06;        // Digital Low Pass Filter: Gyro Bandwidth 5 Hz, delay 19.0 ms

// Other parameters considered
// Rate Noise Spectral Density - 0.005 (deg/s)/(sqrt(Hz)) (at 10 Hz).
// At free run for 3 mins, rate noise density was 0.015 dps/sqrt(Hz). 
// Noise - "Band limited white noise".
// Damping ratio of 2nd order model > 0.5 (to avoid resonance).
// Sensitivity scale facor (for 500 dps) - 65.5 (nominal).
// bias was 0.0213.

// Needs Low Pass Filter with cutoff frequency 5Hz.
// Using 5Hz LPF (built-in): Bias = 0.015 and Rate Noise Density: 0.0013 rad/s/√Hz

// At 10 Hz sampling rate.
// bias = -0.0019 and rate noise density = 0.00023 dps/√Hz

int16_t accelerometer_x, accelerometer_y, accelerometer_z;
float ax, ay, az;
int16_t gyro_x, gyro_y, gyro_z;
float gx, gy, gz;

void setup() {
  Wire.begin();
  // Baud-rate needs to be really high to ensure higher frequencies don't get aliased down to lower frequencies.
  // Gyro output rate is 8KHz when DLPF is disabled.
  // Gyro output rate is KHz when DLPF is enabled.
  Serial.begin(115200); 

  // Wake up the MPU6050 by clearing the sleep bit in PWR_MGMT_1
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); // Clear the sleep bit
  Wire.endTransmission(true);

  // Configure the accelerometer and gyroscope
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(CONFIG);
  Wire.write(DLPF_CFG);
  Wire.write(ACCEL_CONFIG);
  //Wire.write(0x00); // Set accelerometer full scale range to +/- 2g
  Wire.write(AFS_SEL);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(GYRO_CONFIG);
  //Wire.write(0x00); // Set gyroscope full scale range to +/- 250 degrees/s
  Wire.write(FS_SEL);
  Wire.endTransmission(true);
}

void loop() {
  // Read accelerometer data
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true);
  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();

  ax = (float(accelerometer_x)/32767.0)*(2*9.8);
  ay = (float(accelerometer_y)/32767.0)*(2*9.8);
  az = (float(accelerometer_z)/32767.0)*(2*9.8);

  // Read gyroscope data
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true);
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  gx = (float(gyro_x)/32767.0)*(500)*(3.1416/180);
  gy = (float(gyro_x)/32767.0)*(500)*(3.1416/180);
  gz = (float(gyro_x)/32767.0)*(500)*(3.1416/180);

  // Print the data
  // Serial.print("Accelerometer: (m/s2) ");
  // Serial.print(ax);
  // Serial.print(", ");
  // Serial.print(ay);
  // Serial.print(", ");
  // Serial.print(az);
  // Serial.print(" | Gyroscope: (rads/s) ");
  // Serial.print(gx);
  // Serial.print(", ");
  // Serial.print(gy);
  // Serial.print(", ");
  Serial.println(gz, 7);

  delay(100); // Adjust the delay as needed
}
