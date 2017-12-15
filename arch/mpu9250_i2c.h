/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.

 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND

 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
 #ifndef MPU_H
 #define MPU_H

#include <Arduino.h>
#include <Wire.h>
#include "math.h"
#include "3dmath.h"
#include "mpu9250_registers_i2c.h"

class MPU_9250_I2C {

  public:
  // Set initial input parameters
  enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
  };

  enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
  };

  enum Mscale {
    MFS_14BITS = 0, // 0.6 mG per LSB
    MFS_16BITS      // 0.15 mG per LSB
  };

  // Specify sensor full scale
  uint8_t Gscale = GFS_2000DPS;
  uint8_t Ascale = AFS_4G;
  uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
  uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
  float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

  // Pin definitions
  int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
  //float PI = 3.14159265359;

  int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
  int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
  int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
  float gravity[3] = {0,0,0};
  float aclGravFree[3] = {0,0,0};
  float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
  float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
  int16_t tempCount;      // temperature raw count output
  float   temperature;    // Stores the real internal chip temperature in degrees Celsius
  float   SelfTest[6];    // holds results of gyro and accelerometer self test

  // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
  float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
  float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
  // There is a tradeoff in the beta parameter between accuracy and response speed.
  // In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
  // However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
  // Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
  // By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
  // I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
  // the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
  // In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
  float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
  float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
  #define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
  #define Ki 0.0f

  uint32_t delt_t = 0; // used to control display output rate
  uint32_t count = 0, sumCount = 0; // used to control display output rate
  float pitch, yaw, roll;
  float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
  uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
  uint32_t now = 0;        // used to calculate integration interval

  //Low Pass Filter
  float Kaxy_lowpass = 0.005;
  float Axy, Axy_lp;
  float Axy_MagThresh = 0.15;
  int lp_err_running_count = 0;
  int lp_err_count_thresh = 20;

  //Spin threshold
  float spinThreshold = 200;

  //Motion States
  bool rest = true;
  bool spin = false;
  bool oldSpin,oldRest;
  bool landed = false;

  //Max Height
  float throwMaxHeight=0;
  //Raw Measurements
  VectorFloat A, G, M;
  //Intermediate Vectors For High Level Positional FAlgorithm
  VectorFloat Grav, Alin, Awrld, Alast, V, X;
  Quaternion q;
  float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

  uint8_t orientationPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


  // Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
  // (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
  // which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
  // device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
  // The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
  // but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

   // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
   // measured ones.
  void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  void initialize();
  void update();

  //===================================================================================================================
  //====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
  //===================================================================================================================
  void getMres();
  void getGres();
  void getAres();
  void readAccelData(int16_t * destination);
  void readGyroData(int16_t * destination);
  void readMagData(int16_t * destination);
  int16_t readTempData();
  void initAK8963(float * destination);
  void initMPU9250();

  //Motion Intellegence
  //uint8_t dmpGetLinearAccel(float *v, float *vRaw, float *gravity);
  uint8_t dmpGetLinearAccel(VectorFloat &v, VectorFloat &vRaw, VectorFloat &gravity);
  //uint8_t dmpGetGravity(float *g);
  uint8_t dmpGetGravity(VectorFloat &g);
  void calculatePositionalInformation();
  void determineIsSpinning();
  void determineIsRest();
  void determineVelocityNPosition(VectorFloat &Alin, VectorFloat &Vel, VectorFloat &Pos);

  // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
  // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
  void calibrateMPU9250(float * dest1, float * dest2);
  // Accelerometer and gyroscope self test; check calibration wrt factory settings
  void MPU9250SelfTest(float * destination); // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
  // Wire.h read and write protocols
  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
  uint8_t readByte(uint8_t address, uint8_t subAddress);
  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
};

#endif //MPU_H
