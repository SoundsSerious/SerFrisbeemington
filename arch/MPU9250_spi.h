/**
 * Invensense MPU-9250 library using the SPI interface
 *
 * Copyright (C) 2015 Brian Chen
 *
 * Open source under the MIT License. See LICENSE.txt.
 */

#ifndef MPU9250_h
#define MPU9250_h
#include "Arduino.h"
#include "3dmath.h"
#include "math.h"
#include "MPU9250_registers_spi.h"
#include "SPI.h"

// #define AK8963FASTMODE
#define     Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)

class MPU9250_SPI {
public:
    // constructor. Default low pass filter of 188Hz
    MPU9250_SPI(long clock, uint8_t cs, uint8_t low_pass_filter = BITS_DLPF_CFG_188HZ, uint8_t low_pass_filter_acc = BITS_DLPF_CFG_188HZ){
        my_clock = clock;
        my_cs = cs;
        my_low_pass_filter = low_pass_filter;
        my_low_pass_filter_acc = low_pass_filter_acc;
    }
    unsigned int WriteReg(uint8_t WriteAddr, uint8_t WriteData );
    unsigned int slow_WriteReg(uint8_t WriteAddr, uint8_t WriteData );
    unsigned int ReadReg(uint8_t WriteAddr, uint8_t WriteData );
    unsigned int slow_ReadReg(uint8_t WriteAddr, uint8_t WriteData );
    void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes );
    void slow_ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes );

    void initialize();
    void update();

    bool spi_init(bool calib_gyro = true, bool calib_acc = true);
    void read_temp();
    void read_acc();
    void read_gyro();
    unsigned int set_gyro_scale(int scale);
    unsigned int set_acc_scale(int scale);
    void calib_acc();
    void calib_mag();
    void select();
    void slow_select();
    void deselect();
    unsigned int whoami();
    uint8_t AK8963_whoami();
    uint8_t get_CNTL1();
    void read_mag();
    void read_all();
    void calibrate(float *dest1, float *dest2);

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
    uint8_t dmpGetLinearAccel(VectorFloat &v, VectorFloat &vRaw, VectorFloat &gravity);
    //uint8_t dmpGetGravity(float *g);
    uint8_t dmpGetGravity(VectorFloat &g);
    void calculatePositionalInformation();
    void determineIsSpinning();
    void determineIsRest();
    void determineVelocityNPosition(VectorFloat &Alin, VectorFloat &Vel, VectorFloat &Pos);

    float acc_divider;
    float gyro_divider;

    int calib_data[3];
    float Magnetometer_ASA[3];

    float accel_data[3];
    float gyro_data[3];
    float mag_data[3];
    int16_t mag_data_raw[3];

    float randomstuff[3];   // seemed to be an issue with memory being disturbed so allocated random memory space here

private:
    long my_clock;
    uint8_t my_cs;
    uint8_t my_low_pass_filter;
    uint8_t my_low_pass_filter_acc;

    //float randomstuffs[3];

    float g_bias[3];
    float a_bias[3];      // Bias corrections for gyro and accelerometer
};

extern SPIClass SPI2;
#endif
