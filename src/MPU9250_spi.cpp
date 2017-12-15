/**
 * Invensense MPU-9250 library using the SPI interface
 *
 * Copyright (C) 2015 Brian Chen
 *
 * Open source under the MIT License. See LICENSE.txt.
 */


#include "MPU9250_spi.h"
#include "globals.h"
#include <memory>

#define MPU_InitRegNum 17

bool MPU9250::init(bool calib_gyro, bool calib_acc){
    pinMode(my_cs, OUTPUT);
#ifdef CORE_TEENSY
    digitalWriteFast(my_cs, HIGH);
#else
    digitalWrite(my_cs, HIGH);
#endif
    float temp[3];

    if(calib_gyro && calib_acc){
        calibrate(g_bias, a_bias);
    }
    else if(calib_gyro){
        calibrate(g_bias, temp);
    }
    else if(calib_acc){
        calibrate(temp, a_bias);
    }

    uint8_t i = 0;
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
        {BIT_H_RESET, MPUREG_PWR_MGMT_1},        // Reset Device
        {0x01, MPUREG_PWR_MGMT_1},               // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},               // Enable Acc & Gyro
        {my_low_pass_filter, MPUREG_CONFIG},     // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {BITS_FS_2000DPS, MPUREG_GYRO_CONFIG},    // +-250dps
        {BITS_FS_4G, MPUREG_ACCEL_CONFIG},       // +-2G
        {my_low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x22, MPUREG_INT_PIN_CFG},      // Was 0x12
        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
        {0x20, MPUREG_USER_CTRL},        // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
        {0x0D, MPUREG_I2C_MST_CTRL},     // I2C configuration multi-master  IIC 400KHz

        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  // Set the I2C slave addres of AK8963 and set for write.
        //{0x09, MPUREG_I2C_SLV4_CTRL},
        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO},   // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL}, // Enable I2C and set 1 byte

        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
        {0x16, MPUREG_I2C_SLV0_DO},   //0x12 -8hz //0x16-100hz 16bit // Register value to 100Hz continuous measurement in 16bit
        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte

    };

    for(i = 0; i < MPU_InitRegNum; i++) {
        WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        delayMicroseconds(50);  // I2C must slow down the write speed, otherwise it won't work
    }

    set_acc_scale(BITS_FS_4G);
    set_gyro_scale(BITS_FS_2000DPS);

    calib_mag();  // If experiencing problems here, just comment it out. Should still be somewhat functional.
    return 0;
}

//Loop Declarations
void MPU9250::initialize()
{
  frisbeem.com.log("Initializing MPU");

  delay(100);

  SPI.begin(MPU_CLOCK_PIN, MPU_SLAVE_OUT, MPU_DATA_PIN,MPU_CS);

  init(false,false); //Don't Calibrate
  delay(10);
  uint8_t wai = whoami();
  if (wai == 0x71 || wai == 0x73){
    frisbeem.com.log("Successful connection");
  }
  else{
    frisbeem.com.log("Failed connection: ");
    frisbeem.com.log(String(wai));
  }

  uint8_t wai_AK8963 = AK8963_whoami();
  if (wai_AK8963 == 0x48){
    frisbeem.com.log("Successful connection to mag");
  }
  else{
    frisbeem.com.log("Failed connection to mag: ");
    frisbeem.com.log(String(wai_AK8963));
  }

  //Calibrate If Connected
  calib_acc();
  calib_mag();

}

void MPU9250::update()
{
   read_mpu_interrupt();
   if (mpu_read){
     frisbeem.com.log("MPU Interrupt");

     //Gyro
     frisbeem.com.log("Reading Gyro");
     read_gyro();
     G.x = gyro_data[0];
     G.y = gyro_data[1];
     G.z = gyro_data[2];

     //Accelerometer
     frisbeem.com.log("Reading Accelerometer");
     read_acc();
     A.x = accel_data[0];
     A.y = accel_data[1];
     A.z = accel_data[2];
   }


 //Magnometer;
  if (mag_count == 0) {
    read_mag();
  }
  mag_count++;
  if (mag_count > mag_read_count){ mag_count =0; }
  sum += deltat; // sum for averaging filter update rate
  sumCount++;

  enqueue_motion();
  //Meter Algorithm Time --This Shit Gonna Get Crazy
  long strt = micros();
  //frisbeem.com.log("ITM:\t"+String(strt-now));


  calculatePositionalInformation();

  //OverSpeed Correction

  //Kalman Filter - Inertia + BLE Distance

  long now = micros();
  //frisbeem.com.log("CTM:\t"+String(now-strt));
}

void MPU9250::enqueue_motion()
{
  //frisbeem.com.log("Angular Rate: "+String(G.z));
  MotionEvent * motionEvent = new MotionEvent( G.z );
  //std::unique_ptr<MotionEvent> motionEvent( new MotionEvent(G.z));
  frisbeem.event_queue.addEvent( motionEvent );
}

/////////////////////////// Motion Processing //////////////////////////////////
//Positional Information Calculations
void MPU9250::calculatePositionalInformation(){
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
  now = micros();
  deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update

  frisbeem.com.log("Madgwick");
  MadgwickQuaternionUpdate(A.x,A.y,A.z,G.x*PI/180.0f,G.y*PI/180.0f,G.z*PI/180.0f,M.y,M.x,M.z);

  // frisbeem.com.log("Mahony");
  // MahonyQuaternionUpdate(A.x,A.y,A.z,G.x*PI/180.0f,G.y*PI/180.0f,G.z*PI/180.0f,M.y,M.x,M.z);

  dmpGetGravity( Grav );
  dmpGetLinearAccel(Alin, A, Grav);
  Awrld = Alin.getRotated( &q );
  determineIsSpinning();
  determineIsRest();
  determineVelocityNPosition(Awrld,V,X);
  lastUpdate = now;
}

void MPU9250::determineIsSpinning()
{
  oldSpin = spin;
  if ( fabs(G.z) > spinThreshold ){ spin = true;}
  else{ spin = false;}
}
//Determine If At rest
void MPU9250::determineIsRest()
{
  oldRest = rest;
  //Root Sum Square XY acceleration
  Axy = sqrt(Awrld.x*Awrld.x + Awrld.y*Awrld.y);
  //Low Pass Filter
  Axy_lp = Axy_lp + (Axy - Axy_lp) * Kaxy_lowpass;
  //Check For Stable State To Zero Low Pass
  if (Axy < Axy_MagThresh && Axy_lp > Axy_MagThresh){lp_err_running_count += 1;}
  else {lp_err_running_count = 0;}
  if ( !spin & lp_err_running_count > lp_err_count_thresh ){ Axy_lp = 0; }
  //Check If Not Spinning, And Then Set Rest True If Below Criteria
  if ( spin ){ rest = false;}
  else
  {
    if ( Axy_lp < Axy_MagThresh){ rest = true;}
    else if ( Axy_lp >= Axy_MagThresh){ rest = false;}
 }
}
//Performs Double Integration
void MPU9250::determineVelocityNPosition(VectorFloat &Awrld, VectorFloat &Vel, VectorFloat &Pos)
{
  //Store Last Iteration Variables
  float vx,vy,vz;
  float ax,ay,az;

  vx = Vel.x; vy = Vel.y; vz = Vel.z;

  float h = deltat / 2.0; //This Is For The Trapezoid Rule

  if ( rest == false )
  { //Perform Integration If Moving (Not Resting )
    ax = ( Awrld.x + Alast.x ) * GRAVITY;
    ay = ( Awrld.y + Alast.y ) * GRAVITY;
    az = ( Awrld.z + Alast.z ) * GRAVITY;
    Vel.x += ax* h;
    Vel.y += ay* h;
    Vel.z += az* h;
  }
  else
  { //If Resting Set Velocity Zero
    Vel.x = 0;
    Vel.y = 0;
    Vel.z = 0;
  }
  //If Spinning Integrate Position
  if ( spin ){
    float hsq = h*h; //By Coincidence h/2 * h/2 also works for taylor / trapezoid combine (f1+f2)/2 * dt^2/2 
    Pos.x += ( Vel.x + vx ) * h + ax * hsq;
    Pos.y += ( Vel.y + vy ) * h + ay * hsq;
    Pos.z += ( Vel.z + vz ) * h + az * hsq;

    if ( Pos.z > throwMaxHeight){
      //Send Max Height Event
      throwMaxHeight = Pos.z;
    }

    if (Pos.z < 0){ //Protect Against Going Through Floor
      Pos.z = 0;
    }
  }
  //If Resting Set Position To 0,0,1 m off ground
  else if ( rest ){
      Pos.x = 0;
      Pos.y = 0;
      Pos.z = 1;
  }

  if (spin == false)
  { //Check If Was Spinning & Thrown Up
    if ( oldSpin && throwMaxHeight > 3 ) {
      //Send Peak Height
    }
  }

  //Store This Acceleration Value
  Alast.x = Awrld.x; Alast.y = Awrld.y; Alast.z = Awrld.z;

}

uint8_t MPU9250::dmpGetLinearAccel(VectorFloat &v, VectorFloat &vRaw, VectorFloat &gravity) {
    // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet)
    v.x = vRaw.x - gravity.x;
    v.y = vRaw.y - gravity.y;
    v.z = vRaw.z - gravity.z;
    return 0;
}

uint8_t MPU9250::dmpGetGravity(VectorFloat &g) {
    g.x  = 2 * (q.x*q.z - q.w*q.y);
    g.y  = 2 * (q.w*q.x + q.y*q.z);
    g.z  = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
    return 0;
}



/////////////////////////// Data ACcess Routines //////////////////////////////////

void MPU9250::select() {
    //Set CS low to start transmission (interrupts conversion)
    SPI.beginTransaction(SPISettings(backup_clock, MSBFIRST, SPI_MODE3));
#ifdef CORE_TEENSY
    digitalWriteFast(my_cs, LOW);
#else
    digitalWrite(my_cs, LOW);
#endif
}

void MPU9250::select_fast() {
    //Set CS low to start transmission (interrupts conversion)
    SPI.beginTransaction(SPISettings(my_clock, MSBFIRST, SPI_MODE3));
#ifdef CORE_TEENSY
    digitalWriteFast(my_cs, LOW);
#else
    digitalWrite(my_cs, LOW);
#endif
}

void MPU9250::select_spibus() {
    //Set CS low to start transmission (interrupts conversion)
    SPI.beginTransaction(SPISettings(spibus_clock, MSBFIRST, SPI_MODE3));
#ifdef CORE_TEENSY
    digitalWriteFast(my_cs, LOW);
#else
    digitalWrite(my_cs, LOW);
#endif
}

void MPU9250::deselect() {
    //Set CS high to stop transmission (restarts conversion)
#ifdef CORE_TEENSY
    digitalWriteFast(my_cs, HIGH);
#else
    digitalWrite(my_cs, HIGH);
#endif
    SPI.endTransaction();
}

unsigned int MPU9250::WriteReg( uint8_t WriteAddr, uint8_t WriteData )
{
    unsigned int temp_val;

    select();
    SPI.transfer(WriteAddr);
    temp_val=SPI.transfer(WriteData);
    deselect();

    //delayMicroseconds(50);
    return temp_val;
}

unsigned int MPU9250::WriteReg_Fast( uint8_t WriteAddr, uint8_t WriteData )
{
    unsigned int temp_val;

    select_fast();
    SPI.transfer(WriteAddr);
    temp_val=SPI.transfer(WriteData);
    deselect();

    //delayMicroseconds(50);
    return temp_val;
}

unsigned int MPU9250::WriteReg_SpiBus( uint8_t WriteAddr, uint8_t WriteData )
{
    unsigned int temp_val;

    select_spibus();
    SPI.transfer(WriteAddr);
    temp_val=SPI.transfer(WriteData);
    deselect();

    //delayMicroseconds(50);
    return temp_val;
}

unsigned int  MPU9250::ReadReg( uint8_t WriteAddr, uint8_t WriteData )
{
    return WriteReg(WriteAddr | READ_FLAG,WriteData);
}

unsigned int  MPU9250::ReadReg_Fast( uint8_t WriteAddr, uint8_t WriteData )
{
    return WriteReg_Fast(WriteAddr | READ_FLAG,WriteData);
}

void MPU9250::ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )
{
    unsigned int  i = 0;

    select();
    SPI.transfer(ReadAddr | READ_FLAG);
    for(i = 0; i < Bytes; i++)
        ReadBuf[i] = SPI.transfer(0x00);
    deselect();

    //delayMicroseconds(50);
}

void MPU9250::ReadRegs_Fast( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )
{
    unsigned int  i = 0;

    select_fast();
    SPI.transfer(ReadAddr | READ_FLAG);
    for(i = 0; i < Bytes; i++)
        ReadBuf[i] = SPI.transfer(0x00);
    deselect();

    //delayMicroseconds(50);
}


/*                                     INITIALIZATION
 * usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
 * low pass filter value; suitable values are:
 * BITS_DLPF_CFG_256HZ_NOLPF2
 * BITS_DLPF_CFG_188HZ
 * BITS_DLPF_CFG_98HZ
 * BITS_DLPF_CFG_42HZ
 * BITS_DLPF_CFG_20HZ
 * BITS_DLPF_CFG_10HZ
 * BITS_DLPF_CFG_5HZ
 * BITS_DLPF_CFG_2100HZ_NOLPF
 * returns 1 if an error occurred
 */



/*                                ACCELEROMETER SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * accelerometers. Suitable ranges are:
 * BITS_FS_2G
 * BITS_FS_4G
 * BITS_FS_8G
 * BITS_FS_16G
 * returns the range set (2,4,8 or 16)
 */

unsigned int MPU9250::set_acc_scale(int scale){
    unsigned int temp_scale;
    WriteReg(MPUREG_ACCEL_CONFIG, scale);

    switch (scale){
        case BITS_FS_2G:
            acc_divider=16384;
        break;
        case BITS_FS_4G:
            acc_divider=8192;
        break;
        case BITS_FS_8G:
            acc_divider=4096;
        break;
        case BITS_FS_16G:
            acc_divider=2048;
        break;
    }
    temp_scale = WriteReg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);

    switch (temp_scale){
        case BITS_FS_2G:
            temp_scale=2;
        break;
        case BITS_FS_4G:
            temp_scale=4;
        break;
        case BITS_FS_8G:
            temp_scale=8;
        break;
        case BITS_FS_16G:
            temp_scale=16;
        break;
    }
    return temp_scale;
}



/*                                 GYROSCOPE SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * gyroscopes. Suitable ranges are:
 * BITS_FS_250DPS
 * BITS_FS_500DPS
 * BITS_FS_1000DPS
 * BITS_FS_2000DPS
 * returns the range set (250,500,1000 or 2000)
 */

unsigned int MPU9250::set_gyro_scale(int scale){
    unsigned int temp_scale;
    WriteReg(MPUREG_GYRO_CONFIG, scale);

    switch (scale){
        case BITS_FS_250DPS:   gyro_divider = 131;  break;
        case BITS_FS_500DPS:   gyro_divider = 65.5; break;
        case BITS_FS_1000DPS:  gyro_divider = 32.8; break;
        case BITS_FS_2000DPS:  gyro_divider = 16.4; break;
    }

    temp_scale = WriteReg(MPUREG_GYRO_CONFIG|READ_FLAG, 0x00);

    switch (temp_scale){
        case BITS_FS_250DPS:   temp_scale = 250;    break;
        case BITS_FS_500DPS:   temp_scale = 500;    break;
        case BITS_FS_1000DPS:  temp_scale = 1000;   break;
        case BITS_FS_2000DPS:  temp_scale = 2000;   break;
    }
    return temp_scale;
}



/*                                 WHO AM I?
 * usage: call this function to know if SPI is working correctly. It checks the I2C address of the
 * mpu9250 which should be 0x71
 */

unsigned int MPU9250::whoami(){
    unsigned int response;
    response = WriteReg(MPUREG_WHOAMI|READ_FLAG, 0x00);
    return response;
}



/*                                 READ ACCELEROMETER
 * usage: call this function to read accelerometer data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */

void MPU9250::read_acc()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs_Fast(MPUREG_ACCEL_XOUT_H,response,6);
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8)|response[i*2+1];
        data = (float)bit_data;
        accel_data[i] = data/acc_divider - a_bias[i];
    }

}

/*                                 READ GYROSCOPE
 * usage: call this function to read gyroscope data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */

void MPU9250::read_gyro()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs_Fast(MPUREG_GYRO_XOUT_H,response,6);
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
        data = (float)bit_data;
        gyro_data[i] = data/gyro_divider - g_bias[i];
    }

}


/*                                 READ temperature
 * usage: call this function to read temperature data.
 * returns the value in °C
 */

void MPU9250::read_temp(){
    uint8_t response[2];
    int16_t bit_data;
    float data;
    ReadRegs_Fast(MPUREG_TEMP_OUT_H,response,2);

    bit_data = ((int16_t)response[0]<<8)|response[1];
    data = (float)bit_data;
    temperature = (data/340)+36.53;
    //deselect();
}

/*                                 READ ACCELEROMETER CALIBRATION
 * usage: call this function to read accelerometer data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 * returns Factory Trim value
 */

void MPU9250::calib_acc()
{
    uint8_t response[4];
    int temp_scale;
    //READ CURRENT ACC SCALE
    temp_scale=WriteReg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);
    set_acc_scale(BITS_FS_8G);
    //ENABLE SELF TEST need modify
    //temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);

    ReadRegs(MPUREG_SELF_TEST_X,response,4);
    calib_data[0] = ((response[0]&11100000)>>3) | ((response[3]&00110000)>>4);
    calib_data[1] = ((response[1]&11100000)>>3) | ((response[3]&00001100)>>2);
    calib_data[2] = ((response[2]&11100000)>>3) | ((response[3]&00000011));

    set_acc_scale(temp_scale);
}

uint8_t MPU9250::AK8963_whoami(){
    uint8_t response;
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    delayMicroseconds(100);
    response = WriteReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);    //Read I2C
    //ReadRegs(MPUREG_EXT_SENS_DATA_00,response,1);
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C

    return response;
}

void MPU9250::calib_mag(){
    uint8_t response[3];
    float data;
    int i;
    // Choose either 14-bit or 16-bit magnetometer resolution
    //uint8_t MFS_14BITS = 0; // 0.6 mG per LSB
    uint8_t MFS_16BITS =1; // 0.15 mG per LSB
    // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    uint8_t M_8HZ = 0x06; // 8 Hz update
    //uint8_t M_100HZ = 0x06; // 100 Hz continuous magnetometer

    /* get the magnetometer calibration */

    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);   // Set the I2C slave    addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX);                 // I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83);                       // Read 3 bytes from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);                     // Enable I2C and set bytes
    delayMicroseconds(1000);
    //response[0]=WriteReg(MPUREG_EXT_SENS_DATA_01|READ_FLAG, 0x00); //Read I2C

    WriteReg(AK8963_CNTL1, 0x00);                               // set AK8963 to Power Down
    delayMicroseconds(50000);                                                  // long wait between AK8963 mode changes
    WriteReg(AK8963_CNTL1, 0x0F);                               // set AK8963 to FUSE ROM access
    delayMicroseconds(50000);                                                  // long wait between AK8963 mode changes

    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);              // Read I2C
    for(i = 0; i < 3; i++) {
        data=response[i];
        Magnetometer_ASA[i] = ((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
    WriteReg(AK8963_CNTL1, 0x00); // set AK8963 to Power Down
    delayMicroseconds(500);
    // Configure the magnetometer for continuous read and highest resolution.
    // Set bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
    // register, and enable continuous mode data acquisition (bits [3:0]),
    // 0010 for 8 Hz and 0110 for 100 Hz sample rates.
    WriteReg(AK8963_CNTL1, MFS_16BITS << 4 | M_8HZ);            // Set magnetometer data resolution and sample ODR
    delayMicroseconds(500);
}

void MPU9250::read_mag(){
    uint8_t response[7];
    float data;
    int i;

    WriteReg_SpiBus(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);  // Set the I2C slave addres of AK8963 and set for read.
    WriteReg_SpiBus(MPUREG_I2C_SLV0_REG, AK8963_HXL);                 // I2C slave 0 register address from where to begin data transfer
    WriteReg_SpiBus(MPUREG_I2C_SLV0_CTRL, 0x87);                      // Read 6 bytes from the magnetometer

    // delayMicroseconds(1000);
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    mag_overflowByte = response[6];
    // must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
    if(!(mag_overflowByte & 0x08)){
      for(i = 0; i < 3; i++) {
          mag_data_raw[i] = ((int16_t)response[i*2+1]<<8)|response[i*2];
          data = (float)mag_data_raw[i];
          mag_data[i] = data*Magnetometer_ASA[i];
      }
      M.x = mag_data[0];
      M.y = mag_data[1];
      M.z = mag_data[2];
      frisbeem.com.log("Read Mag Data");
    }
    frisbeem.com.log("MAG ST2:\t"+String(mag_overflowByte,BIN));
    mag_overflow = (bitRead(mag_overflowByte,3) == 1 );
}

// void MPU9250::read_mag(){ //Try to parse interrupt in a smart way... come back to this
//     uint8_t response[6];
//     float data;
//     int i;
//
//     WriteReg_SpiBus(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);  // Set the I2C slave addres of AK8963 and set for read.
//     WriteReg_SpiBus(MPUREG_I2C_SLV0_REG, AK8963_ST1);                 // I2C slave 0 register address from where to begin data transfer
//     WriteReg_SpiBus(MPUREG_I2C_SLV0_CTRL, 0x81);                      // Read 1 bytes from the magnetometer
//
//     mag_interruptStatus = ReadReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);
//     mag_read = (bitRead(mag_interruptStatus,0) != 0);
//
//     WriteReg_SpiBus(MPUREG_I2C_SLV0_REG, AK8963_HXL);                 // I2C slave 0 register address from where to begin data transfer
//     WriteReg_SpiBus(MPUREG_I2C_SLV0_CTRL, 0x86);                      // Read 8 bytes from the magnetometer
//     ReadRegs(MPUREG_EXT_SENS_DATA_01,response,6);
//
//     WriteReg_SpiBus(MPUREG_I2C_SLV0_REG, AK8963_ST2);                 // I2C slave 0 register address from where to begin data transfer
//     WriteReg_SpiBus(MPUREG_I2C_SLV0_CTRL, 0x81);                      // Read 8 bytes from the magnetometer
//     mag_overflowByte = ReadReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);
//     mag_overflow = (bitRead(mag_interruptStatus,0) != 0);
//
//     // must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
//     if(!(mag_overflowByte & 0x08)) {
//       for(i = 0; i < 3; i++) {
//             mag_data_raw[i] = ((int16_t)response[i*2+1]<<8)|response[i*2];
//             data = (float)mag_data_raw[i];
//             mag_data[i] = data*Magnetometer_ASA[i];
//       }
//       M.x = mag_data[0];
//       M.y = mag_data[1];
//       M.z = mag_data[2];
//       frisbeem.com.log("MAG READ",true);
//     }
//     frisbeem.com.log("MAG ST1:\t"+String(mag_interruptStatus,BIN),true);
//     frisbeem.com.log("MAG ST2:\t"+String(mag_overflowByte,BIN),true);
//
//     mag_read = (bitRead(mag_interruptStatus,0) != 0);
//     mag_overflow = (bitRead(mag_overflowByte,3) == 1 );
// }

void MPU9250::read_mag_interrupt(){
  WriteReg_SpiBus(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);  // Set the I2C slave addres of AK8963 and set for read.
  WriteReg_SpiBus(MPUREG_I2C_SLV0_REG, AK8963_ST1);                 // I2C slave 0 register address from where to begin data transfer
  WriteReg_SpiBus(MPUREG_I2C_SLV0_CTRL, 0x81);                      // Read 1 bytes from the magnetometer
  mag_interruptStatus = ReadReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);
  mag_read = (bitRead(mag_interruptStatus,0) != 0);

}

void MPU9250::read_mpu_interrupt(){
  mpu_interruptStatus = ReadReg_Fast(MPUREG_INT_STATUS,0x00);
  //Interupt Status
  mpu_read = (bitRead(mpu_interruptStatus,0) != 0);
}

uint8_t MPU9250::get_CNTL1(){
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_CNTL1);              // I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

    // delayMicroseconds(1000);
    return WriteReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00);    //Read I2C
}

void MPU9250::read_all(){
    uint8_t response[21];
    int16_t bit_data;
    float data;
    int i;

    // Send I2C command at first
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL);                // I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);                     // Read 7 bytes from the magnetometer
    // must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

    ReadRegs(MPUREG_ACCEL_XOUT_H,response,21);
    // Get accelerometer value
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
        data = (float)bit_data;
        accel_data[i] = data/acc_divider - a_bias[i];
    }
    // Get temperature
    bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
    data = (float)bit_data;
    temperature = ((data-21)/333.87)+21;
    // Get gyroscope value
    for(i=4; i < 7; i++) {
        bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
        data = (float)bit_data;
        gyro_data[i-4] = data/gyro_divider - g_bias[i-4];
    }
    // Get Magnetometer value
    for(i=7; i < 10; i++) {
        mag_data_raw[i] = ((int16_t)response[i*2+1]<<8) | response[i*2];
        data = (float)mag_data_raw[i];
        mag_data[i-7] = data * Magnetometer_ASA[i-7];
    }
}

void MPU9250::calibrate(float *dest1, float *dest2){
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device
    WriteReg(MPUREG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    WriteReg(MPUREG_PWR_MGMT_1, 0x01);
    WriteReg(MPUREG_PWR_MGMT_2, 0x00);
    delay(200);

    // Configure device for bias calculation
    WriteReg(MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
    WriteReg(MPUREG_FIFO_EN, 0x00);      // Disable FIFO
    WriteReg(MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    WriteReg(MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
    WriteReg(MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    WriteReg(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    WriteReg(MPUREG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    WriteReg(MPUREG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    WriteReg(MPUREG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    WriteReg(MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    WriteReg(MPUREG_USER_CTRL, 0x40);   // Enable FIFO
    WriteReg(MPUREG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    WriteReg(MPUREG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    ReadRegs(MPUREG_FIFO_COUNTH, data, 2); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        ReadRegs(MPUREG_FIFO_R_W, data, 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];

    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accelsensitivity;}

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    // Push gyro biases to hardware registers
    WriteReg(MPUREG_XG_OFFS_USRH, data[0]);
    WriteReg(MPUREG_XG_OFFS_USRL, data[1]);
    WriteReg(MPUREG_YG_OFFS_USRH, data[2]);
    WriteReg(MPUREG_YG_OFFS_USRL, data[3]);
    WriteReg(MPUREG_ZG_OFFS_USRH, data[4]);
    WriteReg(MPUREG_ZG_OFFS_USRL, data[5]);

    // Output scaled gyro biases for display in the main program
    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    ReadRegs(MPUREG_XA_OFFSET_H, data, 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    ReadRegs(MPUREG_YA_OFFSET_H, data, 2);
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    ReadRegs(MPUREG_ZA_OFFSET_H, data, 2);
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for(ii = 0; ii < 3; ii++) {
      if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
    WriteReg(MPUREG_XA_OFFSET_H, data[0]);
    WriteReg(MPUREG_XA_OFFSET_L, data[1]);
    WriteReg(MPUREG_YA_OFFSET_H, data[2]);
    WriteReg(MPUREG_YA_OFFSET_L, data[3]);
    WriteReg(MPUREG_ZA_OFFSET_H, data[4]);
    WriteReg(MPUREG_ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MPU9250::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q.w, q2 = q.x, q3 = q.y, q4 = q.z;   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q.w = q1 * norm;
            q.x = q2 * norm;
            q.y = q3 * norm;
            q.z = q4 * norm;

        }



 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones.
void MPU9250::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {   frisbeem.com.log("START Mahony");
            float q1 = q.w, q2 = q.x, q3 = q.y, q4 = q.z;   // short name local variable for readability
            float norm;
            float hx, hy, bx, bz;
            float vx, vy, vz, wx, wy, wz;
            float ex, ey, ez;
            float pa, pb, pc;

            // Auxiliary variables to avoid repeated arithmetic
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f / norm;        // use reciprocal for division
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
            hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
            bx = sqrt((hx * hx) + (hy * hy));
            bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

            // Estimated direction of gravity and magnetic field
            vx = 2.0f * (q2q4 - q1q3);
            vy = 2.0f * (q1q2 + q3q4);
            vz = q1q1 - q2q2 - q3q3 + q4q4;
            wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
            wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
            wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

            // Error is cross product between estimated direction and measured direction of gravity
            ex = (ay * vz - az * vy) + (my * wz - mz * wy);
            ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
            ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
            if (Ki > 0.0f)
            {
                eInt[0] += ex;      // accumulate integral error
                eInt[1] += ey;
                eInt[2] += ez;
            }
            else
            {
                eInt[0] = 0.0f;     // prevent integral wind up
                eInt[1] = 0.0f;
                eInt[2] = 0.0f;
            }

            // Apply feedback terms
            gx = gx + Kp * ex + Ki * eInt[0];
            gy = gy + Kp * ey + Ki * eInt[1];
            gz = gz + Kp * ez + Ki * eInt[2];

            // Integrate rate of change of quaternion
            pa = q2;
            pb = q3;
            pc = q4;
            q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
            q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
            q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
            q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

            // Normalise quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
            norm = 1.0f / norm;
            q.w = q1 * norm;
            q.x = q2 * norm;
            q.y = q3 * norm;
            q.z = q4 * norm;
            frisbeem.com.log("END Mahony");
        }
