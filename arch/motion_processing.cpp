/////////////////////////// Motion Processing //////////////////////////////////
//Positional Information Calculations
void MPU_9250_I2C::calculatePositionalInformation(){
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

void MPU_9250_I2C::determineIsSpinning()
{
  oldSpin = spin;
  if ( G.z > spinThreshold ){ spin = true;}
  else{ spin = false;}
}
//Determine If At rest
void MPU_9250_I2C::determineIsRest()
{
  oldRest = rest;
  //Root Sum Square XY acceleration
  Axy = sqrt(Awrld.x*Awrld.x + Awrld.y*Awrld.y);
  //Low Pass Filter
  Axy_lp = Axy_lp + (Axy - Axy_lp) * Kaxy_lowpass;
  //Check For Stable State To Zero Low Pass
  if (Axy < Axy_MagThresh && Axy_lp > Axy_MagThresh){lp_err_running_count += 1;}
  else {lp_err_running_count = 0;}
  if ( lp_err_running_count > lp_err_count_thresh ){ Axy_lp = 0; }
  //Check If Not Spinning, And Then Set Rest True If Below Criteria
  if ( spin ){ rest = false;}
  else
  {
    if ( Axy_lp < Axy_MagThresh){ rest = true;}
    else if ( Axy_lp >= Axy_MagThresh){ rest = false;}
 }
}
//Performs Double Integration
void MPU_9250_I2C::determineVelocityNPosition(VectorFloat &Awrld, VectorFloat &Vel, VectorFloat &Pos)
{
  //Store Last Iteration Variables
  float vx,vy,vz;
  vx = Vel.x; vy = Vel.y; vz = Vel.z;

  float h = deltat / 2.0;

  if ( rest == false )
  { //Perform Integration If Moving (Not Resting )
    float h_mps = h * 9.81; //Gravity Beeches
    Vel.x += ( Awrld.x + Alast.x ) * h_mps;
    Vel.y += ( Awrld.y + Alast.y ) * h_mps;
    Vel.z += ( Awrld.z + Alast.z ) * h_mps;
  }
  else
  { //If Resting Set Velocity Zero
    Vel.x = 0;
    Vel.y = 0;
    Vel.z = 0;
  }
  //If Spinning Integrate Position
  if ( spin ){
    Pos.x += ( Vel.x + vx ) * h;
    Pos.y += ( Vel.y + vy ) * h;
    Pos.z += ( Vel.z + vz ) * h;

    if ( Pos.z > throwMaxHeight){throwMaxHeight = Pos.z;}

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
      frisbeem.lights.scale(throwMaxHeight,10);
      throwMaxHeight = 0; // Reset
    }
  }

  //Store This Acceleration Value
  Alast.x = Awrld.x; Alast.y = Awrld.y; Alast.z = Awrld.z;

}

uint8_t MPU_9250_I2C::dmpGetLinearAccel(VectorFloat &v, VectorFloat &vRaw, VectorFloat &gravity) {
    // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet)
    v.x = vRaw.x - gravity.x;
    v.y = vRaw.y - gravity.y;
    v.z = vRaw.z - gravity.z;
    return 0;
}

uint8_t MPU_9250_I2C::dmpGetGravity(VectorFloat &g) {
    g.x  = 2 * (q.x*q.z - q.w*q.y);
    g.y  = 2 * (q.w*q.x + q.y*q.z);
    g.z  = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
    return 0;
}
