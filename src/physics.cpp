#include <physics.h>
#include "globals.h"

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!

KalmanModel::KalmanModel()
{
  //X - State Vector - X,V,A,Rpos,Rssi
  //U - Input Vector - ACL World + Rssi
  //W - Process Noise - ACL World + Rpos,Rssi
  //Z - Measurement Vector - ACL World + R
  //V - Measurement Noise - ACL World + Rpos,Rssi
  //Set Sizes: X, U, W, Z, V
    setDim(NN, UU, WW, ZZ, VV);
}

void KalmanModel::makeBaseA()
{ //(nxn)
  //Take Care Of Idenity
  //1-3: Position
  //4-6: Velocity
  //7-9: Acceleration
  frisbeem.com.log("Making Base A");
  for (unsigned long int i = 1; i <= NN; i++){
    for (unsigned long int j = 1; j <= NN; j++){
      //String elm = "|"+String(i)+","+String(j);
      if (i == j) {   A(i,j) = 1.0;} // On That Identiy
      else        {   A(i,j) = 0.0; }
    }
  }
}



void KalmanModel::makeA()
{ float dt = frisbeem.physics.deltat;
  float dt2 = dt * dt / 2.0;
  for (unsigned long int i = 1; i <= n; i++){
    if (i <= 3) { A(i,i+6) = dt2; } //its dt
    if (i <= 6) { A(i,i+3) = dt; } //its dt
  }
}

void KalmanModel::makeBaseH()
{ //(nxn)
  //Take Care Of Idenity
  //1-3: Position
  //4-6: Velocity
  //7-9: Acceleration
  //10-11: Position Scalar
  //12-13: Velocity Scalar (WIP)
  for (unsigned long int i = 1; i <= m; i++){
    for (unsigned long int j = 1; j <= n; j++){
        H(i,j) = 0.0;
    }
  }
  H(1,7) = 1.0; //X ACL
  H(2,8) = 1.0; //Y ACL
  H(3,9) = 1.0; //Z ACL
}

void KalmanModel::makeH(){
  //Postion Estimateion... Store For makeMeasure()
  Rpos = sqrt(x(1)*x(1) + x(2)*x(2) + x(3)*x(3)); //60us
  float r_factor = 1.0/Rpos; //16us
  dxdR = x(1) * r_factor;//dR/dx //4us
  dydR = x(2) * r_factor;//dR/dy //4us
  dzdR = x(3) * r_factor;//dR/dz //4us
  //Store It YO'
  H(4,1) = dxdR;
  H(4,2) = dydR;
  H(4,3) = dzdR;
}

void KalmanModel::makeBaseQ(){
  //Prediction Noise (nwxnw)
  for (unsigned long int i = 1; i <= nw; i++){
    for (unsigned long int j = 1; j <= nw; j++){
      if (i<=3 && j<=3){
        if (i==j) {Q(i,j)=w_pos*w_pos;}
        else      {Q(i,j) = 0.0;}//(w_pos*w_pos) / 10.0;}
      }
      else if(i>3 && j>3){
        if   (i == j) { Q(i,j)=w_vel*w_vel;}
        else          { Q(i,j)= 0.0;}//w_vel*w_vel/10.0;}
      }
      else{
        Q(i,j) = 0.0;
      }
    }
  }
}

void KalmanModel::makeBaseV(){
  //Measurement Noise (mxnv)
  for (unsigned long int i = 1; i <= m; i++){
    for (unsigned long int j = 1; j <= nv; j++){
      if (i==j) { V(i,j) = 1.0;}
      else {      V(i,j) = 0.0;}
    }
  }
}

void KalmanModel::makeBaseR(){
  //Measurement Noise (nwxnv?)
  for (unsigned long int i = 1; i <= nv; i++){
    for (unsigned long int j = 1; j <= nv; j++){
      if (i<=3 && j<=3){
        if (i==j){R(i,j)=v_acl*v_acl;}
        else{     R(i,j) = 0.0; }
      }
      else if(i>3 && j>3){
        if (i==4 && j==4){ R(i,j)=v_pos*v_pos;}
        else {             R(i,j)= 0.0; }
      }
      else{
        R(i,j) = 0.0;
      }
    }
  }
}

void KalmanModel::makeBaseW(){
  //Process Noise Bool (nxnv)
  //nv -> Ax,Ay,Az,R
  for (unsigned long int i = 1; i < n; i++){
    for (unsigned long int j = 1; j < nv; j++){
      if (i==j) { W(i,j) = 1.0;}
      else {      W(i,j) = 0.0;}
    }
  }
}

void KalmanModel::makeProcess(){
  // This Takes Place Of The Input Matrix.. can be nonlinear
  Vector x_(x.size());
  //Position
  float dt = frisbeem.physics.deltat;
  float dt2 = dt * dt / 2.0;
  x_(1) = x(1)+x(4)*frisbeem.physics.deltat;
  x_(2) = x(2)+x(5)*frisbeem.physics.deltat;
  x_(3) = x(3)+x(6)*frisbeem.physics.deltat;
  //Velocity
  x_(4) = x(4)+u(1)*frisbeem.physics.deltat;
  x_(5) = x(5)+u(2)*frisbeem.physics.deltat;
  x_(6) = x(6)+u(3)*frisbeem.physics.deltat;
  //Acceleration - Input
  x_(7) = u(1);//x(7);
  x_(8) = u(2);//x(8);
  x_(9) = u(3);//x(9);
  //Update
  x.swap(x_);
}

void KalmanModel::makeMeasure(){
  //Straight Up Position Yo
  z(1) = frisbeem.physics.Awrld.x;
  z(2) = frisbeem.physics.Awrld.y;
  z(3) = frisbeem.physics.Awrld.z;

  //Position
  z(4) = Rpos;
}

void KalmanModel::initialize(float ax, float ay, float az, float D){
  //Make Dem Matrix
  P0 = Matrix(11,11);
  P0(1,1) = 10.0*10.0;
  P0(2,2) = 10.0*10.0;
  P0(3,3) = 10.0*10.0;
  P0(4,4) = 10.0;
  P0(5,5) = 10.0;
  P0(6,6) = 10.0;
  P0(7,7) = 1.0;
  P0(8,8) = 1.0;
  P0(9,9) = 1.0;


  //Initial Guess Vector
  x0 = Vector(11);
  x0(1) = 0.1;
  x0(2) = 0.1;
  x0(3) = 0.1;
  x0(4) = 0.0;
  x0(5) = 0.0;
  x0(6) = 0.0;
  x0(7) = 0.0;
  x0(8) = 0.0;
  x0(9) = 0.0;

  //Control Input initialization

  uin = Vector(4);
  uin(1) = ax;
  uin(2) = ay;
  uin(3) = az;
  uin(4) = D;

  //Measurement Vector
  zin = Vector(4);
  zin(1) = ax;
  zin(2) = ay;
  zin(3) = az;
  zin(4) = D;

  //Initalize Kalman Model
  init(x0,P0);

  frisbeem.com.log("Go For Initial Kalman");
  updatePhysicsModel();
  timeUpdateStep(uin);
  measureUpdateStep(zin);

  frisbeem.com.log("First Kalman Update");
  updatePhysicsModel();
}
void KalmanModel::updatePhysicsModel()
{
  //Parse State
  frisbeem.physics.X.x = x(1);
  frisbeem.physics.X.y = x(2);
  frisbeem.physics.X.z = x(3);

  frisbeem.physics.V.x = x(4);
  frisbeem.physics.V.y = x(5);
  frisbeem.physics.V.z = x(6);

  frisbeem.physics.A.x = x(7);
  frisbeem.physics.A.y = x(8);
  frisbeem.physics.A.z = x(9);

  frisbeem.com.str_send_telemetry();
  //frisbeem.com.log("Rpos: "+String(Rpos),true);
}

void KalmanModel::update(float ax, float ay, float az, float D){

  zin(1) = ax;
  zin(2) = ay;
  zin(3) = az;
  //RSSI Input
  zin(4) = D;

  uin(1) = ax;
  uin(2) = ay;
  uin(3) = az;
  //RSSI Input
  uin(4) = D;

  step(uin,zin);
  updatePhysicsModel();
}

void Physics::initialize()
{
  processNewData();
  frisbeem.com.log("Initalizing Kalman Model");
  kalman_model.initialize(Awrld.x,Awrld.y,Awrld.z,C1_Distance);
}

void Physics::update(){
  enqueue_motion();
  //Meter Algorithm Time --This Shit Gonna Get Crazy
  //long strt = micros();
  //frisbeem.com.log("ITM:\t"+String(strt-now));
  processNewData();

  //OverSpeed Correction WIP

  //Kalman Filter - Inertia + BLE Distance
  kalman_model.update(Awrld.x,Awrld.y,Awrld.z,C1_Distance);

  //Do Other Motion Processing
  calculatePositionalInformation();

  //Time Calc
  lastUpdate = now;
  now = micros();
  deltat = (float)((now - lastUpdate)/1E6); //dMicros To dT
  //frisbeem.com.log("CTM:\t"+String(now-strt));
}

void Physics::enqueue_motion()
{
  //frisbeem.com.log("Angular Rate: "+String(G.z));
  MotionEvent * motionEvent = new MotionEvent( frisbeem.mpu.G.z );
  //std::unique_ptr<MotionEvent> motionEvent( new MotionEvent(G.z));
  frisbeem.event_queue.addEvent( motionEvent );
}

/////////////////////////// Motion Processing //////////////////////////////////
//Positional Information Calculations
void Physics::processNewData(){
  frisbeem.com.log("Madgwick");
  MadgwickQuaternionUpdate(frisbeem.mpu.A.x,frisbeem.mpu.A.y,frisbeem.mpu.A.z,
                           frisbeem.mpu.G.x*PI/180.0f,frisbeem.mpu.G.y*PI/180.0f,frisbeem.mpu.G.z*PI/180.0f,
                           frisbeem.mpu.M.y,frisbeem.mpu.M.x,frisbeem.mpu.M.z);

  // frisbeem.com.log("Mahony");
  // MahonyQuaternionUpdate(A.x,A.y,A.z,G.x*PI/180.0f,G.y*PI/180.0f,G.z*PI/180.0f,M.y,M.x,M.z);

  dmpGetGravity( Grav );
  dmpGetLinearAccel(Alin, frisbeem.mpu.A, Grav);
  Awrld = Alin.getRotated( &q );
}
void Physics::calculatePositionalInformation(){
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!
  // Pass gyro rate as rad/s
  determineIsSpinning();
  determineIsRest();
  //determineVelocityNPosition(Awrld,V,X);
}

void Physics::determineIsSpinning()
{
  oldSpin = spin;
  if ( fabs(frisbeem.mpu.G.z) > spinThreshold ){ spin = true;}
  else{ spin = false;}
}
//Determine If At rest
void Physics::determineIsRest()
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
void Physics::determineVelocityNPosition(VectorFloat &Awrld, VectorFloat &Vel, VectorFloat &Pos)
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

uint8_t Physics::dmpGetLinearAccel(VectorFloat &v, VectorFloat &vRaw, VectorFloat &gravity) {
  // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet)
  //Do Inversoin Check with vector projection
  float linAccGravProj = (vRaw.x*gravity.x + vRaw.y*gravity.y + vRaw.z*gravity.z) * gravInversion;
  //Remove Gravity
  if (linAccGravProj > 0){
    v.x = vRaw.x - gravity.x;
    v.y = vRaw.y - gravity.y;
    v.z = vRaw.z - gravity.z;
  }
  else{
    v.x = vRaw.x + gravity.x;
    v.y = vRaw.y + gravity.y;
    v.z = vRaw.z + gravity.z;
  }
  return 0;
}

uint8_t Physics::dmpGetGravity(VectorFloat &g) {
  g.x  = 2 * (q.x*q.z - q.w*q.y);
  g.y  = 2 * (q.w*q.x + q.y*q.z);
  g.z  = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
  return 0;
}


void Physics::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
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
void Physics::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
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
