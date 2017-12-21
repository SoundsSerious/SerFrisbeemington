#ifndef PHYSICS_h
#define PHYSICS_h


#include <Arduino.h>
#include <3dmath.h>
#include <event.h>

#include "sdkconfig.h"

#include "kalman/kvector.h"
#include "kalman/kmatrix.h"
#include "kalman/ekfilter.h"
//#include "kalman/ktypes.h"

typedef Kalman::KVector<float, 1, false> Vector;  //!< Vector type.
typedef Kalman::KMatrix<float, 1, false> Matrix;  //!< Matrix type.

#define GRAVITY ((float)9.80665)
// these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Kp 2.0f * 5.0f
#define Ki 0.0f

#define MASS (float) 0.186 //kg
#define INERTIA (float) 1.17E-3 //kg*m^2

#define NN 9
#define UU 4
#define ZZ 4
#define WW 6
#define VV 4

class KalmanModel : public Kalman::EKFilter<float,1,true,true> {
  //Start at 1 with 32bit precision
  public:
        KalmanModel();
        ~KalmanModel(){};

        void makeBaseA();
        void makeA();
        void makeH();
        void makeBaseH();
        void makeBaseV();
        void makeBaseR();
        void makeBaseW();
        void makeBaseQ();
        void makeProcess();
        void makeMeasure();

        //Custom
        void initialize(float ax, float ay, float az, float D);
        void update(float ax, float ay, float az, float D);

        void updatePhysicsModel();

        void set_dt(float dt);
        float dt = 1.0 / 2000.0; //2000 Updates Per Second

        Matrix P0;
        Vector x0;
        Vector zin;
        Vector uin;

        //Distance Information
        float Rpos = 10.0;
        float dxdR = 1.0;
        float dydR = 1.0;
        float dzdR = 1.0;

        //Sensor Noise
        float v_vel= 2.0;
        float v_pos= 1.0;
        float v_acl= 0.1;
        //Process Noise
        float w_pos = 1.0;
        float w_vel = 0.5;
};

//Where we solve math related to position and orientation of the disk
class Physics
{
  public:

  //Intermediate Vectors For High Level Positional FAlgorithm
  VectorFloat Grav, Alin, Awrld, Alast, A, V, X;
  Quaternion q;
  float linAccGravProj;
  float gravInversion = 1.0/GRAVITY;
  //Store Client Connectivity Strength Informatoin for Localaization
  float C1_Distance = 10.0;

  float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

  KalmanModel kalman_model;

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

  uint8_t orientationPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

  //uint32_t delt_t = 0; // used to control display output rate
  uint32_t count = 0, sumCount = 0; // used to control display output rate
  float pitch, yaw, roll;
  float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
  uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
  uint32_t now = 0;        // used to calculate integration interval

  //Low Pass Filter
  float Kaxy_lowpass = 0.005;
  float Axy, Axy_lp;
  float Axy_MagThresh = 0.25;
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

  void initialize();
  void update();
  void processNewData();

  void enqueue_motion();

  //Motion Intellegence
  //uint8_t dmpGetLinearAccel(float *v, float *vRaw, float *gravity);
  uint8_t dmpGetLinearAccel(VectorFloat &v, VectorFloat &vRaw, VectorFloat &gravity);
  //uint8_t dmpGetGravity(float *g);
  uint8_t dmpGetGravity(VectorFloat &g);
  void calculatePositionalInformation();
  void determineIsSpinning();
  void determineIsRest();
  void determineVelocityNPosition(VectorFloat &Alin, VectorFloat &Vel, VectorFloat &Pos);


  void MadgwickQuaternionUpdate(float ax, float ay, float az,
                                float gx, float gy, float gz,
                                float mx, float my, float mz);
   // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
   // measured ones.
  void MahonyQuaternionUpdate(float ax, float ay, float az,
                              float gx, float gy, float gz,
                              float mx, float my, float mz);
};

#endif //PHYSICS_h
