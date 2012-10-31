
void computeIMU () {
  uint8_t axis;
  static int16_t gyroADCprevious[3] = {0,0,0};
  int16_t gyroADCp[3];
  int16_t gyroADCinter[3];
  static uint32_t timeInterleave = 0;

  //we separate the 2 situations because reading gyro values with a gyro only setup can be acchieved at a higher rate
  //gyro+nunchuk: we must wait for a quite high delay betwwen 2 reads to get both WM+ and Nunchuk data. It works with 3ms
  //gyro only: the delay to read 2 consecutive values can be reduced to only 0.65ms
  #if defined(NUNCHUCK)
    annexCode();
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    ACC_getADC();
    getEstimatedAttitude(); // computation time must last less than one interleaving delay
    while((micros()-timeInterleave)<INTERLEAVING_DELAY) ; //interleaving delay between 2 consecutive reads
    timeInterleave=micros();
    f.NUNCHUKDATA = 1;
    while(f.NUNCHUKDATA) ACC_getADC(); // For this interleaving reading, we must have a gyro update at this point (less delay)

    for (axis = 0; axis < 3; axis++) {
      // empirical, we take a weighted value of the current and the previous values
      // /4 is to average 4 values, note: overflow is not possible for WMP gyro here
      gyroData[axis] = (gyroADC[axis]*3+gyroADCprevious[axis])/4;
      gyroADCprevious[axis] = gyroADC[axis];
    }
  #else
    #if ACC
      ACC_getADC();
      getEstimatedAttitude();
    #endif
    #if GYRO
      Gyro_getADC();
    #endif
    for (axis = 0; axis < 3; axis++)
      gyroADCp[axis] =  gyroADC[axis];
    timeInterleave=micros();
    annexCode();
    if ((micros()-timeInterleave)>650) {
       annex650_overrun_count++;
    } else {
       while((micros()-timeInterleave)<650) ; //empirical, interleaving delay between 2 consecutive reads
    }
    #if GYRO
      Gyro_getADC();
    #endif
    for (axis = 0; axis < 3; axis++) {
      gyroADCinter[axis] =  gyroADC[axis]+gyroADCp[axis];
      // empirical, we take a weighted value of the current and the previous values
      gyroData[axis] = (gyroADCinter[axis]+gyroADCprevious[axis])/3;
      gyroADCprevious[axis] = gyroADCinter[axis]/2;
      if (!ACC) accADC[axis]=0;
    }
  #endif
  #if defined(GYRO_SMOOTHING)
    static int16_t gyroSmooth[3] = {0,0,0};
    for (axis = 0; axis < 3; axis++) {
      gyroData[axis] = (int16_t) ( ( (int32_t)((int32_t)gyroSmooth[axis] * (conf.Smoothing[axis]-1) )+gyroData[axis]+1 ) / conf.Smoothing[axis]);
      gyroSmooth[axis] = gyroData[axis];
    }
  #elif defined(TRI)
    static int16_t gyroYawSmooth = 0;
    gyroData[YAW] = (gyroYawSmooth*2+gyroData[YAW])/3;
    gyroYawSmooth = gyroData[YAW];
  #endif
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 100
#endif

/* Set the Low Pass Filter factor for Magnetometer */
/* Increasing this value would reduce Magnetometer noise (not visible in GUI), but would increase Magnetometer lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef MG_LPF_FACTOR
  //#define MG_LPF_FACTOR 4
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter */
/* Increasing this value would reduce and delay Acc influence on the output of the filter*/#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 400.0f
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter */
/* Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#ifndef GYR_CMPFM_FACTOR
  #define GYR_CMPFM_FACTOR 200.0f
#endif

//****** end of advanced users settings *************

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#if GYRO
  #define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //should be 2279.44 but 2380 gives better result
  // +-2000/sec deg scale
  //#define GYRO_SCALE ((200.0f * PI)/((32768.0f / 5.0f / 4.0f ) * 180.0f * 1000000.0f) * 1.5f)     
  // +- 200/sec deg scale
  // 1.5 is emperical, not sure what it means
  // should be in rad/sec
#else
  #define GYRO_SCALE (1.0f/200e6f)
  // empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
  // !!!!should be adjusted to the rad/sec
#endif 
// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;

int16_t _atan2(float y, float x){
  #define fp_is_neg(val) ((((uint8_t*)&val)[3] & 0x80) != 0)
  float z = y / x;
  int16_t zi = abs(int16_t(z * 100)); 
  int8_t y_neg = fp_is_neg(y);
  if ( zi < 100 ){
    if (zi > 10) 
     z = z / (1.0f + 0.28f * z * z);
   if (fp_is_neg(x)) {
     if (y_neg) z -= PI;
     else z += PI;
   }
  } else {
   z = (PI / 2.0f) - z / (z * z + 0.28f);
   if (y_neg) z -= PI;
  }
  z *= (180.0f / PI * 10); 
  return z;
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X; 
}

static t_fp_vector EstG;
#if defined(AccBaroLPF)
  static float BaroAccLPF[3];
#endif
void getEstimatedAttitude(){
  uint8_t axis;
  int32_t accMag = 0;
#if MAG
  static t_fp_vector EstM;
#endif
#if defined(MG_LPF_FACTOR)
  static int16_t mgSmooth[3]; 
#endif
#if defined(ACC_LPF_FACTOR)
  static float accLPF[3];
#endif
  static uint32_t previousT;                                                         // CrashpilotMod increase to 32 Bit (10 Bytes more)
  uint32_t currentT = micros();
  float scale, deltaGyroAngle[3];

  ACCDeltaTime = currentT - previousT;                                               // CrashpilotMod
  scale = ACCDeltaTime * GYRO_SCALE;
  previousT = currentT;

  // Initialization
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = gyroADC[axis]  * scale;
    #if defined(ACC_LPF_FACTOR)
   
     #if defined(AccBaroLPF)                                                        // AccBaroLPF in percent 0-100% (between lpfmin and lpfmax)
       #define lpfmax 0.04f
       #define lpfmin 0.2f
       #define lpffactor (lpfmin - (((lpfmin-lpfmax)/100.0f)*AccBaroLPF))
       BaroAccLPF[axis] = (BaroAccLPF[axis]*(1.0f-lpffactor))+ (accADC[axis] * lpffactor);
     #endif
    
      accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
      accSmooth[axis] = accLPF[axis];
      #define ACC_VALUE accSmooth[axis]
    #else  
      accSmooth[axis] = accADC[axis];
      #define ACC_VALUE accADC[axis]
    #endif
//    accMag += (ACC_VALUE * 10 / (int16_t)acc_1G) * (ACC_VALUE * 10 / (int16_t)acc_1G);
    accMag += (int32_t)ACC_VALUE*ACC_VALUE ;
    #if MAG
      #if defined(MG_LPF_FACTOR)
        mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
        #define MAG_VALUE mgSmooth[axis]
      #else  
        #define MAG_VALUE magADC[axis]
      #endif
    #endif
  }
  accMag = accMag*100/((int32_t)acc_1G*acc_1G);

  rotateV(&EstG.V,deltaGyroAngle);
  #if MAG
    rotateV(&EstM.V,deltaGyroAngle);
  #endif 

  if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0) {
    f.SMALL_ANGLES_25 = 1;
  } else {
    f.SMALL_ANGLES_25 = 0;
  }

  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  if ( ( 36 < accMag && accMag < 196 ) || f.SMALL_ANGLES_25 )
    for (axis = 0; axis < 3; axis++) {
      int16_t acc = ACC_VALUE;
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + acc) * INV_GYR_CMPF_FACTOR;
    }
  #if MAG
    for (axis = 0; axis < 3; axis++)
      EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + MAG_VALUE) * INV_GYR_CMPFM_FACTOR;
  #endif
  
  // Attitude of the estimated vector
  angle[ROLL]  =  _atan2(EstG.V.X , EstG.V.Z) ;
  angle[PITCH] =  _atan2(EstG.V.Y , EstG.V.Z) ;

  #if MAG
    // Attitude of the cross product vector GxM
    heading = _atan2( EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X , EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z  );
    heading += MAG_DECLINIATION * 10; //add declination
    heading = heading /10;
    if ( heading > 180)      heading = heading - 360;
    else if (heading < -180) heading = heading + 360;
  #endif

}

void getGroundAlt()                                              // Executed in Setup takes approx 4.5 sec
{
    uint8_t gacnt=0;
    while(gacnt < 150){
     while(newbaroalt ==0) Baro_update();                        // Wait for new Baroval
     newbaroalt = 0;                                             // Reset Boolean
     GroundAlt=GroundAlt+BaroAlt;
     gacnt++;}
    GroundAlt=GroundAlt/150;
    accVelScale = 9.80665f/10000.0f/acc_1G;
}

#if BARO
///////////////////////////////////////////////
//Crashpilot1000 Mod getEstimatedAltitude ACC//
///////////////////////////////////////////////
#define VarioTabsize 8
#define BaroTabsize 5
void getEstimatedAltitude()
{
  static uint8_t Vidx = 0,Bidx = 0;
  static int32_t LastEstAltBaro = 0;
  static int8_t VarioTab[VarioTabsize];
  static int32_t BaroTab[BaroTabsize];
  static float velz = 0.0f, accalt = 0.0f;
  int8_t IdxCnt;
  float angle = EstG.V.Z/acc_1G;             //  angle = fsq(angle);
  uint8_t ThrAngle = angle * 100.0f;
  if (ThrAngle>100){ThrAngle = 100; angle = 1;}  
  #if defined(AccBaroLPF)
    int16_t accZ = ((BaroAccLPF[ROLL]*EstG.V.X+BaroAccLPF[PITCH]*EstG.V.Y+BaroAccLPF[YAW]*EstG.V.Z)*InvSqrt(fsq(EstG.V.X)+fsq(EstG.V.Y)+fsq(EstG.V.Z)) - acc_1G)*angle;
  #else
    int16_t accZ = ((accADC[ROLL]*EstG.V.X+accADC[PITCH]*EstG.V.Y+accADC[YAW]*EstG.V.Z)*InvSqrt(fsq(EstG.V.X)+fsq(EstG.V.Y)+fsq(EstG.V.Z)) - acc_1G)*angle;
  #endif
  #if defined(AccBaroDebug)
    debug[0]=accADC[YAW] - acc_1G;
    debug[1]=accZ;
  #endif

  if (abs(accZ)<10) accZ = 0;
  velz = velz+accZ*accVelScale*ACCDeltaTime;
  accalt = accalt+velz*ACCDeltaTime*0.000001f;
  if (newbaroalt!=0)
   {
    newbaroalt = 0;                                                               // Reset Boolean
    BaroTab[Bidx] = BaroAlt-GroundAlt; Bidx++;                                    // Get EstAltBaro
    if (Bidx==BaroTabsize) Bidx=0;
    int32_t tmp32=0; IdxCnt = 0;
    while(IdxCnt<BaroTabsize){tmp32 = tmp32 + BaroTab[IdxCnt]; IdxCnt++;}
    int32_t EstAltBaro = tmp32/BaroTabsize;
    VarioTab[Vidx] = constrain(EstAltBaro - LastEstAltBaro,-127,127); Vidx++;     // Baro Climbrate 
    if (Vidx==VarioTabsize) Vidx=0; 
    LastEstAltBaro = EstAltBaro;
    int16_t tmp16 = 0; IdxCnt = 0;
    while(IdxCnt<VarioTabsize){tmp16 = tmp16 + VarioTab[IdxCnt]; IdxCnt++;}
    float BaroClimbRate = (34*tmp16)/VarioTabsize;                                // BaroClimbRate in cm/sec // + is up // 30ms * 33 = 990ms
    velz = velz * 0.982f + BaroClimbRate * 0.018f;
    accalt = accalt * 0.85f + EstAltBaro * 0.15f;
   }
  EstAlt = accalt;
  BaroP = 0; BaroI = 0; BaroD = 0;
  if (ThrAngle < 40) return;                                                      // Don't do BaroPID if copter too tilted// EstAlt & Baroclimbrate are always done :)
  BaroP = ((AltHold-EstAlt)*conf.P8[PIDALT])/200;
  BaroI = (velz*conf.I8[PIDALT])/50;                                              //  BaroI = constrain(ClimbRate*conf.I8[PIDALT],-150,150);
  BaroD = (((int16_t)conf.D8[PIDALT]) * (100 - ThrAngle))/25;
}
float InvSqrt (float x){ 
  union{  
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1); 
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}
float fsq(float x){return x * x;}
//int32_t fsq(int32_t x){return x * x;}
#endif



