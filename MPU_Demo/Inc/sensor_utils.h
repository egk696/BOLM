#ifndef SENSOR_UTILS_H  //SENSOR_UTILS_H
#define SENSOR_UTILS_H

#include <stdint.h>
#include <math.h>
#define ACCELEROMETER_TO_G 0.000061f //range of +-2g over 4096 counts/g
#define GYROSCOPE_TO_DPS 0.01751f //for a range of +-250dps with 62.5 dps/LSB
#define COMPASS_TO_TESLA 0.00058f //for a range of +-1000 uT with 0.10 uT/LSB
#define GRAVITY_ACC 0.980665f
#define RAD_TO_DEG 57.295779f
#define DEG_TO_RAD 0.017453292f
#define PI 3.14159265f
#define SEC_TO_MS 0.001f
#define SEC_TO_US 0.000001f
#define Kroll 0.92f
#define Kpitch 0.92f
#define Kyaw_am 1.0f
#define Kyaw_gam 0.8f
#define Kgravity 0.97f
#define Theta pitch
#define Phi roll
#define Psi yaw
#define threadshold 0.2f
#define STATUS_HISTORY_SIZE 10
#define NEXT_STATUS(i) (((i) < STATUS_HISTORY_SIZE-1)) ? ((i) + 1) : 0
#define PREV_STATUS(i) ((i) > 0) ? ((i) - 1) : (STATUS_HISTORY_SIZE-1)

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
     
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

const float ypr_hard_offset[3] = {0.0f, 1.0f, -4.0f};

typedef struct{
    float yaw, pitch, roll;
} AngleVector;

typedef struct{
    float x,y,z;
} SensorVector, Vector3D;

typedef struct{
    float a11, a12, a13;
    float a21, a22, a23;
    float a31, a32, a33;
} Matrix3x3;

typedef struct{
    SensorVector accel;
    SensorVector gyro;
    SensorVector mag;
} SensorData;

typedef struct {
    Vector3D pos;
    Vector3D vel;
    Vector3D acc;
    AngleVector angles;
    Vector3D ang_vel;
    Vector3D ang_acc;
    uint32_t ticks;
} StatusData;



//converts the raw accel to g based on the ACCELEROMETER_SENSITIVITY
SensorVector convertRawAccelToG(SensorVector accel);
//converts the raw accel to m/s based on the ACCELEROMETER_SENSITIVITY
SensorVector convertRawAccelToMs(SensorVector accel);
//converts the raw gyro to "deg. per sec." based on the GYROSCOPE_SENSITIVITY
SensorVector convertRawGyroToDps(SensorVector gyro);
//convert the raw magnotometer to gauss based on the COMPASS_SENSITIVITY
SensorVector convertRawMagToGauss(SensorVector mag);
//calculates the YPR, accepts converted data, note: the accelerometer can be raw although the gyro should be in degrees
void calculateYPR(AngleVector *angles, SensorData new_data, SensorVector hardIronV, float dt);
//implements magnetometer derotation based on the NXP Implemting a Tilt-Compensated eCompass
void deRotateMagnetometer(Vector3D *de_rot_mag, Vector3D orig_mag, AngleVector angles, SensorVector hardIron);
//convert raw acceleration to linear without gravity
void convertGravityToLinearAcceleration(SensorVector accel_in, AngleVector angles, SensorVector *gravity, Vector3D *linearAccel);
//implements inner product of two vectors
float innerProdcut(Vector3D v1, Vector3D v2);
//calculates velocity and postion using double integration
void calculatesVelocityPosition(SensorVector acc_new_data, SensorVector * vel, SensorVector * pos, float dt);
//implements matrix multiplication NxN with 1xN
void multiplyVectorTo3x3(Vector3D *mult_v, Vector3D v1, Matrix3x3 m);
//implements a rotation vector by angle in the x,y,z axis order
void rotateVectorByAngleXYZ(Vector3D *rot_v, AngleVector old_data);
//implements a rotation vector by angle in the z,y,x axis order
void rotateVectorByAngleZYX(Vector3D *rot_v, AngleVector old_data);
//implements a cross product on two vectors
void crossProduct(Vector3D *v, Vector3D v1, Vector3D v2);
//implements inner product of two vectors
float innerProduct(Vector3D v1, Vector3D v2);

void centerOfMassAcceleration(  Vector3D *accel_cm, 
                Vector3D accel_no_g, 
                Vector3D ang_vel, 
                Vector3D ang_accel, 
                Vector3D node_coords);

void calculateAngularAcceleration(  
            Vector3D *angularAcceleration, 
            Vector3D oldAngularVelocity, 
            Vector3D newAngularVelocity, 
            float dt);
void calculateAngularVelocity(Vector3D *angularVelocity, SensorVector gyroData, AngleVector angles) ;

void rotateVector(Vector3D *rot_vec, Vector3D vec, AngleVector angles);

void accelerationGRemoveGravity(SensorVector *new_acc, SensorVector acc, AngleVector angles);

static SensorVector acc_old_data = {.x = 0, .y = 0, .z = 0};
static SensorVector vel_old_data = {.x = 0, .y = 0, .z = 0};
static SensorVector pos_old_data = {.x = 0, .y = 0, .z = 0};
static SensorVector hardIronV = {0};

/*Keep history 10 statuses - Needed for the noisy angular acceleration*/
static StatusData status[STATUS_HISTORY_SIZE] = {0}; 

#endif //SENSOR_UTILS_H