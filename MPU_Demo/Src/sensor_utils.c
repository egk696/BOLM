#include "sensor_utils.h"

int status_offset = 0;

//converts the raw accel to g based on the ACCELEROMETER_TO_G
SensorVector convertRawAccelToG(SensorVector accel){
    SensorVector vector = accel;
    vector.x = accel.x * ACCELEROMETER_TO_G;
    vector.y = accel.y * ACCELEROMETER_TO_G;
    vector.z = accel.z * ACCELEROMETER_TO_G;
    return vector;
}
//converts the raw accel to g based on the ACCELEROMETER_SENSITIVITY
SensorVector convertRawAccelToMs(SensorVector accel){
    SensorVector vector = {0};
    vector.x = accel.x * GRAVITY_ACC * ACCELEROMETER_TO_G;
    vector.y = accel.y * GRAVITY_ACC * ACCELEROMETER_TO_G;
    vector.z = accel.z * GRAVITY_ACC * ACCELEROMETER_TO_G;
    return vector;
}

//converts the raw accel to "deg. per sec." based on the GYROSCOPE_TO_DPS
SensorVector convertRawGyroToDps(SensorVector gyro){
    SensorVector vector = gyro;
    vector.x = gyro.x * GYROSCOPE_TO_DPS;
    vector.y = gyro.y * GYROSCOPE_TO_DPS;
    vector.z = gyro.z * GYROSCOPE_TO_DPS;
    return vector;
}

//convert the raw magnotometer to gauss based on the COMPASS_TO_TESLA
SensorVector convertRawMagToGauss(SensorVector mag){
    SensorVector vector = mag;
    vector.x = mag.x * COMPASS_TO_TESLA;
    vector.y = mag.y * COMPASS_TO_TESLA;
    vector.z = mag.z * COMPASS_TO_TESLA;
    return vector;
}


/*Implements magnetometer derotation based on the NXP Implemting a Tilt-Compensated eCompass*/ 
void deRotateMagnetometer(Vector3D *de_rot_mag, Vector3D orig_mag, AngleVector angles, SensorVector hardIron){
     /***********************************************************   
        (Bpx – Vx) cosθ + (Bpy – Vy) sinθ sinφ + (Bpz – Vz) sinθ cosφ
Eq.19   (Bpy – Vy) cosφ – (Bpz – Vz) sinφ
        –(Bpx – Vx) sinθ + (Bpy – Vy) cosθ sinφ + (Bpz – Vz) cosθ cosφ
    ************************************************************/
    /*de-rotate x magnetometer*/
    de_rot_mag->x = (orig_mag.x-hardIron.x)*cos(angles.Theta) 
                + (orig_mag.y-hardIron.y)*sin(angles.Theta)*sin(angles.Phi) 
                + (orig_mag.z-hardIron.z)*sin(angles.Theta)*cos(angles.Phi);
    /*de-rotate y magnetometer*/
    de_rot_mag->y = (orig_mag.y-hardIron.y)*cos(angles.Phi) - (orig_mag.z-hardIron.z)*sin(angles.Phi);
    /*de-rotate z magnetometer*/
    de_rot_mag->z = -(orig_mag.x-hardIron.x)*sin(angles.Theta) 
                + (orig_mag.y-hardIron.y)*cos(angles.Theta)*sin(angles.Phi) 
                + (orig_mag.z-hardIron.z)*cos(angles.Theta)*cos(angles.Phi);
}

//calculates the YPR, accepts converted data, note: the accelerometer can be raw although the gyro should be in degrees
void calculateYPR(AngleVector *angles, SensorData new_data, SensorVector hardIronV, float dt){
    SensorVector de_rot_mag = new_data.mag;
    float pitchAccel=0.0f, rollAccel=0.0f;
    
    /*calculate current roll-accel angle Phi*/
    rollAccel = atan(new_data.accel.y / new_data.accel.z);
    /*calculate current pitch-accel angle Theta*/
    pitchAccel = atan(-new_data.accel.x / new_data.accel.z );//atan2f(-new_data.accel.x, new_data.accel.y * sin(rollAccel) + new_data.accel.z * cos(rollAccel));
    /*de-rotate magnetometer*/
    deRotateMagnetometer(&de_rot_mag, new_data.mag, (AngleVector){.yaw=0.0f, .pitch=pitchAccel, .roll=rollAccel}, hardIronV);

    /*fuse high-passed roll-gyro with roll-accel*/
    angles->roll = (angles->roll + new_data.gyro.y  * dt * DEG_TO_RAD) * Kroll + rollAccel * (1.0f - Kroll);
    //angles->roll = min(max(angles->roll, -PI/2.0f), PI/2.0f);
    /*fuse high-passed pitch-gyro with pitch-accel*/
    angles->pitch = (angles->pitch + new_data.gyro.x * dt * DEG_TO_RAD) * Kpitch + pitchAccel * (1.0f - Kpitch);
    //angles->pitch = min(max(angles->pitch, -PI/2.0f), PI/2.0f);
    /*tilt compensated compass*/
    angles->yaw = atan2(-de_rot_mag.y, de_rot_mag.x);
    //angles->yaw = min(max(angles->yaw, -PI), PI);
}

//convert raw acceleration to linear without gravity by LPF
void convertGravityToLinearAcceleration(SensorVector accel_in, AngleVector angles, SensorVector *gravity, Vector3D *linearAccel){
    gravity->x = gravity->x * Kgravity + accel_in.x * (1.0f - Kgravity);
    gravity->y = gravity->y * Kgravity + accel_in.y * (1.0f - Kgravity);
    gravity->z = gravity->z * Kgravity + accel_in.z * (1.0f - Kgravity);
    
    //gravity->x = sin(angles.pitch);
    //gravity->y = sin(angles.roll);
    //gravity->z = cos(angles.pitch)*cos(angles.roll);
    
    //gravity->x = cos(angles.Theta) + sin(angles.Theta)*sin(angles.Phi) + sin(angles.Theta)*cos(angles.Phi);
    //gravity->y = cos(angles.Phi) - sin(angles.Phi);
    //gravity->z = -sin(angles.Theta) + cos(angles.Theta)*sin(angles.Phi) + cos(angles.Theta)*cos(angles.Phi);
    
    linearAccel->x = accel_in.x - min(max(gravity->x, -1.0f), 1.0f);
    linearAccel->y = accel_in.y - min(max(gravity->y, -1.0f), 1.0f);
    linearAccel->z = accel_in.z - min(max(gravity->z, -1.0f), 1.0f);
}

//implements inner product of two vectors
float innerProdcut(Vector3D v1, Vector3D v2) {
   return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}

//implements matrix multiplication NxN with 1xN
void multiplyVectorTo3x3(Vector3D *mult_v, Vector3D v1, Matrix3x3 m){
    mult_v->x = innerProdcut((Vector3D){.x=m.a11, .y=m.a12, .z=m.a13}, v1);
    mult_v->y = innerProdcut((Vector3D){.x=m.a21, .y=m.a22, .z=m.a23}, v1);
    mult_v->z = innerProdcut((Vector3D){.x=m.a31, .y=m.a32, .z=m.a33}, v1);
}

//calculates velocity and postion using double integration
void calculatesVelocityPosition(SensorVector acc_new_data, SensorVector * vel, SensorVector * pos, float dt)
{

    float xArea = 0, yArea = 0, zArea = 0;
    if(acc_new_data.x < threadshold && acc_new_data.x > - threadshold)
        acc_new_data.x = 0;

    if(acc_new_data.y < threadshold && acc_new_data.y > - threadshold)
        acc_new_data.y = 0;

    if(acc_new_data.y < threadshold && acc_new_data.y > - threadshold)
        acc_new_data.y = 0;


    xArea = (acc_new_data.x - acc_old_data.x) / 2;
    xArea += acc_old_data.x;
    xArea *= dt;
    
    vel->x = xArea + vel_old_data.x;

    yArea = (acc_new_data.y - acc_old_data.y) / 2;
    yArea += acc_old_data.y;
    yArea *= dt;

    vel->y = yArea + vel_old_data.y;

    zArea = (acc_new_data.z - acc_old_data.z) / 2;
    zArea += acc_new_data.z;
    zArea *= dt;

    vel->z = zArea + vel_old_data.z;

    acc_old_data = acc_new_data;
////////////////////////
    xArea = (vel->x - vel_old_data.x) / 2;
    xArea += vel->x;
    xArea *= dt;
    pos->x = xArea + pos_old_data.x;

    yArea = (vel->y - vel_old_data.y) / 2;
    yArea += vel_old_data.y;
    yArea *= dt;
    pos->y = yArea + pos_old_data.y;

    zArea = (vel->z - vel_old_data.z) / 2;
    zArea += vel_old_data.z;
    zArea *= dt;
    pos->z = zArea + pos_old_data.z;

    vel_old_data = *vel;
    pos_old_data = *pos;
}

//implements a rotation vector by angle in the x,y,z axis order
void rotateVectorByAngleXYZ(Vector3D *rot_v, AngleVector old_data){
}

//implements a rotation vector by angle in the z,y,x axis order
void rotateVectorByAngleZYX(Vector3D *rot_v, AngleVector old_data){
}

float innerProduct(Vector3D v1, Vector3D v2) {
    return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}

StatusData *get_status() {
    return &status[status_offset];
}

StatusData *next_status() {
    status_offset = NEXT_STATUS(status_offset);
    return &status[status_offset];
}

/************************************************************************
Calculating the acceleration of the center of mass
Inputs: 
    accel_no_g  -> Acceleration by the accelerometer at Node i, rotated
    ang_vel -> Angular Velocity ω
    ang_accel   -> Angular Velocity ω dot
    node_coords -> Node distance from the CM
Output:
    accel_cm    -> Acceleration of the Center of Mass
Calculation:
    a_cm = a_node - (ω x (ω x R_cm_n)) - (ang_accel x R_cm_n)
************************************************************************/
void centerOfMassAcceleration(Vector3D *accel_cm, 
                Vector3D accel_no_g, 
                Vector3D ang_vel, 
                Vector3D ang_accel, 
//              Vector3D position,
                Vector3D node_coords) {

    Vector3D interm, accel_cor;
    crossProduct(&interm, ang_vel, node_coords);
    crossProduct(&interm, ang_vel, interm);
    crossProduct(&accel_cor, ang_accel, node_coords);
    accel_cm->x = accel_no_g.x - interm.x - accel_cor.x; 
    accel_cm->y = accel_no_g.y - interm.y - accel_cor.y; 
    accel_cm->z = accel_no_g.z - interm.z - accel_cor.z; 
}

void crossProduct(Vector3D *v, Vector3D v1, Vector3D v2) {
//     Vector3D cross;
    v->x = (v1.y * v2.z) - (v1.z * v2.y);
    v->y = (v1.z * v2.x) - (v1.x * v2.z);
    v->z = (v1.x * v2.y) - (v1.y * v2.x);
//     return cross;
}

/**********************


**********************/
void calculateAngularVelocity(Vector3D *angularVelocity, SensorVector gyroData, AngleVector angles) 
{
    rotateVector(angularVelocity, gyroData, angles);
}

/*****************************************************************************
Angular Acceleration:
    Calculate Using: Δω/Δt = (ω_current - ω_prev)/Dt
Can be Compensated Using the Instantanious point of rotation:
    a = Δω/Δt x R + ωx(ωxR)
Angular Velocity has to be rotated to the reference frame
*****************************************************************************/
void calculateAngularAcceleration(  
            Vector3D *angularAcceleration, 
            Vector3D oldAngularVelocity, 
            Vector3D newAngularVelocity, 
            float dt) 
{

    angularAcceleration->x = (newAngularVelocity.x - oldAngularVelocity.x)/dt;      
    angularAcceleration->y = (newAngularVelocity.y - oldAngularVelocity.y)/dt;      
    angularAcceleration->z = (newAngularVelocity.z - oldAngularVelocity.z)/dt;      

}

void array3MultiplyVector(Vector3D *result, Vector3D *array, Vector3D vec) {

        result->x = innerProduct(array[0], vec);    
        result->y = innerProduct(array[1], vec);    
        result->z = innerProduct(array[2], vec);    

}

void rotateVector(Vector3D *rot_vec, Vector3D vec, AngleVector angles) {
/*
φ: roll
θ: pitch
ψ: yaw
⎡ cosθcosψ                  cosθsinψ                    -sinθ    ⎤
⎢ sinφsinθcosψ−cosφsinψ     cosφcosψ+sinφsinθsinψ   sinφcosθ ⎥
⎣ sinφsinψ+cosφsinθcosψ     cosφsinθsinψ−sinφcosψ       cosφcos θ⎦
*/
    float phi, theta, psi;
    phi =  angles.roll;
    theta = angles.pitch;
    psi = angles.yaw;
    
    Vector3D rotMatrix[3] = { 
                {.x=cos(theta)*cos(psi), .y=cos(theta)*sin(psi), .z=-sin(theta)},

                  { .x=sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), 
                    .y=cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi), 
                    .z=sin(phi)*cos(theta)},

                  { .x=sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi), 
                    .y=cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi), 
                    .z=cos(phi)*cos(theta)}
                };

    
    array3MultiplyVector(rot_vec, rotMatrix, vec);
}

void accelerationGRemoveGravity(SensorVector *new_acc, SensorVector acc, AngleVector angles) {

    rotateVector(new_acc, acc, angles);
    new_acc->z -= 1.0f;   

//  new_acc->x = acc.x + sin(theta);    
//  new_acc->y = acc.y - sin(phi) * cos(theta); 
//  new_acc->z = acc.z - cos(theta) * cos(phi); 
}