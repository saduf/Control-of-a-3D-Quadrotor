#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"
#include <math.h>

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
    BaseController::Init();
    
    // variables needed for integral control
    integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
    // Load params from simulator parameter system
    ParamsHandle config = SimpleConfig::GetInstance();
    
    // Load parameters (default to 0)
    kpPosXY = config->Get(_config+".kpPosXY", 0);
    kpPosZ = config->Get(_config + ".kpPosZ", 0);
    KiPosZ = config->Get(_config + ".KiPosZ", 0);
    
    kpVelXY = config->Get(_config + ".kpVelXY", 0);
    kpVelZ = config->Get(_config + ".kpVelZ", 0);
    
    kpBank = config->Get(_config + ".kpBank", 0);
    kpYaw = config->Get(_config + ".kpYaw", 0);
    
    kpPQR = config->Get(_config + ".kpPQR", V3F());
    
    maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
    maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
    maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
    maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);
    
    maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);
    
    minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
    maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
    // load params from PX4 parameter system
    //TODO
    param_get(param_find("MC_PITCH_P"), &Kp_bank);
    param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
    // Convert a desired 3-axis moment and collective thrust command to
    //   individual motor thrust commands
    // INPUTS:
    //   desCollectiveThrust: desired collective thrust [N]
    //   desMoment: desired rotation moment about each axis [N m]
    // OUTPUT:
    //   set class member variable cmd (class variable for graphing) where
    //   cmd.desiredThrustsN[0..3]: motor commands, in [N]
    
    // HINTS:
    // - you can access parts of desMoment via e.g. desMoment.x
    // You'll need the arm length parameter L, and the drag/thrust ratio kappa
    
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    
   
    cmd.desiredThrustsN[0] = 0.25f * (collThrustCmd + sqrt(2)/L * (momentCmd.x + momentCmd.y) - momentCmd.z/kappa); // front left
    cmd.desiredThrustsN[1] = 0.25f * (collThrustCmd - sqrt(2)/L * (momentCmd.x - momentCmd.y) + momentCmd.z/kappa); // front right
    cmd.desiredThrustsN[2] = 0.25f * (collThrustCmd + sqrt(2)/L * (momentCmd.x - momentCmd.y) + momentCmd.z/kappa); // rear left
    cmd.desiredThrustsN[3] = 0.25f * (collThrustCmd - sqrt(2)/L * (momentCmd.x + momentCmd.y) - momentCmd.z/kappa); // rear right
    
   
    /////////////////////////////// END STUDENT CODE ////////////////////////////
    
    return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
    // Calculate a desired 3-axis moment given a desired and current body rate
    // INPUTS:
    //   pqrCmd: desired body rates [rad/s]
    //   pqr: current or estimated body rates [rad/s]
    // OUTPUT:
    //   return a V3F containing the desired moments for each of the 3 axes
    
    // HINTS:
    //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
    //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
    //  - you'll also need the gain parameter kpPQR (it's a V3F)
    
    V3F momentCmd;
    
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // Body rate P controller in Newtons * meters;
    momentCmd.x = Ixx * kpPQR.x * (pqrCmd.x - pqr.x);
    momentCmd.y = Iyy * kpPQR.y * (pqrCmd.y - pqr.y);
    momentCmd.z = Izz * kpPQR.z * (pqrCmd.z - pqr.z);
    
    /////////////////////////////// END STUDENT CODE ////////////////////////////
    
    return momentCmd;
}

// returns a desired roll and pitch rate
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
    // Calculate a desired pitch and roll angle rates based on a desired global
    //   lateral acceleration, the current attitude of the quad, and desired
    //   collective thrust command
    // INPUTS:
    //   accelCmd: desired acceleration in global XY coordinates [m/s2]
    //   attitude: current or estimated attitude of the vehicle
    //   collThrustCmd: desired collective thrust of the quad [N]
    // OUTPUT:
    //   return a V3F containing the desired pitch and roll rates. The Z
    //     element of the V3F should be left at its default value (0)
    
    // HINTS:
    //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
    //  - you'll need the roll/pitch gain kpBank
    //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first
    
    V3F pqrCmd;
    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    
    // Convert Thrust to acceleration
    float cd = collThrustCmd / mass;
    
    // Dimensionless commanded accelerations
    float b_x_c = -CONSTRAIN(accelCmd[0]/cd, -maxTiltAngle, maxTiltAngle);
    float b_y_c = -CONSTRAIN(accelCmd[1]/cd, -maxTiltAngle, maxTiltAngle);
    
    // Rotation matrix elements to account for non-linear transformations
    float R22 = R(2,2);
    float R10 = R(1,0);
    float R02 = R(0,2);
    float R00 = R(0,0);
    float R12 = R(1,2);
    float R11 = R(1,1);
    float R01 = R(0,1);
    
    // P controller for roll and pitc accounting for non-linear transformations from accelerations to body rates
    pqrCmd.x = 1/R22 * (R10 * kpBank * (b_x_c - R02) - R00 * kpBank * (b_y_c - R12));
    pqrCmd.y = 1/R22 * (R11 * kpBank * (b_x_c - R02) - R01 * kpBank * (b_y_c - R12));
    pqrCmd.z = 0.0f;
   
    /////////////////////////////// END STUDENT CODE ////////////////////////////
    
    return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
    // Calculate desired quad thrust based on altitude setpoint, actual altitude,
    //   vertical velocity setpoint, actual vertical velocity, and a vertical
    //   acceleration feed-forward command
    // INPUTS:
    //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
    //   posZ, velZ: current vertical position and velocity in NED [m]
    //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
    //   dt: the time step of the measurements [seconds]
    // OUTPUT:
    //   return a collective thrust command in [N]
    
    // HINTS:
    //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
    //  - you'll need the gain parameters kpPosZ and kpVelZ
    //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
    //  - make sure to return a force, not an acceleration
    //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER
    
    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    float thrust = 0;
    
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    
    // Constrain maximum vertical speeds.
    float acc_vel = kpPosZ * (posZCmd - posZ) + velZCmd;
    acc_vel = CONSTRAIN(acc_vel, -maxDescentRate, maxAscentRate);
    
    // Thrust PID controller accounting for gravity effect
    thrust = -(accelZCmd + kpVelZ * (acc_vel - velZ) - 9.81 + KiPosZ*(posZCmd - posZ)*dt) * mass / R(2,2);
    
    /////////////////////////////// END STUDENT CODE ////////////////////////////
    
    return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmd)
{
    // Calculate a desired horizontal acceleration based on
    //  desired lateral position/velocity/acceleration and current pose
    // INPUTS:
    //   posCmd: desired position, in NED [m]
    //   velCmd: desired velocity, in NED [m/s]
    //   pos: current position, NED [m]
    //   vel: current velocity, NED [m/s]
    //   accelCmd: desired acceleration, NED [m/s2]
    // OUTPUT:
    //   return a V3F with desired horizontal accelerations.
    //     the Z component should be 0
    // HINTS:
    //  - use fmodf(foo,b) to constrain float foo to range [0,b]
    //  - use the gain parameters kpPosXY and kpVelXY
    //  - make sure you cap the horizontal velocity and acceleration
    //    to maxSpeedXY and maxAccelXY
    
    // make sure we don't have any incoming z-component
    accelCmd.z = 0;
    velCmd.z = 0;
    posCmd.z = pos.z;
    
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    

    V3F h_velocity = kpPosXY * (posCmd - pos);
    float h_v_mag = sqrtf(h_velocity.x*h_velocity.x + h_velocity.y*h_velocity.y);
    
    // Cap horizontal velocity
    if(h_v_mag > maxSpeedXY)
        h_velocity = h_velocity * maxSpeedXY / h_v_mag;
    
    
    V3F h_acceleration = kpVelXY * (velCmd - vel);
    float h_a_mag = sqrtf(h_acceleration.x*h_acceleration.x + h_acceleration.y*h_acceleration.y);

    // Cap horizontal acceleration
    if(h_a_mag > maxAccelXY)
        h_acceleration = h_acceleration * maxAccelXY / h_a_mag;
    
    accelCmd = h_velocity + h_acceleration + accelCmd ;

    // PD controller with feed forward acceleration
    accelCmd = kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel) + accelCmd;

    /////////////////////////////// END STUDENT CODE ////////////////////////////
    
    return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
    // Calculate a desired yaw rate to control yaw to yawCmd
    // INPUTS:
    
    //   yawCmd: commanded yaw [rad]
    //   yaw: current yaw [rad]
    // OUTPUT:
    //   return a desired yaw rate [rad/s]
    // HINTS:
    //  - use fmodf(foo,b) to constrain float foo to range [0,b]
    //  - use the yaw control gain parameter kpYaw
    
    float yawRateCmd=0;
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    
    // Make sure psi_cmd is between 0 and 2*pi
    yawCmd = fmodf(yawCmd, 2*M_PI);
    float yaw_err = yawCmd - yaw;
    
    if (yaw_err > M_PI)
        yaw_err = yaw_err - 2.f*M_PI;
    else if (yaw_err < -M_PI)
        yaw_err = yaw_err + 2.f*M_PI;
    
    // Yaw P controller
    yawRateCmd = kpYaw*yaw_err;
    
    
    /////////////////////////////// END STUDENT CODE ////////////////////////////
    
    return yawRateCmd;
    
}
VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
    curTrajPoint = GetNextTrajectoryPoint(simTime);
    
    float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);
    
    // reserve some thrust margin for angle control
    float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
    collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
    
    V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
    
    V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
    desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());
    
    V3F desMoment = BodyRateControl(desOmega, estOmega);
    
    return GenerateMotorCommands(collThrustCmd, desMoment);
}

