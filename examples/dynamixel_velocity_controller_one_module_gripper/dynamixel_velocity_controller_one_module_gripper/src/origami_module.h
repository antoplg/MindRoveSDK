/**********************************************************************************************
* Origami Module for Servo Motor Control
* by Mustafa Mete <mustafa.mete@epfl.ch>
**********************************************************************************************/
#ifndef OrigamiModule
#define OrigamiModule

#define POSITION_CONTROL_PERIOD 50 // [ms]

#include "PID-custom.h"

class Module
{
    private:

    public:
    //int moduleType;      // 1, 2, or 3 DoF
    // Constructor  for 3 DoF
    int motorID1, motorID2, motorID3;
    
    Module(int id1, int id2, int id3);
       
    // STATE variables
    // this should be private and be rachable by get. But how to get an array, but I'm so lazy :/
    float modulePositionX, modulePositionY, modulePositionZ;

    // double moduleLegAngle1, moduleLegAngle2, moduleLegAngle3; // [rad]
    // double moduleLegAnglePrevious1, moduleLegAnglePrevious2, moduleLegAnglePrevious3;
    // double moduleLegAnglularVelocity1, moduleLegAnglularVelocity2, moduleLegAnglularVelocity3;
    

    float moduleLegAngle[3]; // [rad]
    float moduleLegAnglePrevious[3];
    float moduleLegAnglularVelocity[3];


    // DO I USE THEM? why not just setpoint? TARGET variables 
    //double moduleTargetPositionX, moduleTargetPositionY, moduleTargetPositionZ;
    //double moduleTargetLegAngle1, moduleTargetLegAngle2, moduleTargetLegAngle3;
    float moduleTargetLegAngle[3];
 
    // SETPOINT variables
    float setpointPositionX=0, setpointPositionY=0, setpointPositionZ=0;
    // double setpointLegAngle1=0, setpointLegAngle2=0, setpointLegAngle3=0;                           // [rad]
    // double setpointLegAngleVelocity1=0, setpointLegAngleVelocity2=0, setpointLegAngleVelocity3=0;   // [rad/s]

    float setpointLegAngle[3]={0,0,0};                             // [rad]
    float setpointLegAngleVelocity[3]={0,0,0};                     // [rad/s]

    // assign the input and output pins and sets the frequencies
    void setupPINs();

    // ------------------- module forward kinematics----------------//
    // method to calculate the end effector position from leg angles
    void calculateEndEffectorPosition();

    // -------------------module inverse kinematics----------------//
    // method to calculate the desired leg angles from the end effector position
    void calculateTargetAngles(float moduleTargetPositionX, float moduleTargetPositionY, float moduleTargetPositionZ); 
    
    // sub-method
    void convertCartesianToPolarCoordinate(float x, float y, float z, float *r, float *psi, float *delta);

    // -------------------read leg angle----------------//
    // method to set leg angles from servo motor
    void assignLegAngles(float angle1, float angle2, float angle3);
    
    float lowpassFilter(float previousFilteredValue, float input, float beta);
    
    void updateModuleStates();

    // Pouch PID variables and objects
    float positionPIDkP=0, positionPIDkI=0, positionPIDkD=0;

    float setpointLegVelocityFF[3]={0,0,0};
    float setpointLegVelocityFB[3]={0,0,0};
    
    // position PIDs
    PID* position1PID;
    PID* position2PID;
    PID* position3PID;

    void setupPIDs();
    void computePositionController();
    bool isTheSignSame(float, float);// returns  TRUE if they have the same sign, else false
};

#endif