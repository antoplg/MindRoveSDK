/**********************************************************************************************
* Origami Module for Servo Motor Control
* by Mustafa Mete <mustafa.mete@epfl.ch>
**********************************************************************************************/

#include "Arduino.h"
#include "origami_module.h"

/* Constructor*/
// // Constructor  for 3 DoF
Module::Module(int id1, int id2, int id3){
      this->motorID1 = id1; this->motorID2 = id2; this->motorID3 = id3;
    }

// -------- CALCULATE THE STATES!!!  --------////
// module forward kinematics
void Module::calculateEndEffectorPosition(){ // for displaying where the module end effector is
    float phi1 = this->moduleLegAngle[0];
    float phi2 = this->moduleLegAngle[1];
    float phi3 = this->moduleLegAngle[2];

    float newPositionX = ((sin(phi1)*((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi3)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi2)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0)*6.0E1-((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi3)*6.0E1))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi2)*6.0E1))/2.0)*(cos(phi1)*6.0E1+sqrt(3.0)*2.5E1))*((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi3)*6.0E1))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi2)*6.0E1))/2.0)*-2.0)/(pow(fabs((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi3)*6.0E1))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi2)*6.0E1))/2.0),2.0)+pow(fabs((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi3)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi2)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0),2.0)+pow(fabs((sin(phi1)*6.0E1-sin(phi2)*6.0E1)*(cos(phi1)*6.0E1+cos(phi3)*3.0E1+sqrt(3.0)*(7.5E1/2.0))-(sin(phi1)*6.0E1-sin(phi3)*6.0E1)*(cos(phi1)*6.0E1+cos(phi2)*3.0E1+sqrt(3.0)*(7.5E1/2.0))),2.0));
    float newPositionY = ((sin(phi1)*((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi3)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi2)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0)*6.0E1-((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi3)*6.0E1))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi2)*6.0E1))/2.0)*(cos(phi1)*6.0E1+sqrt(3.0)*2.5E1))*((sin(phi1)*6.0E1-sin(phi2)*6.0E1)*(cos(phi1)*6.0E1+cos(phi3)*3.0E1+sqrt(3.0)*(7.5E1/2.0))-(sin(phi1)*6.0E1-sin(phi3)*6.0E1)*(cos(phi1)*6.0E1+cos(phi2)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))*2.0)/(pow(fabs((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi3)*6.0E1))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi2)*6.0E1))/2.0),2.0)+pow(fabs((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi3)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi2)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0),2.0)+pow(fabs((sin(phi1)*6.0E1-sin(phi2)*6.0E1)*(cos(phi1)*6.0E1+cos(phi3)*3.0E1+sqrt(3.0)*(7.5E1/2.0))-(sin(phi1)*6.0E1-sin(phi3)*6.0E1)*(cos(phi1)*6.0E1+cos(phi2)*3.0E1+sqrt(3.0)*(7.5E1/2.0))),2.0));
    float newPositionZ = ((sin(phi1)*((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi3)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi2)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0)*6.0E1-((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi3)*6.0E1))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi2)*6.0E1))/2.0)*(cos(phi1)*6.0E1+sqrt(3.0)*2.5E1))*((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi3)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi2)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0)*2.0)/(pow(fabs((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi3)*6.0E1))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(sin(phi1)*6.0E1-sin(phi2)*6.0E1))/2.0),2.0)+pow(fabs((sqrt(3.0)*(cos(phi2)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi3)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0+(sqrt(3.0)*(cos(phi3)*6.0E1+sqrt(3.0)*2.5E1)*(cos(phi1)*6.0E1+cos(phi2)*3.0E1+sqrt(3.0)*(7.5E1/2.0)))/2.0),2.0)+pow(fabs((sin(phi1)*6.0E1-sin(phi2)*6.0E1)*(cos(phi1)*6.0E1+cos(phi3)*3.0E1+sqrt(3.0)*(7.5E1/2.0))-(sin(phi1)*6.0E1-sin(phi3)*6.0E1)*(cos(phi1)*6.0E1+cos(phi2)*3.0E1+sqrt(3.0)*(7.5E1/2.0))),2.0));

    if( !isnan(newPositionX) && !isnan(newPositionY) && !isnan(newPositionZ)){
      this->modulePositionX = newPositionX;
      this->modulePositionY = newPositionY;
      this->modulePositionZ = newPositionZ;
    }
 }

// reads the potentiometer sensor and calculate the angles // also, calculates the velocity
void Module::assignLegAngles(float angle1, float angle2, float angle3){
  this->moduleLegAngle[0] = fmod(angle1, PI);
  this->moduleLegAngle[1] = fmod(angle2, PI);
  this->moduleLegAngle[2] = fmod(angle3, PI);
}

// -------- SETPOINTS!! --------/////
// module inverse kinematics  - // takes target module position, calculates the target angles
void Module::calculateTargetAngles(float moduleTargetPositionX, float moduleTargetPositionY, float moduleTargetPositionZ){ 
    float psi, delta, r0;
    //convert to polar coordinates! because of the model!
    convertCartesianToPolarCoordinate(moduleTargetPositionX, moduleTargetPositionY, moduleTargetPositionZ, &r0, &psi, &delta);

    float newTargetLegAngle1 = atan((cos(psi)*1.2E2-sqrt(pow(cos(psi),2.0)*1.44E4+pow(cos(delta),2.0)*pow(sin(psi),2.0)*6.9E3-r0*r0+sqrt(3.0)*r0*cos(delta)*sin(psi)*1.0E2))/(r0+cos(delta)*sin(psi)*1.2E2-sqrt(3.0)*cos(delta)*sin(psi)*5.0E1))*2.0;
    float newTargetLegAngle2 = atan((cos(psi)*1.2E2-sqrt(pow(cos(psi),2.0)*1.44E4+pow(cos(delta),2.0)*pow(sin(psi),2.0)*1.725E3-r0*r0+pow(sin(delta),2.0)*pow(sin(psi),2.0)*5.175E3+r0*sin(delta)*sin(psi)*1.5E2-sqrt(3.0)*cos(delta)*sin(delta)*pow(sin(psi),2.0)*3.45E3-sqrt(3.0)*r0*cos(delta)*sin(psi)*5.0E1))/(r0+cos(delta-3.141592653589793*(2.0/3.0))*sin(psi)*1.2E2-sqrt(3.0)*cos(delta-3.141592653589793*(2.0/3.0))*sin(psi)*5.0E1))*2.0;
    float newTargetLegAngle3 = atan((cos(psi)*1.2E2-sqrt(pow(cos(psi),2.0)*1.44E4+pow(cos(delta),2.0)*pow(sin(psi),2.0)*1.725E3-r0*r0+pow(sin(delta),2.0)*pow(sin(psi),2.0)*5.175E3-r0*sin(delta)*sin(psi)*1.5E2+sqrt(3.0)*cos(delta)*sin(delta)*pow(sin(psi),2.0)*3.45E3-sqrt(3.0)*r0*cos(delta)*sin(psi)*5.0E1))/(r0+cos(delta-3.141592653589793*(4.0/3.0))*sin(psi)*1.2E2-sqrt(3.0)*cos(delta-3.141592653589793*(4.0/3.0))*sin(psi)*5.0E1))*2.0;

    if( !isnan(newTargetLegAngle1) && !isnan(newTargetLegAngle2) && !isnan(newTargetLegAngle3)){
      this->moduleTargetLegAngle[0] = newTargetLegAngle1;
      this->moduleTargetLegAngle[1] = newTargetLegAngle2;
      this->moduleTargetLegAngle[2] = newTargetLegAngle3;
    }
}

// -------- DEPENDENCIES!! --------/////
// submethod for the "calculateAngleSetPoints()"
void Module::convertCartesianToPolarCoordinate(float x, float y, float z, float *r, float *psi, float *delta){
  *r = sqrt(x*x + y*y + z*z); 
  *psi = acos(z/sqrt(x*x + y*y + z*z));
  if(y<0){ //correction to the conversion!
    //*psi = -1*acos(z/sqrt(x*x + y*y + z*z)); // dont remember why I did this :( For me psi is always positive
  }
  if((x==0) & (y==0)) // gives nan at this position!
    *delta = 0;
  else
    *delta = atan2(y,x);
}

float Module::lowpassFilter(float previousFilteredValue, float input, float beta){
  return beta * previousFilteredValue + (1 - beta) * input;
}

void Module::updateModuleStates(){}

void Module::setupPIDs(){
  unsigned long positionControlSampleTime = (int)POSITION_CONTROL_PERIOD;
  float antiWindUpLimit = 40.0;
  float speedLimit = 40.0;
  this->position1PID = new  PID(&(this->moduleLegAngle[0]), &(this->setpointLegVelocityFB[0]), &(this->setpointLegAngle[0]), 
  positionPIDkP, positionPIDkI, positionPIDkD, positionControlSampleTime, DIRECT);
  this->position1PID->SetOutputLimits(-speedLimit,speedLimit);
  this->position1PID->IntegratorAntiWindUpLimits(-antiWindUpLimit, antiWindUpLimit);

  this->position2PID = new  PID(&(this->moduleLegAngle[1]), &(this->setpointLegVelocityFB[1]), &(this->setpointLegAngle[1]), 
  positionPIDkP, positionPIDkI, positionPIDkD, positionControlSampleTime, DIRECT);
  this->position2PID->SetOutputLimits(-speedLimit,speedLimit);
  this->position2PID->IntegratorAntiWindUpLimits(-antiWindUpLimit, antiWindUpLimit);

  this->position3PID = new  PID(&(this->moduleLegAngle[2]), &(this->setpointLegVelocityFB[2]), &(this->setpointLegAngle[2]), 
  positionPIDkP, positionPIDkI, positionPIDkD, positionControlSampleTime, DIRECT);
  this->position3PID->SetOutputLimits(-speedLimit,speedLimit);
  this->position3PID->IntegratorAntiWindUpLimits(-antiWindUpLimit, antiWindUpLimit);
}

// ToDos: check this part's performance
void Module::computePositionController(){
  // double antiWindupCoeff=6;
  // anti-windup check
  // if(!isTheSignSame(this->setpointLegVelocityFB1, this->position1PID->myError))
  //   this->position1PID->integrator = this->position1PID->integrator + antiWindupCoeff*positionPIDkI*this->position1PID->myError;
  this->position1PID->Compute();

  // if(!isTheSignSame(this->setpointLegVelocityFB2, this->position2PID->myError))
  //   this->position2PID->integrator = this->position2PID->integrator + antiWindupCoeff*positionPIDkI*this->position2PID->myError;
  this->position2PID->Compute();

  // if(!isTheSignSame(this->setpointLegVelocityFB3, this->position3PID->myError))
    // this->position3PID->integrator = this->position3PID->integrator + antiWindupCoeff*positionPIDkI*this->position3PID->myError;
  this->position3PID->Compute();
}

bool Module::isTheSignSame(float x, float y){
  return (x>0) == (y>0);
}