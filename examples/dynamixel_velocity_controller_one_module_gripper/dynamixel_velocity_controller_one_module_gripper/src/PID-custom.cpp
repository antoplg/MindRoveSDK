/**********************************************************************************************
 * Arduino PID Libraryfor pneumagami
 * by Mustafa Mete <mustafa.mete@epfl.ch>
 **********************************************************************************************/
#include "Arduino.h"
#include "PID-custom.h"

//Constructor
PID::PID(float* Input, float* Output, float* Setpoint,
        float Kp, float Ki, float Kd, unsigned long SampleTime, int ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint; 
    this->SampleTime = SampleTime;              //default Controller Sample Time is 0.01 seconds  // [ms]

    PID::SetOutputLimits(0, 255);				//default output limit corresponds to //the arduino pwm limits
    PID::IntegratorAntiWindUpLimits(0, 255);

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis()-SampleTime;
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      float input = *myInput;
        myError = *mySetpoint - input; // should error be global? er>0 if setpoint>input
        dInput = (input - lastInput);

      float aggresiveError = 1; // [rad] // integrator limit is around 40-60 rad
      if((integrator*myError<0) && ( abs((float)integrator)> (float)aggresiveError) )
            integrator+= 4*(ki * myError);
      else
            integrator+= (ki * myError);


        // clamping the pid integrator
        if(integrator > outMaxIntegrator) integrator = outMaxIntegrator;
        else if(integrator < outMinIntegrator) integrator = outMinIntegrator;

        filterBeta = 0.3;
        // filter the velocity
        //dInputFiltered = this->lowpassFilter(dInputFiltered, dInput, filterBeta);
        dInputFiltered = dInput;

        float output;
        // PID
        output = kp * myError + integrator +  kd * dInputFiltered;

        //clamping the pid output
        if(output > outMax) output = outMax;
        else if(output < outMin) output = outMin;

        *myOutput = output;

        // Remember some variables for next time 
        lastInput = input;
        lastTime = now;
        return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(float Kp, float Ki, float Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;

   float SampleTimeInSec = ((float)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;

  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}



/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
}

void PID::IntegratorAntiWindUpLimits(float Min, float Max){
   if(Min >= Max) return;
   outMinIntegrator= Min;
   outMaxIntegrator= Max;
}


/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.) 
 ******************************************************************************/
void PID::SetControllerDirection(int Direction){
   if(Direction !=controllerDirection){
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
   }
   controllerDirection = Direction;
}

//change this to moving average
float PID::lowpassFilter(float previousFilteredValue, float input, float beta){
  return beta * previousFilteredValue + (1 - beta) * input;
}