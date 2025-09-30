#ifndef PID_custom
#define PID_custom

class PID
{
  public:
  #define DIRECT  0
  #define REVERSE  1

  //commonly used functions **************************************************************************
  PID(float*, float*, float*,        // * constructor.  links the PID to the Input, Output, and 
        float, float, float,  unsigned long, int);//   Setpoint.  Initial tuning parameters are also set here.
                                          //   (overload for specifying proportional mode)
	
  void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

  bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

  void SetOutputLimits(float, float); // * clamps the output to a specific range. 0-255 by default, but
										                      //   it's likely the user will want to change this depending on
										                      //   the application

  void SetTunings(float, float, float);        	    

	void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.

  void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                        //   the PID calculation is performed.  default is 100
										  
  // METE edition
  void IntegratorAntiWindUpLimits(float Min, float Max); // * anti windup thing for pid. not necessary should be equal to the max PID output
	
  float integrator = 0;   	            // integrator
  float dInput = 0, dInputFiltered = 0;  // input velocity and filtered vel. can be monitored as well
  float myError = 0;                           // difference between output and input

  //Display functions ****************************************************************

  float LowpassFilter(float , float , float , float );

  private:
	void Initialize();          // why?
    
	float kp;                  // * (P)roportional Tuning Parameter
  float ki;                  // * (I)ntegral Tuning Parameter
  float kd;                  // * (D)erivative Tuning Parameter

  unsigned long SampleTime = 20;  // [ms]

	int controllerDirection;

  float *myInput;                // * Pointers to the Input, Output, and Setpoint variables
  float lastInput;
  float *myOutput;               //   This creates a hard link between the variables and the 
  float *mySetpoint;             //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
                
	unsigned long lastTime;
  float outMin, outMax;
  float outMinIntegrator, outMaxIntegrator;

  float lowpassFilter(float, float, float);
  float filterBeta = 0.0;              // coeff
};
#endif