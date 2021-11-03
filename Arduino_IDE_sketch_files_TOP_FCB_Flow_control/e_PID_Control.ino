double PID_min, PID_max;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 0.07, 0.9, 0.0, DIRECT);
//PID myPID(&Input, &Output, &Setpoint, 0.095, 0.925, 0.002, DIRECT); // for pulsatile flow

//Initialize PID Controller
void setup_PID(float setpoint, double PID_min, double PID_max, int *SampleTime)
{
  //initialize the variables we're linked to
  Input = 0;
  Setpoint = setpoint;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // set max value for output
  myPID.SetOutputLimits(PID_min, PID_max); 

  // set sampling time
  myPID.SetSampleTime(*SampleTime); //sample time in ms, how often the PID should update the output
}

double PID_flow_control(float *PID_input)
{
  Input = *PID_input;
  myPID.Compute();
  //return Output;
}
