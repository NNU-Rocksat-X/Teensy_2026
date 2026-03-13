/**
 * Stepper class for PID control of stepper motors. 
 * 
 * @author Riley Mark
 * @author December 12, 2022
 * @author Elias Friberg
 * @author March 2026
 */

#include "step.h"


Stepper::Stepper(
    int8_t motor_ID_In,
    bool closedLoop_In,
    int8_t stepPin_ID_In,
    int8_t directionPin_ID_In,
    int8_t encoderPinA_ID_In,
    int8_t encoderPinB_ID_In,
    int encoderResolution_In
  ): encoder(encoderPinA_ID_In, encoderPinB_ID_In)
{
  motor_ID = motor_ID_In;
  closedLoop = closedLoop_In;
  stepPin_ID = stepPin_ID_In;
  directionPin_ID = directionPin_ID_In;
  encoderPinA_ID = encoderPinA_ID_In;     //TODO: Dont think you need dis if you make encoder later
  encoderPinB_ID_In = encoderPinB_ID_In;
  encoderResolution = encoderResolution_In;

  positionCommand = 0;
  econderPosition = 0;

  proportional_gain = 0.3;
  integral_gain = 0.5;
  derivative_gain = 0.01;
  max_integral = 1;

  pinMode(stepPin_ID, OUTPUT);
  pinMode(directionPin_ID, OUTPUT);

  tasks.state = false;
  tasks.elapsedTime = 0;
  tasks.period = 1000000;
    

  if (closedLoop)
    encoder.write(0);
}

int32_t Stepper::getEncoderPosition() const 
{
  return econderPosition;
}

int32_t Stepper::getPositionCommand() const
{
  return positionCommand;
}

void Stepper::setPositionCommand(int32_t input)
{
  positionCommand = input;
}


void Stepper::motorTask() // Sets a new frequency
{
  int velocity;
  velocity = pid_controller(positionCommand, econderPosition);
  
  if(closedLoop)
  {
    econderPosition = encoder.read();
  }

  if (true) // All Joints  (put motor_ID == 1 through motor_ID == 6 if you need to flip motor 7 polarity)
  {  
    if (velocity > 0) 
    {
      direction = HIGH;
    } 
    else 
    {
      direction = LOW;
    }
  } 
  else 
  {    
    if (velocity > 0) 
    {
      direction = LOW;
    } 
    else 
    {
      direction = HIGH;
    }
  }

  // clamp the motor frequency
  if (fabs(velocity) < 0.001) 
  {
    motorFrequency = 1000000;
  } 
  else 
  {
    motorFrequency = (1000 / abs(velocity));
  }

  if (motorFrequency <= 100) 
  {
    motorFrequency = 100;
  } 

  tasks.period = motorFrequency; // Need to test this, make sure it's right
}

/**
 * The PID controller for the ARM motors.
 * 
 * @param desired_angle - position setpoint in encoder steps
 * @param current_pos - current position of the motor in encoder steps
 * 
 * @return int - current instantaneous velocity of motor
 * 
 * TODO: Tune the PID controllers
 */
double Stepper::pid_controller(double desired_angle, double current_pos) 
{
  double now_time = micros();
  double delta_time = now_time - previous_time;
  double error;
  double derivative;

  previous_time = now_time;

  error = desired_angle - current_pos;
  integral += error;
  derivative = (error - previous_error) / delta_time;

  // clamp the integral
  if (integral > max_integral) 
  {  
    integral = max_integral;
  } 
  else if (integral < -max_integral) 
  {
    integral = -max_integral;
  }

  previous_error = error;

  return velocity = error * proportional_gain + integral * integral_gain + derivative * derivative_gain;
}

// ISR Function
void Stepper::stepCheck()
{
  if (tasks.period == 0) 
  {
    return;
  }

  //increase the elapsed time since the last time the function was called
  tasks.elapsedTime += 10;

  if (tasks.elapsedTime >= tasks.period) 
  {
    step();       //call the step function
    tasks.elapsedTime = 0;  //reset the elapsed time
    
    updateClosedLoopMotors();
  }
}

/**
 * Advances the state of the step pin 
 * 
 * @return - none
 */
void Stepper::step() 
{
  if (!highLow) 
  {
    if (direction == HIGH) 
    {
      digitalWrite(directionPin_ID, HIGH);
    }
    else 
    {
      digitalWrite(directionPin_ID, LOW);
    }

    digitalWrite(stepPin_ID, HIGH);
    highLow = HIGH;
  }
  else 
  {
    digitalWrite(stepPin_ID, LOW);
    highLow = LOW;
  }
}

void Stepper::updateClosedLoopMotors()
{
  if(!closedLoop)  // Open Loop
  {
    if (positionCommand - econderPosition > 0.5)  // will have an error buffer of .5 in either direction
    {
      econderPosition++;
    } else if (positionCommand - econderPosition < -0.5) 
    {
      econderPosition--;
    }
  }
}

void Stepper::motorReset()
{
  positionCommand = 0;
  econderPosition = 0;
  resetEncoder();
}

void Stepper::resetEncoder()
{
  encoder.write(0);
}


int Stepper::rad_to_step(int deg)
{
    return deg * 100;
}


int Stepper::step_to_rad(int step)
{
  return step / 100;
}


