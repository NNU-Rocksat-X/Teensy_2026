/**
 * Stepper class for PID control of stepper motors. 
 * 
 * @author Riley Mark
 * @author December 12, 2022
 */
#include "step.h"

// #include <Arduino.h>

/**
 * Stepper class constructor 
 */
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

  currentAngle = 0;
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
      digitalWrite(stepPin_ID, HIGH);
    }
    else 
    {
      digitalWrite(directionPin_ID, LOW);
      digitalWrite(stepPin_ID, HIGH);
    }

    highLow = HIGH;
  }
  else 
  {
    digitalWrite(stepPin_ID, LOW);
    highLow = LOW;
  }
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


/**
 * Converts degrees to steps
 * 
 * @param deg - int of target position in degrees
 * TODO: For future missions this should get converted to a double
 * 
 * @return int - angle in steps
 * 
 * TODO: update this with the updated gear ratios per joint
 * TODO: dont hard code in the gear ratios....
 */
int Stepper::deg_to_step(int deg) 
{
  if (motor_ID == 7 || motor_ID == 8 ) 
  {
    int temp_val = ( 1 / 360.0 );
    return temp_val * deg;
  } 
  else if (motor_ID == 6 ) 
  {
    int temp_val = ((encoderResolution * 4.0 ) / 360.0);
    return temp_val * deg;
  }
  else
  {
    int temp_val = ((encoderResolution * 4.0 * 7.0 ) / 360.0);
    return temp_val * deg;
  } 
}


/**
 * Function to update the frequency and direction of the motor movement.
 * 
 * @param position - current encoder position
 * @param desired_position - desired position from Jetson cmd
 * 
 * @return int - frequency of motor movement
 * 
 * TODO: set the motor polarity once the new motors are wired up
 */
int Stepper::newFrequency(double position, double desired_position) 
{
  int velocity;

  velocity = pid_controller(desired_position, position);


  if (true) // All Joints  (put motor_ID == 1 if you need to flip motor 1 polarity)
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

  return motorFrequency;
}

int32_t Stepper::getEncoder()
{
  return encoder.read();
}

void Stepper::read_encoders()
{ 
  if(closedLoop) {
    econderPosition = getEncoder();
  }
  else{
    econderPosition = currentAngle;  
  }
}

void Stepper::motor_task()
{
  econderPosition = get_position();
  if (motor_ID == 6) 
    tasks.period = newFrequency(get_position(), positionCommand * 10);
  else
    tasks.period = newFrequency(get_position(), positionCommand);


}

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

    if (motor_ID == 7)  // speicial *10 for special joint 7 funcitnallity, ask Elias, this is his stuipid work around, cuz he's to lazy to fix it as of right now, what a bum
    {
      if ((positionCommand * 10) - currentAngle > 0.5)  // will have an error buffer of .5 in either direction
      {
        currentAngle++;
      } else if ((positionCommand * 10) - currentAngle < -0.5) {
        currentAngle--;
      }
    } else if (!closedLoop)  // ajust angle of closed loop joints
    {
      if (positionCommand - currentAngle > 0.5)  // will have an error buffer of .5 in either direction
      {
        currentAngle++;
      } else if (positionCommand - currentAngle < -0.5) {
        currentAngle--;
      }
    }
  }
}


int Stepper::get_position()
{
  if (motor_ID < 5)   // closed loop
    return getEncoder();
  else if (motor_ID == 6)           // speicial command for joint 7, that allows it to get to the desired position, which is outside the current limit of the communcated values 
    return currentAngle; 
  else                                // open loop
    return currentAngle;
}


