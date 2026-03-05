/**
 * Stepper class for PID control of stepper motors. 
 * 
 * @author Riley Mark
 * @author December 12, 2022
 */
#ifndef step_h
#define step_h

//#define EJECTOR_MOTOR_SPEED 100

#include "Arduino.h"
#include "Encoder.h"


class Stepper {
public:
  int8_t motor_ID;
  bool closedLoop;
  double currentAngle;

  int32_t positionCommand;
  int32_t econderPosition;

  

  // Constuctor
  Stepper(
    int8_t motor_ID_In,
    bool closedLoop_In,
    int8_t stepPin_ID_In,
    int8_t directionPin_ID_In,
    int8_t encoderPinA_ID_In,
    int8_t encoderPinB_ID_In,
    int encoderResolution_In
   );

  /**
     * Advances the state of the step pin 
     * 
     * @return - none
     */
  void step(void);

  /**
     * Function to update the frequency and direction of the motor movement.
     * 
     * @param position - current encoder position
     * @param desired_position - desired position from Jetson cmd
     * 
     * @return int - frequency of motor movement
     */
  int newFrequency(double position, double desired_position);

   int32_t getEncoder();
   void resetEncoder();
   void read_encoders();
   void motor_task();
   int get_position();
   void stepCheck();


private:
  int8_t stepPin_ID;
  int8_t directionPin_ID;
  int8_t encoderPinA_ID;
  int8_t encoderPinB_ID;
  int encoderResolution;

  Encoder encoder;   // From Encoder class, Encoder object called encoder


  double current_velocity;
  bool direction;
  int motorFrequency;
  bool highLow;

  double derivative;
  double integral;
  double error;
  double proportional_gain;
  double integral_gain;
  double derivative_gain;
  double max_integral;
   double velocity;

  double previous_error;
  int previous_time;

   struct TaskScheduler 
   {
      bool state;
      volatile int elapsedTime;
      int period;
      void (*function)();
   };

   TaskScheduler tasks;

  /**
     * Converts degrees to steps
     * 
     * @param deg - int of target position in degrees
     * TODO: For future missions this should get converted to a double
     * 
     * @return int - angle in steps
     */
  int deg_to_step(int deg);

  /**
     * The PID controller for the ARM motors.
     * 
     * @param desired_angle - position setpoint in encoder steps
     * @param current_angle - current position of the motor in encoder steps
     * 
     * @return int - current instantaneous velocity of motor
     */
  double pid_controller(double desired_angle, double current_angle);
};


#endif
