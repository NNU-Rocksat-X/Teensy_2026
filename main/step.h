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

   //Getters and Setters
   int32_t getEncoderPosition() const;
   int32_t getPositionCommand() const;
   void setPositionCommand(int32_t);

   //Public Functions
   void stepCheck();
   void read_encoders();
   void motorTask();
   void motorReset();
   void resetEncoder();

private:
   // Private varables:
   // Motor specifics
   int8_t motor_ID;
   bool closedLoop;

   //Positions
   int32_t positionCommand;
   int32_t econderPosition;

   // Pins
   int8_t stepPin_ID;
   int8_t directionPin_ID;
   int8_t encoderPinA_ID;
   int8_t encoderPinB_ID;

   // Encodor
   Encoder encoder;   // From Encoder libary, Encoder object called encoder
   int encoderResolution;

   // Varables for motorTask()
   bool direction;
   int motorFrequency;
   double velocity;

   //Varables for step()
   bool highLow;

   // Varables for pid_controller()
   double integral;
   double proportional_gain;
   double integral_gain;
   double derivative_gain;
   double max_integral;
   double previous_error;
   int previous_time;

   //Task Scheduler
   struct TaskScheduler 
   {
      bool state;
      volatile int elapsedTime;
      int period;
      void (*function)();
   };

   TaskScheduler tasks;

   // Private funtions
   double pid_controller(double desired_angle, double current_angle);

   void step();
   void updateClosedLoopMotors();

   //int newFrequency(double position, double desired_position);

   int rad_to_step(int deg);
   int step_to_rad(int step);

};


#endif
