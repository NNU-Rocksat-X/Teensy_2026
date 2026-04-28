/**
 * Stepper class for PID control of stepper motors. 
 * 
 * @author Riley Mark
 * @author December 12, 2022
 */

/**
 * ChatGPT Cheet Sheet:
 *
 * ===============================================================
 *                       PID TUNING CHEAT SHEET
 * ===============================================================
 *
 * SYSTEM:
 * Position → PID → velocity → step frequency
 *
 * ---------------------------------------------------------------
 * PROPORTIONAL (P)
 * ---------------------------------------------------------------
 * Too Low:
 *   - Slow response
 *   - May not reach target
 *
 * Too High:
 *   - Large oscillations (big swings past target)
 *   - Overshoot, instability
 *
 * Fix:
 *   - Increase until oscillation, then reduce ~20–30%
 *
 * ---------------------------------------------------------------
 * INTEGRAL (I)
 * ---------------------------------------------------------------
 * Too Low:
 *   - Stops near target but not exactly (steady-state error)
 *
 * Too High:
 *   - Wobbling near target (small back-and-forth motion)
 *   - Slow oscillations
 *
 * Extreme (Windup):
 *   - Large overshoot
 *   - Keeps pushing after crossing target
 *
 * Fix:
 *   - Add only if needed
 *   - Increase slowly
 *
 * ---------------------------------------------------------------
 * DERIVATIVE (D)
 * ---------------------------------------------------------------
 * Too Low:
 *   - Overshoot
 *   - Poor stopping behavior
 *
 * Too High:
 *   - Noisy / jittery motion
 *   - Amplifies encoder noise
 *
 * Fix:
 *   - Increase to reduce overshoot
 *   - Keep minimal for smooth motion
 *
 * ---------------------------------------------------------------
 * MAX_INTEGRAL (Anti-Windup Clamp)
 * ---------------------------------------------------------------
 * Too Low:
 *   - Acts like I is too small
 *   - Cannot remove steady-state error
 *
 * Too High:
 *   - Overshoot increases
 *   - Windup effects
 *
 * Fix:
 *   - Set just high enough to remove error
 *
 * ---------------------------------------------------------------
 * OSCILLATION vs WOBBLE
 * ---------------------------------------------------------------
 * Oscillation:
 *   - Large, fast swings around target
 *   - Cause: P too high
 *
 * Wobble Near Target:
 *   - Small, slow twitching at target
 *   - Cause: I too high
 *
 * ---------------------------------------------------------------
 * COMMON SYMPTOMS → FIXES
 * ---------------------------------------------------------------
 * Slow response            → ↑ P
 * Oscillation              → ↓ P or ↑ D
 * Overshoot                → ↑ D or ↓ P
 * Stops short              → ↑ I or ↑ MAX_INTEGRAL
 * Wobbling near target     → ↓ I
 * Big overshoot after move → ↓ MAX_INTEGRAL
 * Jittery motion           → ↓ D
 * Drifting / unstable hold → ↓ I
 *
 * ---------------------------------------------------------------
 * TUNING ORDER
 * ---------------------------------------------------------------
 * 1. Set I = 0, D = 0
 * 2. Increase P until slight oscillation
 * 3. Reduce P ~20–30%
 * 4. Add D to reduce overshoot
 * 5. Add I ONLY if needed
 * 6. Adjust MAX_INTEGRAL
 *
 * ---------------------------------------------------------------
 * IMPORTANT IMPLEMENTATION NOTES
 * ---------------------------------------------------------------
 * - delta_time must be in seconds (micros() / 1e6)
 * - integral += error * delta_time
 * - derivative = (error - prev_error) / delta_time
 * - Clamp integral to prevent windup
 * - Deadband (MAX_VELOCITY) can cause early stopping
 *
 * ===============================================================
 */

#ifndef step_h
#define step_h

#include "Arduino.h"
#include "Encoder.h"

#define NUM_JOINTS 8
#define GEAR_RATIO 5.076923077

#define MIN_FREQUANCY 100.0

#define MIN_VELOCITY 0.00075

#define PROPORTIONAL_GAIN 0.6
#define DERIVATIVE_GAIN  0.00//0025
#define INTERGRAL_GAIN 0.0//1

#define MAX_INTEGRAL 1

#define MAX_ERROR 10

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
   void setCords();
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

   //Varables for step()
   bool highLow;

   // Varables for pid_controller()
   double integral;
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

   int32_t rad_to_step(int32_t deg) const;
   int32_t step_to_rad(int32_t step) const;

};


#endif
