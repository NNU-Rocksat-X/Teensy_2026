#include <Arduino.h>
#include "teensy_comm.h"
#include "step.h"
#include <TimerOne.h>
#include <imxrt.h> // Make sure Teensy core libraries are included

#define LIMIT_SWITCH_PIN 39
#define zeroing_timout_value 8000

static teensy_status_t tnsy_sts = {0};
static teensy_command_t tnsy_cmd = {0};
static teensy_zero_t tnsy_zero = {0};
static teensy_header_t tnsy_hdr = {0};

int32_t position_cmds[NUM_JOINTS];

/*  Stepper Contructor:
Stepper::Stepper(
    int8_t motor_ID_In,
    bool closedLoop_In,
    int8_t stepPin_ID_In,
    int8_t directionPin_ID_In,
    int8_t encoderPinA_ID_In,
    int8_t encoderPinB_ID_In,
    int encoderResolution_In
  )
  */
Stepper myStepper[] = 
{//      motor_ID   ClosedLoop    stepPin   dirPin    encoderPin_A    encoderPin_B  encoderRes.
  Stepper(1,        1,            3,        2,        14,             15,           1000  ),
  Stepper(2,        1,            5,        4,        16,             17,           300   ),
  Stepper(3,        1,            7,        6,        18,             19,           1000  ), 
  Stepper(4,        1,            9,        8,        21,             20,           300   ),
  Stepper(5,        1,            11,       10,       23,             22,           1000  ),
  Stepper(6,        1,            13,       12,       25,             24,           300   ),
  Stepper(7,        1,            28,       29,       27,             26,           1000  ),
  Stepper(8,        1,            28,       29,       27,             26,           1000  )
};


//****************************************    High Level Code       ****************************************

int main(void) 
{
  setup();
  //zeroing();
  loop();
}

void loop(void) 
{
  while (1)
  {
    // Serial Receive
    receive_command();

    // Check Each Motor
    //motor_task();

    // Serial Send
    send_status();

    print_encoder_values();
    print_target_values();
    Serial.println(" ");

    delay(10);
  }
}


//****************************************    Low Level Code       ****************************************
void setup(void) {
  
  Serial.begin(115200);
  Serial.println("Setup will begin ");

  noInterrupts();
  pinMode(13, OUTPUT);

  Timer1.initialize(10);             // interrupt every 10 us
  Timer1.attachInterrupt(motorISR);  // motorISR is the ISR
  Timer1.start();
  
  Serial1.begin(115200, SERIAL_8N1);
  //Serial.begin(115200);

  Serial.println("Setup Complete");

  interrupts();
}

void motor_task()
{
  for (int ii = 0; ii < NUM_JOINTS; ++ii) 
  {
    myStepper[ii].motorTask();
  }
}

int receive_command() 
{
  uint8_t buffer[1024] = { 0 };
  uint8_t bytes_received = 0;
  int ret = 0;

  while (Serial1.available()) 
  {

    if (bytes_received >= sizeof(buffer)) 
    {
      Serial.println("Error: Buffer overflow!");
      return -1;
    }

    Serial1.readBytes(&buffer[bytes_received], 1);
    bytes_received++;

    // Only parse when a full message is received
    if (bytes_received == sizeof(tnsy_cmd))
    {
      ret = parse_message(&buffer[0], bytes_received);

      if (ret == 1) // TODO: make sure all messages are handled
      {
        for (int j = 0; j < NUM_JOINTS; ++j) 
        {
          myStepper[j].setPositionCommand(tnsy_cmd.setpoint_position[j]);
        }
        return 1;
      } 
      else if (ret == 2)
      {
        // could put Zeroing function call
        return 2;
      }
      else 
      {
        return -2;
      }
    }

  }
  return 0;
}


int parse_message(const uint8_t* buf, int size) 
{

  uint16_t calc_crc = 0;
  uint16_t rec_crc = 0;
  uint16_t hdr_chk = 0;

  memcpy(&hdr_chk, buf, sizeof(hdr_chk));

  if (hdr_chk == 0x5555) 
  {
    calc_crc = crc16_ccitt(buf, size - 2);
    memcpy(&rec_crc, buf + size - 2, sizeof(rec_crc));

    if (calc_crc == rec_crc)
    {
      memcpy(&tnsy_hdr, buf, size - 2);

      // Added message type support. TODO: add this to the jetson
      switch (tnsy_hdr.type)
      {
        case 0:
          // TODO: update Jetson with correct message type for generic command message.
          memcpy(&tnsy_cmd, buf, size - 2);
          return 0;
          break;

        case 1:
          memcpy(&tnsy_cmd, buf, size - 2);
          return 1;
          break;

        case 2:
          memcpy(&tnsy_zero, buf, size - 2);
          return 2;
          break;

        default:
          Serial.print("Error with message parsing");
          return -3;
          break;
      }

    } 
    else 
    {
      return -1;
    }
  } 
  else 
  {
    return -2;
  }
}

int send_status (void) 
{
  uint8_t buffer[1024] = {0};

  tnsy_sts.hdr.header = 0x5555;
  tnsy_sts.hdr.seq++;
  tnsy_sts.hdr.len = sizeof(tnsy_sts);
  tnsy_sts.hdr.type = 0;

  for (int ii = 0; ii < NUM_JOINTS; ii++) 
  {
    if (ii == 6)
      tnsy_sts.encoder[ii] = (myStepper[ii].getEncoderPosition() / 10.0);
    else
      tnsy_sts.encoder[ii] = myStepper[ii].getEncoderPosition();
  }

  memcpy(&buffer[0], &tnsy_sts, sizeof(tnsy_sts) - 2);
  tnsy_sts.crc = crc16_ccitt(&buffer[0], sizeof(tnsy_sts) - 2);

  memcpy(&buffer[0], &tnsy_sts, sizeof(tnsy_sts));

  Serial1.write(buffer, sizeof(tnsy_sts));

  return 0;
}

void motorISR(void) 
{
  for (int ii = 0; ii < NUM_JOINTS; ++ii) 
  {
    myStepper[ii].stepCheck();
  }
}

//****************************************    Zeroing       ****************************************

void zeroing(void)
{                           


  motor_reset();
}


void motor_reset()
{
  for (int ii = 0; ii < NUM_JOINTS; ++ii) 
  {
    myStepper[ii].motorReset();
  }
}

void new_pos ( int motor, int goal_pos ) // input motor NUMBER not motor INDEX
{/*
  int cur_pos = 0;

  --motor;
  myStepper[motor].positionCommand = goal_pos;

  for (int i = 0; i < zeroing_timout_value; ++i)
  {
    motor_task();
    cur_pos = get_position(motor);

      if (abs(cur_pos - goal_pos) < 2) // will end function off as soon as it's within an error radius of 2
      {
        return;
      }
      else if (myStepper[motor].motor_id == 7 && abs(cur_pos - goal_pos) < 11)  //special treatment of joint 7 cuz we need it to move outside of range
      {
        return;
      }

      delay(1);
  }*/
  Serial.println("AHAHAHA, THIS MOVEMNET FAILED YOU DUMB NUT");
}

bool readGPIOFast(int pin) 
{
  return *(portInputRegister(pin)) & digitalPinToBitMask(pin);
}


//****************************************    Test Functions       ****************************************
void motor_test(int tested_motor, int speed)
{/*
  Serial.print("Testing Motor ");
  Serial.print(tested_motor);
  Serial.println(" ");

  int i;

  --tested_motor;   // this is so that the above comment is true

  while(1)
  {

    tasks[tested_motor].period = myStepper[tested_motor].newFrequency(0, speed);

    for (i = 0; i < NUM_JOINTS - NUM_EJCT_JOINTS; ++i) 
    {
      if (i != tested_motor)
      {
        tasks[i].period = myStepper[i].newFrequency(myEncoder[i].read(), 0);
      } 
    } /// Keep other motors where they are
    for (i = NUM_JOINTS - NUM_EJCT_JOINTS; i < NUM_EJCT_JOINTS; ++i)
    {
      if (i != tested_motor)
      {
        tasks[i].period = myStepper[i].newFrequency(0, 0);
      }
    }

    //print_encoder_values();

    delay(10);
  }*/
}


//****************************************      Print Functions       ****************************************
void print_lim_switches (int motor_in, int failure)
{
  if( failure )
  {
    Serial.print("Zeroing failed on motor ");
    Serial.print(motor_in);
    Serial.println(" ");
  }
  else
  {
    Serial.print("Check: ");
    Serial.print(motor_in + 1);
    Serial.println(" ");
  }
}

void print_encoder_values (void)
{
  // Encoder readout  
  for ( int ii = 0; ii < NUM_JOINTS; ++ii)
  {
    Serial.print(myStepper[ii].getEncoderPosition());
    Serial.print(" ");
  }
  Serial.println(" ");
}

void print_target_values (void)
{
  for ( int i = 0; i < NUM_JOINTS; ++i)
  {
    Serial.print(myStepper[i].getPositionCommand());
    Serial.print(" ");
  }
  Serial.println(" ");
}

/**

If the permission port permission error occurs, use the following commands on Jetson

sudo usermod -a -G tty <username>
sudo usermod -a -G dialout <username>
sudo usermod -a -G sudo <username>

sudo chmod g+rw /dev/ttyTHS1


*/