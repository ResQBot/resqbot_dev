#include <TLE9879_Group.h>
//Baudrate is defined in the TLE Library

// Shield-Gruppenobjekt
TLE9879_Group *shields;

// FrontLeft_Board  = BOARD4;
// FrontRight_Board = BOARD1;
// RearLeft_Board   = BOARD3;
// RearRight_Board  = BOARD2;

//variables for connection
int16_t stop_delay = 10;
int16_t refresh_rate = 20;            //[Hz]
int16_t DELAY = (1000 / refresh_rate) - stop_delay;  //delay between two loops according to refresh_rate

//variables for movement controll
int16_t FrontLeft   = 0;
int16_t FrontRight  = 0;
int16_t RearLeft    = 0;
int16_t RearRight   = 0;
int16_t default_speed = 2900;

int16_t FrontLeft_mode  = 0;
int16_t FrontRight_mode = 0;
int16_t RearLeft_mode   = 0;
int16_t RearRight_mode  = 0;

void setup(){

  // Initialize the Shield group object with the number of Shields in the stack
  shields = new TLE9879_Group(4);

  // Set the desired mode (FOC, HALL, BEMF)
  shields->setMode(HALL, BOARD4);
  shields->setMode(HALL, BOARD3);
  shields->setMode(HALL, BOARD2);
  shields->setMode(HALL, BOARD1);

  //Set Hall Frequency
  shields->setParameter(HALL_PWM_FREQ, 20000, BOARD4);      //[kHz]
  shields->setParameter(HALL_PWM_FREQ, 20000, BOARD3);      //[kHz]
  shields->setParameter(HALL_PWM_FREQ, 20000, BOARD2);      //[kHz]
  shields->setParameter(HALL_PWM_FREQ, 20000, BOARD1);      //[kHz]
  
  //set number of pole pairs (look up in data sheet)
  shields->setParameter(HALL_POLE_PAIRS, 2, BOARD4);
  shields->setParameter(HALL_POLE_PAIRS, 2, BOARD3);
  shields->setParameter(HALL_POLE_PAIRS, 2, BOARD2);
  shields->setParameter(HALL_POLE_PAIRS, 2, BOARD1);

  //set minimum I value (must be 0 to enable motor speed = 0)
  shields->setParameter(HALL_SPEED_IMIN, 0, BOARD4);
  shields->setParameter(HALL_SPEED_IMIN, 0, BOARD3);
  shields->setParameter(HALL_SPEED_IMIN, 0, BOARD2);
  shields->setParameter(HALL_SPEED_IMIN, 0, BOARD1);
  
  //set max I value (indirectly restricts max current)
  shields->setParameter(HALL_SPEED_IMAX, 29, BOARD4);
  shields->setParameter(HALL_SPEED_IMAX, 29, BOARD3);
  shields->setParameter(HALL_SPEED_IMAX, 29, BOARD2);
  shields->setParameter(HALL_SPEED_IMAX, 29, BOARD1);
  
  //set min P value (must be 0 to enable motor speed = 0)
  shields->setParameter(HALL_SPEED_PIMIN, 0, BOARD4);
  shields->setParameter(HALL_SPEED_PIMIN, 0, BOARD3);
  shields->setParameter(HALL_SPEED_PIMIN, 0, BOARD2);
  shields->setParameter(HALL_SPEED_PIMIN, 0, BOARD1);
  
  //set max P value (indirectly restricts max current)
  shields->setParameter(HALL_SPEED_PIMAX, 29, BOARD4);
  shields->setParameter(HALL_SPEED_PIMAX, 29, BOARD3);
  shields->setParameter(HALL_SPEED_PIMAX, 29, BOARD2);
  shields->setParameter(HALL_SPEED_PIMAX, 29, BOARD1);
  
  /*
  //proportional gain in the PI controller formula
  shields->setParameter(HALL_SPEED_KP, 500, BOARD4);
  
  //integral gain in the PI controller formula
  shields->setParameter(HALL_SPEED_KI, 100, BOARD4);
  
  //shields->setLed(LED_ON, BOARD4);
  shields->setLedColor(COLOR_BLUE, BOARD4);
  */
  
  shields->setMotorSpeed(0, BOARD4);
  shields->setMotorSpeed(0, BOARD3);
  shields->setMotorSpeed(0, BOARD2);
  shields->setMotorSpeed(0, BOARD1);
  
  shields->setMotorMode(START_MOTOR, BOARD4);
  shields->setMotorMode(START_MOTOR, BOARD3);
  shields->setMotorMode(START_MOTOR, BOARD2);
  shields->setMotorMode(START_MOTOR, BOARD1);

}
 
void loop(){
//check if Serial connection is good. If not, stop motors.
  if (Serial) {
//receive message via serial connection
    String rx_msg = Serial.readStringUntil('\n');
    sscanf(rx_msg.c_str(), "FL%iFR%iRL%iRR%i\n", &FrontLeft, &FrontRight, &RearLeft, &RearRight);
  }
  else {
    FrontLeft  = 0;
    FrontRight = 0;
    RearLeft   = 0;
    RearRight  = 0;
  }
  
//correction for motor orientation
  RearLeft  = RearLeft  * -1;
  RearRight = RearRight * -1;

//set front left motor speed
  if (FrontLeft == FrontLeft_mode) {
    delay(stop_delay);
  }
  else {
    shields->setMotorMode(STOP_MOTOR, BOARD4);
    delay(stop_delay);
    shields->setMotorSpeed(FrontLeft * default_speed, BOARD4);
    shields->setMotorMode(START_MOTOR, BOARD4);
    FrontLeft_mode = FrontLeft;
  }

//set front right motor speed
  if (FrontRight == FrontRight_mode) {
    delay(stop_delay);
  }
  else {
    shields->setMotorMode(STOP_MOTOR, BOARD1);
    delay(stop_delay);
    shields->setMotorSpeed(FrontRight * default_speed, BOARD1);
    shields->setMotorMode(START_MOTOR, BOARD1);
    FrontRight_mode = FrontRight;
  }

//set rear left motor speed
  if (RearLeft == RearLeft_mode) {
    delay(stop_delay);
  }
  else {
    shields->setMotorMode(STOP_MOTOR, BOARD3);
    delay(stop_delay);
    shields->setMotorSpeed(RearLeft * default_speed, BOARD3);
    shields->setMotorMode(START_MOTOR, BOARD3);
    RearLeft_mode = RearLeft;
  }

//set rear right motor speed
  if (RearRight == RearRight_mode) {
    delay(stop_delay);
  }
  else {
    shields->setMotorMode(STOP_MOTOR, BOARD2);
    delay(stop_delay);
    shields->setMotorSpeed(RearRight * default_speed, BOARD2);
    shields->setMotorMode(START_MOTOR, BOARD2);
    RearRight_mode = RearRight;
  }
  
/* if the new command is the same as the current motion -> do nothing
 * else stop the motor for a short interval and set the new speed, then restart the motor.
 * stopping the motor allows for residual current to be dispersed and fixes a problem with reversing the motor's direction.
 */

//delay until next cycle  
  delay(DELAY);
}
