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
int16_t refresh_rate = 100;            //[Hz]
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

  // Override serial baud rate to 115200 to match ROS serial_comms.hpp
  // Also set a low timeout so readStringUntil doesn't block for 1000ms on fragmented data
  Serial.begin(115200);
  Serial.setTimeout(10);

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
 // Helper function to figure out if the motor is moving forward, backward, or stopped
int8_t getSign(int16_t val) {
  if (val > 0) return 1;
  if (val < 0) return -1;
  return 0;
}

void loop() {
  bool newData = false;

  // 1. FLUSH THE SERIAL BUFFER (Fixes continuous command lag)
  // Read all pending messages, but only keep the absolute newest one.
  if (Serial.available() > 0) {
    String rx_msg = "";
    while (Serial.available() > 0) {
      String temp = Serial.readStringUntil('\n');
      if (temp.length() > 5) { // Basic check to ensure it's a valid string
        rx_msg = temp;
      }
    }
    
    // Process only the freshest command
    if (rx_msg != "") {
          int tempFL = 0, tempFR = 0, tempRL = 0, tempRR = 0;
          
          // sscanf returns the number of items it successfully matched
          int parsed = sscanf(rx_msg.c_str(), "FL%iFR%iRL%iRR%i", &tempFL, &tempFR, &tempRL, &tempRR);
          
          // Only apply the changes if ALL 4 values were successfully read!
          if (parsed == 4) {
            FrontLeft  = tempFL;
            FrontRight = tempFR;
            // Flip the rears immediately upon successful read
            RearLeft   = tempRL * -1;
            RearRight  = tempRR * -1;
            newData    = true;
          } else {
            // If we got a partial message, we just ignore it. 
            // A full message will arrive in the next millisecond anyway.
            newData = false; 
          }
        }
  }

  // 2. PROCESS MOTORS ONLY IF WE HAVE NEW DATA
  if (newData) {
    bool changeFL = (FrontLeft != FrontLeft_mode);
    bool changeFR = (FrontRight != FrontRight_mode);
    bool changeRL = (RearLeft != RearLeft_mode);
    bool changeRR = (RearRight != RearRight_mode);

    // Check if the change is an actual DIRECTION change (or starting from 0)
    bool reverseFL = changeFL && (getSign(FrontLeft) != getSign(FrontLeft_mode));
    bool reverseFR = changeFR && (getSign(FrontRight) != getSign(FrontRight_mode));
    bool reverseRL = changeRL && (getSign(RearLeft) != getSign(RearLeft_mode));
    bool reverseRR = changeRR && (getSign(RearRight) != getSign(RearRight_mode));

    // 3. APPLY STOPS ONLY FOR DIRECTION CHANGES
    bool needs_delay = false;
    if (reverseFL) { shields->setMotorMode(STOP_MOTOR, BOARD4); needs_delay = true; }
    if (reverseFR) { shields->setMotorMode(STOP_MOTOR, BOARD1); needs_delay = true; }
    if (reverseRL) { shields->setMotorMode(STOP_MOTOR, BOARD3); needs_delay = true; }
    if (reverseRR) { shields->setMotorMode(STOP_MOTOR, BOARD2); needs_delay = true; }

    // Wait ONCE, and only if a motor is actually changing direction
    if (needs_delay) {
      delay(stop_delay); 
    }

    // 4. UPDATE SPEEDS AND RESTART
    if (changeFL) {
      shields->setMotorSpeed(FrontLeft * default_speed, BOARD4);
      if (reverseFL) shields->setMotorMode(START_MOTOR, BOARD4);
      FrontLeft_mode = FrontLeft;
    }
    if (changeFR) {
      shields->setMotorSpeed(FrontRight * default_speed, BOARD1);
      if (reverseFR) shields->setMotorMode(START_MOTOR, BOARD1);
      FrontRight_mode = FrontRight;
    }
    if (changeRL) {
      shields->setMotorSpeed(RearLeft * default_speed, BOARD3);
      if (reverseRL) shields->setMotorMode(START_MOTOR, BOARD3);
      RearLeft_mode = RearLeft;
    }
    if (changeRR) {
      shields->setMotorSpeed(RearRight * default_speed, BOARD2);
      if (reverseRR) shields->setMotorMode(START_MOTOR, BOARD2);
      RearRight_mode = RearRight;
    }
  }

  // 5. Delay until next cycle
  delay(DELAY);
}