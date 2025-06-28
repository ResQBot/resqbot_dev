/*
The normal write example passed the test in ST3215 Servo, 
and if testing other models of ST series servos
please change the appropriate position, speed and delay parameters.
*/

#include <SCServo.h>
#include <SoftwareSerial.h>
#include <Bonezegei_DRV8825.h>
#include <SomeSerial.h>

#include <EEPROM.h>
#include <CNCShield.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);

#define NO_OF_STEPS               200
#define SLEEP_BETWEEN_STEPS_MS    10
#define SPEED_STEPS_PER_SECOND    100

/*
 * Create a CNCShield object and get a pointer to motor 0 (X axis).
 */
CNCShield cnc_shield;
StepperMotor *motor = cnc_shield.get_motor(0);
#define S_RXD 18
#define S_TXD 19

#define shoulderID1 2
#define shoulderID2 3
#define ellbowID 4
#define headHoizontalID 7
#define headVerticalID 6
#define headRotationID 7


#define FORWARD 1
#define REVERSE 0

#define PIN_DIR  8
#define PIN_STEP 9

#define SERVO_COUNT 6

byte ID[SERVO_COUNT] = {
  shoulderID1, shoulderID2,
  ellbowID,
  headHoizontalID, headVerticalID, headRotationID
};

SoftwareSerial swSerial(16, 17);
SomeSerial hwSerial(&swSerial);
const uint8_t sendPin  = 3;
const uint8_t deviceID = 1;
String rsBuffer = "";

SMS_STS st;

// the UART used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.

Bonezegei_DRV8825 stepper(PIN_DIR, PIN_STEP);


int16_t baseSpd = 0, shoulderSpd = 0, ellbowSpd = 0, headHoizontalSpd = 0, headVerticalSpd = 0, headRotationSpd = 0;
int16_t base   = 0;
int16_t shoulder1  = 0;
int16_t shoulder2  = 0;
int16_t ellbow    = 0;
int16_t headHoizontal   = 0;
int16_t headVertical    = 0;
int16_t headRotation   = 0;
int16_t currentBase   = 0;
int16_t currentShoulder  = 0;
int16_t currentEllbow    = 0;
int16_t currentHeadHoizontal   = 0;
int16_t currentHeadVertical    = 0;
int16_t currentHeadRotation   = 0;

int shoulder1Start = 0;
int shoulder2Start = 0;
int ellbowStart = 0;
int headStart = 0;
int headVerticalStart = 0;

int basePos = 0;
int shoulderPos = 0;
int ellbowPos = 0;

int shoulderRatio = 4;
int elbowRatio = 4;

int absolutePosition = 0;
int lastRawPositionS1 = 0;
int lastRawPositionS2 = 0;
int lastRawPositionE = 0;

void initServo(){

  while(st.Ping(shoulderID2) == -1){
    delay(100);
  }

  shoulder1 = st.ReadPos(shoulderID1);
  shoulder2 = st.ReadPos(shoulderID2);
  //Serial.println(shoulder);
  ellbow = st.ReadPos(ellbowID);
  //Serial.println(ellbow);
  headHoizontal = st.ReadPos(headHoizontalID);
  headVertical = st.ReadPos(headVerticalID);
  //headRotation = st.ReadPos(headRotationID);

  shoulder1Start = shoulder1;
  shoulder2Start = shoulder2;
  ellbowStart = ellbow;
  headStart = headHoizontal;
  headVerticalStart = headVertical;
  shoulderSpd = 100;
  ellbowSpd = 100;
  headHoizontalSpd = 100;
  headVerticalSpd = 100;
  prepMotor(2);
  prepMotor(3);
  prepMotor(4);
  prepMotor(7);
  prepMotor(6);
  //prepMotor(7);

  // Serial.println(shoulder1Start);
  // Serial.println(shoulder2Start);
  // Serial.println(ellbowStart);
  // Serial.println(headStart);
  // Serial.println(headVerticalStart);

  st.WritePosEx(shoulderID1, shoulder1Start, 100);
  st.WritePosEx(shoulderID2, shoulder2Start, 100);
  st.WritePosEx(ellbowID, ellbowStart, 100);
  st.WritePosEx(headHoizontalID, headStart, 100);
  st.WritePosEx(headVerticalID, headVerticalStart, 100);
}

void setup(){
  Serial.begin(115200);
  Serial.setTimeout(1);
  Serial1.begin(1000000, SERIAL_8N1);
  Serial1.setTimeout(1);
  // cnc_shield.begin();
  // cnc_shield.enable();
  // motor->set_speed(SPEED_STEPS_PER_SECOND);
  st.pSerial = &Serial1;
  //for (int i = 0 ; i < SERVO_COUNT; i++) {
  //  long pos = EEPROM.read(i);
  //  prepMotor(i+2, pos);
  //  Serial.println(pos);
  //}

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    //Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  //Serial.println("Motor Shield found.");

  myMotor->setSpeed(300);  // 10 rpm
  initServo();

  // st.WritePosEx(shoulderID1, 0, 100);
  // st.WritePosEx(shoulderID2, 0, 100);
  // st.WritePosEx(ellbowID, 0, 100);
  // st.WritePosEx(headHoizontalID, 0, 100);
  // st.WritePosEx(headVerticalID, 0, 100);
  //stepper.begin();
  delay(1000);
  
}
int dir = 0;
long start = 0;
int speedSet = 0;

void loop(){
    String message = "";
    message = Serial.readStringUntil('\n');
    if(st.Ping(shoulderID2) == -1){
      initServo();
    }
    if(message != ""){
      //Serial.println(isValidPattern(message));
      if(isValidPattern(message)){
        sscanf(message.c_str(), "%i:%i/%i:%i/%i:%i/%i:%i/%i:%i/%i:%i/",
                &base, &baseSpd,
                &shoulder2, &shoulderSpd,
                &ellbow, &ellbowSpd,
                &headHoizontal, &headHoizontalSpd,
                &headVertical, &headVerticalSpd,
                &headRotation, &headRotationSpd);
                shoulder2 *= 4;
                ellbow *= 4;
                //Serial.print("base: ");
                //Serial.println(base);
                //Serial.println(message);
      }
    }

    setMotorSpeed();
    String msg = readMotor();

    Serial.println(msg);
    Serial.flush();
}

bool isSignedInteger(const String& text) {
  if (text.length() == 0) return false;

  int start = (text.charAt(0) == '-') ? 1 : 0;
  if (start == text.length()) return false; // string was just "-"

  for (int i = start; i < text.length(); i++) {
    if (!isDigit(text.charAt(i))) return false;
  }
  return true;
}

bool isValidPattern(String input) {
  // Must end with '/'
  if (!input.endsWith("/")) return false;

  int start = 0;
  while (start < input.length()) {
    int slashIndex = input.indexOf('/', start);
    if (slashIndex == -1) return false;

    String segment = input.substring(start, slashIndex);
    int colonIndex = segment.indexOf(':');

    // Check that there is exactly one colon and something on both sides
    if (colonIndex <= 0 || colonIndex == segment.length() - 1) {
      return false;
    }

    String left = segment.substring(0, colonIndex);
    String right = segment.substring(colonIndex + 1);

    // Check both parts are valid signed integers
    if (!isSignedInteger(left) || !isSignedInteger(right)) return false;

    // Move to next segment
    start = slashIndex + 1;
  }

  return true;
}



void setMotorSpeed() {
  //shoulder2 = shoulder2*4;

  int tempShoulder = convert(shoulder2, shoulder2Start, shoulder1Start);
  // Serial.println(tempShoulder);
  // Serial.println(shoulder2);
  // Serial.println(ellbow);
  // Serial.println(headHoizontal);
  // Serial.println(headVertical);

  st.WritePosEx(shoulderID1, tempShoulder, 1000 * 4, 1000);
  st.WritePosEx(shoulderID2, shoulder2, 1000 * 4, 1000);

  st.WritePosEx(ellbowID, ellbow, 1000 * 4, 1000);
  st.WritePosEx(headHoizontalID, headHoizontal, 4000, 1000);
  st.WritePosEx(headVerticalID, headVertical, 4000, 1000);
  //st.WritePosEx(headRotationID, headRotation, 4000, 1000);
  if(base > currentBase){
    myMotor->step(1, FORWARD, DOUBLE);
    currentBase += 1;
    //stepper.step(FORWARD, base);
  } else if ( base < currentBase) {
    myMotor->step(1, BACKWARD, DOUBLE);
    currentBase -= 1;
    //stepper.step(REVERSE, base * -1);
  }
}

int convert(int encodedValue, int startPos1, int startPos2) {
  return startPos2 + startPos1 - encodedValue;
}

String readMotor(){
  int tempShoulderPos1 = int(st.ReadPos(shoulderID1)/4);
  int shoulderSpeed = int(st.ReadSpeed(shoulderID1));
  int tempShoulderPos2 = int(st.ReadPos(shoulderID2)/4);
  int tempEllbowPos = int(st.ReadPos(ellbowID)/4);
  int ellbowSpeed = int(st.ReadSpeed(ellbowID));
  int tempHeadHorizontalPos = st.ReadPos(headHoizontalID);
  int headHorizontalSpeed = int(st.ReadSpeed(headHoizontalID));
  int tempHeadVerticalPos = st.ReadPos(headVerticalID);
  int headVerticalSpeed = int(st.ReadSpeed(headVerticalID));

  char buffer[256];
  sprintf(buffer, "%d:%d/%d:%d/%d:%d/%d:%d/%d:%d/%d:%d", 
      currentBase,0,
      tempShoulderPos2,shoulderSpeed,
      tempEllbowPos, ellbowSpeed,
      tempHeadHorizontalPos, headHorizontalSpeed,
      tempHeadVerticalPos, headVerticalSpeed,
      0, 0);

  return String(buffer);

}

void prepMotor(int ID){
  st.unLockEprom(ID);

  // Enable multi-turn mode (bit4 of 0x12)
  uint8_t phase = st.readByte(ID, 0x12);
  phase |= 0x10;
  //st.writeByte(ID, 0x12, 28);

  // Set angle limits to 0 (unlimited)
  st.writeByte(ID, 0x09, -8192); // Min angle
  st.writeByte(ID, 0x0A, 0); // Min angle high byte
  st.writeByte(ID, 0x0B, 8192); // Max angle
  st.writeByte(ID, 0x0C, 0); // Max angle high byte
  //st.writeByte(ID, 0x2A, value);

  st.LockEprom(ID);
}
