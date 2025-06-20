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

#define S_RXD 18
#define S_TXD 19

#define shoulderID1 2
#define shoulderID2 3
#define ellbowID 4
#define headHoizontalID 5
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

s16 Position[SERVO_COUNT];
u16 Speed[SERVO_COUNT] = {0};
byte ACC[SERVO_COUNT];

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
int16_t shoulder  = 0;
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

int basePos = 0;
int shoulderPos = 0;
int ellbowPos = 0;

int shoulderRatio = 4;
int elbowRatio = 4;

int absolutePosition = 0;
int lastRawPositionS1 = 0;
int lastRawPositionS2 = 0;
int lastRawPositionE = 0;



void setup(){
  Serial.begin(115200);
  Serial.setTimeout(1);
  Serial1.begin(1000000, SERIAL_8N1);
  Serial1.setTimeout(1);
  st.pSerial = &Serial1;
  //for (int i = 0 ; i < SERVO_COUNT; i++) {
  //  long pos = EEPROM.read(i);
  //  prepMotor(i+2, pos);
  //  Serial.println(pos);
  //}


  shoulder = st.ReadPos(shoulderID2)/4;
  //Serial.println(shoulder);
  ellbow = st.ReadPos(ellbowID)/4;
  //Serial.println(ellbow);
  headHoizontal = st.ReadPos(headHoizontalID);
  headVertical = st.ReadPos(headVerticalID);
  headRotation = st.ReadPos(headRotationID);

  prepMotor(2, -shoulder);
  //prepMotor(3, shoulder);
  prepMotor(4, ellbow);
  prepMotor(5, headHoizontal);
  prepMotor(6, headVertical);
  //prepMotor(7, headRotation);
  stepper.begin();
  for (int i = 0; i < SERVO_COUNT; i++) {
    ACC[i] = 50;  // fixed acceleration
  }
  delay(1000);
  
}
int dir = 0;
long start = 0;
int speedSet = 0;

void loop(){
    String message = "";
    message = Serial.readStringUntil('\n');

    if(message != ""){

      speedSet = 1;
      sscanf(message.c_str(), "%i:%i/%i:%i/%i:%i/%i:%i/%i:%i/%i:%i/",
                &base, &baseSpd,
                &shoulder, &shoulderSpd,
                &ellbow, &ellbowSpd,
                &headHoizontal, &headHoizontalSpd,
                &headVertical, &headVerticalSpd,
                &headRotation, &headRotationSpd);

    }

    setMotorSpeed();
    String msg = readMotor();
    if(!speedSet){
      char buffer[256];
      sprintf(buffer, "%d:%d/%d:%d/%d:%d/%d:%d/%d:%d/%d:%d", 
      0,0,
      shoulder,0,
      ellbow, 0,
      headHoizontal, 0,
      headVertical, 0,
      headRotation, 0);
      msg = String(buffer); 
    }
    Serial.println(msg);
    Serial.flush();
}

void setMotorSpeed() {
  if(base > 0){
    stepper.step(FORWARD, base);
  }else {
    stepper.step(REVERSE, base * -1);
  }
  st.WritePosEx(shoulderID1, -shoulder * 4, 1000 * 4, 1000);
  //st.WritePosEx(shoulderID2, shoulder * 4, 1000 * 4, 1000);

  st.WritePosEx(ellbowID, ellbow * 4, 1000 * 4, 1000);
  st.WritePosEx(headHoizontalID, -headHoizontal, 4000, 1000);
  st.WritePosEx(headVerticalID, headVertical, 4000, 1000);
  //st.WritePosEx(headRotationID, headRotation, 4000, 1000);
}

String readMotor(){
  int tempShoulderPos1 = int(st.ReadPos(shoulderID1)/4);
  int shoulderSpeed = int(st.ReadSpeed(shoulderID1));
  int tempShoulderPos2 = int(st.ReadPos(shoulderID2)/4);
  int tempEllbowPos = int(st.ReadPos(ellbowID)/4);
  int ellbowSpeed = int(st.ReadSpeed(ellbowID));
  //int tempShoulderPos1 = computeAbsolutePosition(st.ReadPos(shoulderID1),lastRawPositionS1);
  //int tempShoulderPos2 = computeAbsolutePosition(st.ReadPos(shoulderID2), lastRawPositionS2);
  //int tempEllbowPos = computeAbsolutePosition(st.ReadPos(ellbowID), lastRawPositionE);
  int tempHeadHorizontalPos = st.ReadPos(headHoizontalID);
  int headHorizontalSpeed = int(st.ReadSpeed(headHoizontalID));
  int tempHeadVerticalPos = st.ReadPos(headVerticalID);
  int headVerticalSpeed = int(st.ReadSpeed(headVerticalID));
  //int tempHeadRotationPos = st.ReadPos(headRotationID);
  //int headRotationSpeed = int(st.ReadSpeed(headRotationID));

  char buffer[256];
  sprintf(buffer, "%d:%d/%d:%d/%d:%d/%d:%d/%d:%d/%d:%d", 
      0,0,
      -tempShoulderPos1,shoulderSpeed,
      tempEllbowPos, ellbowSpeed,
      -tempHeadHorizontalPos, headHorizontalSpeed,
      tempHeadVerticalPos, headVerticalSpeed,
      0, 0);

  return String(buffer);

}

void prepMotor(int ID, int value){
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
