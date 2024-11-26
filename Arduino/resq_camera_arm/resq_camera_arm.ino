#include <XYZrobotServo.h>

#include <SoftwareSerial.h>

String NAME = "ArmController";
int CONNECTED = 0;
int NOCOMMANDCOUNT = 0;

SoftwareSerial servoSerial(8, 9);

// Set up a servo object, specifying what serial port to use and
// what ID number to use.
//
// WARNING: Only change the ID number below to a servo that can
// rotate freely without damaging anything.
XYZrobotServo bottomServo(servoSerial, 1);
XYZrobotServo midServo(servoSerial, 2);
XYZrobotServo topServo(servoSerial, 3);
XYZrobotServo headServo(servoSerial, 4);


void setup()
{
  Serial.begin(115200);

  servoSerial.begin(115200);
  bottomServo.setPosition(300, 100);
  midServo.setPosition(100, 100);
  topServo.setPosition(590, 100);
  headServo.setPosition(500, 100);
  delay(2000);
}


int lowerPos = 300;
int midPos = 100;
int topPos = 590;
int headPos = 500;

int speed = 20;

int lowerMax = 730;
int lowerMin = 300;

int midMax = 900;
int midMin = 100;

int topMax = 900;
int topMin = 590;
int singleTopMin = 250;
int singleTopMax = 750;

int headMax = 750;
int headMin = 250;

void loop()
{
  long start = millis();

  if(Serial.available()){
    if (CONNECTED == 1){
      handleArmMovement();
    } else {
      String msg = Serial.readStringUntil('\n');
      if (msg == "Who are you?"){
        handshake();
      } 
    }
  }else {
    NOCOMMANDCOUNT++;
    if (NOCOMMANDCOUNT > 5){
      CONNECTED = 0;
      NOCOMMANDCOUNT = 0;
    }
  }
  
    delay(speed);
}

void handleArmMovement(){
  String input = Serial.readStringUntil(',');
  String inputTurn = Serial.readStringUntil(',');
  String inputLook = Serial.readStringUntil('\n');

  if (input == "UP") {
    lowerPos = min(max(lowerPos + 10, lowerMin), 500);
    midPos = min(max(midPos + 20, midMin), 500);
    topPos = min(max(topPos + 10, topMin), topMax);
  } else if (input == "DOWN") {
    lowerPos = min(max(lowerPos - 10, lowerMin), 500);
    midPos = min(max(midPos - 20, midMin), 500);
    topPos = min(max(topPos - 10, topMin), topMax);
  }

  if (inputTurn == "LEFT") {
    headPos = min(max(headPos + 10, headMin), headMax);
  } else if (inputTurn == "RIGHT") {
    headPos = min(max(headPos - 10, headMin), headMax);
  }

  if (inputLook == "LUP") {
    topPos = min(max(topPos - 10, singleTopMin), singleTopMax);
  } else if (inputLook == "LDOWN") {
    topPos = min(max(topPos + 10, singleTopMin), singleTopMax);
  }

  midServo.setPosition(midPos, 10);
  bottomServo.setPosition(lowerPos, 10);
  topServo.setPosition(topPos, 10);
  headServo.setPosition(headPos, 10);

}

void homePos(){

  bottomServo.setPosition(300, 100);
  midServo.setPosition(100, 100);
  topServo.setPosition(590, 100);
  headServo.setPosition(500, 100);

  delay(1000);
  bottomServo.torqueOff();
  midServo.torqueOff();
}

float translateValueIntoNewRange(float currentvalue, float currentmax, float currentmin, float newmax, float newmin) {
  return (((currentvalue - currentmin) * (newmax - newmin)) / (currentmax - currentmin)) + newmin;
}

void handshake(){ 

  //set confirmation to 0
  bool confirmation = 0;
  
  //answer Handshake
  Serial.println(NAME);

  int start = millis();

  //wait for confirmation
  String rx_msg = "";
  while(!confirmation){
    if (millis() - start > 1500){
      return;
    }
    //wait for answer, then read
    delay(50);
    rx_msg = Serial.readStringUntil('\n');

    //check for correct answer and confirm handshake#
    Serial.println(rx_msg);
    if(rx_msg == "Hello ArmController" || rx_msg == "Who are youHello ArmController"){
      confirmation = 1;
      CONNECTED = 1;
      Serial.println("confirmed");
      return;
    } else {
      confirmation = 0;
    }
  }
}