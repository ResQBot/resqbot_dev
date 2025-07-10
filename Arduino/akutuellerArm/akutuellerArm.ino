
#include <SCServo.h>

#define baseID          8
#define shoulderID1     2
#define shoulderID2     3
#define ellbowID        4
#define headHoizontalID 7
#define headVerticalID  6
#define headRotationID  7




SMS_STS st;

int16_t base            = 0;
int16_t shoulder1       = 0;
int16_t shoulder2       = 0;
int16_t ellbow          = 0;
int16_t headHoizontal   = 0;
int16_t headVertical    = 0;
int16_t headRotation    = 0;
int16_t newBase         = 0;

int baseStart           = 0;
int shoulder1Start      = 0;
int shoulder2Start      = 0;
int ellbowStart         = 0;
int headStart           = 0;
int headVerticalStart   = 0;


long start = 0;

String serialBuffer = "";

void initServo(){

  while(st.Ping(shoulderID2) == -1){
    delay(100);
  }

  newBase = st.ReadPos(baseID);
  shoulder1 = st.ReadPos(shoulderID1);
  shoulder2 = st.ReadPos(shoulderID2);
  ellbow = st.ReadPos(ellbowID);
  headHoizontal = st.ReadPos(headHoizontalID);
  headVertical = st.ReadPos(headVerticalID);

  base = newBase;
  baseStart = newBase;
  shoulder1Start = shoulder1;
  shoulder2Start = shoulder2;
  ellbowStart = ellbow;
  headStart = headHoizontal;
  headVerticalStart = headVertical;

  prepMotor(baseID);
  prepMotor(shoulderID1);
  prepMotor(shoulderID2);
  prepMotor(ellbow);
  prepMotor(headHoizontalID);
  prepMotor(headVerticalID);
  
  st.WritePosEx(baseID, baseStart, 100);
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
  st.pSerial = &Serial1;
  delay(1000);
  initServo();

  delay(1000);
  
}

void loop(){
  //Serial.println(baseStart);

    if(st.Ping(shoulderID2) == -1){
      initServo();
    }

    while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processMessage(serialBuffer);
      serialBuffer = "";
    } else if (isPrintable(c)) {
      serialBuffer += c;
    }
  }
  setMotorSpeed();
  String msg = readMotor();
  Serial.println(msg);
  Serial.flush();
}

void processMessage(String message) {
  if (!isValidPattern(message)){
    return;
  } 

  sscanf(message.c_str(), "%i/%i/%i/%i/%i/",
         &base,
         &shoulder2,
         &ellbow,
         &headHoizontal,
         &headVertical);
  base = int(2.4 * base);
  shoulder2 *= 4;
  ellbow *= 4;
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
  if (!input.endsWith("/")) return false;

  int start = 0;
  int count = 0;

  while (start < input.length()) {
    int slashIndex = input.indexOf('/', start);
    if (slashIndex == -1) return false;

    String segment = input.substring(start, slashIndex);
    if (!isSignedInteger(segment)) return false;

    count++;
    start = slashIndex + 1;
  }

  return count == 5;
}

void setMotorSpeed() {
  int tempShoulder = convert(shoulder2, shoulder2Start, shoulder1Start);

  st.WritePosEx(baseID, base, 1000 * 4, 1000);
  st.WritePosEx(shoulderID1, tempShoulder, 1000 * 4, 1000);
  st.WritePosEx(shoulderID2, shoulder2, 1000 * 4, 1000);

  st.WritePosEx(ellbowID, ellbow, 1000 * 4, 1000);
  st.WritePosEx(headHoizontalID, headHoizontal, 4000, 1000);
  st.WritePosEx(headVerticalID, headVertical, 4000, 1000);
}

int convert(int encodedValue, int startPos1, int startPos2) {
  return startPos2 + startPos1 - encodedValue;
}

String readMotor(){
  int tempBasePos = int(st.ReadPos(baseID)/2.4);
  int tempShoulderPos1 = int(st.ReadPos(shoulderID1)/4);
  int tempShoulderPos2 = int(st.ReadPos(shoulderID2)/4);
  int tempEllbowPos = int(st.ReadPos(ellbowID)/4);
  int tempHeadHorizontalPos = st.ReadPos(headHoizontalID);
  int tempHeadVerticalPos = st.ReadPos(headVerticalID);
  //int headVerticalSpeed = int(st.ReadSpeed(headVerticalID));

  char buffer[256];
  sprintf(buffer, "%d/%d/%d/%d/%d", 
      tempBasePos,
      tempShoulderPos2,
      tempEllbowPos,
      tempHeadHorizontalPos,
      tempHeadVerticalPos);

  return String(buffer);

}

void prepMotor(int ID){
  st.unLockEprom(ID);

  uint8_t phase = st.readByte(ID, 0x12);
  phase |= 0x10;

  st.writeByte(ID, 0x09, -8192);
  st.writeByte(ID, 0x0A, 0);
  st.writeByte(ID, 0x0B, 8192);
  st.writeByte(ID, 0x0C, 0);

  st.LockEprom(ID);
}