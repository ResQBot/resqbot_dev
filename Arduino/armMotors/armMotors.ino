/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

//======================================================================================

// id of OpenCR board : usb-ROBOTIS_OpenCR_Virtual_ComPort_in_FS_Mode_FFFFFFFEFFFF-if00

//======================================================================================

#include <Dynamixel2Arduino.h>
#include <SCServo.h>
#include <SCSCL.h>
#include <SCS.h>
#include <SCSerial.h>
#include <SMS_STS.h>
#include <INST.h>

// For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
// Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.

#define SERVO_SERIAL Serial1
#define BAUDRATE     1000000
#define SERVO_ID_4     0x06  
#define SERVO_ID_5     0x07   
#define SERVO_ID_6     0x08
#define SERVO_ID_3     0x02  
//--------------------------------------------------------------------------------------

// the id of motors that are close to each other are 1 and 2 and the top motor id is 3 

//--------------------------------------------------------------------------------------
const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;
const uint8_t DXL_ID_3 = 3;
static const uint8_t MAX_MOTORS = 5;
int16_t commanded[MAX_MOTORS];

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

SMS_STS st;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

long currentPositionM1 = 0;
long currentPositionM2 = 0;
long currentPositionM3 = 0;
long currentPositionM4 = 0;
long currentPositionM5 = 0;
long currentPositionM6 = 0;
long returnedPostions[MAX_MOTORS];
float tmp[MAX_MOTORS] = {0.0,0.0,0.0,0.0,0.0};

void initializeDxlMotors() {
  dxl.begin(57600);
  
  uint8_t ids[] = {DXL_ID_1, DXL_ID_2};
  for(int i=0; i<2; i++) {
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i], OP_EXTENDED_POSITION);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, ids[i], 15);
    dxl.writeControlTableItem(PROFILE_VELOCITY, ids[i], 40); 
    dxl.torqueOn(ids[i]);
  }
  // Configure Motor 3
  dxl.torqueOff(DXL_ID_3);
  dxl.setOperatingMode(DXL_ID_3, OP_EXTENDED_POSITION);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_3, 15);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_3, 40); 
  dxl.torqueOn(DXL_ID_3);
}

void recoverSTMotors() {
  st.EnableTorque(SERVO_ID_4, 1);
  st.EnableTorque(SERVO_ID_5, 1);
  st.EnableTorque(SERVO_ID_6, 1);
}

// Generic conversion function
int32_t radToPos(float radians, int resolution) {
  return (int32_t)((radians * resolution) / (2.0 * PI));
}
void checkMotorHealth() {
  for(uint8_t id = 1; id <= 3; id++) {
    // Read Temperature (Celsius)
    int16_t temp = dxl.readControlTableItem(PRESENT_TEMPERATURE, id);
    // Read Load (0 to 1000 range, bit 10 is direction)
    int16_t load = dxl.readControlTableItem(PRESENT_LOAD, id);
    
    DEBUG_SERIAL.print("ID: "); DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" | Temp: "); DEBUG_SERIAL.print(temp);
    DEBUG_SERIAL.print("C | Load: "); DEBUG_SERIAL.println(load);

    if (temp > 65) {
      DEBUG_SERIAL.println("!!! WARNING: MOTOR OVERHEATING !!!");
    }
  }
}
void safeReboot(uint8_t id) {
  int16_t temp = dxl.readControlTableItem(PRESENT_TEMPERATURE, id);
  
  if (temp < 60) { // Only reboot if it's cooled down a bit
    dxl.reboot(id);
    delay(500); 
    initializeDxlMotors();
  } else {
  }
}
void setup() {
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  SERVO_SERIAL.begin(BAUDRATE);
  while(!DEBUG_SERIAL);

  st.pSerial = &SERVO_SERIAL;
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  // Get DYNAMIXEL information for both motors
  dxl.ping(DXL_ID_1);
  dxl.ping(DXL_ID_2);
  dxl.ping(DXL_ID_3);  
  // Configure Motor 1, 2
  uint8_t ids[] = {DXL_ID_1, DXL_ID_2};
  for(int i=0; i<2; i++) {
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i], OP_EXTENDED_POSITION);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, ids[i], 15);
    dxl.writeControlTableItem(PROFILE_VELOCITY, ids[i], 50); // 3rd value is speed, 0 disable this value, making it go as fast as possible  
    dxl.torqueOn(ids[i]);
  }
  // Configure Motor 3
  dxl.torqueOff(DXL_ID_3);
  dxl.setOperatingMode(DXL_ID_3, OP_EXTENDED_POSITION);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_3, 15);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_3, 50); 
  dxl.torqueOn(DXL_ID_3);


}

unsigned long lastCheck = 0;

void loop() {
  // if dxl motor lost his power 0.0:0.0:0.0:0.0:0.0
  if (dxl.ping(DXL_ID_1) == false) {
    delay(1000); 
    initializeDxlMotors();
    return; 
  }
  // if st motor lost his power
  if (st.ReadSpeed(SERVO_ID_4) == -1) {
    delay(500);
    recoverSTMotors();
  }

  // Read from DEBUG_SERIAL, which is the connection to your computer/serial monitor
  String rx = DEBUG_SERIAL.readStringUntil('\n');
  
  sscanf(rx.c_str(), "%f:%f:%f:%f:%f:", &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4]); 
      
  commanded[0] = radToPos(tmp[0] , 4096);
  commanded[1] = radToPos(tmp[1] , 4096);
  commanded[2] = radToPos(tmp[2] , 4096);
  commanded[3] = radToPos(tmp[3] , 4096);
  commanded[4] = radToPos(tmp[4] , 4096);

  currentPositionM1 = commanded[0];
  currentPositionM2 = 3 * commanded[1] ; 
  currentPositionM3 = -3 * commanded[1] + 4096;

  long m4_pos = -2 * commanded[2];
  m4_pos = m4_pos % 4096;
  if(m4_pos < 0) {
      m4_pos += 4096; 
  }
  currentPositionM4 = m4_pos;

  currentPositionM5 = commanded[3];
  currentPositionM6 = commanded[4];

      
  // Send commands to motors
  dxl.setGoalPosition(DXL_ID_1, currentPositionM2);
  dxl.setGoalPosition(DXL_ID_2, currentPositionM3);
  dxl.setGoalPosition(DXL_ID_3, currentPositionM4);
  st.WritePosEx(SERVO_ID_3, currentPositionM4, 100, 0);
  //WritePosEx(ID, Position, Speed, Acceleration)  0 for max value
  st.WritePosEx(SERVO_ID_4, currentPositionM6, 100, 0);
  st.WritePosEx(SERVO_ID_5, currentPositionM5, 100, 0);
  st.WritePosEx(SERVO_ID_6, currentPositionM1, 400, 0);
  
  // Format: D1:D3:S4:S5:S6
  int dxl1_pos = (int) dxl.getPresentPosition(DXL_ID_1) / 3;
  int raw_servo3_pos = (int) st.ReadPos(SERVO_ID_3);
  int signed_servo3_pos = raw_servo3_pos;
  if (signed_servo3_pos > 2048) {
      signed_servo3_pos -= 4096;
  }
  int dxl3_pos = signed_servo3_pos / -2;
  String line = "";

  line += String(st.ReadPos(SERVO_ID_6));
  line += ":";
  line += String(dxl1_pos);
  line += ":";
  line += String(dxl3_pos);
  line += ":";
  line += String(st.ReadPos(SERVO_ID_5));
  line += ":";
  line += String(st.ReadPos(SERVO_ID_4));

  DEBUG_SERIAL.println(line);

/*
  DEBUG_SERIAL.print(st.ReadPos(SERVO_ID_6));
  DEBUG_SERIAL.print(":");
  DEBUG_SERIAL.print(dxl1_pos);
  DEBUG_SERIAL.print(":");
  DEBUG_SERIAL.print(dxl3_pos);
  DEBUG_SERIAL.print(":");
  DEBUG_SERIAL.print(st.ReadPos(SERVO_ID_5));
  DEBUG_SERIAL.print(":");
  DEBUG_SERIAL.println(st.ReadPos(SERVO_ID_4));
*/
//          ---------    reboot the motor if it shutdown ------------
/*
for(uint8_t id = 1; id <= 3; id++) {
    uint8_t error = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id);
    if(error != 0) {
      safeReboot(id);
    }
  }
}
*/
}
