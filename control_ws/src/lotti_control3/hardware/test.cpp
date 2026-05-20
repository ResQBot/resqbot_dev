
#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

class test {
    public:


    void send(double speedPercent){
        //SerialPort  serial("/dev/ttyACM0");
        MotorCmd    cmd;
        MotorData   data;
        double speedR = speedPercent * 6;
    
        cmd.motorType = MotorType::GO_M8010_6;
        data.motorType = MotorType::GO_M8010_6;
        //cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
        cmd.id   = 1;
        cmd.kp   = 0.0;
        cmd.kd   = 0.05;
        cmd.q    = 0.0;
        //cmd.dq   = speedR*queryGearRatio(MotorType::GO_M8010_6);
        cmd.tau  = 0.0;
        //serial.sendRecv(&cmd,&data);
        std::cout << speedR*queryGearRatio(MotorType::GO_M8010_6);
    }
};