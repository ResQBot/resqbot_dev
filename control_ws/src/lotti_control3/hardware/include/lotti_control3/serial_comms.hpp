#ifndef lotti_control3_SERIAL_COMMS_HPP
#define lotti_control3_SERIAL_COMMS_HPP

#include <cstring>
#include <sstream>
#include <cstdlib>
#include "libserial/SerialPort.h"
#include <iostream>
#include "sstream"


class SerialComms{

    public:

        SerialComms() = default;

        void connect(const std::string &serial_device){  
            timeout_ms_ = 1000;
            serial_conn_.Open(serial_device);
            serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }


        void disconnect(){
            serial_conn_.Close();
        }


        bool connected() const{
            return serial_conn_.IsOpen();
        }


        void send_msg(const std::string &msg_to_send){
            serial_conn_.FlushIOBuffers(); // Just in case
            serial_conn_.Write(msg_to_send);
        }


        void send_empty_msg(){
            send_msg("\n");
        }


        std::string read_msg(){
            std::string response = "";

            try{
                serial_conn_.ReadLine(response, '\n', timeout_ms_);
            }
            catch (const LibSerial::ReadTimeout&){
                std::cerr << "The ReadByte() call has timed out." << std::endl ;
            }
            return response;
        }


        void set_arm_values(std::vector<double> pos){
            std::stringstream ss;
            for (size_t i = 0; i < 5; i++){
                ss << pos[i] << ":";
            }
            ss << "\n";
            //std::cout << ss.str();
            send_msg(ss.str());
        }

        void set_flipper_values(int FL, int FR, int RL, int RR){
            std::stringstream ss;
            ss << "FL" << RR << "FR" << FR <<  "RL" << RL << "RR" << FL << "\n";
            send_msg(ss.str());
        }

    private:
        LibSerial::SerialPort serial_conn_;
        int timeout_ms_;
};

#endif // lotti_control3_SERIAL_COMMS_HPP