#ifndef LOTTI_CONTROL_FLIPPER_COMMS_HPP
#define LOTTI_CONTROL_FLIPPER_COMMS_HPP

#include <cstring>
#include <sstream>
#include <cstdlib>
#include "libserial/SerialPort.h"
#include <iostream>
#include "sstream"


class SerialComms{

    public:

        SerialComms() = default;

        void connect(){  
            timeout_ms_ = 1000;
            serial_conn_.Open("/dev/ttyACM0");
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


        void set_flipper_values(int FR, int FL, int RR, int RL)
        {
            std::ostringstream ss;
            ss << "FL" << FL << "FR" << FR <<  "RL" << RL << "RR" << RR << "\n";
            send_msg(ss.str());
        }


    private:
        LibSerial::SerialPort serial_conn_;
        int timeout_ms_;
};

#endif // LOTTI_CONTROL_FLIPPER_COMMS_HPP