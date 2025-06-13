#ifndef LOTTI_CONTROL_SERIAL_COMMS_HPP
#define LOTTI_CONTROL_SERIAL_COMMS_HPP

#include <cstring>
#include <cstdlib>
#include "libserial/SerialPort.h"
#include <iostream>
#include "sstream"
#include "rclcpp/rclcpp.hpp"
#include <cstddef>



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
      //RCLCPP_INFO(rclcpp::get_logger("OUT"), msg_to_send.c_str());
    }


    void send_empty_msg(){
      send_msg("\n");
    }


    std::string read_msg(){
      std::string response = "";
      try{
        serial_conn_.ReadLine(response, '\n', timeout_ms_);
      //RCLCPP_INFO(rclcpp::get_logger("IN"), response.c_str());
      }
      catch (const LibSerial::ReadTimeout&){
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
      }
      return response;
    }


/*     void set_arm_values(int pos[6], int vel[6]){
      std::stringstream ss;
      for (size_t i = 0; i < 6; i++){
        ss << pos[i] << ":" << vel[i] << "/";
      }
      ss << "\n";
      send_msg(ss.str());
    } */

    void set_arm_values(int pos[6]){
      std::stringstream ss;
      for (size_t i = 0; i < 6; i++){
        ss << pos[i] << "/";
      }
      ss << "\n";
      send_msg(ss.str());
    }

    void set_flipper_values(int FR, int FL, int RR, int RL){
      std::stringstream ss;
      ss << "FL" << FL << "FR" << FR <<  "RL" << RL << "RR" << RR << "\n";
      send_msg(ss.str());
    }


  private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
    int vel = 0;
      
};

#endif // LOTTI_CONTROL_SERIAL_COMMS_HPP