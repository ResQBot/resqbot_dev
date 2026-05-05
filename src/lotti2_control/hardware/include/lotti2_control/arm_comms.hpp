#ifndef LOTTI2_CONTROL__ARM_COMMS_HPP
#define LOTTI2_CONTROL__ARM_COMMS_HPP

#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include "libserial/SerialPort.h"
#include "rclcpp/rclcpp.hpp"
#include "sstream"


class ArmComms {

  public:
    ArmComms() = default;

    void connect(const std::string& serial_device) {
        timeout_ms_ = 1000;
        serial_conn_.Open(serial_device);
        serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_38400);
    }


    void disconnect() {
        serial_conn_.Close();
    }


    bool connected() const {
        return serial_conn_.IsOpen();
    }


    int64_t readPos(uint8_t motor_id) {
        bool ackStatus = false;
        int64_t motor_pos;
        while (!ackStatus) {
            readRealTimeLocation(motor_id);                   // issue a query position information command for id-motor
            ackStatus = waitingForACK(pos_len, posRxBuffer);  // Wait for the motor to answer
            if (ackStatus == true) {                          // Received location information
                motor_pos = static_cast<int64_t>(
                  static_cast<uint64_t>(posRxBuffer[3]) << 48 |
                  static_cast<uint64_t>(posRxBuffer[4]) << 40 |
                  static_cast<uint64_t>(posRxBuffer[5]) << 32 |
                  static_cast<uint64_t>(posRxBuffer[6]) << 24 |
                  static_cast<uint64_t>(posRxBuffer[7]) << 16 |
                  static_cast<uint64_t>(posRxBuffer[8]) << 8 |
                  static_cast<uint64_t>(posRxBuffer[9]) << 0);
                break;
            }
            // if failed to receive location information
            // 1. Check the connection of the serial cable; 2. Check whether the motor is powered on; 3. Check the device address and baud rate
            else {
                RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Failed to get position data. Retrying ...");
            }
        }
        return (motor_pos);
    }


    int16_t readSpd(uint8_t motor_id) {
        bool ackStatus = false;
        int16_t motor_spd;
        while (!ackStatus) {
            readRealTimeSpeed(motor_id);                      // issue a query velocity information command for id-motor
            ackStatus = waitingForACK(spd_len, spdRxBuffer);  // Wait for the motor to answer
            if (ackStatus == true) {                          // Received velocity information
                motor_spd = static_cast<int16_t>(
                  static_cast<uint16_t>(spdRxBuffer[4]) << 8 |
                  static_cast<uint16_t>(spdRxBuffer[5]) << 0);
            }
            // if failed to receive velocity information
            // 1. Check the connection of the serial cable; 2. Check whether the motor is powered on; 3. Check the slave address and baud rate
            else {
                RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Failed to get velocity data. Retrying ...");
            }
        }
        return (motor_spd);
    }


    void set_arm_values(uint8_t motor_id, uint16_t speed, uint8_t accel, uint32_t position) {
        cmdTxMsg.clear();                                                 // clear old TxMsg buffer
        cmdTxBuffer[0]  = 0xFA;                                           // frame header
        cmdTxBuffer[1]  = motor_id;                                       // motor address
        cmdTxBuffer[2]  = 0xF5;                                           // function code for setting sync mode
        cmdTxBuffer[3]  = static_cast<uint8_t>((speed >> 8) & 0xFF);      // higher 8 bit speed
        cmdTxBuffer[4]  = static_cast<uint8_t>((speed >> 0) & 0xFF);      // lower 8 bit speed
        cmdTxBuffer[5]  = accel;                                          // acceleration
        cmdTxBuffer[6]  = static_cast<uint8_t>((position >> 24) & 0xFF);  // position command bit31 - bit24
        cmdTxBuffer[7]  = static_cast<uint8_t>((position >> 16) & 0xFF);  // position command bit23 - bit16
        cmdTxBuffer[8]  = static_cast<uint8_t>((position >> 8) & 0xFF);   // position command bit15 - bit8
        cmdTxBuffer[9]  = static_cast<uint8_t>((position >> 0) & 0xFF);   // position command bit7  - bit0
        cmdTxBuffer[10] = getCheckSum(cmdTxBuffer, 10);                   // Calculate checksum
        for (std::size_t i = 0; i < 5; i++) {
            cmdTxMsg[i] = cmdTxBuffer[i];  // write TxBuffer to TxMsg to be sent (conversion necessary due to LibSerial Write function)
        }
        serial_conn_.FlushIOBuffers();  // just in case
        serial_conn_.Write(cmdTxMsg);   // the serial port issues a command to read the real-time position

        // return messages are switched off, since position is polled regularly anyways
    }


    void setSync(u_int8_t mode, size_t motor_count) {
        bool ackStatus;
        uint8_t motor_id;
        for (size_t i = 0; i < motor_count; i++) {
            ackStatus = false;
            motor_id  = static_cast<uint8_t>(i) + 1;
            while (!ackStatus) {
                sendSyncMode(mode, motor_id);                       // issue a set sync mode command for all motors
                ackStatus = waitingForACK(mode_len, modeRxBuffer);  // Wait for the motors to answer
                if (ackStatus == true) {                            // Received velocity information
                    if (modeRxBuffer[4] == 1) {                     // motor confirms mode set
                        break;
                    }
                    else {  // motor error setting mode (connection is good, since data was received. Check program)
                        RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Failed to set synchronous execution mode for motor %d. Retrying ...", motor_id);
                    }
                }
                // if failed to receive mode confirmation
                // 1. Check the connection of the serial cable; 2. Check whether the motor is powered on; 3. Check the slave address and baud rate
                else {
                    RCLCPP_ERROR(rclcpp::get_logger("ArmInterface"), "Failed to set synchronous execution mode for motor %d. Retrying ...", motor_id);
                }
            }
        }
    }


    void startSync() {
        syncTxMsg.clear();                               // clear old TxMsg buffer
        syncTxBuffer[0] = 0xFA;                          // frame header
        syncTxBuffer[1] = 0x00;                          // motor address
        syncTxBuffer[2] = 0x4B;                          // function code for requesting motor position
        syncTxBuffer[3] = getCheckSum(syncTxBuffer, 3);  // Calculate checksum
        for (std::size_t i = 0; i < 4; i++) {
            syncTxMsg[i] = syncTxBuffer[i];  // write TxBuffer to TxMsg to be sent (conversion necessary due to LibSerial Write function)
        }
        serial_conn_.FlushIOBuffers();  // just in case
        serial_conn_.Write(syncTxMsg);  // the serial port issues a command to read the real-time position
        /*
        // if not all motors react to the sync action call, repeat as often as needed
        for (size_t i = 0; i <= 3; i++) {
            serial_conn_.Write(TxMsg);   // the serial port issues a command to read the real-time position
            usleep(1);
        }
        */
    }


  private:

    // for serial connection
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;  // timeout before connection error is called
    // for motor communication
    std::vector<uint8_t> cmdTxMsg;
    std::vector<uint8_t> posTxMsg;
    std::vector<uint8_t> spdTxMsg;
    std::vector<uint8_t> syncTxMsg;
    std::vector<uint8_t> modeTxMsg;
    uint8_t cmdTxBuffer[20];
    uint8_t posTxBuffer[20];
    uint8_t spdTxBuffer[20];
    uint8_t syncTxBuffer[20];
    uint8_t modeTxBuffer[20];
    uint8_t posRxBuffer[20];
    uint8_t spdRxBuffer[20];
    uint8_t modeRxBuffer[20];
    uint8_t pos_len  = 10;
    uint8_t spd_len  = 6;
    uint8_t mode_len = 5;


    void readRealTimeLocation(uint8_t device_id) {
        posTxMsg.clear();                              // clear old TxMsg buffer
        posTxBuffer[0] = 0xFA;                         // frame header
        posTxBuffer[1] = device_id;                    // motor address
        posTxBuffer[2] = 0x31;                         // function code for requesting motor position
        posTxBuffer[3] = getCheckSum(posTxBuffer, 3);  // Calculate checksum
        for (std::size_t i = 0; i < 4; i++) {
            posTxMsg[i] = posTxBuffer[i];  // write TxBuffer to TxMsg to be sent (conversion necessary due to LibSerial Write function)
        }
        serial_conn_.FlushIOBuffers();  // just in case
        serial_conn_.Write(posTxMsg);   // the serial port issues a command to read the real-time position
    }


    void readRealTimeSpeed(uint8_t device_id) {
        spdTxMsg.clear();                              // clear old TxMsg buffer
        spdTxBuffer[0] = 0xFA;                         // frame header
        spdTxBuffer[1] = device_id;                    // motor address
        spdTxBuffer[2] = 0x32;                         // function code
        spdTxBuffer[3] = getCheckSum(spdTxBuffer, 3);  // Calculate checksum
        for (std::size_t i = 0; i < 4; i++) {
            spdTxMsg[i] = spdTxBuffer[i];  // write TxBuffer to TxMsg to be sent (conversion necessary due to LibSerial Write function)
        }
        serial_conn_.FlushIOBuffers();  // just in case
        serial_conn_.Write(spdTxMsg);   // the serial port issues a command to read the real-time position
    }


    void sendSyncMode(u_int8_t mode, u_int8_t device_id) {
        modeTxMsg.clear();                               // clear old TxMsg buffer
        modeTxBuffer[0] = 0xFA;                          // frame header
        modeTxBuffer[1] = device_id;                     // motor address
        modeTxBuffer[2] = 0x4A;                          // function code for setting sync mode
        modeTxBuffer[3] = mode;                          // chose sync mode on/off
        modeTxBuffer[4] = getCheckSum(modeTxBuffer, 4);  // Calculate checksum
        for (std::size_t i = 0; i < 5; i++) {
            modeTxMsg[i] = modeTxBuffer[i];  // write TxBuffer to TxMsg to be sent (conversion necessary due to LibSerial Write function)
        }
        serial_conn_.FlushIOBuffers();  // just in case
        serial_conn_.Write(modeTxMsg);  // the serial port issues a command to read the real-time position
    }


    uint8_t getCheckSum(uint8_t* buffer, uint8_t size) {
        uint8_t i;
        uint16_t sum = 0;
        for (i = 0; i < size; i++) {
            sum += buffer[i];  // Calculate accumulated value
        }
        return (static_cast<uint8_t>(sum & 0xFF));  // return checksum
    }


    bool waitingForACK(uint8_t len, uint8_t (&rxBuffer)[]) {
        bool retVal;  // return value to flag success/error
        std::vector<uint8_t> rxMsg;
        serial_conn_.Read(rxMsg, len, 500);  // read received data
        for (std::size_t i = 0; i < len; i++) {
            rxBuffer[i] = rxMsg[i];
        }
        if (rxBuffer[0] == 0xFB) {  // check received header
            if (rxBuffer[len - 1] == getCheckSum(rxBuffer, len - 1)) {
                retVal = true;  // checksum correct
            }
            else {
                retVal = false;  // Verification error, return false
            }
        }
        else {
            retVal = false;  // wrong header
        }
        return (retVal);
    }
};
#endif  // LOTTI2_CONTROL__ARM_COMMS_HPP