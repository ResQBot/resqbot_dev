// original code by Legged AI Lab (SII) https://github.com/well-robotics
// modified by Res.Q Bots

#ifndef CAN_INTERFACE_HPP
#define CAN_INTERFACE_HPP

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
// CAN connection
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "lotti2_control/cubeMars_motor.h"
#include "lotti2_control/lotti2_drive_interface.hpp"


class CANInterface {

  public:
    CANInterface() = default;

    void connect(const char* socketName) {
        // const char* socketIfName = &socketName;
        // int s;  // File descriptor for the socket as everything in Linux/Unix is a file.
        struct sockaddr_can addr;  // structure for CAN sockets : address family number AF_CAN
        struct ifreq ifr;          // from if.h Interface Request structure used for all socket ioctl's. All interface ioctl's must have parameter definitions which begin with ifr name. The remainder may be interface specific.

        int loopback = 0; /* 0 = disabled, 1 = enabled (default) */
        // can_filter rfilter.can_id = 0

        // socket(int domain, int type, int protocol): returns file descriptor int or -1 if fail
        if ((socket_descrp_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("CANInterface: Error While Opening CAN Socket");
        }
        else {
            // If socket was created successfully, apply the can filter for only receiving from motor and not from master.
            // setsockopt(socket_descrp_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

            setsockopt(socket_descrp_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

            // Retrieve the interface index for the interface name (can0, can1, vcan0) to be used to the ifreq struct
            strcpy(ifr.ifr_name, socketName);

            // Send an I/O control call and pass an ifreq structure containing the interface name
            // ioctl() system call manipulates the underlying device parameters of special files.
            // SIOCGIFINDEX Retrieve the interface index of the interface into ifr_ifindex inside ifr struct.
            ioctl(socket_descrp_, SIOCGIFINDEX, &ifr);

            // with the interface index, now bind the socket to the CAN Interface
            // struct sockaddr_can addr;

            // set address to all zeros. Done in example/man pages. But why?
            memset(&addr, 0, sizeof(addr));

            // Setup the interface parameters in the socketcan address struct
            addr.can_family  = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(socket_descrp_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
                perror("CANInterface: Error while binding to the CAN Socket.");
            }
            else {
                std::cout << "The Socket Descriptor is: " << socket_descrp_ << std::endl;
            }
        }
    }

    void disconnect() {
        if (close(socket_descrp_) < 0) {
            perror("CANInterface: Error while disconnecting the CAN Socket.");
        }
    }

    void sendSpd(motorCommand commands_[2]) {
        struct canxl_frame frame_;
        uint8_t len = 4;
        // construct frame
        for (std::size_t i = 0; i < 2; i++) {
            frame_.prio = commands_[i].motor_id | (static_cast<uint32_t>(CAN_PACKET_SET_RPM) << 8);
            frame_.len  = len;
            for (int ind = 0; i < len; i++) {
                frame_.data[ind] = static_cast<__u8>(commands_[i].speed >> 24);
                frame_.data[ind] = static_cast<__u8>(commands_[i].speed >> 16);
                frame_.data[ind] = static_cast<__u8>(commands_[i].speed >> 8);
                frame_.data[ind] = static_cast<__u8>(commands_[i].speed);
            }
            if (write(socket_descrp_, &frame_, sizeof(struct canxl_frame)) != sizeof(struct canxl_frame)) {
                perror("CANInterface: Error writing to CAN Interface.");
            }
        }
    }

    void receiveCANFrame(motorState states_[2]) {
        struct canxl_frame RxMessage;
        // Listen to all CAN messages
        while (read(socket_descrp_, &RxMessage, sizeof(struct canxl_frame)) > 0) {
            uint32_t func = RxMessage.prio >> 8;
            // filter only those that carry motor states
            if (func == 0x29) {
                uint8_t motor_id = RxMessage.prio & 0xff;
                // filter motor states by motor id
                for (std::size_t i = 0; i < 2; i++) {
                    if (motor_id == states_[i].motor_id) {
                        // int16_t pos_int = (RxMessage.data[0] << 8 | RxMessage.data[1]);
                        int16_t spd_int = (RxMessage.data[2] << 8 | RxMessage.data[3]);
                        int16_t cur_int = (RxMessage.data[4] << 8 | RxMessage.data[5]);
                        //*motor_pos      = static_cast<double>(pos_int * 0.1f);   // motor position
                        states_[i].velocity   = static_cast<double>(spd_int) * 10;    // motor velocity in eRPM
                        states_[i].current    = static_cast<double>(cur_int) * 0.01;  // motor current in A
                        states_[i].motor_temp = RxMessage.data[6];                    // motor temperature in °C
                        states_[i].error_code = RxMessage.data[7];                    // motor error code
                    }
                }
            }
        }
    }


  private:
    int socket_descrp_;  // File descriptor for the socket as everything in Linux/Unix is a file.
    // Pre-allocate memory for CAN messages which are overwritten by functions.
    unsigned char CAN_msg_[8];
    unsigned char CAN_reply_msg_[8];
    // unsigned int motorReplyWaitTime = 1;  // time the motor needs to send a reply [micro seconds]

    typedef enum {
        CAN_PACKET_SET_DUTY = 0,       // Duty Cycle Mode
        CAN_PACKET_SET_CURRENT,        // Current Loop Mode
        CAN_PACKET_SET_CURRENT_BRAKE,  // Current Brake Mode
        CAN_PACKET_SET_RPM,            // RPM Mode
        CAN_PACKET_SET_POS,            // Position Mode
        CAN_PACKET_SET_ORIGIN_HERE,    // Set Origin Mode
        CAN_PACKET_SET_POS_SPD,        // Position-Velocity Loop Mode
        CAN_PACKET_SET_MIT = 8,        // MIT mode
    } CAN_PACKET_ID;
};

#endif  // CAN_INTERFACE_HPP
