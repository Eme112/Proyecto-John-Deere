// Code made by Emérico Pedraza and Renato Arcos in 2022 for the Linear Actuator Project.
//
// This class is intended to implement CAN communication between the STM32F401 and the ESP32.

#include <SPI.h>
#include <mcp2515.h>

#ifndef CAN_h
#define CAN_h
#endif

#include "Arduino.h"

class CAN {
    private:
        struct can_frame can_msg_variables;
        struct can_frame can_msg_setpoint;
    public:
        // Constructor
        CAN(MCP2515 &mcp2515);
        // Send setpoint
        void sendSetpoint(MCP2515 &mcp2515, uint8_t setpoint);
        // Receive setpoint
        void receiveSetpoint(MCP2515 &mcp2515, uint8_t &setpoint);
        // Send position, setpoint, error and velocity
        void sendVariables(MCP2515 &mcp2515, uint8_t position, uint8_t setpoint, uint8_t error, uint8_t velocity);
        // Receive position, setpoint, error and velocity
        void receiveVariables(MCP2515 &mcp2515, uint8_t &position, uint8_t &setpoint, uint8_t &error, uint8_t &velocity);
};

CAN::CAN(MCP2515 &mcp2515) {
    mcp2515 = mcp2515;
    mcp2515.reset();
    mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
}

void CAN::sendSetpoint(MCP2515 &mcp2515, uint8_t setpoint) {
    // id 0x01 is for sending setpoint
    can_msg_setpoint.can_id = 0x01;
    can_msg_setpoint.can_dlc = 1;
    can_msg_setpoint.data[0] = setpoint;
    mcp2515.sendMessage(&can_msg_setpoint);
}

void CAN::receiveSetpoint(MCP2515 &mcp2515, uint8_t &setpoint) {
    if (mcp2515.readMessage(&can_msg_setpoint) == MCP2515::ERROR_OK) {
        // id 0x01 is for receiving setpoint
        if (can_msg_setpoint.can_id == 0x01) {
            // Receive setpoint
            setpoint = can_msg_setpoint.data[0];
        }
    }
}

void CAN::sendVariables(MCP2515 &mcp2515, uint8_t position, uint8_t setpoint, uint8_t error, uint8_t velocity) {
    // id 0x02 is for sending variables
    can_msg_variables.can_id = 0x02;
    can_msg_variables.can_dlc = 4;
    can_msg_variables.data[0] = position;
    can_msg_variables.data[1] = setpoint;
    can_msg_variables.data[2] = error;
    can_msg_variables.data[3] = velocity;
    mcp2515.sendMessage(&can_msg_variables);
}

void CAN::receiveVariables(MCP2515 &mcp2515, uint8_t &position, uint8_t &setpoint, uint8_t &error, uint8_t &velocity) {
    if (mcp2515.readMessage(&can_msg_variables) == MCP2515::ERROR_OK) {
        // id 0x02 is for receiving variables
        if (can_msg_variables.can_id == 0x02) {
            // Receive position, setpoint, error and velocity
            position = can_msg_variables.data[0];
            setpoint = can_msg_variables.data[1];
            error = can_msg_variables.data[2];
            velocity = can_msg_variables.data[3];
        }
    }
}