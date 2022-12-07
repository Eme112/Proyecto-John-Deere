#include <SPI.h>
#include <mcp2515.h>

#define MCP 2

// CAN communication variables
MCP2515 mcp_esp32(MCP);
struct can_frame can_msg_variables;
struct can_frame can_msg_setpoint;
uint8_t pos;
uint8_t setpoint = 40;
uint8_t error;
uint8_t velocity;

void sendSetpoint(uint8_t setpoint) {
    // id 0x01 is for sending setpoint
    can_msg_setpoint.can_id = 0x01;
    can_msg_setpoint.can_dlc = 1;
    can_msg_setpoint.data[0] = setpoint;
    Serial.println("Setpoint sent: " + String(can_msg_setpoint.data[0]));
    mcp_esp32.sendMessage(&can_msg_setpoint);
}

void receiveVariables() {
    if (mcp_esp32.readMessage(&can_msg_variables) == MCP2515::ERROR_OK) {
        // id 0x02 is for receiving variables
        if (can_msg_variables.can_id == 0x02) {
            // Receive position, setpoint, error and velocity
            pos = can_msg_variables.data[0];
            setpoint = can_msg_variables.data[1];
            error = can_msg_variables.data[2];
            velocity = can_msg_variables.data[3];
        }
    }
}

void setup() {
  Serial.begin(115200);

  // Initnializing MCP
  mcp_esp32.reset();
  mcp_esp32.setBitrate(CAN_125KBPS);
  mcp_esp32.setNormalMode();
}

void loop() {
  for(int i=0; i<255; i++) {
    setpoint = i;
    sendSetpoint(setpoint);
    receiveVariables();
    Serial.println("Position = " + String(pos));
    Serial.println("Setpoint = " + String(setpoint));
    Serial.println("Error = " + String(error));
    Serial.println("Velocity = " + String(velocity));
    Serial.println(); 
  }
}
