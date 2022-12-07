//#include "Control/Control.h"
#include <SPI.h>
#include <mcp2515.h>

#define SCK2 PB13
#define SI2  PB15
#define SO2  PB14
#define CS2  PB12

//Control control;

// CAN communication variables
MCP2515 mcp_stm32(CS2);
struct can_frame can_msg_variables;
struct can_frame can_msg_setpoint;
uint8_t pos = 8;
uint8_t setpoint = 40;
uint8_t error = 9;
uint8_t velocity = 10;

// Manages interruptions caused by the encoder
/*void interruptHandler() {
  control.pulse_counter++;
  /*if(control.printing) {
    Serial.printf("pulse_counter = %i\n", control.pulse_counter);
  }
}*/

void initializeMcp() {
    mcp_stm32.reset();
    mcp_stm32.setBitrate(CAN_125KBPS);
    mcp_stm32.setNormalMode();
}

void receiveSetpoint() {
    if (mcp_stm32.readMessage(&can_msg_setpoint) == MCP2515::ERROR_OK) {
        // id 0x01 is for receiving setpoint
        if (can_msg_setpoint.can_id == 0x01) {
            // Receive setpoint
            setpoint = can_msg_setpoint.data[0];
            Serial.println("Setpoint recieved: " + String(setpoint));
        } else {
          Serial.println("Setpoint not recieved");
        }
    } else {
      Serial.println("Setpoint not recieved");
    }
}

void sendVariables() {
    // id 0x02 is for sending variables
    can_msg_variables.can_id = 0x02;
    can_msg_variables.can_dlc = 4;
    can_msg_variables.data[0] = pos;
    can_msg_variables.data[1] = setpoint;
    can_msg_variables.data[2] = error;
    can_msg_variables.data[3] = velocity;
    Serial.println("Variables sent");
    mcp_stm32.sendMessage(&can_msg_variables);
}

void setup() {
  Serial.begin(115200);
  initializeMcp();
  
  //pinMode(ENA, OUTPUT);
  //pinMode(IN1, OUTPUT);
  //pinMode(IN2, OUTPUT);
  //pinMode(LINEAR_RES, INPUT);
  pinMode(SCK2, OUTPUT);
  pinMode(SI2,  OUTPUT);
  pinMode(SO2,  INPUT);
  pinMode(CS2,  OUTPUT);

  //analogReadResolution(12);
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interruptHandler, RISING);
}

void loop() {
  //control.PIDControl(setpoint);
  receiveSetpoint();
  sendVariables();
}
