#include "Control/Control.h"

#define ENA PB9
#define IN1 PB8
#define IN2 PB7
#define LINEAR_RES PA7
#define INTERRUPT_PIN PA0

Control control;

// Manages interruptions caused by the encoder
void interruptHandler() {
  control.pulse_counter++;
  /*if(control.printing) {
    Serial.printf("pulse_counter = %i\n", control.pulse_counter);
  }*/
}

// Read SetPoint via serial based on syntax (setpoint)
float readSetpoint() {
  float setpoint = 0;
  float multiplier;
  char curr_char;
  int state = 0;
  
  while(Serial.available()) {
    curr_char = Serial.read();
    switch(state) {
      case 0:
        if(curr_char == '(') {
          state = 1;
        } else {
          Serial.println("Invalid syntax, returning 50");
          state = 3;
        }
        break;
      case 1:
        if(curr_char >= '0' && curr_char <= '9') {
          setpoint *= 10;
          setpoint += (curr_char - '0');
        } else if(curr_char == '.') {
          state = 2;
          multiplier = 0.1;
        } else if(curr_char == ')') {
          return setpoint;
        } else {
          Serial.println("Invalid syntax, returning 50");
          state = 3;
        }
        break;
      case 2:
        if(curr_char >= '0' && curr_char <= '9') {
          setpoint += (curr_char - '0')*multiplier;
          multiplier /= 10;
        } else if(curr_char == ')') {
          return setpoint;
        } else {
          Serial.println("Invalid syntax, returning 50");
          state = 3;
        }
        break;
      case 3:
        while(Serial.available()) {
          Serial.read();
        }
        return 50;
        break;
    }
  }

  return 50;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(LINEAR_RES, INPUT);

  analogReadResolution(12);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interruptHandler, RISING);
}

void loop() {
  float setpoint = 45;
  if(Serial.available()) {
    setpoint = readSetpoint();
  }
  control.PIDControl(setpoint);
}
