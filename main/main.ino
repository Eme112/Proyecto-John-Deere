#include "Control/Control.h"
#include "CAN/CAN.h"

Control control;

// Manages interruptions caused by the encoder
void interruptHandler() {
  control.pulse_counter++;
  /*if(control.printing) {
    Serial.printf("pulse_counter = %i\n", control.pulse_counter);
  }*/
}

void setup() {
  Serial.begin(115200);
  
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
