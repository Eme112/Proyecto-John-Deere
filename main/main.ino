#include "Control/Control.h"

#define ENA PB9
#define IN1 PB8
#define IN2 PB7
#define LINEAR_RES PA7
#define INTERRUPT_PIN PA0

Control control;

void interruptHandler() {
  control.pulse_counter++;
  if(control.printing) {
    Serial.printf("pulse_counter = %i\n", control.pulse_counter);
  }
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
  control.motorMovement(Serial.read());
  control.moveToPosition(70);
}
