#include "Control/Control.h"

#define ENA PB9
#define IN1 PB8
#define IN2 PB7
#define INTERRUPT_PIN PA0
#define LINEAR_RES PA7
#define POT_PIN PA6

Control control;
float pos = 3;
float setp;
float err = 100;
float vel = 45;

// Manages interruptions cau3sed by the encoder
void interruptHandler() {
  control.pulse_counter++;
  /*if(control.printing) {
    Serial.printf("pulse_counter = %i\n", control.pulse_counter);
  }*/
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(LINEAR_RES, INPUT);
  pinMode(POT_PIN, INPUT);

  analogReadResolution(12);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interruptHandler, RISING);
}

void loop() {
  // Read setpoint
  setp = analogRead(POT_PIN);
  setp = map(setp, 0, 4095, 5, 85);

  // Call PID Control function
  control.PIDControl(setp);
}
