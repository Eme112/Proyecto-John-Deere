#include "Control/Control.h"

#define ENA PB10
#define IN1 PB9
#define IN2 PB8
#define LINEAR_RES PA1
#define INTERRUPT_PIN PA0

// Definitions for conversions
#define BIT7 PA2
#define BIT6 PA3
#define BIT5 PA4
#define BIT4 PA5
#define BIT3 PA6
#define BIT2 PA7
#define BIT1 PA8
#define BIT0 PA9
#define SEL1 PA10
#define SEL0 PA11
#define SETP7 PB7
#define SETP6 PB6
#define SETP5 PB5
#define SETP4 PB4
#define SETP3 PB3
#define SETP2 PB12
#define SETP1 PB1
#define SETP0 PB0

Control control;
uint8_t pos = 3;
uint8_t setp;
uint8_t err = 100;
uint8_t vel = 45;

// Manages interruptions cau3sed by the encoder
void interruptHandler() {
  control.pulse_counter++;
  /*if(control.printing) {
    Serial.printf("pulse_counter = %i\n", control.pulse_counter);
  }*/
}

void parallelRead() {
  uint8_t num = 0;
  
  // Read bits
  bool b7 = digitalRead(BIT7);
  bool b6 = digitalRead(BIT6);
  bool b5 = digitalRead(BIT5);
  bool b4 = digitalRead(BIT4);
  bool b3 = digitalRead(BIT3);
  bool b2 = digitalRead(BIT2);
  bool b1 = digitalRead(BIT1);
  bool b0 = digitalRead(BIT0);
  bool s1 = digitalRead(SEL1);
  bool s0 = digitalRead(SEL0);

  if(b7) num += 128;
  if(b6) num += 64;
  if(b5) num += 32;
  if(b4) num += 16;
  if(b3) num += 8;
  if(b2) num += 4;
  if(b1) num += 2;
  if(b0) num += 1;

  if(!s1 && !s0) pos = num;
  if(!s1 && s0) setp = num;
  if(s1 && !s0) err = num;
  if(s1 && s0) vel = num;
}

void readSetpoint() {
  // Read bits
  Serial.println("Starting to read data");
  bool b7 = digitalRead(SETP7);
  bool b6 = digitalRead(SETP6);
  bool b5 = digitalRead(SETP5);
  bool b4 = digitalRead(SETP4);
  bool b3 = digitalRead(SETP3);
  bool b2 = digitalRead(SETP2);
  bool b1 = digitalRead(SETP1);
  bool b0 = digitalRead(SETP0);

  uint8_t num = 0;
  if(b7) num += 128;
  if(b6) num += 64;
  if(b5) num += 32;
  if(b4) num += 16;
  if(b3) num += 8;
  if(b2) num += 4;
  if(b1) num += 2;
  if(b0) num += 1;

  setp = num;
}

void parallelWrite(uint8_t num, char sel) {
  // Write bits
  digitalWrite(BIT7, num & 128);
  digitalWrite(BIT6, num & 64);
  digitalWrite(BIT5, num & 32);
  digitalWrite(BIT4, num & 16);
  digitalWrite(BIT3, num & 8);
  digitalWrite(BIT2, num & 4);
  digitalWrite(BIT1, num & 2);
  digitalWrite(BIT0, num & 1);

  // Select register
  if(sel == 'p') {
    digitalWrite(SEL1, LOW);
    digitalWrite(SEL0, LOW);
  } else if(sel == 'e') {
    digitalWrite(SEL1, LOW);
    digitalWrite(SEL0, HIGH);
  } else if(sel == 'v') {
    digitalWrite(SEL1, HIGH);
    digitalWrite(SEL0, LOW);
  }
}

void writeSetpoint() {
  // Write bits
  digitalWrite(SETP7, setp & 128);
  digitalWrite(SETP6, setp & 64);
  digitalWrite(SETP5, setp & 32);
  digitalWrite(SETP4, setp & 16);
  digitalWrite(SETP3, setp & 8);
  digitalWrite(SETP2, setp & 4);
  digitalWrite(SETP1, setp & 2);
  digitalWrite(SETP0, setp & 1);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Void setup achieved");
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(LINEAR_RES, INPUT);
  
  /*pinMode(BIT7, OUTPUT);
  pinMode(BIT6, OUTPUT);
  pinMode(BIT5, OUTPUT);
  pinMode(BIT4, OUTPUT);
  pinMode(BIT3, OUTPUT);
  pinMode(BIT2, OUTPUT);
  pinMode(BIT1, OUTPUT);
  pinMode(BIT0, OUTPUT);
  pinMode(SEL1, OUTPUT);
  pinMode(SEL0, OUTPUT);*/
  pinMode(SETP7, INPUT);
  pinMode(SETP6, INPUT);
  pinMode(SETP5, INPUT);
  pinMode(SETP4, INPUT);
  pinMode(SETP3, INPUT);
  pinMode(SETP2, INPUT);
  pinMode(SETP1, INPUT);
  pinMode(SETP0, INPUT);

  Serial.println("End of pinModes");

  analogReadResolution(12);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interruptHandler, RISING);
}

void loop() {
  // Read setpoint
  readSetpoint();
  Serial.println("Setpoint: " + String(setp));

  // Call PID Control function
  control.PIDControl(setp);

  // Get results
  pos = control.getPosition();
  err = control.getError();
  vel = control.getVelocity();

  // Send results
  parallelWrite(pos, 'p');
  delay(1);
  parallelWrite(err, 'e');
  delay(1);
  parallelWrite(vel, 'v');
  delay(1);
}
