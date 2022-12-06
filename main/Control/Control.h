// Code made by Emérico Pedraza and Renato Arcos in 2022 for the Linear Actuator Project.
//
// This class is intended to manually control the actuator's movement.
// It also has some functions to return position and lecture of signals.

#ifndef Control_h
#define Control_h
#endif

#define ENA PB9
#define IN1 PB8
#define IN2 PB7
#define LINEAR_RES PA7
#define INTERRUPT_PIN PA0

#include "Arduino.h"

class Control {
    private:
        const float pulses_per_mm = 270;
        uint8_t velocity = 0;
        bool forward;
        bool backward;
    public:
        float pulse_counter = 0;
        bool printing = false;
        // Constructor
        Control();
        // Read the value of the linear resistor
        int valueRead();
        // Returns the position of the actuator based on a given value
        float getPosition(float y);
        // Manages interruptions caused by the encoder
        void interruptHandler();
        // Moves the actuator based on a given input
        void moveMillis(char input, int millis);
        // Prints the lecture of the linear resistor on each millimeter
        void printResistorValues();
        // Manages the movement of the actuator given a character
        void motorMovement(char input);
        // Moves the actuator to a given position
        void moveToPosition(int pos);
};

Control::Control() {
  velocity = 0;
  pulse_counter = 0;
  printing = false;
  forward = false;
  backward = false;
}

int Control::valueRead() {
  int mean_value = 0;
  
  for(int i=0; i<10; ++i) 
    mean_value += analogRead(LINEAR_RES);

  return mean_value / 10;
}

float Control::getPosition(float y) {
  if(y < 1000) {
    return (y+41.754)/17.525;
  } else {
    return (y+4324)/91.392;
  }
}

void Control::interruptHandler() {
  pulse_counter++;
  if(printing) {
    Serial.printf("pulse_counter = %i\n", pulse_counter);
  }
}

void Control::moveMillis(char dir, int mm) {
  pulse_counter = 0;
  if(dir == 'F') {
    if(valueRead() < 4060) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      while(pulse_counter < pulses_per_mm*mm) {
        analogWrite(ENA, 220);
      }
    }
  } else if(dir == 'B') {
    if(valueRead() > 25) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      while(pulse_counter < pulses_per_mm*mm) {
        analogWrite(ENA, 220);
      }
    }
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, 0);
}

void Control::printResistorValues() {
  int mm = 0;
  int val = valueRead();

  for(mm=0; mm<=89; mm++) {
    val = valueRead();
    Serial.printf("%i\t%i\n", mm, val);
    delay(100);
    if(Serial.read() == 'S') break;
    moveMillis('F',1);
  }
}

void Control::motorMovement(char input) {
  if((forward && valueRead() > 4060)
     || (backward && valueRead() < 26)) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);  
  }
  switch(input) {
    case 'F':                   // Moving Forward
      if(valueRead() < 4060) {
        forward = true;
        backward = false;
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        Serial.println("Moving Forward");
      }
      break;
    case 'B':                   // Moving Backward
      if(valueRead() > 25) {
        forward = false;
        backward = true;
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        Serial.println("Moving Backward");
      }
      break;
    case 'S':                   // Stop
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      Serial.println("STOP");
      break;
    case '0'...'9':             // Change velocity
      velocity = (input-'0')*255/9;
      analogWrite(ENA, velocity);
      Serial.printf("Vel = %i\n", velocity);
      break;
    case '?':                   // Read resistance
      Serial.printf("LINEAR_RES = %i\n", valueRead());
      break;
    case 'X':                   // Estimate position
      int pos;
      pos = getPosition(float(valueRead()));
      Serial.printf("Pos = %i mm\n", pos);
      break;
    case 'P':                  // Print or not print
      printing = !printing;
      if(printing) Serial.println("Printing pulses");
      else Serial.println("Not printing pulses");
      break;
    case 'R':                   // Reset pulse counter
      pulse_counter = 0;
      Serial.printf("pulse_counter = %i\n", pulse_counter);
      break;
    case '.':                   // Print resistor values
      printResistorValues();
      break;
  }
}

void Control::moveToPosition(int pos) {
  int current_pos = getPosition(valueRead());
  Serial.printf("current_pos = %i\ttargeted_pos = %i\n", current_pos, pos);
  if(current_pos < pos) {
    moveMillis('F', pos-current_pos);
  } else if(current_pos > pos) {
    moveMillis('B', current_pos-pos);
  }
}