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
        // Constants
        const float pulses_per_mm = 270;
        const int   Tm = 100;   // Interval in milliseconds to measure velocity
        const float Kp = 0.4; // Proportional constant for the PID controller
        const float Ki = 0.1; // Integral constant for the PID controller
        const float Kd = 0.001; // Derivative constant for the PID controller

        // Control variables
        float cv;
        float cv1;
        float error0 = 0;
        float error1 = 0;
        float error2 = 0;

        // Other variables
        float  velocity = 0;  // Velocity measured in pulses per second
        float  position = 0;  // Position measured in millimeters
        unsigned int  vel_pwm = 0;        // Velocity PWM value
        unsigned long last_time = 0;
        bool forward = false;
        bool backward = false;
        
    public:
        int pulse_counter = 0;
        bool printing = true;

        // Constructor
        Control();
        // Read the value of the linear resistor
        int valueRead();
        // Returns the position of the actuator based on a given value
        float getPosition();
        // Updates control variables
        void updateVariables();
        // PID controller
        void PIDControl(float setpoint);
        // Validate if movement is possible
        bool validateMovement(char input);
        // Moves the actuator based on a given input
        void moveMillis(char input, int millis);
        // Prints the lecture of the linear resistor on each millimeter
        void printResistorValues();
        // Manages the movement of the actuator given a character
        void motorMovement(char input);
        // Moves the actuator to a given position
        void moveToPosition(float pos);
};

Control::Control() {

}

int Control::valueRead() {
  int mean_value = 0;
  
  for(int i=0; i<10; ++i) 
    mean_value += analogRead(LINEAR_RES);

  return mean_value / 10;
}

float Control::getPosition() {
  int y = valueRead();
  if(y < 1000) {
    return (y+41.754)/17.525;
  } else {
    return (y+4324)/91.392;
  }
}

void Control::updateVariables() {
  unsigned long current_time = millis();
  position = getPosition();

  if(current_time - last_time >= Tm) {
    velocity = pulse_counter * 1000 / (current_time - last_time);
    last_time = current_time;
    pulse_counter = 0;
    if(printing) {
      Serial.println("velocity = " + String(velocity) + " pulses/s");
      Serial.println("position = " + String(position) + " mm");
    }
  }
}

void Control::PIDControl(float setpoint) {
  char direction;
  float abs_cv;
  unsigned long current_time = millis();
  
  if(current_time - last_time >= Tm) {
    last_time = current_time;
    error0 = setpoint - getPosition();
    // Differential equation of the PID controller
    cv = (Kp + Kd/Tm)*error0 + (-Kp + Ki*Tm - 2*Kd/Tm)*error1 + (Kd/Tm)*error2;
    cv1 = cv;
    error2 = error1;
    error1 = error0;

    // Set the direction of the motor
    if(cv < 0) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      direction = 'B';
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      direction = 'F';
    }

    // If the error is small, stop the motor
    if(abs(error0) < 0.3) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      cv = 0;
    }

    abs_cv = abs(cv);

    // Limiting the velocity
    if(abs_cv > 100) {
      abs_cv = 255;
    } else if(abs_cv < 0) {
      abs_cv = 174;
    } else {
      abs_cv = map(abs_cv, 0, 100, 174, 255);
    }

    // Print values
    if(printing) {
      Serial.println("Position = " + String(getPosition()) + " mm");
      Serial.println("Setpoint = " + String(setpoint) + " mm");
      Serial.println("Error = " + String(error0) + " mm");
      Serial.println("Direction = " + String(direction));
      Serial.println("Control variable = " + String(cv));
      Serial.println("PWM = " + String(abs_cv));
      Serial.println();
    }

    analogWrite(ENA, abs_cv);
  }
}

bool Control::validateMovement(char dir) {
  if(dir == 'F') {
    if(valueRead() < 4060) {
      forward = true;
      backward = false;
      return true;
    } else {
      return false;
    }
  } else if(dir == 'B') {
    if(valueRead() > 22) {
      forward = false;
      backward = true;
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

void Control::moveMillis(char dir, int mm) {
  pulse_counter = 0;
  if(dir == 'F') {
    if(validateMovement('F')) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      while(pulse_counter < pulses_per_mm*mm) {
        analogWrite(ENA, 220);
      }
    }
  } else if(dir == 'B') {
    if(validateMovement('B')) {
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
  if(validateMovement(input)) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);  
  }
  
  switch(input) {
    case 'F':                   // Moving Forward
      if(validateMovement('F')) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        Serial.println("Moving Forward");
      }
      break;
    case 'B':                   // Moving Backward
      if(validateMovement('B')) {
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
    case '0'...'9':             // Change vel_pwm
      vel_pwm = (input-'0')*255/9;
      analogWrite(ENA, vel_pwm);
      Serial.printf("Vel = %i\n", vel_pwm);
      break;
    case '?':                   // Read resistance
      Serial.printf("LINEAR_RES = %i\n", valueRead());
      break;
    case 'X':                   // Estimate position
      float pos;
      pos = getPosition();
      Serial.println("Pos = " + String(pos) + " mm");
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

void Control::moveToPosition(float pos) {
  float current_pos = getPosition();
  Serial.println("current_pos = " + String(current_pos) + "\ttargeted_pos = " + String(pos));
  if(current_pos < pos) {
    moveMillis('F', pos-current_pos);
  } else if(current_pos > pos) {
    moveMillis('B', current_pos-pos);
  }
}