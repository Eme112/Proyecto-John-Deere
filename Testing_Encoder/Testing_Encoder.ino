// MAX DISTANCE = 89mm
// PULSE COUNT = 26289

#define ENA PB9
#define IN1 PB8
#define IN2 PB7
#define LINEAR_RES PA7
#define INTERRUPT_PIN PA0
#define PULSES_PER_MM 270

volatile int pulse_counter = 0;
int val = 0;
bool printing = false;
bool forward;
bool backward;

int valueRead() {
  int mean_value = 0;
  
  for(int i=0; i<10; ++i) 
    mean_value += analogRead(LINEAR_RES);

  return mean_value / 10;
}

double getPosition(double y) {
  if(y < 1000) {
    return (y+41.754)/17.525;
  } else {
    return (y+4324)/91.392;
  }
}

void interruptHandler() {
  pulse_counter++;
  if(printing) {
    Serial.printf("pulse_counter = %i\n", pulse_counter);
  }
}

void moveMillis(char dir, int mm) {
  pulse_counter = 0;
  if(dir == 'F') {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if(dir == 'B') {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
    
  while(pulse_counter < PULSES_PER_MM*mm) {
    analogWrite(ENA, 220);
  }
  analogWrite(ENA, 0);
}

void printResistorValues() {
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

void motorMovement(char input) {
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
      val = (input-'0')*255/9;
      analogWrite(ENA, val);
      Serial.printf("Vel = %i\n", val);
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
  motorMovement(Serial.read());
}
