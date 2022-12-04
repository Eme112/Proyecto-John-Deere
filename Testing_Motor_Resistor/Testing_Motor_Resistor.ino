#define ENA PB9
#define IN1 PB8
#define IN2 PB7
#define LINEAR_RES PA7

int val = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(LINEAR_RES, INPUT);

  analogReadResolution(12);
}

int valueRead() {
  int mean_value = 0;
  
  for(int i=0; i<1000; ++i) 
    mean_value += analogRead(LINEAR_RES);

  return mean_value / 1000;
}

double getPosition(int x) {
  return (x-3837)/2.7379;
}

void motorMovement(char input) {
  if(input == 'F') {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    Serial.println("Moving Forward");
  } else if(input == 'B') {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    Serial.println("Moving Backward");
  } else if(input == 'S') {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    Serial.println("STOP");
  } else if(input >= '0' && input <= '9') {
    val = (input-'0')*255/9;
    analogWrite(ENA, val);
    Serial.printf("Vel = %i\n", val);
  } else if(input == '?') {
    Serial.printf("LINEAR_RES = %i\n", valueRead());
  } else if(input == 'X') {
    int x = getPosition(valueRead());
    Serial.printf("Pos = %i mm\n", x);
  }
}

void loop() {
  motorMovement(Serial.read());
}
