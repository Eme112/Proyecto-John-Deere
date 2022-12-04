//https://www.youtube.com/watch?v=bl2k6eXDAGM

float resis = 1;
int pinA = 3;
volatile int contador = 0;
unsigned long previousMillis = 0;
long interval = 1000;

float cv;
float cv1;
float error;
float error1;
float error2;

float Kp = 1;
float Ki = 1;
float Kd = 0.01;
float Tm = 0.1;

void setup() {
  pinMode(pinA,INPUT);
  Serial.begin(115200);
  attachInterrupt(1,interrupcion,RISING);
}

void loop() {
  
unsigned long currentMilis = millis();
  if((currentMilis - previousMillis) >= interval)
  {
    previousMillis = currentMilis;
    pv = contador * (60.0 / 408.0);   //RPM eje principal
    contador = 0;
  }

//----------SET POINT--------------
  sp = analogRead(Pot_sp)     //Resistencia lineal
  error = sp - pv;

//--------ECUACION DE DIFERENCIAS----------------
cv = cv1 + (Kp + Kd/Tm)*error + (-Kp + Ki*Tm - 2*Kd/Tm)*error1 + (Kd/Tm)*error2;
cv1 = cv;
error2 = error1;
error1 = error;

//-------SARURACION DEL PID---------------------
if (cv > 4080.0){
  cv = 4080.0;
}

if (cv < 1600.0){
  cv = 1600.0;
}

}

void interrupcion() {
  contador++;
}
