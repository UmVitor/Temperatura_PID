#include <PID_v1.h>
#include "DHT.h" 
#define PIN_INPUT A1
#define PIN_OUTPUT 11
#define DHTTYPE DHT11                          
#define pinBotao 7
#define LED1 4
#define LED2 5
#define LED3 6



bool ligado = true;
bool estadoBotao = true;
bool estadoAntBotao = true;
unsigned long delayBotao;

int saida;

 
DHT dht(PIN_INPUT, DHTTYPE); 

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

void setup()
{
  //initialize the variables we're linked to
  Serial.begin(9600);
  dht.begin();
  Input = dht.readTemperature();
  Setpoint = 27;
  pinMode(pinBotao, INPUT_PULLUP);
  pinMode(PIN_OUTPUT, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  
  estadoBotao = digitalRead(pinBotao);
  if (!estadoBotao && estadoAntBotao) {
     if ((millis() - delayBotao) > 20) {
        ligado = !ligado;
        delayBotao = millis();
     }
  }
  estadoAntBotao = estadoBotao;
  
  

  saida = (int) Output;

  if (ligado) {
     analogWrite(PIN_OUTPUT, saida); 
     Input = dht.readTemperature();
     myPID.Compute();
     Serial.print("Temperatura: ");                
     Serial.print(Input);                   
     Serial.println("*C\n");   
     Serial.print(saida);
     Serial.println("\n");
  } else {
     analogWrite(PIN_OUTPUT, 0);
  } 
  if (saida == (int) Output )  { // Sistema estabilizado
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
  }
  if(saida > (int) Output || saida == 0){ //Sistema aquecendo
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
    digitalWrite(LED3, LOW);
   }
  if(saida < (int) Output || saida == 255){ //Sistema esfriando
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, HIGH);
   }
 }
