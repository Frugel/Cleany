#include <Ultrasonic.h>
#include <Servo.h>
#include <IRremote.h>


int pos = 0;  // analog pin used to connect the potentiometer
int dis = 20;

Ultrasonic ultrasonic(11,12, 150 * 58); // (Trig PIN,Echo PIN, Max.TimeOut in µsec )
Servo servo1;  // Crea un Objeto servo

//Motores
#define ENA 7             // Potencia motor A
#define IN1 6            // Control 1 motor A
#define IN2 5            // Control 2 motor A
#define ENB 2             // Potencia motor B
#define IN3 4             // Control 1 motor B
#define IN4 3             // Control 2 motor A

int RECV_PIN = 9;
IRrecv irrecv(RECV_PIN);
decode_results results;
#define btnAdelante  16712445
#define btnAtras  16750695
#define btnDerecha  16748655
#define btnIzquierda  16769055
#define btnParada  16754775

#define btnAuto 16753245

int velDeMarchaIzq = 150;
int velDeMarchaDer = velDeMarchaIzq+15;
int velDeGiroIzq = 130;
int velDeGiroDer = velDeGiroIzq+15;
int val;    // variable to read the value from the analog pin 

void setup() { 
  Serial.begin (9600);

  servo1.attach(10);  // Selecionamos el pin 2 como el pin de control para

  //Motores
  pinMode(ENA, OUTPUT);     
  pinMode(IN1, OUTPUT);     
  pinMode(IN2, OUTPUT);     
  pinMode(ENB, OUTPUT);     
  pinMode(IN3, OUTPUT);     
  pinMode(IN4, OUTPUT);

  irrecv.enableIRIn(); // Start the receiver  
} 

void adelante()
{
  Serial.println("adelante");
  
  analogWrite(ENA, velDeMarchaIzq);
  digitalWrite(IN1, LOW);   
  digitalWrite(IN2, HIGH);   
  
  analogWrite(ENB, velDeMarchaDer);   
  digitalWrite(IN3, LOW);   
  digitalWrite(IN4, HIGH); 
}

void atras()
{
  Serial.println("atras");

  analogWrite(ENA, velDeMarchaIzq);
  digitalWrite(IN1, HIGH);   
  digitalWrite(IN2, LOW);   
  
  analogWrite(ENB, velDeMarchaDer);   
  digitalWrite(IN3, HIGH);   
  digitalWrite(IN4, LOW);
}

void parada()
{
  Serial.println("parada");

  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);   
  digitalWrite(IN2, LOW);  
  
  analogWrite(ENB, 0);   
  digitalWrite(IN3, LOW);   
  digitalWrite(IN4, LOW);
}

void derecha()
{
  Serial.println("derecha");
  
  analogWrite(ENA, velDeGiroDer);
  digitalWrite(IN1, LOW);   
  digitalWrite(IN2, HIGH);
  
  analogWrite(ENB, velDeGiroIzq);   
  digitalWrite(IN3, HIGH);   
  digitalWrite(IN4, LOW);  
}

void izquierda()
{
  Serial.println("izquierda");
  
  analogWrite(ENA, velDeGiroDer);
  digitalWrite(IN1, HIGH);   
  digitalWrite(IN2, LOW); 
  
  analogWrite(ENB, velDeGiroIzq);   
  digitalWrite(IN3, LOW);   
  digitalWrite(IN4, HIGH);
}

void autoModo1()
{
  int centimetros; 
  centimetros = ultrasonic.Ranging(CM);

  if (centimetros>=dis)                           //20 cm es la distancia de emergencia
    {
      adelante();
      
      servo1.write(pos);
      delay(200);
      pos = ((pos+15) % 179);
    
    }
    else if (centimetros<dis)
    {    
      if(pos<60){
        izquierda();
        delay(1000);
      }else if(pos>120){
        derecha();
        delay(1000);
      }else{
        atras();
        delay(2000);
      } 
    }
}


void loop() {
  
  boolean modeAuto = false;
  if (irrecv.decode(&results)) {
    
    switch (results.value) {
    case btnAdelante:
      adelante();
      break;
    case btnAtras:
      atras();
      break;
    case btnDerecha:
      derecha();
      delay(200);
      parada();
      break;
    case btnIzquierda:
      izquierda();
      delay(200);
      parada();
      break;
    case btnParada:
      parada();
      break;
    case btnAuto:
      modeAuto=!modeAuto;
    }
    irrecv.resume(); // Receive the next value
  }
  delay(100);
  
  if(modeAuto){
    autoModo1();
  }
}



