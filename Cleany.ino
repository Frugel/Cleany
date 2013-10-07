#include <Ultrasonic.h>
#include <Servo.h>
#include <IRremote.h>


int pos = 0;  // analog pin used to connect the potentiometer
int inc = 15;
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
boolean modeAuto = false;

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
  Serial.println("Entrando en modod autoModo1");
  
  int centimetros; 
  centimetros = ultrasonic.Ranging(CM);
  
  Serial.println(centimetros);

  if (centimetros>=dis)                           //20 cm es la distancia de emergencia
    {
      adelante();    
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

void autoModo2()
{
  Serial.println("Entrando en modod autoModo2");
  
  int centimetros; 
  centimetros = ultrasonic.Ranging(CM);
  Serial.println("Distancia: "+centimetros);
  
  if (centimetros>=dis)                           //20 centimetros es la distancia de emergencia
  {
    adelante();
    delay(500);
  }
  else if (centimetros<dis)
  {    
    parada();

    int myInts[19];
    servo1.write(0);
    delay(100);
    for (int i=0; i <= 18; i++){

      myInts[i]= ultrasonic.Ranging(CM);
      delay(10);
      myInts[i]=myInts[i] + ultrasonic.Ranging(CM);
      delay(10);
      myInts[i]=myInts[i] + ultrasonic.Ranging(CM);
      delay(10);
      myInts[i]=myInts[i] + ultrasonic.Ranging(CM);
      delay(10);

      myInts[i]= (myInts[i]/4);

      pos =i*10;
      servo1.write(pos);
      delay(100);
    }

    servo1.write(90);
    delay(100);

    Serial.println(myInts[0]);
    Serial.println(myInts[1]);
    Serial.println(myInts[2]);
    Serial.println(myInts[3]);
    Serial.println(myInts[4]);
    Serial.println(myInts[5]);
    Serial.println(myInts[6]);
    Serial.println(myInts[7]);
    Serial.println(myInts[8]);
    Serial.println(myInts[9]);

    if( (myInts[0]>dis) &&  (myInts[1]>dis) && (myInts[2]>dis) && (myInts[3]>dis) && (myInts[4]>dis) 
      && (myInts[5]>dis) && (myInts[6]>dis) && (myInts[7]>dis) && (myInts[8]>dis) && (myInts[9]>dis) ){

      derecha();
      delay(1000);

    }
    else if( (myInts[10]>dis) &&  (myInts[11]>dis) && (myInts[12]>dis) && (myInts[13]>dis) && (myInts[14]>dis) 
      && (myInts[15]>dis) && (myInts[16]>dis) && (myInts[17]>dis) && (myInts[18]>dis) ){

      izquierda();
      delay(1000);

    }
    else{
      atras();
      delay(2000);

      izquierda();
      delay(1000);
    }
  }
}

void avanzarServo() {
  servo1.write(pos);
  delay(75);
  
  if(pos>=180){
    inc = (-15);
  }else if (pos<=0){
    inc = 15;
  }
  pos = pos + inc;
}

void loop() {
  
  avanzarServo();
  
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
      parada();
      break;
    }
    irrecv.resume(); // Receive the next value
  }
  
  if(modeAuto){
    autoModo1();
  }

}




