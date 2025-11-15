#include <SoftwareSerial.h>

char buf;

class DCMotor {  
  int spd = 255, pin1, pin2;
  
public:  
    void Pinout(int in1, int in2){
      pin1 = in1;
      pin2 = in2;
      pinMode(pin1, OUTPUT);
      pinMode(pin2, OUTPUT);
    }

    void Speed(int in1){
      spd = in1;
    }    

    void Forward(){
      analogWrite(pin1, spd);
      digitalWrite(pin2, LOW);
    }

    void Backward(){
      digitalWrite(pin1, LOW);
      analogWrite(pin2, spd);
    }

    void Stop(){
      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
    }
};

DCMotor Motor1, Motor2;

void verificaComandoCelular(){
  int watchDogCounter = 0;

  while(BT.available() > 0){
    buf = BT.read();
    Serial.println(buf);  

    // Frente
    if (buf == 'w'){
      Motor1.Forward();
      Motor2.Forward();
    }

    // Ré no kibe
    if (buf == 's'){
      Motor1.Backward();
      Motor2.Backward();
    }

    // Esquerda frente
    if (buf == 'a'){
      Motor1.Forward();
      Motor2.Stop();
    }

    // Direita frente
    if (buf == 'd'){
      Motor2.Forward();
      Motor1.Stop();
    }

    // Esquerda trás 
    if (buf == 'z'){
      Motor1.Backward();
      Motor2.Stop();
    }

    // Direita trás 
    if (buf == 'c'){
      Motor2.Backward();
      Motor1.Stop();
    }

    // Parar
    if (buf == 'x'){
      Motor1.Stop();
      Motor2.Stop();
    }

    delay(100);
    watchDogCounter++;
    if (watchDogCounter > 500) break;
  }
}

void setup() {
  Serial.begin(9600);

  Motor1.Pinout(5,6);
  Motor2.Pinout(9,10);

  delay(1000);
}

void loop() {
  Motor1.Speed(255);
  Motor2.Speed(255);
  verificaComandoCelular();
}
