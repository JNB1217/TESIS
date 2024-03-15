#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int led = 2;
int led1=3;
#define BUTTON_PIN 4

void setup() {
  // Inicializar la comunicación serie a 9600 baudios
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  pinMode(led, OUTPUT);
  pinMode(4, INPUT_PULLUP);
  Serial.println("Hello Arduino");
}

void loop() {
   byte buttonState = digitalRead(BUTTON_PIN);
  if (Serial.available() > 0) {
    // Si hay datos disponibles para leer
    //char string[1]; // Definir un arreglo de caracteres para almacenar el valor recibido (más el carácter nulo)

    // Leer el dato en forma de string
    String inputString = Serial.readStringUntil('\n');
    String sub_S = inputString.substring(0, 1);
    String sub_T = inputString.substring(1, 2);
    //Serial.println(sub_S);
    //Serial.println(sub_T);
    delay(10);
 if (buttonState == HIGH) {      
    if (sub_S == "1"){
      pwm.setPWM(6, 0, 670);//mano abierta
      delay(10);
      digitalWrite(led, HIGH);
      Serial.println(sub_S);
    } else if(sub_S == "0"){
      pwm.setPWM(6, 0, 150);//mano cerrada
      delay(10);
      digitalWrite(led, LOW);
      Serial.println(sub_S);
}
 }
      
if (buttonState == LOW) {
     if(sub_T == "0"){
      pwm.setPWM(3, 0, 150);//girar muñeca
      delay(10);
      digitalWrite(led1, HIGH);
      Serial.println(sub_T);
    }else if(sub_T == "1"){
      pwm.setPWM(3, 0, 670);//muñeca normal
      delay(10);
      digitalWrite(led1, LOW);
      Serial.println(sub_T);
    }
}

    //Serial.println(inputString);
  }
}
