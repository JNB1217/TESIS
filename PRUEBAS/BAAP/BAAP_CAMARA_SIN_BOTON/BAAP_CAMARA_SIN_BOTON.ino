#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int led = 2;
int led1=3;
int servoMin = 170; // Valor mínimo (0 grados)
int servoMax = 400; // Valor máximo (90 grados)
int servoPos = servoMin;

void setup() {
  // Inicializar la comunicación serie a 9600 baudios
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  pinMode(led, OUTPUT);
  Serial.println("Hello Arduino");
}

void loop() {
  if (Serial.available() > 0) {
    // Si hay datos disponibles para leer
    //char string[1]; // Definir un arreglo de caracteres para almacenar el valor recibido (más el carácter nulo)

    // Leer el dato en forma de string
    String inputString = Serial.readStringUntil('\n');
    String sub_S = inputString.substring(0, 1);
    String sub_T = inputString.substring(1, 2);
    Serial.println(sub_S);
    Serial.println(sub_T);
    delay(1000);

    if (inputString == "11") {

      pwm.setPWM(3, 0, 650);//muñeca estado normal
      pwm.setPWM(2, 0, 600);//mano abierta
      delay(10);
      digitalWrite(led, HIGH);
    } else if(inputString == "00"){
      pwm.setPWM(3, 0, 330);//gira muñeca
      pwm.setPWM(2, 0, 150);//mano cerrada
      delay(10);
      digitalWrite(led, LOW);
    
    }else if(inputString == "10"){
      pwm.setPWM(3, 0, 330);//girar muñeca
      pwm.setPWM(2, 0, 600);//mano abierta
      delay(10);
      digitalWrite(led1, HIGH);
    }else if(inputString == "01"){
      pwm.setPWM(3, 0, 650);//muñeca normal
      pwm.setPWM(2, 0, 150);//mano cerrada
      delay(10);
      digitalWrite(led1, LOW);
    }

    //Serial.println(inputString);
  }
}
