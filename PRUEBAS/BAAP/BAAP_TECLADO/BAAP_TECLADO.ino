#include <Wire.h> // Librería para controlar I2C
#include <Adafruit_PWMServoDriver.h> // Librería para utilizar el módulo PCA9685

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // Dirección I2C del módulo PCA9685

int led = 2;  // Define el pin para el LED
int servoMin = 150;  // Valor mínimo del servo (0 grados)
int servoMax = 270;  // Valor máximo del servo (90 grados)

int servoPos0 = 150;  // Posición inicial del servo #1
int servoPos1 = 150;  // Posición inicial del servo #2

void setup() {
  Serial.begin(9600);  // Inicia la comunicación serial a 9600 bps
  pwm.begin();  // Inicializa el controlador de servo PCA9685
  pwm.setPWMFreq(60);  // Establece la frecuencia PWM en 60 Hz
  pinMode(led, OUTPUT);  // Configura el pin del LED como salida
  Serial.println("Prototipo BAAP");  // Imprime un mensaje en la consola
}

void moveServosSlow(int targetPos0, int targetPos1, int stepDelay) {
  int currentPosition0 = servoPos0;
  int currentPosition1 = servoPos1;

  // Bucle para mover gradualmente los servos a las posiciones deseadas
  for (int pos0 = currentPosition0, pos1 = currentPosition1; pos0 != targetPos0 || pos1 != targetPos1; ) {
    if (pos0 != targetPos0) {
      if (pos0 < targetPos0) {
        pos0++;
      } else if (pos0 > targetPos0) {
        pos0--;
      }
      pwm.setPWM(0, 0, pos0);  // Establece el pulso PWM para el servo #1
    }

    if (pos1 != targetPos1) {
      if (pos1 < targetPos1) {
        pos1++;
      } else if (pos1 > targetPos1) {
        pos1--;
      }
      pwm.setPWM(1, 0, pos1);  // Establece el pulso PWM para el servo #2
    }

    delay(stepDelay);  // Espera antes de la próxima iteración
  }

  servoPos0 = targetPos0;
  servoPos1 = targetPos1;
}

void loop() {
  if (Serial.available() > 0) {
    char string[1];
    Serial.readBytes(string, 2);  // Lee dos bytes desde la entrada serial

    //

    // Compara el primer carácter de la cadena leída y realiza acciones en consecuencia
    if (string[0] == '1') {        // Brazo abajo (número 1)
      moveServosSlow(240, 280, 7); 
      digitalWrite(led, HIGH);  // Enciende el LED
    } else if (string[0] == '0') { // Brazo arriba (número 0)  // Marcador en el servo es canal #1
      moveServosSlow(173, 350, 7);
      digitalWrite(led, LOW);  // Apaga el LED
    } else if (string[0] == '8') { // Mano cerrar dedos (número 8)
      pwm.setPWM(6, 0, 150);
      digitalWrite(led, HIGH);
    } else if (string[0] == '9') { // Mano abrir dedos (número 9)
      pwm.setPWM(6, 0, 670);
      digitalWrite(led, LOW);
    } else if (string[0] == '6') { // Girar muñeca (número 6)
      pwm.setPWM(3, 0, 150);
      digitalWrite(led, HIGH);
    } else if (string[0] == '5') { // Girar muñeca a su lugar original (número 5)
      pwm.setPWM(3, 0, 670);
      digitalWrite(led, LOW);
    }

    Serial.println(string);  // Imprime el carácter leído en la consola
  }
}
