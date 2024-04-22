#include <Servo.h>  // Bliblioteca que permite o funcionamento do servo motor

Servo boxServo;

// Sensor de Cor TCS3200
#define S0 2
#define S1 3
#define S2 8
#define S3 9
#define sensorOut 10
int frequency = 0;  // Variável para armazenar a frequência
int color = 0;      // Variável para armazenar a cor detectada

// Motor L298N
int mRight = 5;    // Pino de controle do motor direito
int dirRight = 4;  // Pino de direção do motor direito
int mLeft = 6;     // Pino de controle do motor esquerdo
int dirLeft = 7;   // Pino de direção do motor esquerdo

// Sensor de Linha
const int numPins = 6;      // Número de pinos analógicos (A0 a A5)
int sensorValues[numPins];  // Array para armazenar leituras analógicas

int sensorVal[6] = { 0, 0, 0, 0, 0, 0 };  // Array para armazenar valores binários dos sensores de linha
int error, last_error, MV, pid_l, pid_r, D, D1, D2, D3, I, I1, I2, I3, P, Pd, bitSensor;
int Max_MV;
unsigned char Kp = 9;  // Kp é um coeficiente que determina a magnitude da correção proporcional aplicada num sistema de controlo,
                          //com base no erro entre a posição desejada e a posição real.
unsigned char Kd = 0;     // Kd  é um coeficiente que determina como a correção do controlador PID é influenciada pela taxa de variação do erro.
unsigned char Ts = 1;
unsigned char maxPwm = 170;      // maxPwm define a velocidade maxima dos motores
unsigned char intersection = 0;  // Variavel para armazenar o numero de interseções

void setup() {
  Serial.begin(9600);  // Inicialização da comunicação serial
  pinMode(12, OUTPUT);
  // Configuração dos pinos do motor L298N
  pinMode(mRight, OUTPUT);
  pinMode(dirRight, OUTPUT);
  pinMode(mLeft, OUTPUT);
  pinMode(dirLeft, OUTPUT);
  // Configuração dos pinos do sensor de linha
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  // Configuração dos pinos do sensor de cor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(12, HIGH);
  delay(100);
  digitalWrite(12, LOW);
  delay(200);
  digitalWrite(12, HIGH);
  delay(100);
  digitalWrite(12, LOW);
  delay(200);
  digitalWrite(12, HIGH);
  delay(100);
  digitalWrite(12, LOW);
  boxServo.attach(11);  // Anexar o servo motor
  boxServo.write(35);   // Movimento inicial do servo motor
}

void (*resetFunc)(void) = 0;  // Função para reiniciar o Arduino

void loop() {
  mission();  // Função principal para execução das operações do robô
}

void readSens() {
  // Função para ler os valores dos sensores de linha
  for (int analogPin = A0; analogPin <= A5; analogPin++) {
    int sensorValue = analogRead(analogPin);  // Leitura do valor analógico do sensor
    Serial.print(sensorValue);                // Imprimir o valor lido
    Serial.print('\t');

    sensorValues[analogPin - A0] = sensorValue;  // Armazenar o valor lido no array
    if (sensorValue >= 125) {
      sensorVal[analogPin - A0] = 1;  // Converter o valor para binário (0 ou 1)
    } else {
      sensorVal[analogPin - A0] = 0;
    }
    Serial.print(sensorVal[analogPin - A0]);  // Imprimir o valor binário
    Serial.print('\t');
  }
  Serial.println();
}

void robotRun() {
  // Função para controlar o movimento do robô com base nos sensores de linha
  readSens();                                                                                                                                   // Ler os sensores de linha
  bitSensor = ((sensorVal[0] * 1) + (sensorVal[1] * 2) + (sensorVal[2] * 4) + (sensorVal[3] * 8) + (sensorVal[4] * 16) + (sensorVal[5] * 32));  // Converter os valores dos sensores para um número binário
  switch (bitSensor) {
    // Determinar o erro com base na posição dos sensores
    case 0b110000: error = -4; break;
    case 0b010000: error = -3; break;
    case 0b011000: error = -2; break;
    case 0b001000: error = -1; break;
    case 0b001100: error = 0; break;
    case 0b000100: error = 1; break;
    case 0b000110: error = 2; break;
    case 0b000010: error = 3; break;
    case 0b000011: error = 4; break;
    case 0b000001: error = 5; break;
  }

  Max_MV = Kp * 5;  // Calcular o máximo de MV
  P = Kp * error;   // Calcular P (proporcional)
  D1 = Kd * 8;
  D2 = D1 / Ts;
  D3 = error - last_error;
  D = D2 * D3;

  last_error = error;
  MV = P + D;

  if (MV >= -Max_MV && MV <= Max_MV) {
    pid_l = maxPwm + MV;
    pid_r = maxPwm - MV;
    if (pid_l < 0) pid_l = 0;
    if (pid_l > 200) pid_l = 200;
    if (pid_r < 0) pid_r = 0;
    if (pid_r > 200) pid_r = 200;
    forward(pid_r, pid_l);
  } else if (MV < -Max_MV) {
    turnLeft(170, 170);
  } else if (MV > Max_MV) {
    turnRight(150, 150);
  } else {
    forward(pid_r, pid_l);
  }
}

int readColor() {
  // Função para ler a cor detectada pelo sensor de cor
  // Leitura da cor vermelha
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  frequency = pulseIn(sensorOut, LOW);
  int R = frequency;
  Serial.print("R= ");
  Serial.print(frequency);
  Serial.print("  ");
  delay(50);
  // Leitura da cor verde
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  frequency = pulseIn(sensorOut, LOW);
  int G = frequency;
  Serial.print("G= ");
  Serial.print(frequency);
  Serial.print("  ");
  delay(50);
  // Leitura da cor azul
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  frequency = pulseIn(sensorOut, LOW);
  int B = frequency;
  Serial.print("B= ");
  Serial.print(frequency);
  Serial.println("  ");
  delay(50);

  // Determinar a cor predominante com base nas leituras
  if (R < 45 && R > 25 && G < 160 && G > 130) {
    Serial.println(" Red !\n");
    color = 1;  // Vermelho
    digitalWrite(12, HIGH);
    delay(750);
    digitalWrite(12, LOW);
  }
  if (R < 170 && R > 120 && G < 135 && G > 100) {
    Serial.println(" Green !\n");
    color = 2;  // Verde
    digitalWrite(12, HIGH);
    delay(750);
    digitalWrite(12, LOW);
  }
  if (G < 65 && G > 55 && B < 35 && B > 20) {
    Serial.println(" Blue !\n");
    color = 3;  // Azul
    digitalWrite(12, HIGH);
    delay(750);
    digitalWrite(12, LOW);
  }
  return color;  // Retornar a cor detectada
}

void mission() {
  // Função para coordenar as operações do robô
  robotRun();          // Executar o controlo do robô
  boxServo.write(35);  // Mover o servo motor
  // Verificar se o robô está em uma interseção
  if (sensorVal[5] == 1 && sensorVal[0] == 1) {
    stopRun();            // Parar o movimento do robô
    color = readColor();  // Ler a cor detectada

    // Executar ações com base na cor detectada
    switch (color) {
      case 1:  // Vermelho: Virar à esquerda
        red();
        break;
      case 2:  // Verde: Seguir em frente
        green();
        break;
      case 3:  // Azul: Virar à direita
        blue();
        break;
      case 0:  // Nenhuma cor detectada: Parar
        stopRun();
        break;
    }
  }
}

// Funções para executar ações com base na cor detectada

void red() {
  intersection++;
  if (intersection == 1) {
    delay(50);
    turnBack(150, 0);  // Virar de volta
  } else if (intersection == 2) {
    delay(50);
    stopRun();  // Parar
    delay(500);
    turnLeft(150, 0);  // Virar à esquerda
  } else if (intersection == 3) {
    stopRun();           // Parar
    boxServo.write(90);  // Mover o servo motor
    delay(1000);
    boxServo.write(35);
    digitalWrite(12, HIGH);
    delay(500);
    digitalWrite(12, LOW);
    delay(1000);
    stopRun();  // Parar
    delay(50);
    turnBack(150, 0);  // Virar de volta
  } else if (intersection == 4) {
    delay(50);
    turnRight(0, 150);  // Virar à direita
  } else if (intersection == 5) {
    resetFunc();  // Reiniciar o Arduino
  }
}

void green() {
  intersection++;
  if (intersection == 1) {
    delay(50);
    turnBack(150, 0);  // Virar de volta
  } else if (intersection == 2) {
    delay(50);
    stright(150, 150);  // Seguir em frente
  } else if (intersection == 3) {
    delay(50);
    stopRun();           // Parar
    boxServo.write(90);  // Mover o servo motor
    delay(1000);
    boxServo.write(35);  // Mover o servo motor
    delay(1000);         // Aguardar
    digitalWrite(12, HIGH);
    delay(500);
    digitalWrite(12, LOW);
    turnBack(150, 0);  // Virar de volta
  } else if (intersection == 4) {
    delay(50);
    stright(150, 150);  // Seguir em frente
  } else if (intersection == 5) {
    delay(50);
    resetFunc();  // Reiniciar o Arduino
  }
}

void blue() {
  intersection++;
  if (intersection == 1) {
    delay(50);
    turnBack(150, 0);  // Virar de volta
  } else if (intersection == 2) {
    turnRight(0, 150);  // Virar à direita
  } else if (intersection == 3) {
    delay(50);
    stopRun();           // Parar
    boxServo.write(90);  // Mover o servo motor
    delay(1000);
    boxServo.write(35);  // Mover o servo motor
    delay(1000);         // Aguardar
    digitalWrite(12, HIGH);
    delay(500);
    digitalWrite(12, LOW);  // Aguardar
    turnBack(150, 0);       // Virar de volta
  } else if (intersection == 4) {
    delay(50);
    turnLeft(150, 0);  // Virar à esquerda
  } else if (intersection == 5) {
    delay(50);
    resetFunc();  // Reiniciar o Arduino
  }
}

// Funções de controlo de movimento

void forward(int valLeft, int valRight) {
  digitalWrite(dirRight, HIGH);   // Definir direção do motor direito
  analogWrite(mRight, valRight);  // Controlar a velocidade do motor direito
  digitalWrite(dirLeft, HIGH);    // Definir direção do motor esquerdo
  analogWrite(mLeft, valLeft);    // Controlar a velocidade do motor esquerdo
}

void stright(int valLeft, int valRight) {
  // Seguir em frente até não detectar mais a linha
  
  digitalWrite(dirRight, HIGH);
  analogWrite(mRight, valLeft);
  digitalWrite(dirLeft, HIGH);
  analogWrite(mLeft, valRight);
      delay(100);
  while (sensorVal[2] == 1) { readSens(); }
  while (sensorVal[3] == 1) { readSens(); }
  while (sensorVal[4] == 1) {
    readSens();
    robotRun();
  }
}

void turnRight(int valLeft, int valRight) {
  // Virar à direita até não detectar mais a linha
  digitalWrite(dirRight, dirRight);
  analogWrite(mRight, valRight);
  digitalWrite(dirLeft, LOW);
  analogWrite(mLeft, 0);
  delay(800);
  while (sensorVal[5] == 1) { readSens(); }
  while (sensorVal[4] == 1) { readSens(); }
  while (sensorVal[3] == 1) { readSens(); }
}

void turnLeft(int valLeft, int valRight) {
  // Virar à esquerda até não detectar mais a linha
  digitalWrite(dirRight, LOW);
  analogWrite(mRight, 0);
  digitalWrite(dirLeft, dirLeft);
  analogWrite(mLeft, valLeft);
  delay(700);
  while (sensorVal[3] == 1) { readSens(); }
  while (sensorVal[4] == 1) { readSens(); }
  while (sensorVal[5] == 1) { readSens(); }
}

void turnBack(int valLeft, int valRight) {
  // Virar 180 graus até não detectar mais a linha
  digitalWrite(dirRight, HIGH);
  analogWrite(mRight, 150);
  delay(850);
  digitalWrite(dirRight, LOW);
  analogWrite(mRight, 0);
  digitalWrite(dirLeft, dirLeft);
  analogWrite(mLeft, valLeft);
  while (sensorVal[0] == 1 && sensorVal[1] == 1) { readSens(); }
  while (sensorVal[0] == 0 && sensorVal[1] == 0) { readSens(); }
  while (sensorVal[0] == 1 && sensorVal[1] == 1) { readSens(); }
  while (sensorVal[2] == 1 && sensorVal[3] == 1) { readSens(); }
  while (sensorVal[2] == 0 && sensorVal[3] == 0) { readSens(); }
  while (sensorVal[2] == 1 && sensorVal[3] == 1) { readSens(); }
}

void stopRun() {
  // Parar o movimento dos motores
  digitalWrite(dirRight, LOW);
  analogWrite(mRight, 0);
  digitalWrite(dirLeft, LOW);
  analogWrite(mLeft, 0);
}
