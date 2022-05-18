// PIVIC - ROPE V3 

#include <AccelStepper.h>

// Pinos conectados ao Step e Dir do Easydriver
int pino_passo = 5; //Step
int pino_direcao = 4; //Dir

int pino_hall = 3;
int pino_led1 = 2;
int pino_led2 = 6;
int pino_buzzer = 7;

#define MotorInterfaceType 1

//Inicialização
// 1 - EasyDriver
// 5 - Porta do STEP
// 4 - Porta do DIR
AccelStepper motor = AccelStepper(MotorInterfaceType, pino_passo, pino_direcao);

// 1 - Anti-Horario
// 0 - Horario
int direcao = 0;

//VAR leitura do sensor hall
int hall;

// Quantidade de passos para uma volta completa
int limite_passos = 360;

// Tempo de cada passo
int microsegundos = 120;

void setup() {
  pinMode(pino_led1, OUTPUT);
  pinMode(pino_led2, OUTPUT);

  pinMode(pino_hall, INPUT);

  Serial.begin(9600);

  motor.setMaxSpeed(3800);
}

void loop(){
  motor.setSpeed(3800);
  motor.runSpeed();
}
