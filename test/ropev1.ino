// PIVIC - ROPE V1

// Pinos conectados ao Step e Dir do Easydriver
int pino_passo = 5;
int pino_direcao = 4;
int pino_hall = 3;
int pino_led1 = 2;
int pino_led2 = 6;
int pino_buzzer = 7;

// 1 - Anti-Horario
// 0 - Horario
int direcao = 0;

//VAR leitura do sensor hall
int hall;

// Quantidade de passos para uma volta completa
int limite_passos = 360;

// Tempo de cada passo
int microsegundos = 120;

// Ciclos
bool conta_ciclo = true;
int ciclos = 0;

void setup() {
  pinMode(pino_passo, OUTPUT);
  pinMode(pino_direcao, OUTPUT);
  pinMode(pino_led1, OUTPUT);
  pinMode(pino_led2, OUTPUT);

  pinMode(pino_hall, INPUT);

  Serial.begin(9600);

  calibrar();
}

void calibrar(){
  Serial.println("Calibrando...");

  tone(pino_buzzer, 1500);

  digitalWrite(pino_led1, HIGH);
  digitalWrite(pino_led2, HIGH);
  
  //Define a direcao
  digitalWrite(pino_direcao, direcao);

  //Le o sensor hall
  hall = digitalRead(pino_hall);

  while (hall != LOW){
    digitalWrite(pino_passo, 1);
    delayMicroseconds(microsegundos);
    digitalWrite(pino_passo, 0);
    delayMicroseconds(microsegundos);
    
    hall = digitalRead(pino_hall);
  };

  digitalWrite(pino_led1, LOW);
  digitalWrite(pino_led2, LOW);

  noTone(pino_buzzer);

  Serial.println("Pronto, iniciando");
  Serial.println("");
  delay(1000);
}

void loop(){
  
  for (int passo = 0 ; passo < limite_passos; passo++){
    digitalWrite(pino_passo, 1);
    delayMicroseconds(microsegundos);
    digitalWrite(pino_passo, 0);
    delayMicroseconds(microsegundos);

    //Le o sensor hall
    hall = digitalRead(pino_hall);
  
    if (hall == HIGH){
      digitalWrite(pino_led1, HIGH);
      digitalWrite(pino_led2, LOW);

      conta_ciclo = true;
    } else {
      digitalWrite(pino_led1, LOW);
      digitalWrite(pino_led2, HIGH);

      if (conta_ciclo == true){
        ciclos += 1;

        Serial.print(ciclos);
        Serial.println(" Ciclos");

        conta_ciclo = false;
      }
      
      tone(pino_buzzer, 500, 100);
    }
  }
}
