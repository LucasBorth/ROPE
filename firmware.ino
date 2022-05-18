//////////////////////////////////////////////////////////////////////////////////////////////
//  Firmeware do ROPE (RObo de Papel Educacional) v02
//  pinos do cnc shield
//          step    dir
//   x       2       5
//   y       3       6
//   z       4       7
//
//  x e z  motoroes de passo, y sensores hall de posicao inicial
//
//  Pino 8 habilita os drivers quando está com nivel baixo e desabilita com nivel alto
//  Sete a corrente dos motores para no maximo 200mA (no motor de 5v com bobinas de 44 ohms) para calcular a corrente maxima meca a resistencia das bobinas e divida a tensão do motor pela resistencia.
//  No caso do driver que usei o DRV8825 eu deixei o pot com 0.9v o que me deu uma velocidde maxima de 750.
//  Depois criar um auto ajuste para ele pegar automatico os parametros como:
//  1 - velocidade maxima
// 
//////////////////////////////////////////////////////////////////////////////////////////////

// Bugs conhecidos e limitações
// Bug - fazer remanejamento das igual foi feito na MoveFoWard() pras funções de giro, do jeito que esta as de giro nao podem ser chamadas em sequencia.
// normalize esta acochada e por isso apos muitas chamadas de funcoes os motores devem desincronizar
// usando volta completa pra liberar o lock, é possivel usar meia volta se funções de giro pegarem o motor nas posições 0 ou +- 1024
// melhorar a modularização
// remover os prints de seriais, eles consomem memoria e deixam o codigo lento, se colocar muitos prints o programa tera pouca ram e ficara instavel



// Includes
// Math lib
#include <Math.h>
// Include the AccelStepper library for stepper control
#include <AccelStepper.h>
// Include miles timmer library for serial control
#include <elapsedMillis.h>
// include IRremote ken shirriff,z3t0, arminjo for control robot test
#include <IRremote.h>

// defines
// Define accelstepper variables
#define MAXSPEED 750
#define ACCELARATION  50
#define SPEED 400

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
// Pinout is CNCShield 3.0 Compatible
#define motorInterfaceType 1
#define stepperAmount 2
// step and dir pins for drivers in x and z
#define stepPin1 2
#define dirPin1 5
#define stepPin2 4
#define dirPin2 7

// enable drivers pin and activity led
// enable dont use accelsteper enable control
#define enablePin 8
#define ledPin 13

// hall sensor pins use y driver dir and step pins
#define  hallSensor1 3
#define  hallSensor2 6

// IrPin z+ or z- endstop for IR control
#define IRsensorPin 11

// macros for better semantic
#define driversON digitalWrite(enablePin, LOW)
#define driversOFF digitalWrite(enablePin, HIGH)
#define ledON digitalWrite(ledPin, HIGH)
#define ledOFF digitalWrite(ledPin, LOW)
#define readHall1  digitalRead(hallSensor1)
#define readHall2  digitalRead(hallSensor2)
#define DIFFERENCE(x, y) (x > y) ? (x - y) : (y - x) // calcula a diferença absoluta entre dois valores

//Variables

//Accelstepper variables
// Create instances of the AccelStepper class:
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);
//Stick pointers to those AccelStepper objects into an array, for acess steppers using indexes ex: steppers[0]
//you have to do it this way to be global variables!
AccelStepper* steppers[stepperAmount] ={ &stepper1, &stepper2, };
int speed = SPEED;

// Hall Sensor
int hallSensors[stepperAmount*2] = {0}; // array of hall sensors *2 to store previous state and present stat
int hallSensorSize[2]={0}; // armazena o tamanho da area de sensitividade dos sensores hall em pulsos (varia com distancia e posicionamento do sensor)


// Millis timer
elapsedMillis serialTimer; //for serial print

// Motor Control
int stepByTurn[2] = {2048,2048}; // armazena o numero de passos por volta do motor no 28byj-48 é 2048 depois iremos capturar automaticamente o valor
int hallSizeCounter[2] = {0}; // armazena o tamanho do sensor hall pra função de achar centro
//int motorBlock[2] = {0}; //bloqueia pulso dos motores nas funções de movimento para manter sincronizado com o centro do sensor
long motorPosition[2] = {0}; // armazena as posicões dos motores para a rotina de achar meio
long motorSincPosition[2]= {0};// posições onde os motores estarão sincados e giroblock pode ser liberado
// armazena um valor que representa a ultima funcão de movimento que acessou o motor, sem incluir a de stop 
// 1 - MotorFoward(), 2 - MotorBackWard(),  3 - TurnRight(), 4 - TurnLeft()
int lastSub = 0;  //Variavel que indica a ultima subrotina que fez step nos motores
int giroBlock = 0; //Variave que redireciona as subrotinas de movimento para as subs de girar
long motorZero[] = {0}; //posição inicial do giro do motor
int reqStop=0;// requisita saida do giro

// IR control variables
IRrecv irRecv(IRsensorPin); //ativa ir no pino de recepção
decode_results results; // armazena codigo do IR
long codeAnt = 0; // armazena o codigo da funcao chamada anteriormente em exeCom para que o robo não seja floodado com comandos
long exeCom = 0; // comando a ser executado

void setup() {

  // initialize serial communication at 115200 bits per second, more slow serial slowdown the motor
  Serial.begin(115200);

  // enable IR recevier
  irRecv.enableIRIn();

  //now i can just make a for loop and iterate through the steppers array and call whatever function I want
  //Note: you have to use -> to call that function instead of a .
  // Set the maximum speed, acceleration e speed for two instances in steps per second:
  for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){
   steppers[stepperNumber]->setMaxSpeed(MAXSPEED);
   steppers[stepperNumber]->setAcceleration(ACCELARATION);
   steppers[stepperNumber]->setSpeed(SPEED);
  }
  
  // initialize the LED pin and enablePin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // initialize hall pins as an input with pullup (no external resistor)
  pinMode(hallSensor1, INPUT_PULLUP);
  pinMode(hallSensor2, INPUT_PULLUP);

  HallInit();
}

void loop() {
 // if the sensor is receive any signal 
  if (irRecv.decode(&results)){
    //Serial.println(results.value); // para ver os e/hex no terminal
    //resume function according to hex code
    irRecv.resume();
  }
  
  // if the incoming data is "defined hex code" then run the motors functions
  // 2155845735 retorna
  //2155813095 vol +
  //2155809015 vol -
  if (codeAnt != results.value){ 

   if(results.value == 2155860525){ //GetStepsByTurn(1) canal +
      //Serial.println("GetStepsByTurn(1)");
      //GetStepsByTurn(1);
   } 
  if(results.value == 2155868685){ //GetStepsByTurn(2) canal -
     // Serial.println("GetStepsByTurn(2)");
      //GetStepsByTurn(2);
   }  
   if(results.value == 2155841655){ // hallInit()
      Serial.println("hallInit()");
      HallInit();
   }
   if(results.value == 2155833495){ // MotorForward()
      Serial.println("MotorForward()");
      exeCom = 1;
   }
   if(results.value == 2155829415) { //MotorBackward()
     Serial.println("MotorBackward()");
     exeCom = 2;
   }
   if(results.value == 2155809525) { //TurnRight();
     Serial.println("TurnRight()");
     exeCom = 3;
   }
   if(results.value == 2155842165) { //TurnLeft();
     Serial.println("TurnLeft()");
     exeCom = 4;
   }
   if(results.value == 2155857975) { //MotorStop();
     Serial.println("MotorStop()");
     exeCom = 5;
   }
  }
  codeAnt = results.value;
  if (exeCom == 1) {
    MotorForward();
    if(serialTimer >= 100){
     //Serial.println("giroblock= " + String(giroBlock) + " lastSub = " + String(lastSub));
     serialTimer = 0;
    }
  }
  if (exeCom == 2) {
    MotorBackward();
    if(serialTimer >= 100){
     //Serial.println("giroblock= " + String(giroBlock) + " lastSub = " + String(lastSub));
    serialTimer = 0;
   }
  }
  if (exeCom == 3) {
    TurnRight();
    if(serialTimer >= 100){
    //Serial.println("giroblock= " + String(giroBlock) + " lastSub = " + String(lastSub));
    serialTimer = 0;
    }
  }
  if (exeCom == 4) TurnLeft();
  if (exeCom == 5) MotorStop();

 if(serialTimer >= 100){
    for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){ // set current position to 0 and speed
    //steppers[stepperNumber]->setSpeed(SPEED);
    //steppers[stepperNumber]->setCurrentPosition(0);
   //Serial.println("posicão motor"  +String(stepperNumber) + ": " + String(steppers[stepperNumber]->currentPosition()));
  }
   int temp = HallRead();
  // if(temp != 0)  Serial.println("Sinc das rodas: " + String(temp));
   serialTimer = 0;
 }
}


///////////////////////////////////////////////////////////////////////////
//
//  Subroutines
//
//////////////////////////////////////////////////////////////////////////

// Initialize hall sensors array and homing legs
void HallInit(){
  Serial.println("Inicializando motores!");
  hallSensors[0] = readHall1;
  hallSensors[1] = readHall2; 
  hallSensors[2] = hallSensors[0];
  hallSensors[3] = hallSensors[1];
  driversON;
  //Serial.println("leitura halls: " + String(hallSensors[0]) + " " + String(hallSensors[0])); 
  // Os motores podem estar nos sensores, mas não existe garantia que estão exatamente no meio dos sensores
  // por isso vamos tirar eles dos sesnores para reposicionar em seguida 
  int hallvalue = 1;
  if(hallSensors[0] != 1 || hallSensors[1] != 1 ){  // motors on sensor, moving magnet out of sensor
    //Serial.println("Movendo motores pra fora dos sensores");
    while(hallvalue){  //move motor 1 outside
      steppers[0]->setSpeed(SPEED);
      steppers[0]->run();
      for(int n=0; n<5; n++){// para garantir que realmente tirei o motor do sensor so paro apos 5 1s lidos em seguida.
        hallvalue = hallvalue + readHall1;
      }
      if (hallvalue==6) hallvalue = 0;
      else hallvalue = 1;
    }// executo o memso pro motor 2
    hallvalue =1;  
    while(hallvalue){ //move motor 2 outside       
      steppers[1]->setSpeed(SPEED);
      steppers[1]->run();
      for(int n=0; n<5; n++){// para garantir que realmente tirei o motor do sensor
        hallvalue = hallvalue + readHall2;
      }
      if (hallvalue==6) hallvalue = 0;
      else hallvalue = 1;
     } 
  }
  // Posiciono o motor bem no centro do sensor
  // para isso eu conto quantos pulsos em 0  existe na posição do sensor, depois divido esse valor por 2 e mando o motor retornar a essa posição
  //Serial.println("Ajustando motores");   
  if(readHall1 == 1){  // if motor 1 dont is in setpoint
   // Serial.println("Motor 1 - buscando sensor!");
    // encontro o inicio do sensor
    while(readHall1 != 0){// move motor 1 to setpoint        
      steppers[0]->setSpeed(SPEED);
      steppers[0]->run();
    }
    //Serial.println("Sensor encontrado, centralizando no sensor...");
    steppers[0]->setCurrentPosition(0);// zero a posição do motor bem no inicio do sensor
    while(readHall1 != 1){// movo ate o fim do sensor      
      steppers[0]->setSpeed(SPEED);
      steppers[0]->run();
    }
    hallSensorSize[0] = steppers[0]->currentPosition();
    // Serial.println("Tamanho do sensor: " + String(hallSensorSize[0]));   
    //Serial.println("Movendo para o centro do sensor");
    // agoro movo o magneto pro exato meio do sensor
    int ajuste = hallSensorSize[0]/2;
    steppers[0]->moveTo(ajuste);
    steppers[0]->setSpeed(SPEED/2);
    while (steppers[0]->distanceToGo() != 0) {
      steppers[0]->runSpeedToPosition();
    }
    //Serial.println("Motor 1 ajustado :)");
  } 
  if(readHall2 == 1){  // if motor 2 dont is in setpoint
    //Serial.println("Motor 2 - buscando sensor!");
    while(readHall2 != 0){// move motor 2 to setpoint        
      steppers[1]->setSpeed(SPEED);
      steppers[1]->run();
    }
    //Serial.println("Sensor encontrado, centralizando no sensor...");
    steppers[1]->setCurrentPosition(0);
    while(readHall2 != 1){        
      steppers[1]->setSpeed(SPEED);
      steppers[1]->run();
    }
    hallSensorSize[1] = steppers[1]->currentPosition();
    //Serial.println("Tamanho do sensor: " + String(hallSensorSize[1])); 
    //Serial.println("Movendo para o centro do sensor");
    int ajuste = hallSensorSize[1]/2;
    steppers[1]->moveTo(ajuste);
    steppers[1]->setSpeed(SPEED/2);
    while (steppers[1]->distanceToGo() != 0) {
      steppers[1]->runSpeedToPosition();
    }
    //Serial.println("Motor 2 ajustado :)");
  }
  driversOFF;
  for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){ // set current position to 0 and speed
    steppers[stepperNumber]->setSpeed(SPEED);
    steppers[stepperNumber]->setCurrentPosition(0);
  }
  Serial.println("Inicializacao finalizada!!!!"); 
  HallRead();   
  giroBlock =0;
}   


// Read hall sensor array and update the previous values
// Detect edges in the hall sensors, if return value is 0 no change in sensors.
// Other values ​​encode changes in sensors:
// 1 - falling edge sensor 1 and no change in 2
// 2 - rising edge sensor 1 and no change in 2
// 4 - falling edge sensor 2 and no change in 1
// 5 - falling edge sensor 1 and falling edge sensor 2 -> synced sensors
// 6 - rising edge sensor 1 and falling edge sensor 2 -> synced sensors
// 8 - rising edge sensor 2 and no change in 1
// 9 - falling edge sensor 1 and rising edge sensor 2 -> synced sensors
// 10 - rising edge sensor 1 and rising edge sensor 2 -> synced sensors
int HallRead() {
  int val = 0;
   hallSensors[2] = hallSensors[0];
   hallSensors[3] = hallSensors[1];
   hallSensors[0] = readHall1;
   hallSensors[1] = readHall2;
  if(hallSensors[0] == 1 && hallSensors[2] == 0) // falling edge sensor 1
    val = val + 1;
  if(hallSensors[0] == 0 && hallSensors[2] == 1) // rising edge sensor 1
    val = val + 2;
  if(hallSensors[1] == 1 && hallSensors[3] == 0) // falling edge sensor 2
    val = val + 4;
  if(hallSensors[1] == 0 && hallSensors[3] == 1) // rising edge sensor 2
    val = val + 8;
  return val;
}

// Subs de movimento 

//Encontra o centro do motor pela analise do movimento, essa rotina diz:
// 1 - quando o motor passa pelo centro e reseta a posição dos motores
// ativa as flags de bloqueio do motor que estiver adiantado pelo numero necessário de pulsos
// usa as seguintes variaveis
//int stepByTurn[2] = {2048,2048}; // armazena o numero de passos por volta do motor no 28byj-48 é 2048 depois iremos capturar automaticamente o valor
//int hallSizeCounter[2] = {0}; // armazena o tamanho do sensor hall pra função de achar centro
//int motorBlock[2] = {0}; //bloqueia pulso dos motores nas funções de movimento para manter sincronizado com o centro do sensor
//long motorPosition[2] = {0}; // armazena as posicões dos motores para a rotina de achar meio
int CenterFind() {
  int out =0;// saida da função:
  // 0 não achou um centro,
  // 1 centro do motor 1, 
  // 2 centro do motor 2
  // 3 centro dos dois motores in sinc
  //Faz uma copia da posição dos motoroes para temporario
  //for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){
  // motorPosition[stepperNumber] = steppers[stepperNumber]->currentPosition();
  //}
 int borda =  HallRead();
 if(borda != 0) {// alguma borda ocorreu, a contagem de meio deve parar ou iniciar
    if(borda == 2 || borda == 6 || borda == 10) { //aconteceu borda de subida em 1, inicia contagem de meio
     // Serial.println("borda subida motor 1 :"+ String(steppers[0]->currentPosition()));
    }
    if(borda == 8|| borda == 9 || borda == 10) { //aconteceu borda de subida em 2, inicia contagem de meio
     // Serial.println("borda subida motor 2 :"+ String(steppers[1]->currentPosition()));
    }
    if(borda == 1|| borda == 5 || borda == 9) { //aconteceu borda de descida em 1, termina contagem de meio
      // Serial.println("borda descida motor 1 :"+ String(steppers[0]->currentPosition()));
    }
   if(borda == 4|| borda == 5 || borda == 6) { //aconteceu borda de descida em 2, termina contagem de meio
      // Serial.println("borda descida motor 2 :"+ String(steppers[1]->currentPosition()));
    }
  }
 return out;
}


//seta as velicidades dos motores 1 positiva, -1 negativa 
void SetSteppersSpeed(int motor1, int motor2){
  for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){
    if (stepperNumber == 0) { 
      steppers[stepperNumber]->setSpeed(motor1 * speed);
    }
    else {
      steppers[stepperNumber]->setSpeed(motor2 * speed);
    }
  }
}

// move pra frente, codigo 1
void MotorForward() {
  if(giroBlock != 0) { // se a funcão de giro pediu o lock do motor
    if (giroBlock == 3) {
      TurnRight();
      reqStop = 1;
      }
    if (giroBlock == 4) {
      TurnLeft();
      reqStop = 1;
    }   
  }
  else{
   reqStop = 0;
   driversON;
   // aqui se o numero de passos do motor for <= ao numero de passos do outro motor o metodo rum será executado e o motor ira executar mais um passo.
   // porem se o motor estiver adiantado ( pasos > que o outro) run não será executado até os dois mnotores estarem sincronizados.
   for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){
     steppers[stepperNumber]->setSpeed(speed);
     if( steppers[stepperNumber]->currentPosition() <= steppers[1 - stepperNumber]->currentPosition()) {
       steppers[stepperNumber]->runSpeed();
      }
    }
    lastSub = 1;// codigo da sub MotorFoward()
  }
}

// move pra trás, codigo 2
void MotorBackward() {
  if(giroBlock != 0) { // se a funcão de giro pediu o lock do motor
    if (giroBlock == 3) {
      TurnRight();
      reqStop = 1;
      }
    if (giroBlock == 4) {
      TurnLeft();
      reqStop = 1;
    }   
  }
  else{
   reqStop = 0;
   driversON;
   // aqui se o numero de passos do motor for <= ao numero de passos do outro motor o metodo rum será executado e o motor ira executar mais um passo.
   // porem se o motor estiver adiantado ( pasos > que o outro) run não será executado até os dois mnotores estarem sincronizados.
   for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){
     steppers[stepperNumber]->setSpeed(-speed);
     if( steppers[stepperNumber]->currentPosition() >= steppers[1 - stepperNumber]->currentPosition()) {
       steppers[stepperNumber]->runSpeed();
      }
    }
    lastSub = 2;// codigo da sub MotorFoward()
  }
}

// calcula novas posições de parada e atualiza as posições iniciais 
void StopPosition(){
  //para cada motor seta a posição corrente com o resto da divisão pelo numero de passos por volta
 for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){ 
    //Serial.println("StopPosition() -> posicão motor "  +String(stepperNumber) + " antes de limitar: " + String(steppers[stepperNumber]->currentPosition()));
    steppers[stepperNumber]->setCurrentPosition(steppers[stepperNumber]->currentPosition() % stepByTurn[stepperNumber] );
    //Serial.println("StopPosition() -> posicão motor "  +String(stepperNumber) + " apos limitar: " + String(steppers[stepperNumber]->currentPosition()));
    motorZero[stepperNumber]= steppers[stepperNumber]->currentPosition();//armazena o "zero" dos motores
  }
  //Serial.println("StopPosition() Motorzero 1 = " + String(motorZero[0]) + " Motorzero 2 = " + String(motorZero[1]));
}

// salva as posições iniciais dos motores
void MotorZero(){
  int debug = 1;
 for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){ 
    //Serial.println("StopPosition() -> posicão motor "  +String(stepperNumber) + " antes de limitar: " + String(steppers[stepperNumber]->currentPosition()));
    //steppers[stepperNumber]->setCurrentPosition(steppers[stepperNumber]->currentPosition() % stepByTurn[stepperNumber] );
    //Serial.println("MotorZero() -> posicão zero do motor "  +String(stepperNumber) +  String(steppers[stepperNumber]->currentPosition()));
    motorZero[stepperNumber]= steppers[stepperNumber]->currentPosition();//armazena o "zero" dos motores
  }
  if(debug == 1){
    //Serial.println("MotorZero() 1 = " + String(motorZero[0]) + " e 2 = " + String(motorZero[1]) + " fim!");
  }
}

//seta a posição dos dois motores
void SetMotorPosition(long motor1, long motor2){
  int debug = 1;
  long temp[2]= {motor1, motor2};
 for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){ 
   steppers[stepperNumber]->setCurrentPosition(temp[stepperNumber]);
   if(debug == 1){
     //Serial.println("SetMotorPosition -> posicão do motor "  +String(stepperNumber) +  String(steppers[stepperNumber]->currentPosition()));
    }
  }
}


//Calcula posições seguras de liberar o block de giro
// no modo atual ele usara voltas completas, mas no futuro será possivel liberar o lock com meia volta para as funções de giro e direto entre as funções de frente e tras
// o sentido é pra colocar o sinal certo para o movimento do motor 1 sentido horario -1 antihorario
void SincPosition(int sentido1, int sentido2){
  int debug =1;
  int temp[2] = {sentido1, sentido2};// transforma os sentidos em uma matriz
  //Serial.println("SincPosition() Motorzero 1 = " + String(motorZero[0]) + " Motorzero 2 = " + String(motorZero[1]) + " inicio!");
  //calcula as posições de parada possiveis para liberar o giroblock
  for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){
    //Serial.println("SincPosition(" + String(temp[stepperNumber}) + ")posicão motor"  +String(stepperNumber) + ": " + String(steppers[stepperNumber]->currentPosition()) + "  temp = " + String(temp[stepperNumber]));
    // posição para liberar sinc uso o motor, como zerei a posição ao entrar na rotina de 
    motorSincPosition[stepperNumber] =  steppers[stepperNumber]->currentPosition() + temp[stepperNumber] * (stepByTurn[stepperNumber]); // no futuro divida o numero de voltas por 2 para meia volta
    if(debug==1){
     //Serial.println("SincPosition() "  +String(stepperNumber) + ": " + String(motorSincPosition[stepperNumber]));
    }
  }
  if(debug == 1){
   //Serial.println("SincPosition() Motorzero 1 = " + String(motorZero[0]) + " Motorzero 2 = " + String(motorZero[1]) + " fim!");
  }
}

//quando as posições dos motores ficam com sinais contrários ocorre um problema com
//as rotinas de andar pra frente e para trás, elas simplesmente travam
// essa funcão deixa as duas coordenadas com o mesmo sinal
// 1 deixa os valores com sinal positivo, -1 com sinal negativo
void NormalizePosition(int sinal){
  /*if(sinal == 1){
    for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){ 
     Serial.println(" NormalizePosition() -> posicão motor "  +String(stepperNumber) + " antes de normalizar: " + String(steppers[stepperNumber]->currentPosition()));
     steppers[stepperNumber]->setCurrentPosition(steppers[stepperNumber]->currentPosition() % stepByTurn[stepperNumber] );
     if( steppers[stepperNumber]->currentPosition() < 0){// se é o valor negativo
       steppers[stepperNumber]->setCurrentPosition(2048 + steppers[stepperNumber]->currentPosition()) ;
      }
      else{// se for positivo
      // if
        //steppers[stepperNumber]->setCurrentPosition(2048 - steppers[stepperNumber]->currentPosition()); 
      }
     Serial.println("NormalizePosition() -> posicão motor "  +String(stepperNumber) + " apos normalizar: " + String(steppers[stepperNumber]->currentPosition()));
     motorZero[stepperNumber]= steppers[stepperNumber]->currentPosition();//armazena o "zero" dos motores
    }
    int dif = DIFFERENCE(steppers[0]->currentPosition(), steppers[1]->currentPosition());
    if( dif > 15){
      Serial.println("NormalizePosition() diferença: " + String(DIFFERENCE(steppers[0]->currentPosition(), steppers[1]->currentPosition())) );
      steppers[1]->setCurrentPosition(2048 - steppers[1]->currentPosition());
      Serial.println("NormalizePosition() -> posicão motor 1 corrigida apos normalizar: " + String(steppers[1]->currentPosition()));
      //motorZero[1]= steppers[1]->currentPosition();//armazena o "zero" dos motores
    
  }
  else {
    //codigo do -1


  }*/
  // devieria normalizar, mas está dando muito problema, desse jeito o robo funcionara por um tempo, quando enviar muitos comandos difernetes ele deve desincronizar
  // será reparado em futuras versões
  for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){ 
    steppers[stepperNumber]->setCurrentPosition(0);
    motorZero[stepperNumber] = 0;
     }
  Serial.println("Nomalize() Motorzero 1 = " + String(motorZero[0]) + " Motorzero 2 = " + String(motorZero[1]));
}

//Run sicrono em dois motores
void RunGiroSinc(){
 int debug = 1;
  for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){
    //Aqui fazemos o ajuste de passo pra sincar os dois motores, verificamos se o valor absoluto da distancia percorida atual do motor é igual ou menor a distancia do outro motor.  
    //caso não seja o motor não recebe run.  
    if( abs(steppers[stepperNumber]->currentPosition() - motorZero[stepperNumber] ) <= abs(steppers[1 - stepperNumber]->currentPosition() - motorZero[1 - stepperNumber] ) ) {
      if(debug==1){// debug da função
        //Serial.println("RunGiroSinc() motor " + String(stepperNumber) + " - " + String(steppers[stepperNumber]->currentPosition()));
        //Serial.println( "RunGiroSinc() motor  " + String(stepperNumber)+ "  - " + String(abs(steppers[stepperNumber]->currentPosition() - motorZero[stepperNumber] )) + " <= " + String(abs(steppers[1 - stepperNumber]->currentPosition() - motorZero[1 - stepperNumber] )));
      }
      steppers[stepperNumber]->run();// gira os motores
    }   
  }
}

// run sincrono em um motor :
// 0 - motor 1 e 1 - motor 2
void RunGiroSinc(int motor){
 int debug = 0;
 //Aqui fazemos o ajuste de passo pra sincar o motor, verificamos se o valor absoluto da distancia percorida atual do motor é igual ou menor a distancia do outro motor.  
  if( abs(steppers[motor]->currentPosition() - motorZero[motor] ) <= abs(steppers[1 - motor]->currentPosition() - motorZero[1 - motor] ) ) {
    if(debug==1){// debug da função
     // Serial.println("RunGiroSinc() motor " + String(motor) + " - " + String(steppers[motor]->currentPosition()));
     // Serial.println( "RunGiroSinc(int) " + String(abs(steppers[motor]->currentPosition() - motorZero[motor] )) + " <= " + String(abs(steppers[1 - motor]->currentPosition() - motorZero[1 - motor] )));
    }
    steppers[motor]->run();// gira o motor
 } 
}



// Gira direita, codigo 3
void TurnRight() {
  int debug = 1;
  //Este if inicializa a rotina de giro só é executado uma vez quando entra na rotina pela primeira vez.
  //também pega o lock dos motores
  if(giroBlock == 0 && lastSub != 3 ){
    //salva as posições de entrada na rotina dos motores,as posições serão usadas para sincronizar os motores
    MotorZero();
    //calcula as posições de parada possiveis para liberar o giroblock, nesta versão ela será a cada volta completa, é possivel em futuras versões liberar a roda a cada meio giro
    // pois os motores se encontram en sincronismo no zero e nas posições 1024. Porém para se fazer isso é necessário garantir que a rotina sempre recebe o motor em zero ou 1024.
    // Assim é necessrio modificar todas para serem no estilo das funçoes de giro
    SincPosition(-1,1);
    giroBlock = 3;// adquire Lock
    if(debug==1){// debug da função
      //Serial.println("turnRight() lock obtido, executado MotorZero();,SincPosition(-1,1); e giroblock =3  ");
      //Serial.println("turnRight() reqStop = " + String(reqStop));
    }
  } 
  //A cada giro completo os motores se sincronizam
  else{// se a sub foi inicializada agora só cai aqui
   for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){ // para cada motor
     if(motorSincPosition[stepperNumber] == steppers[stepperNumber]->currentPosition()) {//se posição de parada é atingida    
       if (reqStop == 1){
         //Serial.println("lock liberado pelo motor" + String(stepperNumber) + " valor de reqStop: " + String(reqStop)); 
         giroBlock = 0;
         reqStop = 0;
         NormalizePosition(1);
        }    
       else{
         //ativa os drivers
         driversON;
         //seta a direção dos motores
         SetSteppersSpeed(-1, 1);
         //gira o motor em sincronia
         RunGiroSinc(stepperNumber);
         //realcula as novas posições
         if(stepperNumber  == 1){// so no segundo motor
           MotorZero();
           SincPosition(-1,1);
          }
        }
      }
      else{
        //ativa os drivers
        driversON;
        //seta a direção dos motores
        SetSteppersSpeed(-1, 1);
        //Gira o motor em sincronia
        RunGiroSinc(stepperNumber);
      }
    }
  }
  lastSub = 3;// codigo da sub TurnRight()
}


// Gira direita, codigo 3
void TurnRightOld() {
  //Este if inicializa a rotina de giro só é executado uma vez quando entra na rotina pela primeira vez.
  //também pega o lock dos motores
  if(giroBlock == 0 && lastSub != 3 ){
    //reseta as posições iniciais dos motores
    StopPosition();
    //calcula as posições de parada possiveis para liberar o giroblock
    SincPosition(-1,1);
    giroBlock = 3;// adquire Lock
  } 

  //Acada meio giro os motores se sincronizam
  else{// se a sub foi inicializada agora so cai aqui
   driversON;
   //seta a direção dos motores
   SetSteppersSpeed(-1, 1);
   //agora vamos sincar os dois motores a nivel de passos fazendo dar rum ou não nos motores ao comparar seus valores de passo atual com o queteoricamente deveria ser
   //pra isso usamos o motor positivo pra obter o quanto ele se deslocou e comparamos se o negativo se deslocou o mesmo se algum motoro estiver adiantado ou atrasado 
   // a rotina vai ajustalos por não enviar run pro motor
   //os sinais mudam pois agora a velocidade é negativa e a posição corrente pode se tornar negativa.
   for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){
    //Aqui fazemos o ajuste de passo pra sincar os dois motores, verificamos se o valor absoluto da distancia percorida atual do motor é igual a distancia que ele deveria percorrer
    if (stepperNumber == 0){//motor 1 negativo 
      if( abs(steppers[stepperNumber]->currentPosition() - motorZero[stepperNumber] ) <= abs(steppers[1 - stepperNumber]->currentPosition() - motorZero[1 - stepperNumber] ) ) {
        Serial.println( String(abs(steppers[stepperNumber]->currentPosition() - motorZero[stepperNumber] )) + " <= " + String(abs(steppers[1 - stepperNumber]->currentPosition() - motorZero[1 - stepperNumber] )));
       steppers[stepperNumber]->run();
     }
    }
    else {//motor 2 positivo
      if( abs(steppers[stepperNumber]->currentPosition() - motorZero[stepperNumber] ) <= abs(steppers[1 - stepperNumber]->currentPosition() - motorZero[1 - stepperNumber] ) ) {
       Serial.println( "If motor 2 " + String(abs(steppers[stepperNumber]->currentPosition() - motorZero[stepperNumber] )) + " <= " + String(abs(steppers[1 - stepperNumber]->currentPosition() - motorZero[1 - stepperNumber] )));
        steppers[stepperNumber]->run();  
     }
    }
    if(motorSincPosition[stepperNumber] == steppers[stepperNumber]->currentPosition()) {//se posição de parada é atingida giroblock é liberado     
      if (reqStop == 1){
        Serial.println("lock liberado pelo motor" + String(stepperNumber) + " valor de reqStop: " + String(reqStop)); 
        giroBlock = 0;
        reqStop = 0;
        NormalizePosition(1);
      }
      else{
       //realcula as novas posições
       StopPosition();
       SincPosition(-1,1);
      }
    }
    else{
      // quem sabe precise né?
    }

  }
  lastSub = 3;// codigo da sub TurnRight()
  }
}

 

// Gira esquerda, codigo 4
void TurnLeft() {
    int debug = 1;
  //Este if inicializa a rotina de giro só é executado uma vez quando entra na rotina pela primeira vez.
  //também pega o lock dos motores
  if(giroBlock == 0 && lastSub != 4 ){
    //salva as posições de entrada na rotina dos motores,as posições serão usadas para sincronizar os motores
    MotorZero();
    //calcula as posições de parada possiveis para liberar o giroblock, nesta versão ela será a cada volta completa, é possivel em futuras versões liberar a roda a cada meio giro
    // pois os motores se encontram en sincronismo no zero e nas posições 1024. Porém para se fazer isso é necessário garantir que a rotina sempre recebe o motor em zero ou 1024.
    // Assim é necessrio modificar todas para serem no estilo das funçoes de giro
    SincPosition(1,-1);
    giroBlock = 4;// adquire Lock
    if(debug==1){// debug da função
      Serial.println("TurnLeft() lock obtido, executado MotorZero();,SincPosition(-1,1); e giroblock =3  ");
      Serial.println("TurnLeft() reqStop = " + String(reqStop));
    }
  } 
  //A cada giro completo os motores se sincronizam
  else{// se a sub foi inicializada agora só cai aqui
   for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){ // para cada motor
     if(motorSincPosition[stepperNumber] == steppers[stepperNumber]->currentPosition()) {//se posição de parada é atingida    
       if (reqStop == 1){
         Serial.println("lock liberado pelo motor" + String(stepperNumber) + " valor de reqStop: " + String(reqStop)); 
         giroBlock = 0;
         reqStop = 0;
         NormalizePosition(1);
        }    
       else{
         //ativa os drivers
         driversON;
         //seta a direção dos motores
         SetSteppersSpeed(1, -1);
         //gira o motor em sincronia
         RunGiroSinc(stepperNumber);
         //realcula as novas posições
         if(stepperNumber  == 1){// so no segundo motor
           MotorZero();
           SincPosition(1,-1);
          }
        }
      }
      else{
        //ativa os drivers
        driversON;
        //seta a direção dos motores
        SetSteppersSpeed(1, -1);
        //Gira o motor em sincronia
        RunGiroSinc(stepperNumber);
      }
    }
  }
  lastSub = 4;// codigo da sub TurnLeft()
}


void MotorStop() {
  for(int stepperNumber = 0; stepperNumber < stepperAmount; stepperNumber++){
   steppers[stepperNumber]->setSpeed(0);
  }
 driversOFF;
 //Serial.println("posicão motor 1 "   + String(steppers[0]->currentPosition()));
 //Serial.println("posicão motor 2 "   + String(steppers[1]->currentPosition()));
 lastSub = 5;// codigo da sub MotorBAckward()
}

// pega o numero de passos por volta
void GetStepsByTurn(int motor) {
  driversON;
 if(motor == 1){ 
   while(readHall1 != 1){//move motor 1 outside
     steppers[0]->setSpeed(SPEED);
     steppers[0]->run();
    }
   steppers[0]->setCurrentPosition(0);
   while(readHall1 == 1){        
      steppers[0]->setSpeed(SPEED);
      steppers[0]->run();
    }
    Serial.println("passos por volta: " + String(steppers[0]->currentPosition()+ hallSensorSize[0]));
  }
   if(motor == 2){ 
   while(readHall2 != 1){//move motor 1 outside
     steppers[1]->setSpeed(SPEED);
     steppers[1]->run();
    }
   steppers[1]->setCurrentPosition(0);
   while(readHall2 == 1){        
      steppers[1]->setSpeed(SPEED);
      steppers[1]->run();
    }
    Serial.println("passos por volta: " + String(steppers[1]->currentPosition() + hallSensorSize[1]));
  }  
  driversOFF;
}

