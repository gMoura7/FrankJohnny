#include <DualVNH5019MotorShield.h>
#include <NXTShield.h>
#include <Wire.h>
#include <QTRSensors.h>

bool M1turning = false, M2turning = false;

/*Ultrassonicos------------------------------------------*/
int ultrapwm = 34, ultratrig = 35;
int esquerdaPwm = 0, esquerdaTrig = 0;
uint8_t EmPwmCmd[4] = {
  0x44, 0x22, 0xbb, 0x01
};

/*Sensores-IR---------------------------------------------*/
QTRSensorsAnalog qtra((unsigned char[]) {
  A15, A14, A13, A12, A11, A10, A9
}
, 7);
unsigned int sensors[7];
/*PID/Motores------------------------------------------*/
const float KP = 0.13, KI = 0, KD = 0;
const int Motors = -150;
int P = 0, I = 0, D = 0;
int erro = 0, last_erro = 0, deltaTime = 0, last_time = 0;
DualVNH5019MotorShield md;

/*Setup------------------------------------------*/
void setup() {
  ultraSetup(ultrapwm, ultratrig);
  calibrar(-250, 250);
  Serial.begin(9600);
}
/*------------------------------------------*/
void loop() {
  // GAP();
  if (ultraCheck(ultrapwm, ultratrig) <= 10) desvio(esquerdaPwm, esquerdaTrig); //Procura os obstaculos
  //encruzilhada();
  sigaLinha();
}
/*------------------------------------------*/
void sigaLinha() {
  int position = qtra.readLine(sensors);
  erro = 3500 - position; //Segue a linha preta(devido a montagem invertida dos motores)
  //erro = position - 3500; //"Foge" da linha preta(devido a montagem invertida dos motores)
  deltaTime = millis() - last_time;
  last_time = millis();
  last_erro = erro;

  P = erro * KP;
  I += (erro * KI) * deltaTime;
  D = (last_erro - erro) * KD / deltaTime;

  int velocidade = P + I + D;

  int m1 = Motors - velocidade;
  int m2 = Motors + velocidade;

  m1 = constrain(m1, -250, 250);
  m2 = constrain(m2, -250, 250);
  md.setSpeeds(m1, m2);
}
/*------------------------------------------*/
void GAP() {
  while ( (sensors[0] < 600 || sensors[0] > 6000) && (sensors[1] < 600 || sensors[1] > 6000) && (sensors[2] < 600 || sensors[2] > 6000) && (sensors[3] < 600 || sensors[3] > 6000)
          && (sensors[4] < 600 || sensors[4] > 6000) && (sensors[5] < 600 || sensors[5] > 6000) && (sensors[6] < 600 || sensors[6] > 6000) && (sensors[7] < 600 || sensors[7] > 6000) ) {
    md.setSpeeds(-200, -200);
    qtra.readLine(sensors);
  }
}
/*------------------------------------------*/
void ultraSetup(int ultraPWM, int ultraTrig) {
  pinMode(ultraTrig, OUTPUT);                           // A low pull on pin COMP/TRIG
  digitalWrite(ultraTrig, HIGH);                        // Set to HIGH
  pinMode(ultraPWM, INPUT);                             // Sending Enable PWM mode command
  for(int x =0; x < 4; x++) Serial.write(EnPwmCmd[x]);
}
/*------------------------------------------*/
int ultraCheck(int ultrapwm, int ultratrig) {
  unsigned int distance = 0;
  digitalWrite(ultratrig, LOW);
  digitalWrite(ultratrig, HIGH);
  unsigned long medicao = pulseIn(ultrapwm, LOW);
  if (medicao < 50000 && medicao >= 2) distance = medicao / 50;
  else return 50000;
  return distance;
}


/*-------------------------------------------
 * A funçao "desvio" a seguir sera explicada em 6 passos, desde o inicio ao fim da mesma.
 * 1º Passo: O robo, apos ter identificado o obstaculo antes, ira girar 90º graus para a Direita.
 * 2º Passo: A seguir ele andara para frente, enquanto estiver "enxergando" o obstaculo.
 * 3º Passo: Ira girar 90º graus para a Esquerda.
 * 4º Passo: Logo apos ele ira andar novamente para frente enquanto estiver vendo o obstaculo.(Observaçao: O delay de 0.5 Segundos serve para que o robo consiga completar a lateral do obstaculo)
 * 5º Passo: O robo ira girar 45º graus para a Esquerda.
 * 6º Passo: E, finalmente, ira andar ate encontrar a linha preta
 * Observacao: Os resets dos encoders nao estao incluidos nos passos pois eles sao essenciais para o funcionamento da funçao, sendo desnecessario mencionana-los.
 -------------------------------------------*/
void desvio(int esquerdaPwm, int esquerdaTrig){ //Fun��o para desviar do obst�culo
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o ac�mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o ac�mulo de valores
  
  do{
  move1(FORWARD, 200, 425, BRAKE); //Faz o motor 1 andar para "frente" com for�a 200 por 150 rota��es
  move2(BACKWARD, 200, 425, BRAKE);  //Faz o motor 2 andar para "tr�s" com for�a 200 por 150 rota��es
  }while (M1turning || M2turning); //Espera um dos motores terminar seu movimento
  
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o ac�mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o ac�mulo de valores
  
  do{//Anda para frente com 150 rota��es
  move1(BACKWARD, 200, 700, BRAKE); //Faz o motor 1 andar para "tr�s" com for�a 200 por 150 rota��es
  move2(BACKWARD, 200, 700, BRAKE); //Faz o motor 2 andar para "tr�s" com for�a 200 por 150 rota��es
  }while (M1turning || M2turning);
  
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o ac�mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o ac�mulo de valores

  do{//faz a 2� Virada
  move1(BACKWARD, 200, 450, BRAKE); //Faz o motor 1 andar para "tr�s" com for�a 200 por 150 rota��es
  move2(FORWARD, 200, 450, BRAKE);  //Faz o motor 2 andar para "frente" com for�a 200 por 150 rota��es
  }while (M1turning || M2turning); //Espera um dos motores terminar seu movimento
  
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o ac�mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o ac�mulo de valores

  do{//Anda para frente at� a hora da 3� Virada
  move1(BACKWARD, 200, 900, BRAKE); //Faz o motor 1 andar para "tr�s" com for�a 200 por 150 rota��es
  move2(BACKWARD, 200, 900, BRAKE); //Faz o motor 2 andar para "tr�s" com for�a 200 por 150 rota��es
  }while (M1turning || M2turning);

  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o ac�mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o ac�mulo de valores
  
  do{
  move1(BACKWARD, 200, 250, BRAKE); //Faz o motor 1 andar para "tr�s" com for�a 200
  move2(FORWARD, 200, 250, BRAKE);  //Faz o motor 2 andar para "frente" com for�a 200...
  }while (M1turning || M2turning); //Espera um dos motores terminar seu movimento

  while(qtra.readLine(sensors) < 3000 || qtra.readLine(sensors) > 3900){ //Gira os motores para a Esquerda enquanto a linha preta n�o � encontrada
    move1(BACKWARD, 200 );  //Faz o motor 1 andar para "tr�s" com for�a 200
    move2(BACKWARD, 200);  //Faz o motor 2 andar para "frente" com for�a 200
    //Faz o rob� girar para a Esquerda com a mesma velocidade em ambos os motores
  }
} //Fim da fun��o

/*------------------------------------------*/
void calibrar(int vel1, int vel2) {
  md.setSpeeds(200, -200); 
  for (int x = 0; x < 90; x++) {
      qtra.calibrate();
      delay(20);
  }
  while (qtra.readLine(sensors) < 3000 || qtra.readLine(sensors) > 3900) {
    md.setSpeeds(200, -200);
  }
}
/*------------------------------------------*/
void move1(int direction, int speed, int rotation, int halt){
  bool brake = false;
  int pos = 0, speeDef = 0;

  if(halt == BRAKE) brake = true;
  if(direction == FORWARD && speed < 0) speeDef = -speed;
  else if(direction == BACKWARD && speed > 0) speeDef = -speed;
  else speeDef = speed;
  
  md.setM1Speed(speeDef);

  if(direction == FORWARD){ 
    pos = (Motor1.readPosition()) - rotation;
  
    if(pos > 0) M1turning = true;
    else {
      M1turning = false;
      if(brake == true) md.setM1Speed(0);
    }
  }
  
  else if(direction == BACKWARD) {
    pos = (Motor1.readPosition()) + rotation;
    
    if(pos < 0) M1turning = true;
    else {
      M1turning = false;
      if(brake == true) md.setM1Speed(0);
    }
  }
}
/*------------------------------------------*/
void move2(int direction, int speed, int rotation, int halt){
  bool brake = false;
  int pos = 0, speeDef = 0;
  
  if(halt == BRAKE) brake = true;
  if(direction == FORWARD && speed < 0) speeDef = -speed;
  else if(direction == BACKWARD && speed > 0) speeDef = -speed;
  else speeDef = speed;
  
  md.setM2Speed(speeDef);

  if(direction == FORWARD){ 
    pos = (Motor2.readPosition()) - rotation;
  
    if(pos > 0) M2turning = true;
    else {
      M2turning = false;
      if(brake == true) md.setM2Speed(0);
    }
  }
  
  else if(direction == BACKWARD) {
    pos = (Motor2.readPosition()) + rotation;
    
    if(pos < 0) M2turning = true;
    else {
      M2turning = false;
      if(brake == true) md.setM2Speed(0);
    }
  }
}