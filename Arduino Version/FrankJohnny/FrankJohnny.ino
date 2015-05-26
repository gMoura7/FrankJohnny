#include <NXTShield.h>
#include <Wire.h>
#include <DualVNH5019MotorShield.h>
#include <QTRSensors.h>

/*Ultrassonicos------------------------------------------*/
int ultrapwm = 34, ultratrig = 35;
int esquerdaPwm = 0, esquerdaTrig = 0;
uint8_t EmPwmCmd[4] = {
  0x44,0x22,0xbb,0x01};
/*Iniciar-Motores------------------------------------------*/
DualVNH5019MotorShield md;
/*Sensores-IR---------------------------------------------*/
QTRSensorsAnalog qtra((unsigned char[]) {
  A15,A14,A13,A12,A11,A10,A9}
, 7);
unsigned int sensors[7];
/*PID/Motores------------------------------------------*/
const float KP = 0.13, KI = 0, KD = 0;
const int Motors = -150;
int P = 0, I = 0, D = 0;
int erro = 0, last_erro = 0, deltaTime = 0, last_time = 0;
/*Setup------------------------------------------*/
void setup(){
  md.init();
  ultraSetup(ultrapwm, ultratrig);
  calibrar(-250, 250);
}
/*------------------------------------------*/
void loop(){
  GAP();
  if(ultraCheck(ultrapwm, ultratrig) < 10) desvio(esquerdaPwm, esquerdaTrig); //Procura os obst�culos
  //encruzilhada();
  sigaLinha();
}
/*------------------------------------------*/
void sigaLinha(){
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
void GAP(){
  while ( (sensors[0]<600 || sensors[0]> 6000) && (sensors[1]<600 || sensors[1]> 6000) && (sensors[2]<600 || sensors[2]> 6000) && (sensors[3]<600 || sensors[3]> 6000)
    && (sensors[4]<600 || sensors[4]> 6000) && (sensors[5]<600 || sensors[5]> 6000) && (sensors[6]<600 || sensors[6]> 6000) && (sensors[7]<600 || sensors[7]> 6000) ){
    md.setSpeeds(-200, -200); //Verifica GAP na pista
    qtra.readLine(sensors);
  }
}
/*------------------------------------------*/
void ultraSetup(int ultraPWM, int ultraTrig){
  pinMode(ultraTrig, OUTPUT);                           // A low pull on pin COMP/TRIG
  digitalWrite(ultraTrig, HIGH);                        // Set to HIGH
  pinMode(ultraPWM, INPUT);                             // Sending Enable PWM mode command
}
/*------------------------------------------*/
int ultraCheck(int ultrapwm, int ultratrig){
  unsigned int distance = 0;
  digitalWrite(ultratrig, LOW);
  digitalWrite(ultratrig, HIGH);
  unsigned long medicao = pulseIn(ultrapwm, LOW);
  if(medicao < 50000) distance = medicao/50;
  else return 50000;
  return distance;
}
/*-------------------------------------------*/
void desvio(int esquerdaPwm, int esquerdaTrig){ //Esta fun��o deve ser � prova de erros

  Motor1.resetPosition(); //Reseta o encoder para que n�o acumule valores, resultando em erro na pr�xima instru��o

  while( (Motor1.readPosition()) != -225) md.setSpeeds(-250, 250); //Enquanto na linha preta, vira cerca de 45� � direita

  while( (ultraCheck(esquerdaPwm, esquerdaTrig)) < 10 ) md.setSpeeds(-200,-200); //Segue em frente enquanto o ultrassonico da esquerda "enxergar" o obstáculo

  Motor1.resetPosition();

  while( (Motor1.readPosition()) != 225) md.setSpeeds(250, -250); //Assim que o ultrass�nico n�o "enxergar" mais, o rob� vira 45� � esquerda

  while( (ultraCheck(esquerdaPwm, esquerdaTrig)) < 10) md.setSpeeds(-200,-200); 

  Motor1.resetPosition();

  while( (Motor1.readPosition()) != 225) md.setSpeeds(250, -250); 

  while( ( (qtra.readLine(sensors)) -3500) < -300 || ( (qtra.readLine(sensors)) -3500) > 300) md.setSpeeds(-200,-200);//Trocar essa instru��o por alguma que interrompa o loop assim que a linha preta seja encontrada
}
/*------------------------------------------*/
void calibrar(int vel1, int vel2){
  md.setSpeeds(vel1, vel2);
  for(int x = 0; x < 90; x++){
    qtra.calibrate();
    delay(20);
  }
  while(qtra.readLine(sensors) < 3000 || qtra.readLine(sensors) > 3900) md.setSpeeds(-200, 200);
}
/*------------------------------------------*/

