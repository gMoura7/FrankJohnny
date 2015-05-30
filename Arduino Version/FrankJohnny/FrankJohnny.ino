#include <NXTShield.h>
#include <Wire.h>
#include <QTRSensors.h>

/*Ultrassonicos------------------------------------------*/
int ultrapwm = 34, ultratrig = 35;
int esquerdaPwm = 0, esquerdaTrig = 0;
uint8_t EmPwmCmd[4] = {
  0x44,0x22,0xbb,0x01};
  
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
  ultraSetup(ultrapwm, ultratrig);
  calibrar(-250, 250);
}
/*------------------------------------------*/
void loop(){
  GAP();
  if(ultraCheck(ultrapwm, ultratrig) <= 10) desvio(esquerdaPwm, esquerdaTrig); //Procura os obstáculos
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
  Motor1.move( (sentido(m1)), (setspeeds(m1)) );
  Motor2.move( (sentido(m2)), (setspeeds(m2)) );
}
/*------------------------------------------*/
void GAP(){
  while ( (sensors[0]<600 || sensors[0]> 6000) && (sensors[1]<600 || sensors[1]> 6000) && (sensors[2]<600 || sensors[2]> 6000) && (sensors[3]<600 || sensors[3]> 6000)
    && (sensors[4]<600 || sensors[4]> 6000) && (sensors[5]<600 || sensors[5]> 6000) && (sensors[6]<600 || sensors[6]> 6000) && (sensors[7]<600 || sensors[7]> 6000) ){
    Motor1.move(FORWARD, 200 );
    Motor2.move(FORWARD, 200);  
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


/*-------------------------------------------
 * A função "desvio" a seguir será explicada em 6 passos, desde o início ao fim da mesma.
 * 1º Passo: O robô, após ter identificado o obstáculo antes, irá girar 90º graus para a Direita.
 * 2º Passo: A seguir ele andará para frente, enquanto estiver "enxergando" o obstáculo.
 * 3º Passo: Irá girar 90º graus para a Esquerda.
 * 4º Passo: Logo após ele irá andar novamente para frente enquanto estiver vendo o obstáculo.(Observação: O delay de 0.5 Segundos serve para que o robô consiga completar a lateral do obstáculo)
 * 5º Passo: O robô irá girar 45º graus para a Esquerda.
 * 6º Passo: E, finalmente, irá andar até encontrar a linha preta
 * Observação: Os resets dos encoders não estão incluídos nos passos pois eles são essenciais para o funcionamento da função, sendo desnecessário mencionaná-los.
 -------------------------------------------*/
void desvio(int esquerdaPwm, int esquerdaTrig){ //Função para desviar do obstáculo
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o acúmulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o acúmulo de valores
  
  do{
    Motor1.move(FORWARD, 200); //Faz o motor 1 andar para "trás" com força 200
    Motor2.move(BACKWARD, 200);  //Faz o motor 1 andar para "trás" com força 200...
    //Simplificando, faz o robô girar para a Direita com a mesma velocidade em ambos os motores
  } while( (Motor1.readPosition()) > -500 && (Motor2.readPosition()) < 500); //...Enquanto a posição do motor 1 for maior que -500 (-90 graus) E a posição do motor 2 for menor que 500 (90 graus)

  while( (ultraCheck(esquerdaPwm, esquerdaTrig)) <= 10){ //Enquanto a distância medida pelo sensor ultrassônica da esquerda for menor que 10cm
    Motor1.move(BACKWARD, 200); //Mova o motor 1 para "trás" com força 200
    Motor2.move(BACKWARD, 200); //Mova o motor 2 para "trás" com força 200
    delay(500); //"Atrasa" 0.5 Segundos antes de checar novamente a condição
    //Simplificando, faz o robô andar para "trás" (que é a nossa frente devido à montagem) com força 200
  }
  
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o acúmulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o acúmulo de valores
  
  do{ //O "do" executa o bloco de código primeiro para depois checar a condição do  while
    Motor1.move(BACKWARD, 200); //Mova o motor 1 para "frente" com força 200
    Motor2.move(FORWARD, 200); //Mova o motor 2 para "trás" com força 200...
    //Simplificando, faz o robô girar para a Esquerda com a mesma velocidade em ambos os motores
  }while( (Motor1.readPosition()) < 500 && (Motor2.readPosition()) > -500); //...Enquanto a posição do motor 1 for menor que 500 E a do motor 2 for maior que -500
  
  do{ 
    Motor1.move(BACKWARD, 200); //Mova o motor 1 para "trás" com força 200
    Motor2.move(BACKWARD, 200); //Mova o motor 2 para "trás" com força 200
    delay(500); //Aguarde 0,5 segundos antes (nesse período o robô não parará de andar)
  }while( (ultraCheck(esquerdaPwm, esquerdaTrig)) <= 10); //Enquanto a distância medida pelo sensor ultrassônica da esquerda for menor que 10cm
  
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o acúmulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o acúmulo de valores
  
  do{
    Motor1.move(FORWARD, 200); //Mova o motor 1 para "trás" com força 200
    Motor2.move(BACKWARD, 200); //Mova o motor 2 para "frente" com força 200
  }while( (Motor1.readPosition()) > -250 && (Motor2.readPosition()) < 250); //...Enquanto a posição do motor 1 for maior que -250 (-45 graus) E a do motor 2 for menor que 250 (45 graus)
  
  do{
    Motor1.move(BACKWARD, 200); //Mova o motor 1 para "trás" com força 200
    Motor2.move(BACKWARD, 200); //Mova o motor 2 para "trás" com força 200...
  }while( (qtra.readLine(sensors)) < 3000 || (qtra.readLine(sensors)) > 3900); //...Enquanto o robô não encontrar a linha preta
} //Fim da função

/*------------------------------------------*/
void calibrar(int vel1, int vel2){
    Motor1.move(FORWARD, 200 ); //Faz o motor 1 andar para "frente" com força 200
    Motor2.move(BACKWARD, 200 );  //Faz o motor 2 andar para "trás" com força 200
    //Faz o robô girar para a Direita com a mesma velocidade em ambos os motores
  for(int x = 0; x < 90; x++){
    qtra.calibrate(); //Calibra os sensores
    delay(20); //Aguarda 0.02 segundos antes de prosseguir o loop
  }
  
  while(qtra.readLine(sensors) < 3000 || qtra.readLine(sensors) > 3900){ //Gira os motores para a Esquerda enquanto a linha preta não é encontrada
    Motor1.move(BACKWARD, 200 );  //Faz o motor 1 andar para "trás" com força 200
    Motor2.move(FORWARD, 200);  //Faz o motor 2 andar para "frente" com força 200
    //Faz o robô girar para a Esquerda com a mesma velocidade em ambos os motores
  }
}
/*------------------------------------------*/

int setspeeds(int vel) { 
  if(vel < 0) return -vel; //Se o valor (velocidade neste caso) entregue a função for negativo, retorne o oposto desse valor, isto é, multiplique o valor negativo por "-1".
  return vel; //...Retorne o mesmo valor
}

int sentido(int vel){
  if(vel < 0) return BACKWARD; //Se o valor (velocidade neste caso) entregue a função for negativo, retorne o sentido como sendo para TRÁS, caso contrário...
  return FORWARD; //...Retorne o sentido como sendo para FRENTE
}
