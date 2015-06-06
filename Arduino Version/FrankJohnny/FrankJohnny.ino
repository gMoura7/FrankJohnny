
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
  // GAP();
  if(ultraCheck(ultrapwm, ultratrig) <= 10) desvio(esquerdaPwm, esquerdaTrig); //Procura os obst�culos
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
  if(medicao < 50000 && medicao >= 2) distance = medicao/50;
  else return 50000;
  return distance;
}


/*-------------------------------------------
 * A fun��o "desvio" a seguir ser� explicada em 6 passos, desde o in�cio ao fim da mesma.
 * 1� Passo: O rob�, ap�s ter identificado o obst�culo antes, ir� girar 90� graus para a Direita.
 * 2� Passo: A seguir ele andar� para frente, enquanto estiver "enxergando" o obst�culo.
 * 3� Passo: Ir� girar 90� graus para a Esquerda.
 * 4� Passo: Logo ap�s ele ir� andar novamente para frente enquanto estiver vendo o obst�culo.(Observa��o: O delay de 0.5 Segundos serve para que o rob� consiga completar a lateral do obst�culo)
 * 5� Passo: O rob� ir� girar 45� graus para a Esquerda.
 * 6� Passo: E, finalmente, ir� andar at� encontrar a linha preta
 * Observa��o: Os resets dos encoders n�o est�o inclu�dos nos passos pois eles s�o essenciais para o funcionamento da fun��o, sendo desnecess�rio mencionan�-los.
 -------------------------------------------*/
void desvio(int esquerdaPwm, int esquerdaTrig){ //Fun��o para desviar do obst�culo
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o ac�mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o ac�mulo de valores
  
  //1� Virada do robo
  Motor1.move(FORWARD, 200, 425, BRAKE); //Faz o motor 1 andar para "frente" com for�a 200 por 150 rota��es
  Motor2.move(BACKWARD, 200, 425, BRAKE);  //Faz o motor 2 andar para "tr�s" com for�a 200 por 150 rota��es
  while (Motor1.isTurning() || Motor2.isTurning()); //Espera um dos motores terminar seu movimento
  
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o ac�mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o ac�mulo de valores
  
//Anda para frente com 150 rota��es
  Motor1.move(BACKWARD, 200, 700, BRAKE); //Faz o motor 1 andar para "tr�s" com for�a 200 por 150 rota��es
  Motor2.move(BACKWARD, 200, 700, BRAKE); //Faz o motor 2 andar para "tr�s" com for�a 200 por 150 rota��es
  while (Motor1.isTurning() || Motor2.isTurning());
  
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o ac�mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o ac�mulo de valores

  //faz a 2� Virada
  Motor1.move(BACKWARD, 200, 450, BRAKE); //Faz o motor 1 andar para "tr�s" com for�a 200 por 150 rota��es
  Motor2.move(FORWARD, 200, 450, BRAKE);  //Faz o motor 2 andar para "frente" com for�a 200 por 150 rota��es
  while (Motor1.isTurning() || Motor2.isTurning()); //Espera um dos motores terminar seu movimento
  
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o ac�mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o ac�mulo de valores

  //Anda para frente at� a hora da 3� Virada
  Motor1.move(BACKWARD, 200, 900, BRAKE); //Faz o motor 1 andar para "tr�s" com for�a 200 por 150 rota��es
  Motor2.move(BACKWARD, 200, 900, BRAKE); //Faz o motor 2 andar para "tr�s" com for�a 200 por 150 rota��es
  while (Motor1.isTurning() || Motor2.isTurning());

  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o ac�mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o ac�mulo de valores
  
  Motor1.move(BACKWARD, 200, 250, BRAKE); //Faz o motor 1 andar para "tr�s" com for�a 200
  Motor2.move(FORWARD, 200, 250, BRAKE);  //Faz o motor 2 andar para "frente" com for�a 200...
  while (Motor1.isTurning() || Motor2.isTurning()); //Espera um dos motores terminar seu movimento

  while(qtra.readLine(sensors) < 3000 || qtra.readLine(sensors) > 3900){ //Gira os motores para a Esquerda enquanto a linha preta n�o � encontrada
    Motor1.move(BACKWARD, 200 );  //Faz o motor 1 andar para "tr�s" com for�a 200
    Motor2.move(BACKWARD, 200);  //Faz o motor 2 andar para "frente" com for�a 200
    //Faz o rob� girar para a Esquerda com a mesma velocidade em ambos os motores
  }
} //Fim da fun��o

/*------------------------------------------*/
void calibrar(int vel1, int vel2){
    Motor1.move(FORWARD, 200 ); //Faz o motor 1 andar para "frente" com for�a 200
    Motor2.move(BACKWARD, 200 );  //Faz o motor 2 andar para "tr�s" com for�a 200
    //Faz o rob� girar para a Direita com a mesma velocidade em ambos os motores
  for(int x = 0; x < 90; x++){
    qtra.calibrate(); //Calibra os sensores
    delay(20); //Aguarda 0.02 segundos antes de prosseguir o loop
  }
  
  while(qtra.readLine(sensors) < 3000 || qtra.readLine(sensors) > 3900){ //Gira os motores para a Esquerda enquanto a linha preta n�o � encontrada
    Motor1.move(BACKWARD, 200 );  //Faz o motor 1 andar para "tr�s" com for�a 200
    Motor2.move(FORWARD, 200);  //Faz o motor 2 andar para "frente" com for�a 200
    //Faz o rob� girar para a Esquerda com a mesma velocidade em ambos os motores
  }
}
/*------------------------------------------*/

int setspeeds(int vel) { 
  if(vel < 0) return -vel; //Se o valor (velocidade neste caso) entregue a fun��o for negativo, retorne o oposto desse valor, isto �, multiplique o valor negativo por "-1".
  return vel; //...Retorne o mesmo valor
}

int sentido(int vel){
  if(vel < 0) return BACKWARD; //Se o valor (velocidade neste caso) entregue a fun��o for negativo, retorne o sentido como sendo para TR�S, caso contr�rio...
  return FORWARD; //...Retorne o sentido como sendo para FRENTE
}
