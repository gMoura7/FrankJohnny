#include <NXTShield.h>
#include <Wire.h>
#include <QTRSensors.h>

/*-----------------Sensor-de-Cor-------------------------*/
int LDR = A0; // ldr sensor on analog pin 0
const int redLed = 0;
const int greenLed = 1;
const int blueLed = 2;
const int originLDR = LDR;
int ledArray[] = {redLed, greenLed, blueLed};

bool CalibrarCorState = false;

float colorArray[] = {0, 0, 0};
float whiteArray[] = {422, 353, 322};
float blackArray[] = {229, 142, 141};

//int avgRead;

int ldrvalue = 0;

// Values for detected colors
int redval = 0;
int greenval = 0;
int blueval = 0;

const int TIMESG = 5;

bool balanced = false;

/*Ultrassonicos------------------------------------------*/
int ultrapwm = 34, ultratrig = 35;
uint8_t EnPwmCmd[4] = {
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
const int MOtors = -150;
int P = 0, I = 0, D = 0;
int erro = 0, last_erro = 0, deltaTime = 0, last_time = 0;

const int andar = 43, calibrarMotor = 41, HG = 44;
bool calibrateComplete = false;
bool sala3 = false;

/*Setup------------------------------------------*/
void setup() {
  pinMode (calibrarMotor, INPUT);
  pinMode (andar, INPUT);
  pinMode (HG, INPUT);
  
  ultraSetup(ultrapwm, ultratrig); //seta os ultrass�nicos
  
  while (!calibrateComplete) {
    if (digitalRead(calibrarMotor) == LOW)
      calibrar(250);

    if (digitalRead(andar) == LOW) calibrateComplete = true;
  }
}
/*------------------------------------------*/
void loop() {
  rampa(HG);
  GAP();
  noventaGraus();
  encruzilhada();
  if(ultraCheck(ultrapwm, ultratrig) <= 7) desvio();
  //Serial.println( (ultraCheck(ultrapwm, ultratrig)) );
  sigaLinha(MOtors);
}
/*-------Seguir-Linha-----------------------*/
void sigaLinha(int Motors) {
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
/*-----------------GAPs---------------------*/
void GAP() {
  while ( (sensors[0] < 600 || sensors[0] > 6000) && (sensors[1] < 600 || sensors[1] > 6000) && (sensors[2] < 600 || sensors[2] > 6000) && (sensors[3] < 600 || sensors[3] > 6000)
          && (sensors[4] < 600 || sensors[4] > 6000) && (sensors[5] < 600 || sensors[5] > 6000) && (sensors[6] < 600 || sensors[6] > 6000) && (sensors[7] < 600 || sensors[7] > 6000) ) {
    Motor1.move(FORWARD, 200 );
    Motor2.move(FORWARD, 200);
    qtra.readLine(sensors);
  }
}
/*----------Obstaculo----------------------*/
void ultraSetup(int ultraPWM, int ultraTrig) {
  pinMode(ultraTrig, OUTPUT);                           // A low pull on pin COMP/TRIG
  digitalWrite(ultraTrig, HIGH);                        // Set to HIGH
  pinMode(ultraPWM, INPUT);                             // Sending Enable PWM mode command
  //for(int x =0; x <4; x++) Serial.write(EnPwmCmd[x]);
}
/*-----------Obstaculo0---------------------*/
int ultraCheck(int ultraPwm, int ultraTrig) {
  int distance = 0;
  digitalWrite(ultraTrig, LOW);
  digitalWrite(ultraTrig, HIGH);
  unsigned long medicao = pulseIn(ultraPwm, LOW);
  if (medicao < 50000 && medicao > 150) {
    distance = medicao / 50;
    //desvio();
  }
  return distance;
}


/*-------------Obstaculo---------------------
 * A funï¿½ï¿½o "desvio" a seguir serï¿½ explicada em 6 passos, desde o inï¿½cio ao fim da mesma.
 * 1ï¿½ Passo: O robï¿½, apï¿½s ter identificado o obstï¿½culo antes, irï¿½ girar 90ï¿½ graus para a Direita.
 * 2ï¿½ Passo: A seguir ele andarï¿½ para frente, enquanto estiver "enxergando" o obstï¿½culo.
 * 3ï¿½ Passo: Irï¿½ girar 90ï¿½ graus para a Esquerda.
 * 4ï¿½ Passo: Logo apï¿½s ele irï¿½ andar novamente para frente enquanto estiver vendo o obstï¿½culo.(Observaï¿½ï¿½o: O delay de 0.5 Segundos serve para que o robï¿½ consiga completar a lateral do obstï¿½culo)
 * 5ï¿½ Passo: O robï¿½ irï¿½ girar 45ï¿½ graus para a Esquerda.
 * 6ï¿½ Passo: E, finalmente, irï¿½ andar atï¿½ encontrar a linha preta
 * Observaï¿½ï¿½o: Os resets dos encoders nï¿½o estï¿½o incluï¿½dos nos passos pois eles sï¿½o essenciais para o funcionamento da funï¿½ï¿½o, sendo desnecessï¿½rio mencionanï¿½-los.
 ---------------Obstaculo-------------------*/
void desvio() { //Funï¿½ï¿½o para desviar do obstï¿½culo
  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o acï¿½mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o acï¿½mulo de valores

  //1ï¿½ Virada do robo
  Motor1.move(FORWARD, 200, 425, BRAKE); //Faz o motor 1 andar para "frente" com forï¿½a 200 por 150 rotaï¿½ï¿½es
  Motor2.move(BACKWARD, 200, 425, BRAKE);  //Faz o motor 2 andar para "trï¿½s" com forï¿½a 200 por 150 rotaï¿½ï¿½es
  while (Motor1.isTurning() || Motor2.isTurning()); //Espera um dos motores terminar seu movimento

  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o acï¿½mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o acï¿½mulo de valores

  //Anda para frente com 150 rotaï¿½ï¿½es
  Motor1.move(BACKWARD, 200, 700, BRAKE); //Faz o motor 1 andar para "trï¿½s" com forï¿½a 200 por 150 rotaï¿½ï¿½es
  Motor2.move(BACKWARD, 200, 700, BRAKE); //Faz o motor 2 andar para "trï¿½s" com forï¿½a 200 por 150 rotaï¿½ï¿½es
  while (Motor1.isTurning() || Motor2.isTurning());

  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o acï¿½mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o acï¿½mulo de valores

  //faz a 2ï¿½ Virada
  Motor1.move(BACKWARD, 200, 450, BRAKE); //Faz o motor 1 andar para "trï¿½s" com forï¿½a 200 por 150 rotaï¿½ï¿½es
  Motor2.move(FORWARD, 200, 450, BRAKE);  //Faz o motor 2 andar para "frente" com forï¿½a 200 por 150 rotaï¿½ï¿½es
  while (Motor1.isTurning() || Motor2.isTurning()); //Espera um dos motores terminar seu movimento

  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o acï¿½mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o acï¿½mulo de valores

  //Anda para frente atï¿½ a hora da 3ï¿½ Virada
  Motor1.move(BACKWARD, 200, 900, BRAKE); //Faz o motor 1 andar para "trï¿½s" com forï¿½a 200 por 150 rotaï¿½ï¿½es
  Motor2.move(BACKWARD, 200, 900, BRAKE); //Faz o motor 2 andar para "trï¿½s" com forï¿½a 200 por 150 rotaï¿½ï¿½es
  while (Motor1.isTurning() || Motor2.isTurning());

  Motor1.resetPosition(); //Reseta o encoder 1 para previnir o acï¿½mulo de valores
  Motor2.resetPosition(); //Reseta o encoder 2 para previnir o acï¿½mulo de valores

  Motor1.move(BACKWARD, 200, 250, BRAKE); //Faz o motor 1 andar para "trï¿½s" com forï¿½a 200
  Motor2.move(FORWARD, 200, 250, BRAKE);  //Faz o motor 2 andar para "frente" com forï¿½a 200...
  while (Motor1.isTurning() || Motor2.isTurning()); //Espera um dos motores terminar seu movimento

  while (qtra.readLine(sensors) < 3000 || qtra.readLine(sensors) > 3900) { //Gira os motores para a Esquerda enquanto a linha preta nï¿½o ï¿½ encontrada
    Motor1.move(BACKWARD, 200 );  //Faz o motor 1 andar para "trï¿½s" com forï¿½a 200
    Motor2.move(BACKWARD, 200);  //Faz o motor 2 andar para "frente" com forï¿½a 200
    //Faz o robï¿½ girar para a Esquerda com a mesma velocidade em ambos os motores
  }
} //Fim da funï¿½ï¿½o

/*---------Motores--------------------------*/
void calibrar(int vel) {
  Motor1.move(FORWARD, vel ); //Faz o motor 1 andar para "frente" com forï¿½a 200
  Motor2.move(BACKWARD, vel );  //Faz o motor 2 andar para "trï¿½s" com forï¿½a 200
  //Faz o robï¿½ girar para a Direita com a mesma velocidade em ambos os motores
  for (int x = 0; x < 100; x++) {
    qtra.calibrate(); //Calibra os sensores
    delay(20); //Aguarda 0.02 segundos antes de prosseguir o loop
  }

  /*while (qtra.readLine(sensors) < 3000 || qtra.readLine(sensors) > 3900) { //Gira os motores para a Esquerda enquanto a linha preta nï¿½o ï¿½ encontrada
    Motor1.move(FORWARD, vel );  //Faz o motor 1 andar para "trï¿½s" com forï¿½a 200
    Motor2.move(BACKWARD, vel);  //Faz o motor 2 andar para "frente" com forï¿½a 200
    //Faz o robï¿½ girar para a Esquerda com a mesma velocidade em ambos os motores
  }
  */
}
/*-----------Motores------------------------*/

int setspeeds(int vel) {
  if (vel < 0) return -vel; //Se o valor (velocidade neste caso) entregue a funï¿½ï¿½o for negativo, retorne o oposto desse valor, isto ï¿½, multiplique o valor negativo por "-1".
  return vel; //...Retorne o mesmo valor
}
/*-----------Motores------------------------*/
int sentido(int vel) {
  if (vel < 0) return BACKWARD; //Se o valor (velocidade neste caso) entregue a funï¿½ï¿½o for negativo, retorne o sentido como sendo para TRï¿½S, caso contrï¿½rio...
  return FORWARD; //...Retorne o sentido como sendo para FRENTE
}


void rampa(int hg) {
  if (digitalRead(hg) == HIGH) {
    sigaLinha(-200);
    sala3 = true;
  }
  else {
    sala3 = false;
  }
}

boolean encruzilhada() {
  Motor1.resetPosition();
  Motor2.resetPosition();
  if(sensors[0] > 600 && sensors[1] > 600 && sensors[2] > 600 && sensors[3] < 600 &&
     sensors[4] < 600 && sensors[5] < 600 && sensors[6] < 600 && sensors[7] < 600){
    if(checkingGreen() > 10 && checkingGreen() <70) {
      Motor1.move(BACKWARD, 100, 50, BRAKE);
      Motor2.move(FORWARD, 100, 50, BRAKE);
      while (Motor1.isTurning() || Motor2.isTurning());
      return true;
    }

    else if(checkingGreen() > 10 && checkingGreen() <70) {
      Motor1.resetPosition();
      Motor2.resetPosition();
      Motor1.move(FORWARD, 100, 50, BRAKE);
      Motor2.move(BACKWARD, 100, 50, BRAKE);
      while (Motor1.isTurning() || Motor2.isTurning());
      return true;
    }

    else{
      Motor1.resetPosition();
      Motor2.resetPosition();
      Motor1.move(BACKWARD, 200, 50, BRAKE);
      Motor2.move(BACKWARD, 200, 50, BRAKE);
      while (Motor1.isTurning() || Motor2.isTurning());
      return false;
    }
  }
}

boolean noventaGraus(){
  Motor1.resetPosition();
  Motor2.resetPosition();
  if(sensors[0] > 600 && sensors[1] > 600 && sensors[2] > 600 && sensors[3] < 600 && sensors[4] < 600 &&
     sensors[5] < 600 && sensors[6] < 600 && sensors[7] < 600){
    Motor1.move(BACKWARD, 200, 100, BRAKE);
    Motor2.move(BACKWARD, 200, 100, BRAKE);
  while (Motor1.isTurning() || Motor2.isTurning());
    if(qtra.readLine(sensors) > 3000 || qtra.readLine(sensors) < 3900){
      Motor1.resetPosition();
      Motor2.resetPosition();
      Motor1.move(FORWARD, 200, 150, BRAKE);
      Motor2.move(FORWARD, 200, 150, BRAKE);
      while (Motor1.isTurning() || Motor2.isTurning());
      
      LDR = A15; //IMPORTANTE: Vari�vel do LDR, como s�o 2 sensores de cor(2 LDRs) apenas troca-se a porta de cada um na vari�vel de leitura, economizando fun��es, resumindo: GAMBIARRA
      
      if(checkingGreen() > 10 && checkingGreen() <70) {
	Motor1.resetPosition();
	Motor2.resetPosition();
	Motor1.move(FORWARD, 100, 50, BRAKE);
	Motor2.move(BACKWARD, 100, 50, BRAKE);
	while (Motor1.isTurning() || Motor2.isTurning());
	LDR = originLDR;
	return true;
      }

      else if(checkingGreen() > 10 && checkingGreen() <70) {
        LDR = originLDR;
	Motor1.resetPosition();
	Motor2.resetPosition();
	Motor1.move(FORWARD, 100, 50, BRAKE);
	Motor2.move(BACKWARD, 100, 50, BRAKE);
	while (Motor1.isTurning() || Motor2.isTurning());
	return true;
      }
      else{
	Motor1.resetPosition();
	Motor2.resetPosition();
	Motor1.move(BACKWARD, 200, 50, BRAKE);
	Motor2.move(BACKWARD, 200, 50, BRAKE);
	while (Motor1.isTurning() || Motor2.isTurning());
	LDR = originLDR;
	return false;
      }
    }
    else {
      Motor1.resetPosition();
      Motor2.resetPosition();
      Motor1.move(FORWARD, 200, 150, BRAKE);
      Motor2.move(FORWARD, 200, 150, BRAKE);
      while (Motor1.isTurning() || Motor2.isTurning());
      LDR = A15;
      if(checkingGreen() > 10 && checkingGreen() <70) {
	Motor1.resetPosition();
	Motor2.resetPosition();
	Motor1.move(FORWARD, 100, 50, BRAKE);
	Motor2.move(BACKWARD, 100, 50, BRAKE);
	while (Motor1.isTurning() || Motor2.isTurning());
	LDR = originLDR;
	return true;
      }
      
      else if(checkingGreen() > 10 && checkingGreen() <70) {
        LDR = originLDR;
	Motor1.resetPosition();
	Motor2.resetPosition();
	Motor1.move(FORWARD, 100, 50, BRAKE);
	Motor2.move(BACKWARD, 100, 50, BRAKE);
	while (Motor1.isTurning() || Motor2.isTurning());
	return true;
      }
    }
  LDR = originLDR;
  }
  else if(sensors[0] < 600 && sensors[1] < 600 && sensors[2] < 600 && sensors[3] < 600 && sensors[4] < 600 && sensors[5] < 600 && sensors[6] > 600 && sensors[7] > 600){
    Motor1.resetPosition();
    Motor2.resetPosition();
    Motor1.move(BACKWARD, 200, 100, BRAKE);
    Motor2.move(BACKWARD, 200, 100, BRAKE);
    while (Motor1.isTurning() || Motor2.isTurning());
    
    if(qtra.readLine(sensors) > 3000 || qtra.readLine(sensors) < 3900){
      Motor1.resetPosition();
      Motor2.resetPosition();
      Motor1.move(FORWARD, 200, 150, BRAKE);
      Motor2.move(FORWARD, 200, 150, BRAKE);
      while (Motor1.isTurning() || Motor2.isTurning());
      
      LDR = A15; //IMPORTANTE: Vari�vel do LDR, como s�o 2 sensores de cor(2 LDRs) apenas troca-se a porta de cada um na vari�vel de leitura, economizando fun��es, resumindo: GAMBIARRA
      
      if(checkingGreen() > 10 && checkingGreen() <70) {
	Motor1.resetPosition();
	Motor2.resetPosition();
	Motor1.move(FORWARD, 100, 50, BRAKE);
	Motor2.move(BACKWARD, 100, 50, BRAKE);
	while (Motor1.isTurning() || Motor2.isTurning());
	LDR = originLDR;
	return true;
      }
      
      else if(checkingGreen() > 10 && checkingGreen() <70) {
        LDR = originLDR;
	Motor1.resetPosition();
	Motor2.resetPosition();
	Motor1.move(FORWARD, 100, 50, BRAKE);
	Motor2.move(BACKWARD, 100, 50, BRAKE);
	while (Motor1.isTurning() || Motor2.isTurning());
	return true;
      }
      else{
	Motor1.resetPosition();
	Motor2.resetPosition();
	Motor1.move(BACKWARD, 200, 50, BRAKE);
	Motor2.move(BACKWARD, 200, 50, BRAKE);
	while (Motor1.isTurning() || Motor2.isTurning());
	LDR = originLDR;
	return false;
      }
    }
    else {
      Motor1.resetPosition();
      Motor2.resetPosition();
      Motor1.move(FORWARD, 200, 150, BRAKE);
      Motor2.move(FORWARD, 200, 150, BRAKE);
      while (Motor1.isTurning() || Motor2.isTurning());
      LDR = A15;
      if(checkingGreen() > 10 && checkingGreen() <70) {
	Motor1.resetPosition();
	Motor2.resetPosition();
	Motor1.move(FORWARD, 100, 50, BRAKE);
	Motor2.move(BACKWARD, 100, 50, BRAKE);
	while (Motor1.isTurning() || Motor2.isTurning());
	LDR = originLDR;
	return true;
      }
      
      else if(checkingGreen() > 10 && checkingGreen() <70) {
        LDR = originLDR;
	Motor1.resetPosition();
	Motor2.resetPosition();
	Motor1.move(FORWARD, 100, 50, BRAKE);
	Motor2.move(BACKWARD, 100, 50, BRAKE);
	while (Motor1.isTurning() || Motor2.isTurning());
	return true;
      }
    }
  LDR = originLDR;
  }
} //Fim da Funcao

/*-----------------Sensor-de-Cor-------------------------*/
void CorSetup() {
  while (CalibrarCorState == HIGH);
  pinMode (redLed, OUTPUT);
  pinMode (greenLed, OUTPUT);
  pinMode (blueLed, OUTPUT);

}

int checkingGreen() {
  int avg = 0, sum = 0;
  //if (digitalRead(calibrarCor) == LOW) {
    for (int i = 0; i < TIMESG; i++) {
      rgbSense();
      //printColors();
      sum += colorArray[1];
    }
    avg = sum / TIMESG;
    //Serial.println();
    //Serial.print("Average: ");
    //Serial.println(avg);
  //}
  return avg;
}

void CalibrarCor() {
  // White Balance: operador deve colocar amostra branca sobre o sensor.
  //Serial.println ("White balance [press CalibrarCor to start] ...");
  while (CalibrarCorState == HIGH);
  delay(200);
  // Le os valores red/green/blue para a amostra branca.
  //Serial.print ("White Array: [");
  for (int i = 0; i < 3; i++) {
    digitalWrite (ledArray[i], HIGH);
    delay(100);
    whiteArray[i] = getAvgRead(5);
    digitalWrite (ledArray[i], LOW);
    //Serial.print (whiteArray[i]);
    delay(100);
  }
  //Serial.println ("] OK");

  // Black Balance: operador deve colocar amostra preta sobre o sensor.
  //Serial.println ("Black balance [press CalibrarCor to start] ...");
  while (CalibrarCorState == HIGH);
  delay(200);
  //Serial.print ("Black Array: [");
  for (int i = 0; i < 3; i++) {
    digitalWrite (ledArray[i], HIGH);
    delay(100);
    blackArray[i] = getAvgRead(5);
    digitalWrite (ledArray[i], LOW);
    //Serial.print (blackArray[i]);
    delay(100);
  }
  //Serial.println ("] OK");
  balanced = true;
  delay(2000);
}

void rgbSense () {
  float greyDiff = 0;
  for (int i = 0; i < 3; i++) {
    digitalWrite (ledArray[i], HIGH);
    delay(100);
    colorArray[i] = getAvgRead(5);
    greyDiff = whiteArray[i] - blackArray[i];
    colorArray[i] = (colorArray[i] - blackArray[i]) / greyDiff * 255;
    digitalWrite (ledArray[i], LOW);
    delay(100);
  }
}

int getAvgRead (int times) {
  int value = 0;
  int summ = 0;
  int avg = 0;
  for (int i = 0; i < times; i++) {
    value = analogRead(LDR);
    summ = value + summ;
    delay(10);
  }
  avg = summ / times;
  return avg;
}

void printColors () {
  for (int x = 0; x < 3; x++) if (colorArray[x] < 0) colorArray[x] = 0;
  Serial.print ("R: ");
  Serial.println(int(colorArray[0]));
  Serial.print("G = ");
  Serial.println(int(colorArray[1]));
  Serial.print("B = ");
  Serial.println(int(colorArray[2]));
  Serial.println();
  analogWrite(2, colorArray[0]);
  analogWrite(3, colorArray[1]);
  analogWrite(4, colorArray[2]);
  //delay(5000);
}

