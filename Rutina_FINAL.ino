#include <PID_v1.h>
#include <Servo.h>
#include <Adafruit_ADS1X15.h>


//ver forma de conexionar sensor IR para detectar que el polo se halla desplazado y donde ponerlo.
//ver si el segundo PID funciona con los delays, sino por pasos para no complicar 

//ADS
  Adafruit_ADS1015 ads;

// variables -----------------------------------------------------------------------------------------------------------------------------------------
  bool S1=false;
  bool S2=false;
  bool S3=false;
  bool S4=false;
  bool S5=false;
  bool S6=false;
  bool S7=false;
  bool S8=false;
  bool S9=false;
  bool S10=false;
  bool S11=false;
  bool S12=false;
  bool S11_5=false;
  bool S13=false;
  bool S14=false;
  bool S15=false;

  int VarillasDespPos=0;
  int ang=107;
  bool prueba = false;
  bool grippersClosed=false;
  bool placaElevada=false;

// Pines para los motores y encoders-------------------------------------------------------------------------------------------------------------------
  const int dirPinIzquierda = 30;     // Pin de dirección del motor 1 en el puente H L298N
  const int pwmPinIzquierda = 3;    // Pin PWM para velocidad del motor 1
  const int encoderPinAIzquierda = 18;  // Canal A del encoder del motor 1 amarillo
  const int encoderPinBIzquierda = 28;  // Canal B del encoder del motor 1

  const int dirPinDerecha = 31;     // Pin de dirección del motor 2 en el puente H L298N
  const int pwmPinDerecha = 2;     // Pin PWM para velocidad del motor 2
  const int encoderPinADerecha = 21;  // Canal A del encoder del motor 2 amarillo
  const int encoderPinBDerecha = 22;  // Canal B del encoder del motor 2

  // Variables de los encoders
  volatile long encoderCountIzquierda = 0;    // Contador del encoder motor 1
  volatile long encoderCountDerecha = 0;    // Contador del encoder motor 2

// Prametros PID ---------------------------------------------------------------------------------------------------------------------------------------
  int targetPositionIzquierda = 14000;         // Posición objetivo motor 1 en pulsos del encoder motor izquirda 15000
  int targetPositionDerecha = 28000;          // Posición objetivo motor 2 en pulsos del encoder deracha 30000

  // Variables para el PID
  double inputIzquierda, outputIzquierda, setpointIzquierda;
  double inputDerecha, outputDerecha, setpointDerecha;
  double KpIzquierda = 0.375, KiIzquierda = 0, KdIzquierda = 0;  // Ajusta estos valores según tu sistema
  double KpDerecha = 0.375, KiDerecha = 0, KdDerecha = 0;  // Ajusta estos valores según tu sistema
  PID myPID1(&inputIzquierda, &outputIzquierda, &setpointIzquierda, KpIzquierda, KiIzquierda, KdIzquierda, DIRECT);
  PID myPID2(&inputDerecha, &outputDerecha, &setpointDerecha, KpDerecha, KiDerecha, KdDerecha, DIRECT);

  // Variables para registro de datos
  unsigned long startTime;

// Configuración de Servos -----------------------------------------------------------------------------------------------------------------------
  Servo myServoDerecha;
  Servo myServoIzquierda;

  Servo myServoPlaca;

    // Servo Digital
  Servo ServoBase;
  int servo_min_pulse=500;    //minimum pulse time in microseconds
  int servo_max_pulse=2500;   //maximum pulse time in microseconds

  //Servo ServoSoporte;

  Servo ServoDespDer;
  Servo ServoDespIzq;


// Configuración de LDR -------------------------------------------------------------------------------------------------------------------------
  int ldrPinDerecha = A4;
  int ldrPinIzquierda = A3;
  int ldrThresholdDerecha = 1000;
  int ldrThresholdIzquierda = 1100;
  int ldrValueDerecha;
  int ldrValueIzquierda;

// Pines Cilindros y finales de carrera --------------------------------------------------------------------------------------------------------------
  //cilindro base
  const int D_2 = 52;  // Pin del sensor de final de carrera
  const int D_1 = 53;  // Pin del sensor de final de carrera
  const int D_0 = 51;  // Pin del sensor de final de carrera
  const int D_mas = 49;   // Pin para activar el avance del cilindro
  const int D_menos = 48; // Pin para activar el retroceso del cilindro

  //Cilindro izquierda
  const int A_1 = 44;  // Pin del sensor de final de carrera
  const int A_0 = 45;  // Pin del sensor de final de carrera
  const int A_mas = 47;   // Pin para activar el avance del cilindro
  const int A_menos = 46; // Pin para activar el retroceso del cilindro

  //Cilindro derecha
  const int B_1 = 43;  // Pin del sensor de final de carrera
  const int B_0 = 42;  // Pin del sensor de final de carrera
  const int B_mas = 41;   // Pin para activar el avance del cilindro
  const int B_menos = 40; // Pin para activar el retroceso del cilindro

  ////Cilindro inferior
  const int C_1 = 27;  // Pin del sensor de final de carrera
  const int C_0 = 38;  // Pin del sensor de final de carrera
  const int C_mas = 37;   // Pin para activar el avance del cilindro
  const int C_menos = 36; // Pin para activar el retroceso del cilindro


void setup() {
  Serial.begin(9600);   //Inicialización del monitor serial
  Serial.println("Iniciando Setup...");

// Setup motores----------------------------------------------------------------------------------------------------------------------------------------
  // Configuración de pines para motor 1
  pinMode(dirPinIzquierda, OUTPUT);
  pinMode(pwmPinIzquierda, OUTPUT);
  pinMode(encoderPinAIzquierda, INPUT);
  pinMode(encoderPinBIzquierda, INPUT);

  // Configuración de pines para motor 2
  pinMode(dirPinDerecha, OUTPUT);
  pinMode(pwmPinDerecha, OUTPUT);
  pinMode(encoderPinADerecha, INPUT);
  pinMode(encoderPinBDerecha, INPUT);

  // Interrupciones para los encoders
  attachInterrupt(digitalPinToInterrupt(encoderPinAIzquierda), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinADerecha), encoderISR2, CHANGE);

// Confniguracion PID ----------------------------------------------------------------------------------------------------------------------------------
  setpointIzquierda = 0;
  setpointDerecha = 0;
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(-135, 135);  // Ajusta el rango de salida para PWM motor 1
  myPID2.SetOutputLimits(-255, 255);  // Ajusta el rango de salida para PWM motor 2
  myPID1.SetSampleTime(10);           // Tiempo de muestreo del PID en ms
  myPID2.SetSampleTime(10);           // Tiempo de muestreo del PID en ms

  startTime = millis();  // Iniciar el tiempo

// Setup Servos ----------------------------------------------------------------------------------------------------------------------------------------
  myServoDerecha.attach(5);
  myServoIzquierda.attach(4);
  openGrippers();
  

  myServoPlaca.attach(6);
  bajarPlaca();

  ServoBase.attach(9,servo_min_pulse,servo_max_pulse);  //servo connected to pin 9
  ServoBase.write(125);


  ServoDespDer.attach(10);
  ServoDespIzq.attach(11);
  VarillasDesp0();
  

// Setup cilindros -----------------------------------------------------------------------------------------------------------------------------------
  // cilindro base
  pinMode(D_2, INPUT);      // Configura el pin del sensor magnetico como entrada
  pinMode(D_1, INPUT);      // Configura el pin del sensor magnetico como entrada
  pinMode(D_0, INPUT);      // Configura el pin del sensor magnetico como entrada
  pinMode(D_mas, OUTPUT);      // Configura el pin de avance como salida
  pinMode(D_menos, OUTPUT);   // Configura el pin de retroceso como salida  

  digitalWrite(D_mas, HIGH);
  digitalWrite(D_menos, HIGH);

  // cilindro izquierda
  pinMode(A_1, INPUT_PULLUP);      // Configura el pin del sensor como entrada
  pinMode(A_0, INPUT_PULLUP);      // Configura el pin del sensor como entrada
  pinMode(A_mas, OUTPUT);      // Configura el pin de avance como salida
  pinMode(A_menos, OUTPUT);   // Configura el pin de retroceso como salida

  digitalWrite(A_mas, HIGH);
  digitalWrite(A_menos, LOW);
  delay(10);
  digitalWrite(A_menos, HIGH);

  //Cilindro derecha
  pinMode(B_1, INPUT_PULLUP);      // Configura el pin del sensor como entrada
  pinMode(B_0, INPUT_PULLUP);      // Configura el pin del sensor como entrada
  pinMode(B_mas, OUTPUT);      // Configura el pin de avance como salida
  pinMode(B_menos, OUTPUT);   // Configura el pin de retroceso como salida

  digitalWrite(B_mas, HIGH);
  digitalWrite(B_menos, LOW);
  delay(10);
  digitalWrite(B_menos, HIGH);

  //Cilindro inferior
  pinMode(C_1, INPUT_PULLUP);      // Configura el pin del sensor como entrada
  pinMode(C_0, INPUT_PULLUP);      // Configura el pin del sensor como entrada
  pinMode(C_mas, OUTPUT);      // Configura el pin de avance como salida
  pinMode(C_menos, OUTPUT);   // Configura el pin de retroceso como salida

  digitalWrite(C_mas, LOW);
  digitalWrite(C_menos, HIGH);
  delay(10);
  digitalWrite(C_mas, HIGH);

// Inicio rutina----------------------------------------------------------------------------------------------------------------------------------------
 S1=true;
}

void loop() {

//Paso 1: Elevar placa 
  if(S1){
    varillasDesp0();
    elevarPlaca();
    Serial.println("S1");
    if (placaElevada){
      S1=false;
      S2=true;
      
    }
  }

//Paso 2: Lectura LDRs
  if (S2){
    Serial.println("S2");


    ldrValueDerecha = analogRead(ldrPinDerecha);
    ldrValueIzquierda = analogRead(ldrPinIzquierda);

    Serial.print("LDR Derecha: ");
    Serial.println(ldrValueDerecha);
    Serial.print("LDR Izquierda: ");
    Serial.println(ldrValueIzquierda);

    if ( ldrValueDerecha > ldrThresholdDerecha && ldrValueIzquierda > ldrThresholdIzquierda) {  //
      closeGrippers();
      delay(500);    
      S2=false;
      S3=true;

    } 
    
  }

//Paso 3: Setear setpoints PID 
  if (S3){
    encoderCountDerecha=0;
    encoderCountIzquierda=0;
    setpointIzquierda = targetPositionIzquierda;
    setpointDerecha = targetPositionDerecha;
    Serial.println("S3");

    if (setpointIzquierda!=0  && setpointDerecha!=0){
    S3=false;
    S4=true;}
  }

//Paso 4: Trasladar polos con PID
  if (S4){

  inputIzquierda = encoderCountIzquierda;   // Actualiza la posición actual del motor 1
  inputDerecha = encoderCountDerecha;   // Actualiza la posición actual del motor 2
  myPID1.Compute();         // Calcula el PID para el motor 1
  myPID2.Compute();         // Calcula el PID para el motor 2

  // Control del motor 1 según el valor del PID
  if (outputIzquierda < 0) {
    digitalWrite(dirPinIzquierda, LOW);       // Dirección hacia adelante
    analogWrite(pwmPinIzquierda, -outputIzquierda);  // Control de velocidad del motor 1
  } else if (outputIzquierda > 0) {
    digitalWrite(dirPinIzquierda, HIGH);        // Dirección hacia atrás
    analogWrite(pwmPinIzquierda, 255 - outputIzquierda); // Control de velocidad del motor 1 (inverso)
  } else {
    analogWrite(pwmPinIzquierda, 0);           // Apaga el motor 1
  }

  // Control del motor 2 según el valor del PID
  if (outputDerecha < 0) {
    digitalWrite(dirPinDerecha, LOW);       // Dirección hacia adelante
    analogWrite(pwmPinDerecha, -outputDerecha);  // Control de velocidad del motor 2
  } else if (outputDerecha > 0) {
    digitalWrite(dirPinDerecha, HIGH);        // Dirección hacia atrás
    analogWrite(pwmPinDerecha, 255 - outputDerecha); // Control de velocidad del motor 2 (inverso)
  } else {
    analogWrite(pwmPinDerecha, 0);           // Apaga el motor 2
  }

  if (inputDerecha>17000 && inputIzquierda>8500){ //elevar base
    digitalWrite(D_mas,LOW);
  }

  if (inputDerecha>16000 && inputIzquierda>8000){  //enderezar base
    ServoBase.write(85);
  }

  // Registrar datos: tiempo, setpoints, posiciones actuales y salidas PID
  unsigned long currentTime = millis() - startTime;
  //Serial.print(currentTime);      // Tiempo transcurrido en ms
  //Serial.print(",");
  // Mostrar en Serial Monitor
  Serial.print("Input Izquierda: ");
  Serial.print(inputIzquierda);
  Serial.print(" | Setpoint Izquierda: ");
  Serial.print(setpointIzquierda);
  Serial.print(" | Output Izquierda: ");
  Serial.println(outputIzquierda);

  Serial.print("Input Derecha: ");
  Serial.print(inputDerecha);
  Serial.print(" | Setpoint Derecha: ");
  Serial.print(setpointDerecha);
  Serial.print(" | Output Derecha: ");
  Serial.println(outputDerecha);
  delay(10);  // Espera breve para estabilidad

  int errorDerecha=abs(setpointDerecha-inputDerecha);
  int errorIzquierda=abs(setpointIzquierda-inputIzquierda);

  if (errorDerecha<=100 && errorIzquierda<=100){
    analogWrite(pwmPinDerecha, 0);
    analogWrite(pwmPinIzquierda, 0);
    digitalWrite(dirPinDerecha, LOW);
    digitalWrite(dirPinIzquierda, LOW);
    ServoBase.write(95);
    S4=false;
    S5=true;
  }
  }

//Paso 5: Soltar polo y sujetarlo
  if(S5){

    Serial.println("S5");
    openGrippers();
    delay(10);
    bajarPlaca();
    if (!placaElevada && !grippersClosed){
      S5=false;
      S6=true;
    }
  }

//Paso 6: Desactivar cilindro baso y bajar un nivel
  if(S6){
    Serial.println("S6");
    delay(500);
    digitalWrite(D_mas,HIGH);
    delay(50);
    digitalWrite(D_menos,LOW);
    if (digitalRead(D_0)==HIGH){
      S6=false;
      S7=true;
    }
  }
//Paso 7: Doblez  +
  if (S7){
    Serial.println("S7");
    digitalWrite(A_mas, LOW);
    digitalWrite(B_mas, LOW);
    delay(50);
    if (digitalRead(A_1)==LOW && digitalRead(B_1)==LOW){
      S7=false;
      S8=true;
    }
  }
//Paso 8: Inclinar base

  if (S8){

    Serial.println("S8");
    ServoBase.write(79);
    delay(2000);
    S8=false;
    S9=true;  
  }
//Paso 9: Doblez -
  if (S9){
    Serial.println("S9");
    digitalWrite(A_mas, HIGH);
    digitalWrite(B_mas, HIGH);
    delay(50);
    digitalWrite(B_menos, LOW);
    digitalWrite(A_menos, LOW);

    delay(50);
    
    if (digitalRead(B_0)==LOW && digitalRead(A_0)==0){
      digitalWrite(B_menos, HIGH);
      digitalWrite(A_menos, HIGH);
      S9=false;
      S10=true;
      S1=false;
    }
  }
//Paso 10: Inclinacion base
  if (S10){
    delay(500);
    VarillasDesp1();
    delay(1200);
    Serial.println("S10");
    ServoBase.write(115);
    delay(600);
    S10=false;
    S11=true;
  }
// Paso11: C-

  if (S11){
    Serial.println("S11");
    digitalWrite(C_menos, LOW);

    if (digitalRead(C_0)==LOW ){
      S11=false;
      S12=true;
    }
  }

//Paso 12: Inclinacion base
  if (S12){
    delay(400);
    Serial.println("S12");
    ServoBase.write(95);
    delay(400);
    ServoBase.write(107);
    delay(100);
    S12=false;
    S13=true;
  }

// Paso13: C+

  if (S13){
    
    digitalWrite(C_menos, HIGH);
    delay(50);
    digitalWrite(C_mas, LOW);
    Serial.println("S13.2");
    if (digitalRead(C_1)==LOW ){
      Serial.println("S13.3");
      S13=false;
      S14=true;
    }
  }
// Paso 14 entregar polo
  if(S14){
    Serial.println("S14");
    for (ang=107;ang<130;ang++){
      ServoBase.write(ang);
      delay(20);
    }
    delay(800);
    mitadPlaca();
    delay(800);
    VarillasDesp2();
    delay( 1000);
    S14=false;
    S15=true;
  }

}

// Cuenta encoders--------------------------------------------------------------------------------------------------------------------------------------
  // Función de interrupción para leer el encoder motor 1
  void encoderISR1() {
    int stateA = digitalRead(encoderPinAIzquierda);
    int stateB = digitalRead(encoderPinBIzquierda);
    if (stateA == stateB) {
      encoderCountIzquierda++;
    } else {
      encoderCountIzquierda--;
    }
    }

  // Función de interrupción para leer el encoder motor 2
  void encoderISR2() {
    int stateA = digitalRead(encoderPinADerecha);
    int stateB = digitalRead(encoderPinBDerecha);
    if (stateA == stateB) {
      encoderCountDerecha++;
    } else {
      encoderCountDerecha--;
    }
    }

// Funciones servos ------------------------------------------------------------------------------------------------------------------------------------
  void closeGrippers() {
    myServoDerecha.write(0);
    delay(10);
    myServoIzquierda.write(180);
    grippersClosed = true;
    Serial.println("Grippers cerrados");
    }

  void openGrippers() {
    myServoDerecha.write(50);
    delay(10);
    myServoIzquierda.write(130);
    grippersClosed = false; 
    Serial.println("Grippers abiertos");
    }

  void elevarPlaca() {
    myServoPlaca.write(67);
    delay(10);
    placaElevada = true;
    }

  void bajarPlaca() {
    myServoPlaca.write(0);
    delay(10);
    placaElevada = false;
    }

  void mitadPlaca() {
    myServoPlaca.write(17);
    delay(10);
    placaElevada = true;
    }

  void VarillasDesp0() {
    ServoDespDer.write(7);
    delay(10);
    ServoDespIzq.write(180);
    int VarillasDespPos = 0;
    }

  void VarillasDesp1() {
    ServoDespDer.write(97);
    delay(10);
    ServoDespIzq.write(88);
    int VarillasDespPos = 1;
    }

  void VarillasDesp2() {
    ServoDespDer.write(180);
    delay(10);
    ServoDespIzq.write(0);
    int VarillasDespPos = 2;
    }