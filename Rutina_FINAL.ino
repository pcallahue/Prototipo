#include <PID_v1.h>
#include <Servo.h>

// Prototipos de funciones
void processManualCommand(String cmd);
void openGrippers();
void closeGrippers();
void openGrippers2();
void desplazamientoPlaca(int placa_agulonuevo);
void desplazamientoBase();
void VarillasDesp0();
void VarillasDesp1();
void VarillasDesp2();
void encoderISR1();
void encoderISR2();
void enviarEstadosStages();
void stopMotors();  // Función para detener los motores

// VARIABLES -------------------------------------------------------------------------
bool S1 = false;
bool S1_5 = false;
bool S2 = false;
bool S3 = false;
bool S4 = false;
bool S5 = false;
bool S6 = false;
bool S7 = false;
bool S8 = false;
bool S9 = false;
bool S10 = false;
bool S11 = false;
bool S12 = false;
bool S12_5 = false;
bool S13 = false;
bool S13_2 = false;
bool S13_5 = false;
bool S13_55 = false;
bool S13_6 = false;
bool S14 = false;
bool S15 = false;
bool S16 = false;
bool S17 = false;
bool S18 = false;
bool S19 = false;
bool prueba = false;
bool SR = false;


bool grippersClosed = false;
bool placaElevada = false;

// Flags para comandos manuales
bool STOP = false;
bool manualMode = false;
bool RESET = false;

// Variables para registro de posición inicial de los motores
long cuentasmotorIzq = 0;
long cuentasmotorDer = 0;

int VarillasDespPos = 0;

int bajarPlaca = 0;
int mitadPlaca = 15;
int elevarPlaca = 67;

int base_anguloprevio = 95;
int base_angulonuevo = 95;

int placa_anguloprevio = 0;
int placa_angulonuevo;

// PINES PARA MOTORES Y ENCODERS ----------------------------------------------
const int dirPinIzquierda     = 30;
const int pwmPinIzquierda     = 3;
const int encoderPinAIzquierda= 20;
const int encoderPinBIzquierda= 23;

const int dirPinDerecha       = 31;
const int pwmPinDerecha       = 2;
const int encoderPinADerecha  = 21;
const int encoderPinBDerecha  = 22;

volatile long encoderCountIzquierda = 0;
volatile long encoderCountDerecha   = 0;


// Paametros motor ---------------------------------------------------------

unsigned long prevTimeIzquierda = 0;
unsigned long prevTimeDerecha = 0;
long prevEncoderIzquierda = 0;
long prevEncoderDerecha = 0;
double measuredSpeedIzquierda = 0; // en rpm
double measuredSpeedDerecha = 0;   // en rpm
double filteredSpeedIzquierda = 0; // Para filtrado 
double filteredSpeedDerecha = 0;   // Para filtrado 
const int pulsesPerRevolution = 800;

// PARÁMETROS PID ------------------------------------------------------------
int targetPositionIzquierda = 6950;  // Pulsos para motor 1
int targetPositionDerecha   = -6950;  // Pulsos para motor 2

double inputIzquierda, outputIzquierda, setpointIzquierda;
double inputDerecha, outputDerecha, setpointDerecha;

double posPID_OutIzquierda = 0;   
double posPID_OutDerecha = 0;  

double desiredSpeedIzquierda = 0;  
double desiredSpeedDerecha = 0;

double pwmOutputIzquierda = 0;     
double pwmOutputDerecha = 0; 

double KpPos = 0.75023, KiPos = 0.053044, KdPos = 1.560123;

PID posPIDIzquierda(&inputIzquierda, &posPID_OutIzquierda, &setpointIzquierda, 
                      KpPos, KiPos, KdPos, DIRECT);
PID posPIDDerecha(&inputDerecha, &posPID_OutDerecha, &setpointDerecha, 
                    KpPos, KiPos, KdPos, DIRECT);

// Inner loop (velocidad) – parámetros de ejemplo; ajústalos según tu sistema.
double KpVel = 0.1111, KiVel = 0.5557, KdVel = 0;  
PID velPIDIzquierda(&filteredSpeedIzquierda, &pwmOutputIzquierda, &desiredSpeedIzquierda, 
                    KpVel, KiVel, KdVel, DIRECT);
PID velPIDDerecha(&filteredSpeedDerecha, &pwmOutputDerecha, &desiredSpeedDerecha, 
                  KpVel, KiVel, KdVel, DIRECT);


unsigned long startTime = 0;

// CONFIGURACIÓN DE SERVOS ---------------------------------------------------
Servo myServoDerecha;
Servo myServoIzquierda;
Servo myServoPlaca;
Servo ServoBase;
int servo_min_pulse = 500;
int servo_max_pulse = 2500;
Servo ServoSoporte;
Servo ServoDespDer;
Servo ServoDespIzq;

// CONFIGURACIÓN DE LDR ------------------------------------------------------
int ldrPinDerecha         = A2;
int ldrPinIzquierda       = A3;
int ldrThresholdDerecha   = 15;
int ldrThresholdIzquierda = 15;
int ldrValueDerecha;
int ldrValueIzquierda;

// PINES DE CILINDROS Y FINALES DE CARRERA ------------------------------------
const int D_1    = 53;
const int D_0    = 52;
const int D_mas  = 51;
const int D_menos= 50;

const int A_1    = 41;
const int A_0    = 40;
const int A_mas  = 49;
const int A_menos= 48;

const int B_1    = 39;
const int B_0    = 38;
const int B_mas  = 47;
const int B_menos= 46;

const int C_1    = 37;
const int C_0    = 36;
const int C_mas  = 45;
const int C_menos= 44;

// CONTROL DE ENVÍO PERIÓDICO (cada ~1.8 s)
unsigned long ultimoEnvio = 0;

// Comunicación con ESP32 (usamos Serial1 en Mega)
#define espSerial Serial1

// ---------------------------------------------------------------------------
//                    PROCESO DE COMANDOS MANUALES
// ---------------------------------------------------------------------------
void processManualCommand(String cmd) {
  cmd.trim();
  Serial.println("CMD Manual recibido: " + cmd);

  if (cmd.startsWith("MANUAL=")) {
    String val = cmd.substring(7);
    if (val == "ON") {
      manualMode = true;
      Serial.println("Modo manual ACTIVADO");
    } else if (val == "OFF") {
      manualMode = false;
      Serial.println("Modo manual DESACTIVADO");
    }
  }
  else if (cmd.startsWith("S1=")) {
    String val = cmd.substring(3);
    Serial.print("Valor recibido para S1: ");
    Serial.println(val);
    if (val == "true") {
      S1 = true;
      Serial.println("Stage S1 ACTIVADO");
      if (cuentasmotorIzq == 0 && cuentasmotorDer == 0) {
        cuentasmotorIzq = encoderCountIzquierda;
        cuentasmotorDer = encoderCountDerecha;
        Serial.print("Posición inicial: Izq=");
        Serial.print(cuentasmotorIzq);
        Serial.print(", Der=");
        Serial.println(cuentasmotorDer);
      }
    } else {
      S1 = false;
      Serial.println("Stage S1 DESACTIVADO");
    }
  }
  else if (cmd.startsWith("STOP=")) {
    String val = cmd.substring(5);
    Serial.print("Valor recibido para STOP: ");
    Serial.println(val);
    if (val == "true") {
      STOP = true;
      Serial.println("STOP ACTIVADO");
    } else {
      STOP = false;
      Serial.println("STOP DESACTIVADO");
    }
  }
  else if (cmd.startsWith("RESET=")) {
    String val = cmd.substring(6);
    Serial.print("Valor recibido para RESET: ");
    Serial.println(val);
    if (val == "true") {
      RESET = true;
      SR = true;
      // Paso 1: Desactivar A_mas y B_mas (ponerlos en HIGH)
      digitalWrite(A_mas, HIGH);
      digitalWrite(B_mas, HIGH);
      // Paso 2: Activar C_mas (ponerlo en LOW)
      digitalWrite(C_mas, LOW);
      // Esperar hasta que C_1 esté en LOW
      while (digitalRead(C_1) == LOW) {
        delay(10);
      }
      delay(200);
      // Paso 3: Mover la base y la placa
      ServoBase.write(95);
      myServoPlaca.write(0);
      delay(200);
      // Paso 4: Activar A_menos y B_menos (ponerlos en LOW)
      digitalWrite(A_menos, LOW);
      digitalWrite(B_menos, LOW);
      // Esperar hasta que se verifique que A_0 y B_0 están en LOW
      while ((digitalRead(A_0) != LOW) || (digitalRead(B_0) != LOW)) {
        delay(10);
      }
      delay(200);
      // Paso 5: Activar D_mas (HIGH) y D_menos (LOW)
      digitalWrite(D_mas, HIGH);
      digitalWrite(D_menos, LOW);
      // Esperar hasta que D_0 esté en HIGH
      while (digitalRead(D_0) != HIGH) {
        delay(10);
      }
      // Reiniciar stages, relés y setpoints
      S1 = false; S1_5 = false; S2 = false; S3 = false; S4 = false;
      S5 = false; S6 = false; S7 = false; S8 = false;
      S9 = false; S10 = false; S11 = false; S12 = false;
      S12_5 = false; S13 = false; S14 = false; S15 = false; SR = false;
      digitalWrite(A_mas, HIGH);
      digitalWrite(A_menos, HIGH);
      digitalWrite(B_mas, HIGH);
      digitalWrite(B_menos, HIGH);
      digitalWrite(C_mas, HIGH);
      digitalWrite(C_menos, HIGH);
      digitalWrite(D_mas, HIGH);
      digitalWrite(D_menos, HIGH);
      setpointIzquierda = -cuentasmotorIzq;
      setpointDerecha   = -cuentasmotorDer;
      S16 = true;
      Serial.println("RESET ejecutado");
      RESET = false;
      SR = false;
    } else {
      RESET = false;
      Serial.println("RESET desactivado");
    }
  }
}

// Función para detener los motores
void stopMotors() {
  analogWrite(pwmPinIzquierda, 0);
  analogWrite(pwmPinDerecha, 0);
  digitalWrite(dirPinIzquierda, LOW);
  digitalWrite(dirPinDerecha, LOW);
}

// ---------------------------------------------------------------------------
//                                SETUP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  delay(2000);

  pinMode(dirPinIzquierda, OUTPUT);
  pinMode(pwmPinIzquierda, OUTPUT);
  pinMode(encoderPinAIzquierda, INPUT);
  pinMode(encoderPinBIzquierda, INPUT);

  pinMode(dirPinDerecha, OUTPUT);
  pinMode(pwmPinDerecha, OUTPUT);
  pinMode(encoderPinADerecha, INPUT);
  pinMode(encoderPinBDerecha, INPUT);

  Serial.println("Config int0");
  attachInterrupt(digitalPinToInterrupt(encoderPinAIzquierda), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinADerecha), encoderISR2, RISING);
  Serial.println("Config int1");

  // PID
  setpointIzquierda = 0;
  setpointDerecha = 0;
  
  // Se ponen los nuevos PID de posición y velocidad en modo AUTOMÁTICO
  posPIDIzquierda.SetMode(AUTOMATIC);
  posPIDDerecha.SetMode(AUTOMATIC);
  velPIDIzquierda.SetMode(AUTOMATIC);
  velPIDDerecha.SetMode(AUTOMATIC);
  
  // Se pueden ajustar los tiempos de muestreo (si se desea)
  posPIDIzquierda.SetSampleTime(10);
  posPIDDerecha.SetSampleTime(10);
  velPIDIzquierda.SetSampleTime(10);
  velPIDDerecha.SetSampleTime(10);

  // Servos
  myServoDerecha.attach(5);
  myServoIzquierda.attach(4);
  myServoPlaca.attach(6);
  myServoPlaca.write(0);
  ServoBase.attach(9, servo_min_pulse, servo_max_pulse);
  ServoBase.write(95);
  ServoDespDer.attach(10);
  ServoDespIzq.attach(12);
  VarillasDesp0();
  closeGrippers();
  // Cilindros
  pinMode(D_1, INPUT);
  pinMode(D_0, INPUT);
  pinMode(D_mas, OUTPUT);
  pinMode(D_menos, OUTPUT);
  digitalWrite(D_mas, HIGH);
  digitalWrite(D_menos, LOW);
  delay(10);
  digitalWrite(D_menos, HIGH);

  pinMode(A_1, INPUT_PULLUP);
  pinMode(A_0, INPUT_PULLUP);
  pinMode(A_mas, OUTPUT);
  pinMode(A_menos, OUTPUT);
  digitalWrite(A_mas, HIGH);
  digitalWrite(A_menos, LOW);
  delay(10);
  digitalWrite(A_menos, HIGH);

  pinMode(B_1, INPUT_PULLUP);
  pinMode(B_0, INPUT_PULLUP);
  pinMode(B_mas, OUTPUT);
  pinMode(B_menos, OUTPUT);
  digitalWrite(B_mas, HIGH);
  digitalWrite(B_menos, LOW);
  delay(10);
  digitalWrite(B_menos, HIGH);

  pinMode(C_1, INPUT_PULLUP);
  pinMode(C_0, INPUT_PULLUP);
  pinMode(C_mas, OUTPUT);
  pinMode(C_menos, OUTPUT);
  digitalWrite(C_mas, LOW);
  digitalWrite(C_menos, HIGH);
  delay(10);
  digitalWrite(C_mas, HIGH);
  

  //prueba = true;
  S1 = false;

  Serial.println("Done setup");
  cuentasmotorIzq = 0;
  cuentasmotorDer = 0;
}

// ---------------------------------------------------------------------------
//                                LOOP
// ---------------------------------------------------------------------------
void loop() {
  // Procesar comandos manuales (Serial1)
  if (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('\n');
    command.trim();
    processManualCommand(command);
  }

  if (manualMode) return;

  // Si se activa STOP, detener motores y permanecer en pausa
  if (STOP) {
    Serial.println("STOP activado: pausa la rutina");
    stopMotors();
    while (STOP) {
      if (Serial1.available() > 0) {
        String cmd = Serial1.readStringUntil('\n');
        cmd.trim();
        processManualCommand(cmd);
      }
      delay(50);
    }
    Serial.println("STOP desactivado: reanudando rutina");
  }

  // RUTINA DE CONTROL AUTOMÁTICO
  if (prueba) {
    closeGrippers();
    myServoPlaca.write(0);
    VarillasDesp0();
  }

  if (S1) {
    S1_5 = true;
    S1 = false;
  }

  // Paso 1: Elevar placa y registrar posición inicial
  if (S1_5) {
    if (cuentasmotorIzq == 0 && cuentasmotorDer == 0) {
      cuentasmotorIzq = encoderCountIzquierda;
      cuentasmotorDer = encoderCountDerecha;
    }
    openGrippers();
    base_angulonuevo = 130;
    desplazamientoBase();
    digitalWrite(D_mas, LOW);

    placa_angulonuevo = elevarPlaca;
    desplazamientoPlaca(elevarPlaca);
    delay(500);

    Serial.println("S1");
    if (digitalRead(D_1) == HIGH) {
      S1_5 = false;
      S2 = true;
    }
  }

  // Paso 2: Lectura de LDR
  if (S2) {
    Serial.println("S2");
    ldrValueDerecha = analogRead(ldrPinDerecha);
    ldrValueIzquierda = analogRead(ldrPinIzquierda);
    Serial.print("LDR Derecha: ");
    Serial.println(ldrValueDerecha);
    Serial.print("LDR Izquierda: ");
    Serial.println(ldrValueIzquierda);
    if (ldrValueDerecha < ldrThresholdDerecha) {
      closeGrippers();
      delay(1000);  
      S2 = false;
      S3 = true;
    }
  }

  // Paso 3: Configurar setpoints PID (avanzar 0 → 7000)
  if (S3) {
    encoderCountDerecha = 0;
    encoderCountIzquierda = 0;
    setpointIzquierda = targetPositionIzquierda;
    setpointDerecha = targetPositionDerecha;
    Serial.println("S3");
    if (setpointIzquierda != 0 && setpointDerecha != 0) {
      S3 = false;
      S4 = true;
    }
  }

  // Paso 4: Control PID (ir de 0 → 7000)
  if (S4) {
    unsigned long currentTime = millis();
    float dtIzq = (currentTime - prevTimeIzquierda) / 1000.0; // en segundos
    float dtDer = (currentTime - prevTimeDerecha) / 1000.0;
    if (dtIzq > 0) {
      measuredSpeedIzquierda = ((encoderCountIzquierda - prevEncoderIzquierda) * 60.0) / (dtIzq * pulsesPerRevolution);
    }
    if (dtDer > 0) {
      measuredSpeedDerecha = ((encoderCountDerecha - prevEncoderDerecha) * 60.0) / (dtDer * pulsesPerRevolution);
    }
    // Para este ejemplo se asigna directamente (sin filtrado avanzado)
    filteredSpeedIzquierda = measuredSpeedIzquierda;
    filteredSpeedDerecha = measuredSpeedDerecha;
    // Actualización de variables para el cálculo diferencial
    prevEncoderIzquierda = encoderCountIzquierda;
    prevEncoderDerecha = encoderCountDerecha;
    prevTimeIzquierda = currentTime;
    prevTimeDerecha = currentTime;
    
    // --- Outer Loop (posición): se usa la posición medida (encoderCount) ---
    inputIzquierda = encoderCountIzquierda;
    inputDerecha = encoderCountDerecha;
    posPIDIzquierda.Compute();
    posPIDDerecha.Compute();
    
    // --- Acción Feedforward: se suma (-0.08 * setpoint) a la salida del PID de posición ---
    desiredSpeedIzquierda = posPID_OutIzquierda + (-0.08 * setpointIzquierda);
    desiredSpeedDerecha = posPID_OutDerecha + (-0.08 * setpointDerecha);
    
    // --- Inner Loop (velocidad): se compara la velocidad filtrada con la velocidad deseada ---
    velPIDIzquierda.Compute();
    velPIDDerecha.Compute();
    
    // --- Actuación en los motores usando la misma lógica if original ---
    if (pwmOutputIzquierda < 0) {
      digitalWrite(dirPinIzquierda, LOW);       // Dirección hacia atrás
      analogWrite(pwmPinIzquierda, -(int)abs(pwmOutputIzquierda));
    } else if (pwmOutputIzquierda > 0) {
      digitalWrite(dirPinIzquierda, HIGH);        // Dirección hacia adelante
      analogWrite(pwmPinIzquierda, 255 - (int)pwmOutputIzquierda);
    } else {
      analogWrite(pwmPinIzquierda, 0);
    }
    
    if (pwmOutputDerecha < 0) {
      digitalWrite(dirPinDerecha, LOW);       // Dirección hacia adelante
      analogWrite(pwmPinDerecha, -(int)abs(pwmOutputDerecha));
    } else if (pwmOutputDerecha > 0) {
      digitalWrite(dirPinDerecha, HIGH);        // Dirección hacia atrás
      analogWrite(pwmPinDerecha, 255 - (int)pwmOutputDerecha);
    } else {
      analogWrite(pwmPinDerecha, 0);
    }
    
    if (inputDerecha<-2500 && inputIzquierda>2500){
      if (base_anguloprevio>95){
        int ang1 = base_anguloprevio-6;
        base_angulonuevo = ang1;
        desplazamientoBase();
      }
    }
    // --- Verificación de convergencia (similar a la versión original) ---
    int errorDerecha = abs(setpointDerecha - encoderCountDerecha);
    int errorIzquierda = abs(setpointIzquierda - encoderCountIzquierda);
    if (errorDerecha <= 100 && errorIzquierda<=100) {
      analogWrite(pwmPinDerecha, 0);
      analogWrite(pwmPinIzquierda, 0);
      digitalWrite(dirPinDerecha, LOW);
      digitalWrite(dirPinIzquierda, LOW);
      setpointIzquierda = 0;
      setpointDerecha = 0;
      S4 = false;
      S5 = true;
    }
  }

  // Paso 5: Soltar polo y sujetarlo
  if (S5) {
    Serial.println("S5");
    openGrippers2();
    delay(10);
    placa_angulonuevo = bajarPlaca;
    desplazamientoPlaca(bajarPlaca);
    if (!placaElevada && !grippersClosed) {
      S5 = false;
      S6 = true;
    }
  }

  // Paso 6: Desactivar cilindro base y bajar un nivel
  if (S6) {

    delay(500);
    Serial.println("S6");
    digitalWrite(D_mas, HIGH);
    delay(50);
    digitalWrite(D_menos, LOW);
    if (digitalRead(D_0) == HIGH) {
      S6 = false;
      S7 = true;
    }
  }

  // Paso 7: Doblez +
  if (S7) {
    Serial.println("S7");
    digitalWrite(A_mas, LOW);
    digitalWrite(B_mas, LOW);
    delay(1500);
    if (digitalRead(A_1) == LOW || digitalRead(B_1) == LOW) {
      S7 = false;
      S8 = true;
    }
  }

  // Paso 8: Inclinar base
  if (S8) {
    Serial.println("S8");
    base_angulonuevo = 98;
    desplazamientoBase();
    delay(50);

    base_angulonuevo = 98;
    desplazamientoBase();

    S8 = false;
    S9 = true;
  }

  // Paso 9: Doblez -
  if (S9) {
    Serial.println("S9");
    digitalWrite(A_mas, HIGH);
    digitalWrite(B_mas, HIGH);
    delay(50);
    digitalWrite(B_menos, LOW);
    digitalWrite(A_menos, LOW);
    delay(50);
    if (digitalRead(B_0) == LOW && digitalRead(A_0) == LOW) {
      digitalWrite(B_menos, HIGH);
      digitalWrite(A_menos, HIGH);
      S9 = false;
      S10 = true;
      S1 = false;
    }
  }

  // Paso 10: Inclinación base y activación de varillas
  if (S10) {
    closeGrippers();
    delay(500);
    VarillasDesp1();
    delay(1200);
    Serial.println("S10");
    base_angulonuevo = 111;
    desplazamientoBase();
    delay(400);
    S10 = false;
    S11 = true;
  }

  // Paso 11: C-
  if (S11) {
    Serial.println("S11");
    digitalWrite(C_mas, HIGH);
    digitalWrite(C_menos, LOW);
    if (digitalRead(C_0) == LOW) {
      S11 = false;
      S12 = true;
    }
  }

  // Paso 12: Inclinación base
  if (S12) {
    delay(400);
    Serial.println("S12");

    delay(40);
    base_angulonuevo = 60;
    desplazamientoBase();
     
    delay(40);
    base_angulonuevo = 115;
    desplazamientoBase();
    delay(50);
    S12 = false;
    S13 = true;
  }

  // Paso 13: C+
  if (S13) {

    delay(800);

    digitalWrite(C_menos, HIGH);
    delay(50);
    digitalWrite(C_mas, LOW);
    Serial.println("S13");
    if (digitalRead(C_1) == LOW) {
      Serial.println("S13.3");
      S13 = false;
      S14 = true;
    }
  }

  if(S13_2){
    base_angulonuevo = 130;
    desplazamientoBase();
    delay(200);

    S13_2 = false;
    S13_5 = true;
  }

  // Paso 11: C-
  if (S13_5) {


    delay(800);

    Serial.println("S11");
    digitalWrite(C_mas, HIGH);
    digitalWrite(C_menos, LOW);
    if (digitalRead(C_0) == LOW) {
      S13_5 = false;
      S13_55 = true;
    }
  }

  if (S13_55){

    base_angulonuevo = 55;
    desplazamientoBase();

    delay(100);

    base_angulonuevo = 130;
    desplazamientoBase();
    delay(200);

    S13_55 = false;
    S13_6 = true;

  }
  
  // Paso 13: C+
  if (S13_6) {



    digitalWrite(C_menos, HIGH);
    delay(50);



    digitalWrite(C_mas, LOW);
    Serial.println("S13.2");
    if (digitalRead(C_1) == LOW) {
      Serial.println("S13.3");
      S13_6 = false;
      S14 = true;
    }
  }

  // Paso 14: Entregar polo
  if (S14) {
    base_angulonuevo = 130;
    desplazamientoBase();
    delay(100);
    placa_angulonuevo = mitadPlaca;
    desplazamientoPlaca(mitadPlaca);
    delay(50);
    VarillasDesp2();
    delay(4000);
    S14 = false;
    S15 = true;
  }

  // Paso 15: Preparar RETORNO (motores: 7000 → 0)
  if (S15) {
    Serial.println("S15");
    VarillasDesp0();
    delay(1000);
    base_angulonuevo = 95;
    desplazamientoBase();
    digitalWrite(D_menos, HIGH);
    // Se deja el encoder en su valor actual (~7000) y se establece el setpoint a 0 para el retorno
    setpointIzquierda = 0;
    setpointDerecha   = 0;
    S15 = false;
    S16 = true;
  }

  // Paso 16: Control PID para RETORNO (de ~7000 → 0)
  if (S16) {
    placa_angulonuevo = bajarPlaca;
    desplazamientoPlaca(bajarPlaca);
    Serial.println("S16");

    unsigned long currentTime = millis();
    float dtIzq = (currentTime - prevTimeIzquierda) / 1000.0;
    float dtDer = (currentTime - prevTimeDerecha) / 1000.0;
    if (dtIzq > 0) {
      measuredSpeedIzquierda = ((encoderCountIzquierda - prevEncoderIzquierda) * 60.0) / (dtIzq * pulsesPerRevolution);
    }
    if (dtDer > 0) {
      measuredSpeedDerecha = ((encoderCountDerecha - prevEncoderDerecha) * 60.0) / (dtDer * pulsesPerRevolution);
    }
    filteredSpeedIzquierda = measuredSpeedIzquierda;
    filteredSpeedDerecha = measuredSpeedDerecha;
    prevEncoderIzquierda = encoderCountIzquierda;
    prevEncoderDerecha = encoderCountDerecha;
    prevTimeIzquierda = currentTime;
    prevTimeDerecha = currentTime;
    
    inputIzquierda = encoderCountIzquierda;
    inputDerecha = encoderCountDerecha;

    posPIDIzquierda.Compute();
    posPIDDerecha.Compute();
    
    desiredSpeedIzquierda = posPID_OutIzquierda + (-0.08 * setpointIzquierda);
    desiredSpeedDerecha = posPID_OutDerecha + (-0.08 * setpointDerecha);
    
    velPIDIzquierda.Compute();
    velPIDDerecha.Compute();
    
    if (pwmOutputIzquierda < 0) {
      digitalWrite(dirPinIzquierda, LOW);
      analogWrite(pwmPinIzquierda, -(int)abs(pwmOutputIzquierda));
    } else if (pwmOutputIzquierda > 0) {
      digitalWrite(dirPinIzquierda, HIGH);
      analogWrite(pwmPinIzquierda, 255 - (int)pwmOutputIzquierda);
    } else {
      analogWrite(pwmPinIzquierda, 0);
    }
    
    if (pwmOutputDerecha < 0) {
      digitalWrite(dirPinDerecha, LOW);
      analogWrite(pwmPinDerecha, -(int)abs(pwmOutputDerecha));
    } else if (pwmOutputDerecha > 0) {
      digitalWrite(dirPinDerecha, HIGH);
      analogWrite(pwmPinDerecha, 255 - (int)pwmOutputDerecha);
    } else {
      analogWrite(pwmPinDerecha, 0);
    }
    
    int errorDerecha = abs(setpointDerecha - encoderCountDerecha);
    int errorIzquierda = abs(setpointIzquierda - encoderCountIzquierda);
    if (errorDerecha <= 100) {
      analogWrite(pwmPinDerecha, 0);
      analogWrite(pwmPinIzquierda, 0);
      digitalWrite(dirPinDerecha, LOW);
      digitalWrite(dirPinIzquierda, LOW);
      setpointIzquierda = 0;
      setpointDerecha = 0;
      S16 = false;
      S1 = false;
    };
  };
  // Bloque de envío periódico de datos cada ~1.8 s (independiente del stage)
  unsigned long ahora = millis();

  if (ahora - ultimoEnvio >= 1800) {
    ultimoEnvio = ahora;
    enviarEstadosStages();
  };
}

// ---------------------------------------------------------------------------
//           ENVÍO DE ESTADOS POR UART (cada ~1.8 s)
// ---------------------------------------------------------------------------
void enviarEstadosStages() {
  Serial1.println("------ Actualizacion de Stages ------");
  Serial1.print("S1=");     Serial1.println(S1 ? "true" : "false");
  Serial1.print("S2=");     Serial1.println(S2 ? "true" : "false");
  Serial1.print("S3=");     Serial1.println(S3 ? "true" : "false");
  Serial1.print("S4=");     Serial1.println(S4 ? "true" : "false");
  Serial1.print("S5=");     Serial1.println(S5 ? "true" : "false");
  Serial1.print("S6=");     Serial1.println(S6 ? "true" : "false");
  Serial1.print("S7=");     Serial1.println(S7 ? "true" : "false");
  Serial1.print("S8=");     Serial1.println(S8 ? "true" : "false");
  Serial1.print("S9=");     Serial1.println(S9 ? "true" : "false");
  Serial1.print("S10=");    Serial1.println(S10 ? "true" : "false");
  Serial1.print("S11=");    Serial1.println(S11 ? "true" : "false");
  Serial1.print("S12=");    Serial1.println(S12 ? "true" : "false");
  Serial1.print("S12_5=");  Serial1.println(S12_5 ? "true" : "false");
  Serial1.print("S13=");    Serial1.println(S13 ? "true" : "false");
  Serial1.print("S14=");    Serial1.println(S14 ? "true" : "false");
  Serial1.print("S15=");    Serial1.println(S15 ? "true" : "false");

  bool ldrEstadoDerecha = (ldrValueDerecha < ldrThresholdDerecha);
  bool ldrEstadoIzquierda = (ldrValueIzquierda < ldrThresholdIzquierda);
  Serial1.print("LDR_Derecha=");
  Serial1.println(ldrEstadoDerecha ? "true" : "false");
  Serial1.print("LDR_Izquierda=");
  Serial1.println(ldrEstadoIzquierda ? "true" : "false");

  int avanceIzq = map(encoderCountIzquierda, 0, targetPositionIzquierda, 0, 100);
  int avanceDer = map(encoderCountDerecha, 0, targetPositionDerecha, 0, 100);
  avanceIzq = constrain(avanceIzq, 0, 100);
  avanceDer = constrain(avanceDer, 0, 100);
  Serial1.print("AV_Izq=");
  Serial1.println(avanceIzq);
  Serial1.print("AV_Der=");
  Serial1.println(avanceDer);

  Serial1.print("In_Izq=");
  Serial1.println(inputIzquierda);
  Serial1.print("SP_Izq=");
  Serial1.println(setpointIzquierda);
  Serial1.print("Out_Izq=");
  Serial1.println(outputIzquierda);

  Serial1.print("In_Der=");
  Serial1.println(inputDerecha);
  Serial1.print("SP_Der=");
  Serial1.println(setpointDerecha);
  Serial1.print("Out_Der=");
  Serial1.println(outputDerecha);

  Serial1.println("-------------------------------------");
}

// ---------------------------------------------------------------------------
//                INTERRUPCIONES PARA CONTAR ENCODERS
// ---------------------------------------------------------------------------
void encoderISR1() {
  int stateA = digitalRead(encoderPinAIzquierda);
  int stateB = digitalRead(encoderPinBIzquierda);
  if (stateA == stateB) {
    encoderCountIzquierda++;
  } else {
    encoderCountIzquierda--;
  }
}

void encoderISR2() {
  int stateA = digitalRead(encoderPinADerecha);
  int stateB = digitalRead(encoderPinBDerecha);
  if (stateA == stateB) {
    encoderCountDerecha++;
  } else {
    encoderCountDerecha--;
  }
}

// ---------------------------------------------------------------------------
//                         FUNCIONES DE SERVOS
// ---------------------------------------------------------------------------
void closeGrippers() {
  myServoDerecha.write(180);
  delay(10);
  myServoIzquierda.write(0);
  grippersClosed = true;
  Serial.println("Grippers cerrados");
}

void openGrippers() {
  myServoDerecha.write(150);
  delay(10);
  myServoIzquierda.write(30);
  grippersClosed = false;
  Serial.println("Grippers abiertos");
}

void openGrippers2() {
  myServoDerecha.write(130);
  delay(10);
  myServoIzquierda.write(50);
  grippersClosed = false;
  Serial.println("Grippers abiertos");
}

void desplazamientoPlaca(int placa_agulonuevo) {
  if(placa_anguloprevio!=placa_angulonuevo){
  int q0p = placa_anguloprevio;
  int qfp = placa_angulonuevo;
  float delayTimep = 0.025;              // Tiempo entre puntos (segundos)
  float tfp = abs(q0p - qfp) * delayTimep;    // Tiempo total de la trayectoria

  // Cálculo de coeficientes del polinomio cúbico:
  // q(t) = a3*t^3 + a2*t^2 + a1*t + a0
  float a0p = q0p;                        // q(0) = q0
  float a1p = 0;                         // Velocidad inicial 0
  float a2p = -3.0 * (q0p - qfp) / (tfp * tfp);
  float a3p = 2.0 * (q0p - qfp) / (tfp * tfp * tfp);

  // Inicializar el tiempo actual
  float currentTimep = 0.0;
  Serial.println("done trayectoria");
  while (currentTimep <= tfp) {
    // Calcular la posición en el tiempo currentTime
    float qp = a3p * currentTimep * currentTimep * currentTimep + 
              a2p * currentTimep * currentTimep + 
              a1p * currentTimep + 
              a0p;

    int q_intp = round(qp);
    myServoPlaca.write(q_intp);
    
    // Imprimir tiempo y posición en el monitor serial
    Serial.print("tp: ");
    Serial.print(currentTimep, 3);
    Serial.print(" s, qp: ");
    Serial.println(q_intp);

    delay(delayTimep * 1000);  // Convertir a milisegundos

    // Incrementar currentTime para avanzar en la trayectoria
    currentTimep += delayTimep;
  }
  
  placa_anguloprevio = placa_angulonuevo;

  }
  placaElevada = (placa_agulonuevo != 0);
}

void desplazamientoBase() {
  if(base_anguloprevio!=base_angulonuevo){
  int q0 = base_anguloprevio;
  int qf = base_angulonuevo;
  float delayTime = 0.045;              // Tiempo entre puntos (segundos)
  float tf = abs(q0 - qf) * delayTime;    // Tiempo total de la trayectoria

  // Cálculo de coeficientes del polinomio cúbico:
  // q(t) = a3*t^3 + a2*t^2 + a1*t + a0
  float a0 = q0;                        // q(0) = q0
  float a1 = 0;                         // Velocidad inicial 0
  float a2 = -3.0 * (q0 - qf) / (tf * tf);
  float a3 = 2.0 * (q0 - qf) / (tf * tf * tf);

  // Inicializar el tiempo actual
  float currentTime = 0.0;
  Serial.println("done trayectoria");
  while (currentTime <= tf) {
    // Calcular la posición en el tiempo currentTime
    float q = a3 * currentTime * currentTime * currentTime + 
              a2 * currentTime * currentTime + 
              a1 * currentTime + 
              a0;

    int q_int = round(q);
    ServoBase.write(q_int);
    
    // Imprimir tiempo y posición en el monitor serial
    Serial.print("t: ");
    Serial.print(currentTime, 3);
    Serial.print(" s, q: ");
    Serial.println(q_int);

    delay(delayTime * 1000);  // Convertir a milisegundos

    // Incrementar currentTime para avanzar en la trayectoria
    currentTime += delayTime;
  }
  
  base_anguloprevio = base_angulonuevo;

  }
}
void VarillasDesp0() {
  ServoDespDer.write(0);
  delay(10);
  ServoDespIzq.write(180);
  VarillasDespPos = 0;
}

void VarillasDesp1() {
  ServoDespDer.write(97);
  delay(10);
  ServoDespIzq.write(90);
  VarillasDespPos = 1;
}

void VarillasDesp2() {
  ServoDespDer.write(180);
  delay(10);
  ServoDespIzq.write(0);
  VarillasDespPos = 2;
}
