#include "thingProperties.h"
#include "HardwareSerial.h"

HardwareSerial espSerial(2);

// Variables auxiliares para detectar cambios en las propiedades
bool prevModoManual = false;
bool prevS1 = false;      // Para detectar cambios en S1 (Stage 1)
bool prevSTOP = false;    // Para detectar cambios en STOP
bool prevRESET = false;   // Para detectar cambios en RESET

void setup() {
  // --- Serial principal para ver mensajes en el Monitor Serie ---
  Serial.begin(9600);
  delay(1500); 

  // --- Inicialización de las variables y conexión a Arduino IoT Cloud ---
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  // --- Inicializamos UART2 a 9600 baudios, RX=27, TX=26 ---
  espSerial.begin(9600, SERIAL_8N1, 27, 26);
}

void loop() {
  // Mantiene la sincronía con Arduino IoT Cloud
  ArduinoCloud.update();

  // --- Enviar cambio en S1 (Stage 1) ---
  if (S1 != prevS1) {
    espSerial.println(String("S1=") + (S1 ? "true" : "false"));
    Serial.println("Enviando cambio S1: " + String("S1=") + (S1 ? "true" : "false"));
    prevS1 = S1;
  }
  
  // --- Enviar cambio en STOP ---
  if (STOP != prevSTOP) {
    espSerial.println(String("STOP=") + (STOP ? "true" : "false"));
    Serial.println("Enviando cambio STOP: " + String("STOP=") + (STOP ? "true" : "false"));
    prevSTOP = STOP;
  }
  
  // --- Enviar cambio en RESET ---
  if (RESET != prevRESET) {
    espSerial.println(String("RESET=") + (RESET ? "true" : "false"));
    Serial.println("Enviando cambio RESET: " + String("RESET=") + (RESET ? "true" : "false"));
    prevRESET = RESET;
  }
  
  // --- Envío de cambios en el modo manual ---
  if (modoManual != prevModoManual) {
    if (modoManual) {
      espSerial.println("MANUAL=ON");
      Serial.println("Modo manual ACTIVADO, enviando: MANUAL=ON");
    } else {
      espSerial.println("MANUAL=OFF");
      Serial.println("Modo manual DESACTIVADO, enviando: MANUAL=OFF");
    }
    prevModoManual = modoManual;
  }
  
  // --- Si el modo manual está activo, se envían periódicamente los estados de los botones de los relés ---
  if (modoManual) {
    espSerial.println(String("A_mas=")   + (btnA_mas   ? "ON" : "OFF"));
    espSerial.println(String("A_menos=") + (btnA_menos ? "ON" : "OFF"));
    espSerial.println(String("B_mas=")   + (btnB_mas   ? "ON" : "OFF"));
    espSerial.println(String("B_menos=") + (btnB_menos ? "ON" : "OFF"));
    espSerial.println(String("C_mas=")   + (btnC_mas   ? "ON" : "OFF"));
    espSerial.println(String("C_menos=") + (btnC_menos ? "ON" : "OFF"));
    espSerial.println(String("D_mas=")   + (btnD_mas   ? "ON" : "OFF"));
    espSerial.println(String("D_menos=") + (btnD_menos ? "ON" : "OFF"));
  }
  
  // --- LECTURA DE DATOS POR UART2 ---
  if (espSerial.available()) {
    // Leemos hasta el salto de línea
    String recibido = espSerial.readStringUntil('\n');
    recibido.trim();
    // Mostramos el mensaje recibido en el Monitor Serial
    Serial.println("Mensaje recibido: " + recibido);
    // Parseamos el mensaje para actualizar las propiedades y los valores (si es necesario)
    parseMessage(recibido);
  }
  
  // Aquí puedes añadir más lógica si lo deseas
}


void parseMessage(const String& msg) {
  if (msg == "S1=true") {
    S1 = true;
  } else if (msg == "S1=false") {
    S1 = false;
  }
  else if (msg == "S2=true") {
    S2 = true;
  } else if (msg == "S2=false") {
    S2 = false;
  }
  else if (msg == "S3=true") {
    S3 = true;
  } else if (msg == "S3=false") {
    S3 = false;
  }
  else if (msg == "S4=true") {
    S4 = true;
  } else if (msg == "S4=false") {
    S4 = false;
  }
  else if (msg == "S5=true") {
    S5 = true;
  } else if (msg == "S5=false") {
    S5 = false;
  }
  else if (msg == "S6=true") {
    S6 = true;
  } else if (msg == "S6=false") {
    S6 = false;
  }
  else if (msg == "S7=true") {
    S7 = true;
  } else if (msg == "S7=false") {
    S7 = false;
  }
  else if (msg == "S8=true") {
    S8 = true;
  } else if (msg == "S8=false") {
    S8 = false;
  }
  else if (msg == "S9=true") {
    S9 = true;
  } else if (msg == "S9=false") {
    S9 = false;
  }
  else if (msg == "S10=true") {
    S10 = true;
  } else if (msg == "S10=false") {
    S10 = false;
  }
  else if (msg == "S11=true") {
    S11 = true;
  } else if (msg == "S11=false") {
    S11 = false;
  }
  else if (msg == "S12=true") {
    S12 = true;
  } else if (msg == "S12=false") {
    S12 = false;
  }
  else if (msg == "S13=true") {
    S13 = true;
  } else if (msg == "S13=false") {
    S13 = false;
  }
  else if (msg == "S14=true") {
    S14 = true;
  } else if (msg == "S14=false") {
    S14 = false;
  }
  else if (msg == "S15=true") {
    S15 = true;
  } else if (msg == "S15=false") {
    S15 = false;
  }
  // LDR_Derecha
  else if (msg == "LDR_Derecha=true" || msg == "LDR_Derecha=1") {
    lDRderecha = true;
  } else if (msg == "LDR_Derecha=false" || msg == "LDR_Derecha=0") {
    lDRderecha = false;
  }
  // LDR_Izquierda
  else if (msg == "LDR_Izquierda=true" || msg == "LDR_Izquierda=1") {
    lDRizquierda = true;
  } else if (msg == "LDR_Izquierda=false" || msg == "LDR_Izquierda=0") {
    lDRizquierda = false;
  }
  // Avance del motor (escala 0-100)
  else if (msg.startsWith("AV_Izq=")) {
    String valor = msg.substring(7);
    avanceizquierda = valor.toInt();
  }
  else if (msg.startsWith("AV_Der=")) {
    String valor = msg.substring(7);
    avancederecha = valor.toInt();
  }
  // Valores PID para el motor izquierdo (0.0058 CONVERSION DE CUENTAS DE ENCODER A cm)
  else if (msg.startsWith("In_Izq=")) {
    String valor = msg.substring(7);
    inputIzquierda = 0.0058 * (valor.toFloat());
  }
  else if (msg.startsWith("SP_Izq=")) {
    String valor = msg.substring(7);
    setpointIzquierda = 0.0058 * (valor.toInt());
  }
  else if (msg.startsWith("Out_Izq=")) {
    String valor = msg.substring(8);
    outputIzquierda = valor.toFloat();
  }
  // Valores PID para el motor derecho
  else if (msg.startsWith("In_Der=")) {
    String valor = msg.substring(7);
    inputDerecha = 0.0058 * (valor.toFloat());
  }
  else if (msg.startsWith("SP_Der=")) {
    String valor = msg.substring(7);
    setpointDerecha = 0.0058 * (valor.toInt());
  }
  else if (msg.startsWith("Out_Der=")) {
    String valor = msg.substring(8);
    outputDerecha = valor.toFloat();
  }
}
/*
  Since BtnAMas is READ_WRITE variable, onBtnAMasChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onBtnAMasChange()  {
  // Add your code here to act upon BtnAMas change
}
/*
  Since BtnAMenos is READ_WRITE variable, onBtnAMenosChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onBtnAMenosChange()  {
  // Add your code here to act upon BtnAMenos change
}
/*
  Since BtnBMas is READ_WRITE variable, onBtnBMasChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onBtnBMasChange()  {
  // Add your code here to act upon BtnBMas change
}
/*
  Since BtnBMenos is READ_WRITE variable, onBtnBMenosChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onBtnBMenosChange()  {
  // Add your code here to act upon BtnBMenos change
}
/*
  Since BtnCMas is READ_WRITE variable, onBtnCMasChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onBtnCMasChange()  {
  // Add your code here to act upon BtnCMas change
}
/*
  Since BtnCMenos is READ_WRITE variable, onBtnCMenosChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onBtnCMenosChange()  {
  // Add your code here to act upon BtnCMenos change
}
/*
  Since BtnDMas is READ_WRITE variable, onBtnDMasChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onBtnDMasChange()  {
  // Add your code here to act upon BtnDMas change
}
/*
  Since BtnDMenos is READ_WRITE variable, onBtnDMenosChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onBtnDMenosChange()  {
  // Add your code here to act upon BtnDMenos change
}
/*
  Since ModoManual is READ_WRITE variable, onModoManualChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onModoManualChange()  {
  // Add your code here to act upon ModoManual change
}
/*
  Since RESET is READ_WRITE variable, onRESETChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onRESETChange()  {
  // Add your code here to act upon RESET change
}
/*
  Since S1 is READ_WRITE variable, onS1Change() is
  executed every time a new value is received from IoT Cloud.
*/
void onS1Change()  {
  // Add your code here to act upon S1 change
}
/*
  Since STOP is READ_WRITE variable, onSTOPChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onSTOPChange()  {
  // Add your code here to act upon STOP change
}