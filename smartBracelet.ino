#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "Arduino_BMI270_BMM150.h"
#include "DFRobotDFPlayerMini.h"
#include <ArduinoBLE.h>

// ---------------- MAX30102 ----------------
MAX30105 sensor;
DFRobotDFPlayerMini myDFPlayer;

// UUID-urile TREBUIE să fie identice cu cele din HTML
BLEService alertService("19b10000-e8f2-537e-4f6c-d104768a1214");
BLEStringCharacteristic alertCharacteristic("19b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify, 50);

unsigned long previousMillis = 0;
const long intervalNormal = 500;  // Trimite status normal la fiecare 0.5 secunde
const long intervalAlerta = 100;  // Când e criză, trimite FOARTE DES (0.1 secunde)

unsigned long pulsZ = 0; 

float irEMA = 0;
float alpha = 0.05;
const int medianWindow = 7;
long medianBuffer[medianWindow];
int medianIndex = 0;

int buttonState = LOW;
int lastButtonState = LOW;
bool buttonPressed = false;

boolean audio=false;
boolean audio2=false;
unsigned long startAudio = 0;
unsigned long startAudio2 = 0;
const unsigned long audioDuration = 4000; 
const unsigned long audioDuration2 = 6000; 
const unsigned long ct = 8002; 

unsigned long verificare=0; 
const unsigned long timpVerificare = 120000; 

unsigned long lastBeat = 0;

const int bpmWindow = 4;
float bpmBuffer[bpmWindow];
int bpmIndex = 0;
bool bpmFull = false;
float bpmFiltered = 0;
float lastBpmFiltered = 0; // pastrat pt compatibilitate

// ---------------- PRIMELE BĂTĂI ----------------
int beatCount = 0;

// ---------------- IMU ----------------
float ax, ay, az;
float lastMagnitude = 0;
int countBrusque = 0;
int countBrahi=0;
int countTahi=0;
const float MAG_THRESHOLD = 1.2;
const float DIFF_THRESHOLD = 0;
const unsigned long windowConv = 10000;
unsigned long startConv = 0;

// ---------------- LED + BUTON ----------------
const int ledExternPin = 7;   // D7
const int buttonPin = 3;      // D3

// ---------------- ALERTĂ GLOBALĂ ----------------
bool alertConvulsii = false;
bool alertPuls = false;
bool alertaActiva = false;  // True când LED-ul extern e aprins

// ---------------- PRAGURI PULS ----------------
float tachyBPM = 10; 
float bradyBPM = 10; 
int variabilityCount = 0;
float jumpBPMThreshold = 12;
int jumpCountThreshold = 3;

// ---------------- FILTER ----------------
long medianFilter(long newValue) {
  medianBuffer[medianIndex] = newValue;
  medianIndex = (medianIndex + 1) % medianWindow;
  long tmp[medianWindow];
  for (int i = 0; i < medianWindow; i++) tmp[i] = medianBuffer[i];
  for (int i = 0; i < medianWindow - 1; i++)
    for (int j = i + 1; j < medianWindow; j++)
      if (tmp[j] < tmp[i]) { long t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
  return tmp[medianWindow / 2];
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
 // while (!Serial);   // așteaptă Serial

  Serial1.begin(9600); // DFPlayer
  if (!myDFPlayer.begin(Serial1)) {
      Serial.println("DFPlayer error!");
  } else {
      myDFPlayer.volume(25);
  }

  // Configurare BLE
  if (!BLE.begin()) {
    Serial.println("Eroare initializare BLE!");
    while (1);
  }
  BLE.setLocalName("ConvulsionAlert");
  BLE.setAdvertisedService(alertService);
  alertService.addCharacteristic(alertCharacteristic);
  BLE.addService(alertService);
  alertCharacteristic.writeValue("ALERTA OFF");
  BLE.advertise();
  Serial.println("BLE pornit, astept conectare...");

  // PinMode-uri
  pinMode(ledExternPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  // IMU și MAX30102
  if (!IMU.begin()) { Serial.println("Eroare IMU!"); while(1); }
  if (!sensor.begin()) { Serial.println("Eroare MAX30102!"); while(1); }

  sensor.setup();
  sensor.setPulseAmplitudeRed(0x1F);
  sensor.setPulseAmplitudeIR(0x1F);

  startConv = millis();
  lastBeat = millis();

  digitalWrite(ledExternPin, LOW);

  Serial.println("System ready.");
}


// ---------------- LOOP ----------------
void loop() {

  BLE.poll(); // Mentine conexiunea vie
  BLEDevice central = BLE.central();

  // ---------------- IMU ----------------
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    float magnitude = sqrt(ax*ax + ay*ay + az*az);
    float diff = abs(magnitude - lastMagnitude);

    if (!alertConvulsii) {
      if (millis() - startConv < windowConv) {
        if (magnitude > MAG_THRESHOLD && diff > DIFF_THRESHOLD) countBrusque++;
      } else 
      {
        if (countBrusque > 4)
      {
        alertConvulsii = true;
        verificare = millis(); // Reset timer auto-reset
        if (central && central.connected()) {
          alertCharacteristic.writeValue("CONVULSII");
        }
      }

          countBrusque = 0;
        startConv = millis();

      }
      
    }
    lastMagnitude = magnitude;
  }

  // ---------------- PULS ----------------
  long ir = sensor.getIR();
  irEMA = alpha * ir + (1 - alpha) * irEMA;
  long irFiltered = medianFilter(irEMA);

  if (checkForBeat(irFiltered)) {
      pulsZ=millis();
      
      unsigned long now = millis();
      int delta = now - lastBeat;
      lastBeat = now;

      if (delta > 0) {
        float bpm = 60.0 / (delta / 1000.0);

        if(bpm > 20 && bpm < 260) // Filtru simplu
        {
          bpmBuffer[bpmIndex++] = bpm;
          beatCount++;
        }
        

        if (bpmIndex >= bpmWindow) { bpmIndex = 0; bpmFull = true; }

        int count = bpmFull ? bpmWindow : bpmIndex;
        float sum = 0;

        int index=0;
        for (int i = 0; i < count; i++) 
        {
          if(bpmBuffer[i]!=0)
          {
             sum += bpmBuffer[i];
             index++;
          }
        }
  
        if(index>0) bpmFiltered = (sum / index);

        // Logica ta pt tahicardie
        if (bpmFull) {
           if(bpmFiltered!=0 && bpmFiltered >=tachyBPM) countTahi++;
           if(bpmFiltered!=0 && bpmFiltered <=bradyBPM) countBrahi++;
         }

        // Alertă puls
        if (beatCount>0 && (countTahi>2 || countBrahi>2))
        {
          if(!alertPuls) {
             alertPuls = true;
             verificare = millis();
             if (central && central.connected()) {
                 // MODIFICARE: Adaugam valoarea la mesaj
                 String msg = "PB. PULS " + String(bpmFiltered);
                 alertCharacteristic.writeValue(msg.c_str());
             }
          }
        }
      }
  }
  else
  {
    if(millis()-pulsZ>=30000)
    {
       bpmFiltered=0;
    }
  }

  // ---------------- APRINDERE LED EXTERN ----------------
  if (alertConvulsii && alertPuls && !alertaActiva) {
    alertaActiva = true;
    digitalWrite(ledExternPin, HIGH); // aprindem LED-ul extern
    Serial.println("ALERTA MARE ACTIVATA!");

     if (central && central.connected()) {
      alertCharacteristic.writeValue("ALERTA");
    }
  }

    
    if(alertaActiva)
    {
      if (!audio) {
        myDFPlayer.play(1);       // pornește melodia
        startAudio = millis(); 
        startAudio2=startAudio;   // salvează momentul de start
        audio = true;
    } else{
        // verificăm dacă melodia s-a terminat
         if(millis() - startAudio2 >= ct)
         {
            if (!audio2) {
                myDFPlayer.play(2);       // pornește melodia
                startAudio = millis(); 
                audio2 = true;
            }
            else
            {
              if (millis() - startAudio >= audioDuration2) {
                  myDFPlayer.play(2);   // reia melodia
                  startAudio = millis();
              }
            }
          }
        else if (millis() - startAudio >= audioDuration) {
            myDFPlayer.play(1);   // reia melodia
            startAudio = millis();
        }
      }
    }

  // --- TRIMITERE DATE BLUETOOTH ---
  unsigned long currentMillis = millis();
  
  // Determinăm viteza: dacă e pericol, trimitem rapid. Dacă e calm, trimitem rar.
  long intervalCurent = (alertConvulsii || alertPuls) ? intervalAlerta : intervalNormal;

  if (currentMillis - previousMillis >= intervalCurent) {
    previousMillis = currentMillis;

    if (!alertaActiva && alertConvulsii) { 
       // Trimitem Alerta
       alertCharacteristic.writeValue("CONVULSII");
       String msg = "PULS " + String(bpmFiltered); 
       alertCharacteristic.writeValue(msg.c_str());
    } 
    else if (!alertaActiva && alertPuls) { 
       // Trimitem Puls + Valoare
       
       String msg = "PB. PULS " + String(bpmFiltered);
       alertCharacteristic.writeValue(msg.c_str());
    }
    else
    {
       String msg = "PULS " + String(bpmFiltered);
       alertCharacteristic.writeValue(msg.c_str());
    }
  }

  // ---------------- AUTO RESET (ALARMA FALSA) ----------------
  if(!alertaActiva && (alertPuls==true || alertConvulsii==true) && (millis() - verificare>=timpVerificare))
  {
         if(alertPuls==true)
         alertPuls = false;

        if(alertConvulsii==true)
         alertConvulsii = false;

         //countBrusque = 0;
         //countTahi = 0;
        // countBrahi = 0;
         //bpmIndex = 0;
         //bpmFull = false;

         alertCharacteristic.writeValue("ALARMA FALSA");

         verificare=millis();
  }
      

  // ---------------- SERIAL MONITOR ----------------
  float magnitude = sqrt(ax*ax + ay*ay + az*az);
  Serial.print("IR="); Serial.print(irFiltered); 
  Serial.print(" BPM="); Serial.print(bpmFiltered);
  Serial.print(" | mag="); Serial.print(magnitude);
  Serial.print(" | conv="); Serial.print(alertConvulsii);
  Serial.print(" | pulsAlert="); Serial.print(alertPuls);
  Serial.print(" | alertaActiva="); Serial.println(alertaActiva);

  // ---------------- BUTTON LOGIC ----------------
  buttonState = digitalRead(buttonPin);

  // detectăm o NOUĂ apăsare (LOW -> HIGH)
  if(alertaActiva)
  {
    if (buttonState== HIGH && lastButtonState==LOW) {
    buttonPressed = true;
  } 
  lastButtonState = buttonState;
  }
  else
  {
    buttonPressed = false;
        lastButtonState=LOW;
  }


  // ---------------- RESET MANUAL ----------------
  if (buttonPressed && alertaActiva) {
        digitalWrite(ledExternPin, LOW);
        alertaActiva = false;
        alertPuls = false;
        alertConvulsii = false;
        countBrusque = 0;
        countTahi = 0;
        countBrahi = 0;
        bpmIndex = 0;
        bpmFull = false;
        lastBeat = millis();
        startConv = millis();
        audio = false;
        audio2 = false;
        buttonPressed = false;
        lastButtonState=LOW;
        myDFPlayer.stop();

        if (central && central.connected()) {
            alertCharacteristic.writeValue("ALERTA OFF");
        }

        Serial.println("RESET MANUAL REALIZAT.");
        delay(200);
    }

  delay(10);
}