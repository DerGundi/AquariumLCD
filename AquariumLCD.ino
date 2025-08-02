/*
  LCD-Pinbelegung

 * LCD RS pin to digital pin 7
 * LCD Enable pin to digital pin 8
 * LCD D4 pin to digital pin 9
 * LCD D5 pin to digital pin 10
 * LCD D6 pin to digital pin 11
 * LCD D7 pin to digital pin 12
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 http://www.arduino.cc/en/Tutorial/LiquidCrystal
 */

//Bibliotheken
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//definiere pins
int tempPin = 5;
int rpmInPin = 2;
int rpmOutPin = 3;

//LCD mit passenden Pins als Objekt definieren
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

//Für die RPM-Messung
volatile long rpmCount = 0;

//Definiere Sensor als OneWire Objekt
OneWire tempSensor(tempPin);
//DallasTemperature braucht Objektadresse von OneWire Objekt
DallasTemperature sensors(&tempSensor);

//Startwerte
float temperatur;
float vorherigeTemperatur;
volatile int seconds;
unsigned long lastTime = 0;

//Erhöhung des rpmCounts in einem Interrupt, damit der Wert nicht verfälscht wird
//Quasi eine Art Semaphorenprinzip
void rpmISR(){
    rpmCount++;
}

//Startprozedur, ausgelagert da möglicherweise mehrfach aufgerufen werden kann
void startUpProcedure(){
  //Temperatursonde; wir müssen der Bibliothek tatsächlich sagen, dass sie anfangen soll
  sensors.begin();

  //Vorschreiben von Werten auf LCD:
  //Temperatur:
  lcd.print("Temperatur:");
  //Last:
  lcd.setCursor(0,1);
  lcd.print("DTY:"); //DTY für Duty
  //RPM:
  lcd.setCursor(8,1);
  lcd.print("RPM:");
}

void temperaturSteuerung(){
  //Lüftersteuerung und Lastausgabe:
  lcd.setCursor(4,1);
  if(temperatur < 19.4){
    analogWrite(rpmOutPin, 0); //Unter 19.4 kann aus
    lcd.print("0");            //PWM Duty ausgeben
    lcd.print("  ");           //Leerzeichen um alte Zahlen zu entfernen
  }else if(temperatur > 20.5){
    analogWrite(rpmOutPin, 63); //Vollast ab 20.6 Grad
    lcd.print("100");           //PWM Duty ausgeben
  }else{
    analogWrite(rpmOutPin, round(100 * temperatur - 1940) * 63 / 120); //Lineare Steigung
    //Temperatur - maximal relevante Temperatur (19,4) * die maximale Anzahl an PWM-Stufen (63) / 120 zur Normalisierung
    lcd.print(round(100 * temperatur - 1940) * 100 / 120 ); //PWM Duty in Prozent ausgeben
    lcd.print(" ");
  }
}

//Falls Sensor defekt, Lüfter auf 100%. Kühle ist besser als Wärme
void sensorDefekt(){
  analogWrite(rpmOutPin, 63);
  lcd.clear();
  delay(1000); //LCD Zeit geben, um zu leeren
  lcd.setCursor(0,0);
  lcd.print("Tempsensor");
  lcd.setCursor(0,1);
  lcd.print("defekt!");
  while(temperatur <= 10 || temperatur >= 31){
    temperatur = sensors.getTempCByIndex(0);
    delay(5000); //Sensor Zeit geben um wieder gefunden werden zu können
  }
  lcd.clear();
  lcd.setCursor(0,0);
  //Von vorn anfangen
  setup();
}

void setup() {
  //Console aktivieren
  Serial.begin(9600);

  //LCDs Höhe und Breite definieren
  lcd.begin(16, 2);

  //PWM Pin auf 25kHz erhöhen, dafür Timer2 anpassen
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // Fast PWM, non-inverting auf OC2B
  TCCR2B = (1 << WGM22) | (1 << CS20); // WGM22=1 für TOP=OCR2A, Prescaler=1
  OCR2A = 63;

  //PWM OUTPUT
  pinMode(rpmOutPin, OUTPUT);
  //analogWrite(rpmOutPin, 63); //PWM Signal für Lüfter, Intervall 0-63
  //analogWrite(rpmOutPin, 0);

  //RPM INPUT
  pinMode(rpmInPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rpmInPin), rpmISR, RISING);

  startUpProcedure();
}

void loop() {
  //Sensordaten abfragen
  sensors.requestTemperatures();
  //RPM zurücksetzen
  noInterrupts();
  rpmCount = 0;
  interrupts();

  delay(1000); //Sensor Zeit geben, dabei Pulse zählen

  //Pulse auswerten
  noInterrupts();
  long measuredRPM = rpmCount * 30; // RPM / Zeit * 2, also RPM / 60 * 2 -> rpm * 30
  interrupts();

  //Temperatur abfragen
  temperatur = round(sensors.getTempCByIndex(0) * 10.0) / 10.0; //Auf eine Dezimalstelle runden
  
  //10 Sekunden Threshold, um die Lüfter zu steuern
  if(temperatur != vorherigeTemperatur && temperatur < 20.5 && temperatur > 19.5){
    seconds++;
    Serial.print("Changed Seconds: ");
    Serial.println(seconds);
    if(seconds > 9){
      seconds =  0;
      vorherigeTemperatur = temperatur;
      Serial.println("Seconds reset!");
      temperaturSteuerung();
  }
}else{
    vorherigeTemperatur = temperatur;
    seconds = 0;
  }



   //Plausibilitätsprüfung, wenn wir über und unter diesen Grenzen sind, kann der Sensor nicht funktionieren
   //oder die Lotl sind zu Eiswürfeln bzw. Fischstäbchen geworden.
  if(temperatur <= 10 || temperatur >= 31){
    sensorDefekt();
  }





  temperaturSteuerung();
  
  


  //Ausgabe auf LCD
  lcd.setCursor(11,0);
  lcd.print(temperatur); //Runden auf eine Nachkommastelle
  lcd.write(223); //° Gradzeichen
  lcd.print("C");

  lcd.setCursor(12,1);
  lcd.print(measuredRPM);

  //Ausgabe in die Console
  Serial.print("Temp: ");
  Serial.print(temperatur);
  Serial.print(", PWM: ");
  Serial.print(constrain(round(100 * temperatur - 1940) * 63 / 120, 0, 63)); //Falls Wert rechnerisch über 63 
  Serial.print(", RPM: ");
  Serial.print(measuredRPM);
  Serial.println("");
}

