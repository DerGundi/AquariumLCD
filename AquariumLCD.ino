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
volatile unsigned long rpmCount = 0;

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

void temperaturSteuerung(){
  //Lüftersteuerung und Lastausgabe:
  lcd.setCursor(4,1);
  if(temperatur < 19.4){
    analogWrite(rpmOutPin, 0); //Unter 19.4 kann aus
    lcd.print("0");            //PWM Duty ausgeben
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

  //RPM INPUT
  pinMode(rpmInPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rpmInPin), rpmISR, RISING);

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

  //Ab jetzt die Temperatursteuerung schonmal das erste Mal laufen lassen
  //Temperaturen setzen, vorherige bekommen zum start die selbe
 // temperatur = round(sensors.getTempCByIndex(0) * 10.0) / 10.0;
  //vorherigeTemperatur = temperatur;
  temperaturSteuerung();
}

void loop() {
  //RPM Zählung
  //Damit beim Zurücksetzen des rpmCounts nichts dazwischenfunkt, die Interrupts ausschalten
  noInterrupts(); 
  rpmCount = 0;
  interrupts();
  //und wieder einschalten. Dadurch werden die Interrups ausgeführt und die Anzahl der Impulse des RPM-Gebers gezählt
  /*unsigned long currentTime = millis();
  while(currentTime - lastTime < 1001){
    currentTime = millis();
    if(currentTime - lastTime > 1001){
      break;
    }
  }
  lastTime = currentTime;*/
  delay(1000);
  //Dafür hat das ganze eine Sekunde Zeit

  //Selbes Spiel wie gerade, damit der Wert von seconds nicht verfälscht wird, schalten wir die Interrupts solagne aus und danach wieder ein
  noInterrupts();
  unsigned long rcount = rpmCount;
  interrupts();

  //Berechnung der rpm: Da wir doppelt so viel Messen wie nötig, wird Wert um 2 geteilt und dann mit 60 (die Sekuden) multipliziert
  int rpm = (rcount / 2) * 60;
  //Ausgabe der rpm auf LCD
  lcd.setCursor(12, 1);
  lcd.print(rpm);
  lcd.print("   "); //Lerzeichen, damit alte Zahlen gelöscht werden
  
  //Temperatur abfragen
  sensors.requestTemperatures();

  //Sollte der Sensor keinen(!) Defekt aufweisen
  if(sensors.getTempCByIndex(0) != DEVICE_DISCONNECTED_C){
  //auf eine Dezimalstelle runden
  temperatur = round(sensors.getTempCByIndex(0) * 10.0) / 10.0;
  
  //Lüftergeschwindigkeit soll sich erst ändern, wenn der vorherige Wert 10 Sekunden lang verändert ist
 /* float changedTemp = temperatur - vorherigeTemperatur;
  if((abs(changedTemp) >= 0.10)){
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
    seconds = 0;
  }*/
  temperaturSteuerung();
  lcd.setCursor(11, 0);
  // Temperatur anzeigen
  lcd.print(temperatur);
  //° Gradzeichen
  lcd.setCursor(15, 0);
  lcd.write(223);

  }else{ //Wenn der Sensor defekt ist
    lcd.clear();
    lcd.print("TEMPSENSOR");
    lcd.setCursor(0,1);
    lcd.print("DEFEKT!");
    while(sensors.getTempCByIndex(0) == DEVICE_DISCONNECTED_C){
      //Wenn Temperatursensor ausfällt, ist Kühlung besser als Hitzestau, daher Volllast.
      analogWrite(rpmOutPin, 63);
    }
    Serial.print("Sperre aufgehoben");
    lcd.clear();
    lcd.flush();
    delay(750);
    setup();
  }

  //Ausgabe PWM und RPM in die Console
  Serial.print("TempJ: ");
  Serial.print(temperatur);
  Serial.print(", TempV: ");
  Serial.print(vorherigeTemperatur);
  Serial.print(", PWM: ");
  Serial.print(constrain(round(100 * temperatur - 1940) * 63 / 120, 0, 63)); //Falls Wert rechnerisch über 63 
  Serial.print(", RPM: ");
  Serial.println(rpm);
}

