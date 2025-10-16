//Bibliotheken
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

//definiere pins
int tempPin = 5;
int rpmInPin = 2;
int rpmOutPin = 3;

//LCD mit passenden Pins als Objekt definieren
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

//Temperatursensor
OneWire tempSensor(tempPin);
DallasTemperature sensors(&tempSensor);

//Für die RPM-Messung
volatile long rpmCount = 0;
int measuredRPM = 0;

//Interrupt-Service-Routine um den rpmCount zu erhöhen
void rpmISR(){
  rpmCount++;
}

//Temperatur-Threshold
float minTemp = 19.3;
float maxTemp = 20.7;
float currentTemp = 0.0;
float newTemp = 0.0;
float diffTemp = 0.0;
int newTempCounter = 0;
int pwm = 0;

void tempChange(){
  lcd.setCursor(11,0);
  lcd.print(currentTemp);
  delay(200);
  lcd.setCursor(4,1);
  if(currentTemp < minTemp){
    OCR2B = 0;
    lcd.print("0  "); //PWM Duty
  }else if(currentTemp > maxTemp){
    OCR2B = 79;
    lcd.print("100"); //PWM Duty
  }else{
    pwm = round((currentTemp - minTemp) * (79.0 / (maxTemp - minTemp)));
    OCR2B = pwm;
    lcd.print(round((pwm / 79.0) * 100.0)); //Für Anzeige, bis 100%
    lcd.print(" ");
  }
}


void setup() {
  Serial.begin(115200);

  sensors.begin(); //Temperatursensor starten
  lcd.begin(16,2); //16 Zeichen bei je 2 Zeilen
  //Den Sensoren Zeit geben
  delay(500);

  // Timer2 auf 25 kHz PWM für Pin 3
TCCR2A = 0;
TCCR2B = 0;
OCR2A = 79;  // TOP-Wert für 25 kHz mit Prescaler=8
OCR2B = 0;
TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  // Fast PWM, non-inverting
TCCR2B = (1 << WGM22) | (1 << CS21);  // Prescaler=8 (nicht 1!)

  //Inputs und Outputs setzen
  pinMode(rpmOutPin, OUTPUT);
  pinMode(rpmInPin, INPUT); //externer 10kOhm Pullup, daher kein interner
  attachInterrupt(digitalPinToInterrupt(rpmInPin), rpmISR, RISING);

  sensors.requestTemperatures();

  //Vorschreiben auf LCD
  lcd.setCursor(0,0);
  lcd.print("Temperatur:");

  lcd.setCursor(0,1);
  lcd.print("DTY:");
  lcd.setCursor(8,1);
  lcd.print("RPM:");
  delay(500); //Dem LCD Zeit geben

  //Damit im ersten Durchlauf die tempChange getriggert werden kann
  currentTemp = round(sensors.getTempCByIndex(0) * 10) / 10.0;
  tempChange();
}

void loop() {
  sensors.requestTemperatures();

  noInterrupts();
  rpmCount = 0;
  interrupts();
  delay(1000); //Hier werden dann die Impulse vom Tachosignal gezählt
  noInterrupts();
  measuredRPM = rpmCount * 30; //RPM durch Zeit durch 2, da für eine Umdrehung 2 Impulse gezählt werden
  interrupts();


  //Temperaturhandling
  newTemp = round(sensors.getTempCByIndex(0) * 10) / 10.0; //Temp abfragen und auf eine Nachkommastelle runden
  diffTemp = fabs(newTemp - currentTemp);

  if(newTempCounter == 10){
    currentTemp = newTemp;
    newTempCounter = 0;
    tempChange();
    Serial.println("Counter zurückgesetzt, neue Temperatur übernommen!");
  }else if (diffTemp < 0.3 && diffTemp > 0.0){ //Damit man Extremausschläge unterbindet
    newTempCounter++;
    Serial.println(" ");
    Serial.println("Temperaturänderung!");
    Serial.print("Vorher: ");
    Serial.print(currentTemp);
    Serial.print("°C, Jetzt:");
    Serial.print(newTemp);
    Serial.println("°C");
    Serial.println(" ");
  }else{
    //Wenn wir die Temperatur nicht 10 Sekunden wirklich halten, dann fangen wir von vorne an.
    //Niemand mag Lüfter, die dauernd die Geschwindigkeit ändern.
    newTempCounter = 0;
  }

  //Ausgabenhandling
  lcd.setCursor(12,1);
  lcd.print(measuredRPM);
  lcd.print("   ");
  Serial.print("Aktuelle Temperatur: ");
  Serial.print(currentTemp);
  Serial.print("°C, Aktuelle PWM: ");
  Serial.print(pwm);
  Serial.print(", Aktuelle RPM:");
  Serial.println(measuredRPM);
}