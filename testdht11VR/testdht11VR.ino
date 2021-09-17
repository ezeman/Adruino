#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define DHTTYPE DHT21
#define SetAirHudPin      A0
#define SetAirTempPin     A1
#define SetSoilHudPin     A2
#define SetSoilTempPin    A3
#define SoilHudPin        A7
#define ONE_WIRE_BUS      4
#define DHTPIN            5
#define WaterRelayPin     6
#define FanRelayPin       7
#define FogRekayPin       8
#define I2C_ADDR 0x27
#define BACKLIGHT_PIN 3

DHT dht(DHTPIN, DHTTYPE, 21); // 11 works fine for ESP8266
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(I2C_ADDR,2,1,0,4,5,6,7);

//int humidity, temp_f;  // Values read from sensor

int WaterRelayPinState, FanRelayPinState, FogRekayPinState;

void setup(void) {
  // You can open the Arduino IDE Serial Monitor window to see what the code is doing
// Serial.begin(9600);  // Serial connection from ESP-01 via 3.3v console cable
  dht.begin();           // initialize temperature sensor
  sensors.begin();
  lcd.begin (16,2);
 lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
lcd.setBacklight(HIGH);

 
 pinMode(WaterRelayPin, OUTPUT);
 pinMode(FanRelayPin, OUTPUT);
 pinMode(FogRekayPin, OUTPUT);

digitalWrite(FanRelayPin,HIGH);
digitalWrite(FogRekayPin,HIGH);
digitalWrite(WaterRelayPin,LOW);
WaterRelayPinState = 0;
FanRelayPinState = 0;
FogRekayPinState = 0;
 }
 
void loop(void) {

   int humidity = dht.readHumidity();          // Read humidity (percent)
   int temp_f = dht.readTemperature();     // Read temperature as Fahrenheit

 sensors.requestTemperatures();
 int SoilTemp = sensors.getTempCByIndex(0);

  int SetAirHud = analogRead(SetAirHudPin);
  int SetAirTemp = analogRead(SetAirTempPin);
  int SetSoilHud = analogRead(SetSoilHudPin);
  int SetSoilTemp = analogRead(SetSoilTempPin);
  int SoilHudSensor = analogRead(SoilHudPin);
  
 int AirHud100 = map(SetAirHud, 0, 1023, 20, 81);
 int AirTemp100 = map(SetAirTemp, 0, 1023, 20, 51);
 int SoilHud100 = map(SetSoilHud, 0, 1023, 0, 11);
 int SoilTemp100 = map(SetSoilTemp, 0, 1023, 20, 51);
 int SoilHudSensor100 = map(SoilHudSensor, 0, 1023, 11, 0);

//digitalWrite(FanRelayPin,LOW);
//digitalWrite(FogRekayPin,LOW);

if(humidity<=AirHud100){
  digitalWrite(FogRekayPin,LOW);
  FogRekayPinState = 1;
  
}
else{
  digitalWrite(FogRekayPin,HIGH);
  FogRekayPinState = 0;
 
}



if(temp_f>=AirTemp100){
  digitalWrite(FanRelayPin,LOW);
  FanRelayPinState =1;

}
else{
  digitalWrite(FanRelayPin,HIGH);
  FanRelayPinState = 0;

}


if(humidity>=91 && FanRelayPinState==0){
  

  digitalWrite(FanRelayPin,LOW);


  
}



if(SoilHudSensor100<=SoilHud100){
  digitalWrite(WaterRelayPin,HIGH);
  WaterRelayPinState = 1;

}
else{
  digitalWrite(WaterRelayPin,LOW);
  WaterRelayPinState = 0;
}

/*
 Serial.print("FogRekayPinState=");
 Serial.print(FogRekayPinState);
 Serial.print("--");

  Serial.print("FanRelayPinState=");
 Serial.print(FanRelayPinState);
 Serial.print("--");

  Serial.print("WaterRelayPinState=");
 Serial.print(WaterRelayPinState);
 Serial.println();
*/


if(SoilTemp>=SoilTemp100 && WaterRelayPinState==0){
  

  digitalWrite(WaterRelayPin,HIGH);


  
}



lcd.clear();
lcd.setCursor(0,0); 
lcd.print("Ah");
lcd.print(humidity); 
lcd.print("/"); 
lcd.print(AirHud100); 


lcd.setCursor(8,0);
lcd.print("At");
lcd.print(temp_f); 
lcd.print("/"); 
lcd.print(AirTemp100); 


lcd.setCursor(0,1);
lcd.print("Sh"); 
lcd.print(SoilHudSensor100); 
lcd.print("/"); 
lcd.print(SoilHud100); 

lcd.setCursor(8,1);
lcd.print("St");
lcd.print(SoilTemp); 
lcd.print("/"); 
lcd.print(SoilTemp100); 

/*
   
Serial.print(humidity);
Serial.print(",");
Serial.print(temp_f);
Serial.print(",");
Serial.print(SoilTemp);
Serial.print(",");
Serial.print(AirHud100);
Serial.print(",");
Serial.print(AirTemp100);
Serial.print(",");
Serial.print(SoilHud100);
Serial.print(",");
Serial.print(SoilTemp100);
Serial.print(",");
Serial.print(SoilHudSensor100);
Serial.println();
*/

delay(1000);
}
