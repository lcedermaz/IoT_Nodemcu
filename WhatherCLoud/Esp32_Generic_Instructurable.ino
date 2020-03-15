 // Este es un proyecot hecho por el siguiente loco:
 // https://www.instructables.com/id/ESP32-Weathercloud-Weather-Station/
 
 //wi-fi
#include <WiFi.h>
const char* ssid     = "SSID";
const char* password = "KEY";
WiFiClient client;
const int httpPort = 80;
const char* host = "http://api.weathercloud.net";
const char* streamId   = "....................";
const char* privateKey = "....................";
//-------------------------------------------------------------------------------------
//temperature
#include <OneWire.h>
#include <DallasTemperature.h>
//-------------------------------------------------------------------------------------
OneWire oneWire(4); 
DallasTemperature sensors(&oneWire);
DeviceAddress groundThermometer = { 0x28,  0xD4,  0xB0,  0x26,  0x0,  0x0,  0x80,  0xBC };
DeviceAddress airThermometer = { 0x28,  0xF4,  0xBC,  0x26,  0x0,  0x0,  0x80,  0x2B };
//-------------------------------------------------------------------------------------
//humidity and pressure
#include <BME280I2C.h>
#include <Wire.h>
#include "DHT.h"
DHT dht(18, DHT21);
BME280I2C bme;
//-------------------------------------------------------------------------------------
//solar radiation
#include <BH1750.h>
BH1750 lightMeter;
//-------------------------------------------------------------------------------------
//wind speed
boolean state = false;
int clicked;
unsigned long lastMillis = 0;
//-------------------------------------------------------------------------------------
//rainfall
#include <Wire.h>
int input, rainfall, rainrate;
//-------------------------------------------------------------------------------------
//RTC
#include <Wire.h>
#include "RTClib.h"
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//-------------------------------------------------------------------------------------
void setup(){
    Serial.begin(115200);
    //wi-fi
    delay(100);
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    //-------------------------------------------------------------------------------------
    //temperature
    sensors.begin();
    sensors.getAddress(airThermometer, 1); 
    sensors.getAddress(groundThermometer, 0); 
    sensors.setResolution(airThermometer, 15);
    sensors.setResolution(groundThermometer, 15);
    //-------------------------------------------------------------------------------------
    //humidity and pressure
    Wire.begin();
    bme.begin();
    //-------------------------------------------------------------------------------------
    //solar radiation
    lightMeter.begin();
    //-------------------------------------------------------------------------------------
    //wind speed
    pinMode(26, INPUT);
    //-------------------------------------------------------------------------------------
    //RTC
    rtc.begin();
}
//----------------------------------------------------------------------------------------------------------
void loop(){
    //temperature
    sensors.requestTemperatures();
    delay(10);
    float tempair =  sensors.getTempC(airThermometer);
    float tempground = sensors.getTempC(groundThermometer);
    int temp = int(tempair*10);
    int tempin = int(tempground*10);
    //-------------------------------------------------------------------------------------
    //humidity and pressure
    float tempe(0), hum(0), pres(0);
    bme.read(pres, tempe, hum, BME280::TempUnit_Celsius, BME280::PresUnit_Pa);
    int bar = int(pres/10);
    //-------------------------------------------------------------------------------------
    //heat index
    float hic = dht.computeHeatIndex(tempair, hum, false);
    int heat = int(hic*10);
    //-------------------------------------------------------------------------------------
    //dew point
    double gamma = log(hum / 100) + ((17.62 * tempair) / (243.5 + tempair));
    double dp = (243.5 * gamma / (17.62 - gamma));
    int dew = int(dp*10);
    //-------------------------------------------------------------------------------------
    //solar radiation
    float lux = lightMeter.readLightLevel();
    int solarrad = lux*0.79;
    //-------------------------------------------------------------------------------------
    //UV radiation
    float voltage = (analogRead(34) * (3.3 / 4095));
    float UVIndex = (voltage - 1)* 7.5;
    if(voltage > 2.46) UVIndex = 11;
    if(voltage < 1) UVIndex = 0;
    int uvi = int(UVIndex*10);
    //-------------------------------------------------------------------------------------
    //wind speed
    Serial.println("Starting wind speed measurment period.");
    lastMillis = xTaskGetTickCount();
    while(xTaskGetTickCount() - lastMillis < 10000){
        if(digitalRead(26) == HIGH) if(state == false){
            delay(50);
            clicked++;
            state = true;
        }
        if(digitalRead(26) == LOW) if(state == true) state = false;
    }
    float mps = clicked * 0.0333;
    float kph = mps * 3.6;
    int wspd = int(mps*10);
    //-------------------------------------------------------------------------------------
    //wind direction
    int wdir;
    int ar = analogRead(35);
    if(ar > 2155 && ar < 2207)   wdir = 0;
    if (ar > 502  && ar < 554) wdir = 22.5;
    if (ar > 649  && ar < 699) wdir = 45;
    //if (ar >  && ar < ) wdir = 67.5;
    //if (ar > 0 && ar < 0) wdir = 90;
    if (ar > 49 && ar < 90) wdir = 112.5;
    if (ar > 89 && ar < 111) wdir = 135;
    if (ar > 0 && ar < 11) wdir = 157.5;
    if (ar > 242 && ar < 280) wdir = 180;
    if (ar > 159 && ar < 211) wdir = 202.5;
    if (ar > 1239 && ar < 1301) wdir = 225;
    if (ar > 1129 && ar < 1161) wdir = 247.5;
    if (ar > 4069 && ar < 5001) wdir = 270;
    if (ar > 2470 && ar < 2516) wdir = 292.5;
    if (ar > 3101 && ar < 3321) wdir = 315;
    if (ar > 1629 && ar < 1656) wdir = 337.5;
    if(ar == 0){
         delay(200);
         ar = analogRead(35);
         if(ar == 0) wdir = 90;
         else wdir = 157.5;
    }
    //-------------------------------------------------------------------------------------
    //rainfall
    Wire.requestFrom(105, 4);
    while (Wire.available()){ 
    input = Wire.read();
        if(input == 1){
        rainfall++;
        delay(100);
        }
    }
    DateTime now = rtc.now();
    int hodina = now.hour();
    int minuta = now.minute();
    if(hodina == 0) if(minuta == 0) rainfall = 0;
    //-------------------------------------------------------------------------------------
    //wind chill
    float chill = (13.12 + (0.62 * tempair) - (11.37 * (pow(kph, 0.16))) + (0.39 * tempair * (pow(kph, 0.16))));
    int wchill = int(chill*10);
    if(clicked < 2)wchill = temp;
    if(tempair > 30)wchill = temp;
    //-------------------------------------------------------------------------------------
    //send data
    if (!client.connect(host, httpPort)) {
        Serial.println("Connection with server failed.");
        return;
    }
    client.print("GET /set/wid/WID/key/KEY/temp/");
    client.print(temp);
    client.print("/tempin/");
    client.print(tempin);
    client.print("/chill/");
    client.print(wchill);
    client.print("/dew/");
    client.print(dew);
    client.print("/heat/");
    client.print(heat);
    client.print("/hum/");
    client.print(hum);
    client.print("/wspd/");
    client.print(wspd);
    client.print("/wdir/");
    client.print(wdir);
    client.print("/bar/");
    client.print(bar);
    client.print("/rain/");
    client.print(rainfall*10);
    client.print("/uvi/");
    client.print(uvi);
    client.print("/solarrad/");
    client.print(solarrad);
    //-------------------------------------------------------------------------------------
    //end cycle
    client.println("/ HTTP/1.1");
    client.println("Host: api.weathercloud.net");
    client.println("Connection: close");
    client.println();
    Serial.println();
    Serial.println("closing connection");
    delay(5000);
    clicked = 0;
}
