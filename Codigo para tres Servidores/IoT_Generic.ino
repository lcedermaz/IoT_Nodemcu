/*
 *  Personnal Weather Station sketch for Wunderground.com  Thingspeak.com and Weathercloud.net
 *  
 *  Main board: NodeMCU ESP12
 *  
 *  Sensors:  
 *  Temperature -DS18B20, 
 *  Humidity and preasure - BME280,
 *  Anenometer and wind direction - Davis 6410
 *  Rain gauge - Ventus W174
 *  
 *  Last edit: 2020 02 01
 *  
 *  https://www.instructables.com/id/NodeMCU-Wireless-Weather-Station/
 *  https://hackaday.io/project/166172-iot-personal-nodemcu-esp12-wifi-weather-station
 */
 
#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino/archive/master.zip
#include <WiFiUdp.h>
#include <ESP8266WiFiMulti.h>
#include <DallasTemperature.h> // https://github.com/milesburton/Arduino-Temperature-Control-Library
#include "cactus_io_BME280_I2C.h" // static.cactus.io/downloads/library/bme280/cactus_io_BME280_I2C.zip
#include <SPI.h>
#include <Wire.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>  //https://github.com/jandrassy/ArduinoOTA

ESP8266WiFiMulti wifiMulti;


//-------------------------------BME280----------------------------------------
BME280_I2C bme(0x76);  // I2C using address 0x76


//------------------------------DS18B20---------------------------------------
#define ONE_WIRE_BUS D4              // DS18B20 sensor data wire is plugged on the ESP-12F  GPIO 2
OneWire oneWire(ONE_WIRE_BUS);       // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature DS18B20(&oneWire); // Pass our oneWire reference to Dallas Temperature.


//------------------------------Rain------------------------------------------------
int RainSensorPin = D6;              //Rain REED-ILS sensor GPIO 12
#define Bucket_Size_US 0.019         // rain bucket size inches 
#define Bucket_Size_EU 0.5           // rain bucket size milimetres ( 0.5mm)

int last_localhour;
volatile unsigned long tipCount1h;   // bucket tip counter used in interrupt routine
volatile unsigned long tipCount24h;  // bucket tip counter used in interrupt routine  
unsigned int counter = 16;
volatile unsigned long contactTime;


//--------------------------------Wind  speed----------------------------------------
int WindSensorPin = D5;               // Wind speed -ILS sensor (anemometer) GPIO 14
volatile unsigned long Rotations;     // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime;  // Timer to avoid contact bounce in interrupt routine


//-------------------------------Wind direction-----------------------------------------------
//int WindVanePin = A0;   // The pin the wind vane sensor is connected to A0 (wind direction)

float wind_avg; // average wind direction
int vane_value;// raw analog value from wind vane
int Direction;// translated 0 - 360 direction
int CalDirection;// converted value with offset applied
#define Offset 0;
int windDirections[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };

//--------------------------------Setup Wifi----------------------------------------------
const char* ssid1     = "xxxxxxxxx";   //I use multiple SSID In my home, but you can only define one.
const char* password1 = "xxxxxxx";
const char* ssid2     = "xxxxxxxxxx";
const char* password2 = "xxxxxxxxx";
const char* ssid3     = "xxxxxxxxxx";
const char* password3 = "xxxxxxxxx";

const int sleepTimeS = 300; //18000 for Half hour, 300 for 5 minutes etc.


//--------------------------------Time  settings ------------------------------------------
int UTC_OFFSET = 3 ;   // (GMT +3 )  
volatile unsigned long OFFSET_S;   // offset in seconds


//--------------------------------PWS  settings ------------------------------------------
float altitudepws = 28.00;  //Local  altitude of the PWS to get relative pressure  meters (m)


//----------------------------Wunderground.com  ID PSW ----------------------------------
char server [] = "rtupdate.wunderground.com";
char WEBPAGE [] = "GET /weatherstation/updateweatherstation.php?";
char ID [] = "IKERTS15";
char Key [] = "ewbiuy8g";


//----------------------------Weathercloud.net  ID Key ----------------------------------
char server2 [] = "http://api.weathercloud.net";

char ID2 [] = "b5839e2508564738";
char Key2 [] = "e481b3d042b910e92ghgfrt200";

//----------------------------Thingspeak.com  API Key ----------------------------------
const char* server3 = "api.thingspeak.com";
String apiKey ="KRBFTRETYNOTAI";


//-----------------------------------NTP VAR---------------------------------------------
unsigned int localPort = 2390;
IPAddress timeServerIP;

// const char* ntpServerName = "time.nist.gov";
//const char* ntpServerName = "time.google.com";
const char* ntpServerName = "pool.ntp.org";
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[ NTP_PACKET_SIZE];
WiFiUDP udp;
unsigned long epoch;

//--------------------------------WEATHER VAR---------------------------------------------
float tempc;             // Temp celsius DS18B20
float tempc_min= 100;    // Minimum temperature C
float temp2c;            // Temp celsius  BMP280
float humidity;          // Humidity BMP280
float temp_f;            // Temp farenheit BMP280
float dewpt_f;           // dew point (F)
float dewpt_c;           // dew point (C)
float heat_f;            // heat index (F)
float heat_c;            // heat index (C)
float windSpeed = 0;     // Wind speed (mph)
float wind_speed_min = 100; // Minimum wind speed (mph)
float wind_speed_avg;    // 10 minutes average wind speed ( mph)
float windgustmph = 0;   // Wind gust speed( mph)
float windmax = 0;
float barompa;           // Preasure pascal Pa
float last_baromhpa;     // previous preasure value
float baromin;           // Preasure inches of mercury InHg
float baromhpa;          // Preasure hectopascal hPa
float rain1h = 0;        // Rain inches over the past hour
float rain = 0;          // Rain milimetres over the past hour
float rain24h = 0;       // Rain inches over the past 24 hours
float rainrate = 0;      // Rain milimetres over the past 24 hours
float chill_f;           // Windchill (F)
float chill_c;           // Windchill (C)
int dBm;                 // WiFi signal strenght dBm
int quality;             // WiFi signal quality %


//-------------------------------- Heat Index ---------------------------------------------
#define c1 (-42.379)
#define c2 (2.04901523)
#define c3 (10.14333127)
#define c4 (-0.22475541)
#define c5 (-0.00683783)
#define c6 (-0.05481717)
#define c7 (0.00122874)
#define c8 (0.00085282)
#define c9 (-0.00000199)

bool debug = 1;           //debug = 1 -> enable debug

//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////MAIN PROGRAM START/////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void setup()
{
  Rotations = 0;   // Set Rotations to 0 ready for calculations
  tipCount1h = 0;
  tipCount24h = 0;
  wind_speed_min=100;
  
 
  Serial.begin(115200);
  delay(7000);
  WiFi.hostname("Weather_station_Davis");
  
  Serial.print("Start NodeMCU Weather Station ");
  Serial.println(ID);
  Serial.println();
  OFFSET_S = UTC_OFFSET * -3600;  //Offset in seconds  
  DS18B20.begin();
  delay(2000);  
  Wire.begin(D2,D1); 

 if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  bme.setTempCal(-1);

  startwifi();

  ArduinoOTA.setHostname("NodeMCU_weather_station");

  pinMode(RainSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(RainSensorPin), isr_rg, FALLING);  
  pinMode(WindSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);  // change if low to high >> RISING
  sei();
  
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  
}


void loop()
{
 read_data();             // read different sensors data from analog and digital pins of ESP8266
 getRain();
 getWindSpeed();
 isr_rotation();
 getWindDirection();
 RSSIdBm();               // WiFi signal quality (RSSI)
 print_data();            // print data  in serial monitor
 wunderground();          // sends data to wunderground.com
 weathercloud();          // sends data to weathercloud.net
 thingspeak();            // sends data to thingspeak.com
 }

//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////READ THE DATA FROM SENSORS/////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
 void read_data(void)
 {
     delay(50);
     DS18B20.requestTemperatures(); // Send the command to get temperatures from DS18B20 sensor
     delay(500);
     tempc = DS18B20.getTempCByIndex(0);
     tempc = (tempc - 1);
    if (tempc_min > tempc ) {
     tempc_min = tempc;
}
     bme.readSensor();
     delay(500); 
     


     baromhpa = bme.getPressure_MB()+(altitudepws * 0.1);
     
     if (baromhpa < 950 || baromhpa > 1050 ) {
     baromhpa = last_baromhpa;
        }
     last_baromhpa = baromhpa;
     baromin = (baromhpa)/ 33.86;

     
     temp2c = bme.getTemperature_C();
     humidity = bme.getHumidity();
     
     temp_f =  (tempc_min * 9.0)/ 5.0 + 32.0;
  
     dewpt_c = (dewPoint(tempc_min, humidity));  //dew point
     dewpt_f = (dewpt_c * 9.0)/ 5.0 + 32.0; //  converting dew point to F


     windchill();
     heatindex();
     Counter();
  }
//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////Get wind speed  /////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void getWindSpeed(void)
  {
 
     Rotations = 0; // Set Rotations count to 0 ready for calculations
     //sei(); // Enables interrupts
     ArduinoOTA.handle();
     delay (5000); // Wait 5 seconds to average wind speed
     ArduinoOTA.handle();
     delay (5000); // Wait 5 sedconds to average wind speed
     ArduinoOTA.handle();
     delay (5000); // Wait 5 seconds to average wind speed
     ArduinoOTA.handle();
     delay (5000); // Wait 5 seconds to average wind speed
     ArduinoOTA.handle();
     delay (5000); // Wait 5 seconds to average wind speed
     ArduinoOTA.handle();
     delay (5000); // Wait 5 seconds to average wind speed
     //cli(); // Disable interrupts
     
     /* convert to mp/h using the formula V=P(2.25/T)
      V = P(2.25/30) = P * 0.075       V - speed in mph,  P - pulses per sample period, T - sample period in seconds */
     windSpeed = Rotations * 0.15; // 30 seconds
     Rotations = 0;   // Reset count for next sample
     
  if (windSpeed > windgustmph) {
     windgustmph = windSpeed;
}
 if (wind_speed_min > windSpeed ) {
     wind_speed_min = windSpeed;
}

 wind_speed_avg = (windgustmph + wind_speed_min) * 0.5;   // average wind speed mph per 10 minutes
 
}

// This is the function that the interrupt calls to increment the rotation count
//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////ISR rotation//////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void isr_rotation(void)   
{
  if ((millis() - ContactBounceTime) > 30 ) {  // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
}
// Convert MPH to Knots
float getKnots(float speed) {
   return speed * 0.868976;          //knots 0.868976;
}
// Convert MPH to m/s
float getms(float speed) {
   return speed * 0.44704;           //metric m/s 0.44704;;
}

// Get Wind Direction
//-------------------------------------------------------------------------------------------------------------
/////////////////////////////////// Wind direction ////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void getWindDirection(void) 
{
vane_value = analogRead(A0);
Direction = map(vane_value, 0, 1023, 0, 360);
CalDirection = Direction + Offset;

if(CalDirection > 360)
CalDirection = CalDirection - 360;

if(CalDirection < 0)
CalDirection = CalDirection + 360;

 DirectionCheck();

//getHeading(CalDirection);
}

void DirectionCheck()
{

  if(vane_value >= 0 && vane_value < 32)
   wind_avg = 0;
  else if(vane_value >= 32 && vane_value < 96)
  wind_avg = 22.5;
  else if(vane_value >= 96 && vane_value < 160)
  wind_avg = 45;
  else if(vane_value >= 160 && vane_value < 224)
  wind_avg = 67.5;
  else if(vane_value >= 224 && vane_value < 288)
  wind_avg = 90;
  else if(vane_value >= 288 && vane_value < 352)
  wind_avg = 112.5;
  else if(vane_value >= 352 && vane_value < 416)
  wind_avg = 135;
  else if(vane_value >= 416 && vane_value < 480)
  wind_avg = 157.5;
  else if(vane_value >= 480 && vane_value < 544)
  wind_avg = 180;
  else if(vane_value >= 544 && vane_value < 608)
  wind_avg = 202.5;
  else if(vane_value >= 608 && vane_value < 672)
  wind_avg = 225;
  else if(vane_value >= 672 && vane_value < 736)
  wind_avg = 247.5;
  else if(vane_value >= 736 && vane_value < 800)
  wind_avg = 270;
  else if(vane_value >= 800 && vane_value < 864)
  wind_avg = 292.5;
  else if(vane_value >= 864 && vane_value < 928)
  wind_avg = 315;
  else if(vane_value >= 928 && vane_value < 992)
  wind_avg = 337.5;
  else if(vane_value >= 992 && vane_value < 1025)
  wind_avg = 0;
  
  
  DirectionAvg();

}

void DirectionAvg() 
{
  int index = wind_avg / 22.5;
  if ((index < 0) or (index > 15))
  {exit;}
  windDirections[index]++;
}


int getWindDirectionMax() 
{
  int max = windDirections[0]; 
  int index = 0;
  for (int i = 1; i < 16; i++) 
  {
    if (max < windDirections[i])
    {
      max = windDirections[i]; // find max value
      index = i;
    }
  }
  return index * 22.5; //  return average wind direction 
}


// Converts compass direction to heading
void getHeading(int direction) {
if(direction < 22.5)
Serial.print("N ");
else if (direction < 67.5)
Serial.print("NE ");
else if (direction < 112.5)
Serial.print("E ");
else if (direction < 157.5)
Serial.print("SE ");
else if (direction < 202.5)
Serial.print("S ");
else if (direction < 247.5)
Serial.print("SW ");
else if (direction < 292.5)
Serial.print("W ");
else if (direction < 337.5)
Serial.print("NW ");
else
Serial.print("N ");
} 

// converts wind speed to wind strength
void getWindStrength(float speed)
{
  if(speed < 1)
    Serial.println("Calm");
  else if(speed >= 1 && speed < 3)
    Serial.println("Light Air");
  else if(speed >= 3 && speed < 7)
    Serial.println("Light Breeze");
  else if(speed >= 7 && speed < 12)
    Serial.println("Gentle Breeze");
  else if(speed >= 12 && speed < 18)
    Serial.println("Moderate Breeze");
  else if(speed >= 18 && speed < 24)
    Serial.println("Fresh Breeze");
  else if(speed >= 24 && speed < 31)
    Serial.println("Strong Breeze");
  else if(speed >= 31 && speed < 38)
    Serial.println("High wind");
  else if(speed >= 38 && speed < 46)
    Serial.println("Fresh Gale");
  else if(speed >= 46 && speed < 54)
    Serial.println("Strong Gale");
  else if(speed >= 54 && speed < 63)
    Serial.println("Storm");
  else if(speed >= 63 && speed < 72)
    Serial.println("Violent storm"); 
  else if(speed >= 72 && speed)
    Serial.println("Hurricane");    
}
//------------------------------------------------------------------------------------------------------------
///////////////////////////////// Get Rain data //////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
  void getRain(void)
   {
      cli();         //Disable interrupts
      
      rainrate = tipCount1h * Bucket_Size_EU;   
      rain1h = tipCount1h * Bucket_Size_US;    // 0.3/25.4 = 0.012 bucket size inches
      rain = tipCount24h * Bucket_Size_EU;
      rain24h = tipCount24h * Bucket_Size_US;  // 0.3/25.4 = 0.012 bucket size inches


  if (last_localhour != localhour()) 
{
  tipCount1h = 0; //reset  Rain counter each 1h
  for (int i = 0; i < sizeof(windDirections) / sizeof(windDirections[0]); i++)
{
  windDirections[i] = 0;  //reset  Wind direction average  each 1h
}

  last_localhour = localhour();
}

     if ((localhour() >= 23) && (localmin() >= 58))    //reset Daily Rain each 24
     {
      Serial.println("Reset Daily Rain");
    tipCount24h = 0;
    ESP.restart();
     }
   sei();         //Enables interrupts
   ntptime();   
}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////////// RAIN Interrupt ///////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------

// Interrrupt handler routine that is triggered when the W174 detects rain   

void isr_rg() {

  if((millis() - contactTime) > 500 ) { // debounce of sensor signal 
    tipCount1h++;
    tipCount24h++;
   
    contactTime = millis(); 
  } 
} 

//------------------------------------------------------------------------------------------------------------
////////////////////////////////////////////////// RSSI dBm //////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------

void RSSIdBm() {

  
while (wifiMulti.run() != WL_CONNECTED) {
  delay(1000);
  Serial.print(".");
  }
  dBm = WiFi.RSSI();
  quality = 2 * (dBm + 100);
  if (dBm >= -50)
  quality = 100;
 
}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////PRINT DATA IN SERIAL MONITOR/////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
  void print_data(void) 
 {
   if(debug)
   {
    Serial.println(" ");

    Serial.print("DS18B20 = "); Serial.print(tempc_min); Serial.println(" C  ");

    Serial.print(F("BME280 = ")); Serial.print(temp2c); Serial.print(" C  "); Serial.print(temp_f); Serial.print(" F  "); Serial.print(humidity); Serial.print(" %  ");
    Serial.print(baromhpa);Serial.print(" hPa  "); Serial.print(baromin); Serial.println(" inHg  ");
    //Serial.print(F("BME280 Approx altitude = "));
    //Serial.print(bme.readAltitude(1015)); // this should be adjusted to your local forcase http://www.mide.com/pages/air-pressure-at-altitude-calculator
    //Serial.println(" m");

    Serial.print("Wind_Strength = "); getWindStrength(windSpeed);
    Serial.print("Wind_Speed = "); Serial.print(windSpeed);Serial.print(" mph  "); Serial.print(getKnots(windSpeed));Serial.print(" knots  ");
    Serial.print(getms(windSpeed));Serial.println(" m/s  ");

    
    Serial.print("Min = ");  //Minimum wind speed
    Serial.print(wind_speed_min / 2.236 );Serial.print(" m/s ");

    Serial.print("Avg = "); //Average wind speed
    Serial.print(wind_speed_avg  / 2.236);Serial.print(" m/s ");

    Serial.print("Gust = "); //Maximum wind gust speed
    Serial.print(windgustmph  / 2.236);Serial.println(" m/s ");
    
    Serial.print("Wind_Direction = ");
    getHeading(CalDirection); Serial.println(CalDirection);
    Serial.print("Wind Avg = "); Serial.println(getWindDirectionMax());     
    Serial.print("Vane Value = "); Serial.println(vane_value);

    Serial.print("Rain_Tip_Count = "); Serial.println(tipCount24h);
    Serial.print("Precip_Rate = ");
    Serial.print(rain1h); Serial.print(" in  ");   Serial.print(" Precip_Accum_Total: "); Serial.print(rain24h); Serial.println(" in  "); 
    Serial.print("Rain = ");Serial.print(rain); Serial.print(" mm  ");  Serial.print("Rain rate = "); Serial.print(rainrate); Serial.println(" mm"); 

   Serial.print("Dew point = "); Serial.print(dewpt_c);Serial.print(" C "); Serial.print(dewpt_f);Serial.println(" F ");  
   Serial.print("Heat index = "); Serial.print(heat_c);Serial.println(" C "); 
   Serial.print("Wind chill = "); Serial.print(chill_c);Serial.println(" C "); 
   Serial.println(" ");
   Serial.print("Signal quality  = ");Serial.println(quality);
   Serial.print("RSSI  = ");Serial.print(dBm);Serial.println("dBm ");
   Serial.println(" ");
   
   Serial.print("Counter = ");Serial.println(counter);
   Serial.println(" ");
 }    
 }

//------------------------------------------------------------------------------------------------------------
///////////////////////////////// SEND DATA TO Wunderground.com //////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void wunderground(void)
{
  Serial.print("Connecting to ");
  Serial.println(server);
  WiFiClient client;
 if (client.connect(server, 80)) {
    Serial.print("connected to ");
    Serial.println(client.remoteIP());
    delay(100);
  } else {
    Serial.println("connection failed");
  }
  client.print(WEBPAGE);
  client.print("ID=");
  client.print(ID);
  client.print("&PASSWORD=");
  client.print(Key);
  client.print("&dateutc=now&winddir=");
  client.print(CalDirection);
  client.print("&tempf=");
  client.print(temp_f);
  client.print("&windspeedmph=");
  client.print(wind_speed_avg);    
  client.print("&windgustmph=");
  client.print(windgustmph);
  client.print("&dewptf=");
  client.print(dewpt_f);
  client.print("&humidity=");
  client.print(humidity);
  client.print("&baromin=");
  client.print(baromin);
  client.print("&rainin=");
  client.print(rain1h);
  client.print("&dailyrainin=");
  client.print(rain24h);
  client.print("&softwaretype=NodeMCU-ESP12&action=updateraw&realtime=1&rtfreq=30");
  client.print("/ HTTP/1.1\r\nHost: rtupdate.wunderground.com:80\r\nConnection: close\r\n\r\n");
  Serial.println(" ");
  delay(1000);
  
  //sleepMode();
}
//------------------------------------------------------------------------------------------------------------
/////////////////////////////////  SEND DATA TO  Weathercloud.net ////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------

void weathercloud(void)
{
if (counter == 17){
{
  Serial.print("connecting to ");
  Serial.print(server2);
  Serial.println("...");
  WiFiClient client;
  if (client.connect(server2, 80)) {
    Serial.print("connected to ");
    Serial.println(client.remoteIP());
  } else {
    Serial.println("connection failed");
  }
  client.print("GET /set/wid/");
  client.print(ID2);
  client.print("/key/");
  client.print(Key2);
  client.print("/temp/");
  client.print(tempc_min*10);
  client.print("/chill/");
  client.print(chill_c*10);
  client.print("/dew/");
  client.print(dewpt_c*10);
  client.print("/heat/");
  client.print(heat_c*10);
  client.print("/hum/");
  client.print(humidity);
  client.print("/wspd/");
  client.print(windSpeed*4.47);
  client.print("/wspdavg/");
  client.print(wind_speed_avg*4.47);
  client.print("/wspdhi/");
  client.print(windgustmph*4.47);
  client.print("/wdir/");
  client.print(CalDirection);
  client.print("/wdiravg/");
  client.print(getWindDirectionMax());  
  client.print("/bar/");
  client.print(baromhpa*10); 
  client.print("/rain/");
  client.print(rain*10);
  client.print("/rainrate/");
  client.print(rainrate*10);
  client.print("/tempin/");
  client.print(tempc_min*10);
  client.print("/dewin/");
  client.print(dewpt_c*10);
  client.print("/heatin/");
  client.print(heat_c*10);
  client.print("/humin/");
  client.print(humidity);
  //client.print("/uvi/");
  //client.print(uvi);
  client.println("/ HTTP/1.1");
  client.println("Host: api.weathercloud.net");
  client.println();
  delay(1000);
 
  if (!client.connected()) {
    Serial.println("client disconected.");
      if (client.connect(server2, 80)) {
    delay(100);
    Serial.print("connected to ");
    Serial.println(client.remoteIP());

  } else {
    Serial.println("connection failed");
  }
  }
}
}
 delay(100);
}


//------------------------------------------------------------------------------------------------------------
/////////////////////////////////  SEND DATA TO  Thingspeak.com ////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------

void thingspeak(void)
{
if (counter == 17){
{  
  Serial.print("connecting to ");
  Serial.print(server2);
  Serial.println("...");
  WiFiClient client;
  if (client.connect(server3, 80)) { // use ip 184.106.153.149 or api.thingspeak.com
  Serial.print("connected to ");
  Serial.println(client.remoteIP());
   
   String postStr = apiKey;
   postStr += "&field1=";
   postStr += String(tempc_min);
   postStr += "&field2=";
   postStr += String(humidity);
   postStr += "&field3=";
   postStr += String(baromhpa);
   postStr += "&field4=";
   postStr += String(wind_speed_avg * 0.447);
   postStr += "&field5=";
   postStr += String(windgustmph * 0.447);
   postStr += "&field6=";
   postStr += String(rain);
   postStr += "&field7=";
   postStr += String(getWindDirectionMax());
   postStr += "&field8=";
   postStr += String(quality);
   postStr += "\r\n\r\n";
   
   client.print("POST /update HTTP/1.1\n");
   client.print("Host: api.thingspeak.com\n");
   client.print("Connection: close\n");
   client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
   client.print("Content-Type: application/x-www-form-urlencoded\n");
   client.print("Content-Length: ");
   client.print(postStr.length());
   client.print("\n\n");
   client.print(postStr);
   delay(1000);
   
   }
 client.stop();
}
}
}


//------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////// Counter /////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void Counter()
{
if (counter == 18)
{
windgustmph = 0;
wind_speed_avg = 0;
wind_speed_min = 100;
tempc_min = 100;
counter = 0;  //10 minutes loop     30*20=600s = 10min
}
counter++;
}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////// Calculate Heat Index  ///////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void heatindex()
{
if (temp_f > 80.0)   //The heat index only works for temperatures  above 80°F 
{
  heat_f = c1+c2*(temp_f)+c3*(humidity)+c4*(temp_f)*(humidity)+c5*(pow(temp_f,2))+c6*(pow(humidity,2))+c7*(pow(temp_f, 2))*(humidity)+c8*(temp_f)*(pow(humidity, 2))+c9*(pow(temp_f, 2))*(pow(humidity, 2));
  heat_c = (heat_f - 32)*5/9;  //  converting to C
  }
  else
{
  heat_c = tempc_min;
}
}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////// Calculate Windchill  ////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void windchill()
{
if ((temp_f <50.0) && (windgustmph > 3.0)) //The wind chill only works for temperatures  below 50°F and wind speed above 3 mph. (10°C , 1 m/s)

{
 chill_f =35.74+0.6215*temp_f-35.75*pow(wind_speed_avg,0.16)+0.4275*temp_f*pow(wind_speed_avg,0.16);
 chill_c = (chill_f - 32)*5/9;  //  converting to C
}

else
{
  chill_c = tempc_min;
}
}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////// Calculate dew Point C ////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
double dewPoint(double tempc_min, double humidity)
{
        double RATIO = 373.15 / (273.15 + tempc_min);  // RATIO was originally named A0, possibly confusing in Arduino context
        double SUM = -7.90298 * (RATIO - 1);
        SUM += 5.02808 * log10(RATIO);
        SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
        SUM += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
        SUM += log10(1013.246);
        double VP = pow(10, SUM - 3) * humidity;
        double T = log(VP/0.61078);   // temp var
        return (241.88 * T) / (17.558 - T);
}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////////////////// ESP8266 sleep mode ///////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void sleepMode() {
  Serial.print(F("Sleeping..."));
  ESP.deepSleep(sleepTimeS * 1000000);
}

//------------------------------------------------------------------------------------------------------------
////////////////////////////////////////////////// WIFI SETUP ////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void startwifi()
{
  Serial.print("Connecting to Wifi ");
  delay(1000);

  wifiMulti.addAP(ssid1, password1);        //if you have less SSID, delete the others
  wifiMulti.addAP(ssid2, password2);
  wifiMulti.addAP(ssid3, password3);

  while (wifiMulti.run() != WL_CONNECTED) {
  delay(1000);
  Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(500);
  startudp();
}
//------------------------------------------------------------------------------------------------------------
////////////////////////////////////////////////// UDP NTP////////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void startudp()
{
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
}

//------------------------------------------------------------------------------------------------------------
////////////////////////////////////////////////// NTP request ///////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
unsigned long sendNTPpacket(IPAddress& address)
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

//------------------------------------------------------------------------------------------------------------
////////////////////////////////////////////////// NTP Time //////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
unsigned long ntptime()
{
  WiFi.hostByName(ntpServerName, timeServerIP); 
  sendNTPpacket(timeServerIP);
  delay(1000);
  
  int cb = udp.parsePacket();
  if (!cb) {
    Serial.println("no NTP packet yet");
  }
  else {
    Serial.print("NTP packet received, length=");
    Serial.println(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    // now convert NTP time into everyday time:
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    epoch = secsSince1900 - seventyYears;

    if (debug) {
    
      Serial.print("Seconds since Jan 1 1900 = " );
      Serial.println(secsSince1900);
  
      // print Unix time:
      Serial.print("Unix time = ");
      Serial.println(epoch);
  
  
      // print the hour, minute and second:
      Serial.print("The local time UTC "); 
      Serial.println(UTC_OFFSET);  //  Offset In seconds  https://www.epochconverter.com/timezones?q=
      Serial.print(((epoch-( OFFSET_S ))  % 86400L) / 3600); // print the hour (86400 equals secs per day)
      Serial.print(':');
      if ( (((epoch-(OFFSET_S * 7)) % 3600) / 60) < 10 ) {
        // In the first 10 minutes of each hour, we'll want a leading '0'
        Serial.print('0');
      }
      Serial.print(((epoch-(OFFSET_S ))  % 3600) / 60); // print the minute (3600 equals secs per hour)
      Serial.print(':');
      if ( ((epoch-(OFFSET_S )) % 60) < 10 ) {
        // In the first 10 seconds of each minute, we'll want a leading '0'
        Serial.print('0');
      }
      Serial.println((epoch-(OFFSET_S )) % 60); // print the second
    }
  }
}

int localhour()
{
  return (((epoch-(OFFSET_S ))  % 86400L) / 3600);
}

int localmin()  
{
  return (((epoch-(OFFSET_S  ))  % 3600) / 60);
}

int localsec()
{
  return ((epoch-(OFFSET_S )) % 60);
}
