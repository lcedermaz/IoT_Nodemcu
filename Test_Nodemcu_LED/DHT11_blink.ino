// Example testing sketch for various DHT humidity/temperature sensors
// Written by ladyada, public domain

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"
#include <ESP8266WiFi.h>

#define DHTPIN 4     // (2) Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.


#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);


//--------------------LED indicador de Grabacion

//#define LED_OK 8    // Funciona solo con el 8, 2 y 3 no responde
int ledState = LOW ; // Estado inicial para el parpadeo del led indicador


//--------------------Tiempos (LED indicador de Grabacion)
unsigned long previousMillis_3 = 0;
const long interval_3 = 100 ;


// Wi-Fi Settings
const char* ssid = "Fibertel WiFi493 2.4GHz"; // your wireless network name (SSID)
const char* password = "0068357885"; // your Wi-Fi network password
WiFiClient client;

// ThingSpeak Settings
const int channelID = 544264;
String apiKey = "NUBYH45AO92Q6I50"; // write API key for your ThingSpeak Channel
const char* server = "api.thingspeak.com";
const int postingInterval = 5 * 1000; // post data every 20 seconds


void setup() {
  Serial.begin(9600);
  Serial.println(F("DHTxx test!"));
  dht.begin();

 //--Estado de indicacion al servidor
   Serial.println("Connecting to ");
   Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");

  //--Configuramos LED indicador
  pinMode(LED_BUILTIN, OUTPUT) ;
}

void loop() {

  Datos_OK (); // Blink para ativar cuando se realiza la conexión del equipo (falta configurar tema tiempo)
  
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

  float hif = dht.computeHeatIndex(f, h);
  float hic = dht.computeHeatIndex(t, h, false);

    
  // Chequea el sensor si está bien
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
   if (client.connect(server, 80)) {

    //--------Canales habilitados para Thingspeak------//   
   
    // Construct API request body (Temp)
      String postStr = apiKey;
      postStr +="&field1=";
      postStr += String(t);
      postStr +="&field2=";
      postStr += String(h);
      postStr += "\r\n\r\n";

      client.print("POST /update HTTP/1.1\n");
      client.print("Host: api.thingspeak.com\n");
      client.print("Connection: close\n");
      client.print("X-THINGSPEAKAPIKEY: "+apiKey+"\n");
      client.print("Content-Type: application/x-www-form-urlencoded\n");
      client.print("Content-Length: ");
      client.print(postStr.length());
      client.print("\n\n");
      client.print(postStr);

        Serial.print(F("HR: "));
        Serial.print(h);
        Serial.print(F("%  Temp: "));
        Serial.print(t);
        Serial.print(F("°C "));
        Serial.print(f);
        Serial.print(F("°F  Heat index: "));
        Serial.print(hic);
        Serial.print(F("°C "));
        Serial.print(hif);
        Serial.println(F("°F"));      
               }
      client.stop();
  // wait and then post again
  delay(postingInterval); 
}


void Datos_OK () {

  unsigned long currentMillis_3 = millis();

  if (currentMillis_3 - previousMillis_3 >= interval_3) {
    previousMillis_3 = currentMillis_3;

    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(LED_BUILTIN, ledState);
  }
}
 
 
