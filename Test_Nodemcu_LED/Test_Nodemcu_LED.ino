/*#define LED_BUILTIN 2    //
int ledState = LOW ; // Estado inicial para el parpadeo del led indicador de grabación

unsigned long previousMillis = 0;
const long interval = 500 ;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  Toggle_LED ();

}

void Toggle_LED () {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(LED_BUILTIN, ledState);
  }
}*/
