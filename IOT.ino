#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <stdlib.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> // by Benoit Blanchon
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <Wire.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include "net_misc.h"
#include "DHT.h"
#include <Preferences.h>
#define DHTTYPE DHT11
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
#define ONBOARD_LED  2

Preferences preferences;
const int buttonPin = 0;
bool isEmit = false;
bool configured = false;
String header;
const char* SSID = "ssid";
const char* MDP = "mdp";
const char* USERID = "userid";
const char* FREQ = "freq";
String userID;
unsigned int freq = 10;
AsyncWebServer server(80);
struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};


Button button1 = {0, 0, false};
void IRAM_ATTR isr() {
  button1.numberKeyPresses += 1;
  button1.pressed = true;
}

// variables will change:
int buttonState = 0;

/*============= GPIO ======================*/
Adafruit_BMP280 bmp; // I2C
const int ledPin = 19; // LED Pin
const int photo_resistor_pin = A5;
// DHT Sensor
uint8_t DHTPin = 4;
DHT dht(DHTPin, DHTTYPE);
const char* mqttUser = "iot";
const char* mqttPassword = "salutcestleprojetiot";
float Temperature;
float Humidity;


OneWire oneWire(23);
DallasTemperature tempSensor(&oneWire);

WiFiClient espClient; // Wifi
PubSubClient client(espClient) ; // MQTT client

String whoami; // Identification de CET ESP au sein de la flotte

//StaticJsonBuffer<200> jsonBuffer;
/*===== MQTT broker/server and TOPICS ========*/
const char* mqtt_server = "152.228.216.110";
#define TOPIC "sensors/IOTMIAGE/datas"
#define TOPIC_LED "sensors/aaa/led"
/*=============== SETUP =====================*/
void setup () {
  // Serial
  Serial.begin (9600);
  pinMode(ONBOARD_LED,OUTPUT);

  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
    "try a different address!"));
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


    // Gpio
    pinMode(button1.PIN, INPUT_PULLUP);
    pinMode (ledPin , OUTPUT);
    attachInterrupt(button1.PIN, isr, FALLING);
    pinMode(DHTPin, INPUT);
    /* Wifi */
    dht.begin();
    connect_wifi();

    /*  L'ESP est un client du mqtt_server */
    client.setServer(mqtt_server, 8883);
    //client.setServer(mqtt_server, 8883); // securite !!!

    // set callback when publishes arrive for the subscribed topic
    // methode a effet local => on n'a pas a initier/verifier la connection.
    client.setCallback(mqtt_pubcallback);

    /* Choix d'une identification pour cet ESP ---*/
    // whoami = "esp1";
    whoami =  String(WiFi.macAddress());
  }

  /*============== MQTT CALLBACK ===================*/


  void mqtt_pubcallback(char* topic, byte* message, unsigned int length) {
    /*
    *  Callback if a message is published on this topic.
    */

    // Byte list to String ... plus facile a traiter ensuite !
    // Mais sans doute pas optimal en performance => heap ?
    String messageTemp ;
    for(int i = 0 ; i < length ; i++) {
      messageTemp += (char) message[i];
    }

    Serial.print("Message : ");
    Serial.println(messageTemp);
    Serial.print("arrived on topic : ");
    Serial.println(topic) ;

    // Analyse du message et Action
    if(String (topic) == TOPIC_LED) {
      // Par exemple : Changes the LED output state according to the message
      Serial.print("Action : Changing output to ");
      if(messageTemp == "on") {
        Serial.println("on");
        set_pin(ledPin,HIGH);

      } else if (messageTemp == "off") {
        Serial.println("off");
        set_pin(ledPin,LOW);
      }
    }
  }
  /*============= MQTT SUBSCRIBE =====================*/


  void mqtt_mysubscribe(char* topic) {
    /*
    * ESP souscrit a ce topic. Il faut qu'il soit connecte.
    */
   preferences.begin("credentials", false);
  userID = preferences.getString("userid", "");
  freq = preferences.getUInt("freq", 0);
            preferences.end();
      Serial.print(freq);
        Serial.print(userID);
    while(!client.connected() && !isEmit) { // Loop until we are reconnected
      if (button1.pressed) {
        isEmit = true;
        button1.pressed = false;
      }

      Serial.print("Attempting MQTT connection...");
      if(client.connect("esp32", mqttUser, mqttPassword )) { // Attempt to connect
        Serial.println("connected");
        client.subscribe(topic); // and then Subscribe
      } else { // Connection failed
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println("try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(2*1000);
        digitalWrite(ONBOARD_LED,0.5);
        delay(2*1000);
        digitalWrite(ONBOARD_LED,1);
      }
    }
    digitalWrite(ONBOARD_LED,LOW);
  }


  /*============= ACCESSEURS ====================*/

  float get_temperature() {
    /* float temperature;
    tempSensor.requestTemperaturesByIndex(0);

    temperature = tempSensor.getTempCByIndex(0);
    bmp.readPressure()*/
    return bmp.readTemperature();
  }

  float get_pressure() {
    return bmp.readPressure();
  }
  float get_alt() {
    return bmp.readAltitude();
  }

  float get_light(){
    return analogRead(photo_resistor_pin);
  }
  float get_temp(){
    return dht.readTemperature();
  }
  float get_humidity(){
    return dht.readHumidity();
  }


  void set_pin(int pin, int val){
    digitalWrite(pin, val) ;
  }

  int get_pin(int pin){
    return digitalRead(pin);
  }

  /*================= LOOP ======================*/


  const char index_html[] PROGMEM = R"rawliteral(
    <!DOCTYPE HTML><html><head>
    <title>Configuration ESP </title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    </head><body>
    <form action="/get">
    SSID: <input type="text" name="ssid">
    <br>
    Mot de passe: <input type="text" name="mdp">
    <br>
    ID de l'user sur le site web: <input type="text" name="userid">
    <br>
    Frequence d'envoie de l'ESP: <input type="number" name="freq">
    <br>
    <input type="submit" value="Submit">
        </form><br>
    </body></html>

  )rawliteral";


    const char enregistre[] PROGMEM = R"rawliteral(
      <!DOCTYPE HTML><html><head>
      <title>Configuration ESP </title>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      </head><body>
      Vos preferences ont bien ete sauvegardes
      <br>
      <a href="/">Retourner a l'accueil</a>
      <br>
      Appuyez sur le boutton "boot" de votre ESP pour quitter le mode configuration
      </body></html>)rawliteral";

      void loop () {
        if (button1.pressed) {
          isEmit = !isEmit;
          configured = false;
          button1.pressed = false;
        }
        if(!isEmit)
        {
          if (!configured)
          {
            WiFi.mode(WIFI_STA);
            connect_wifi();
            configured = true;
          }
          digitalWrite(ONBOARD_LED,LOW);
          buttonState = digitalRead(buttonPin);
          Serial.println(buttonState);
          char data[250];
          String payload; // Payload : "JSON ready"
          int32_t period = freq * 1000l; // Publication period


          /* Subscribe to TOPIC_LED if not yet ! */
          if (!client.connected()) {
            mqtt_mysubscribe((char*) (TOPIC_LED));
          }


          /* char tempString[8];
          dtostrf(temperature, 1, 2, tempString);
          client.publish(TOPIC_TEMP, tempString); */

          /* Publish Light periodically */
          payload = "{\"id\": \"" + whoami + "\", \"userId\": \"" + userID + "\", \"lumiere\": \"" + get_light() + "\", \"pression\": \"" + get_pressure() + "\", \"altitude\": \"" + get_alt() + "\", \"humidite\": \"" + get_humidity() + "\", \"temperature\": \"" + get_temperature() +"\"" + "}";
          payload.toCharArray(data, (payload.length() + 1));

          Serial.println(data);
          client.publish(TOPIC, data);
          delay(period);
          // Process MQTT ... obligatoire une fois par loop()
        }

        else {
          if (!configured)
          {
            Serial.print("Setting AP (Access Point)…");
            WiFi.softAP("Esp32", NULL);

            IPAddress IP = WiFi.softAPIP();
            Serial.print("AP IP address: ");
            Serial.println(IP);
            // Send web page with input fields to client
            server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
              request->send_P(200, "text/html", index_html);
            });
            // Send a GET request to <ESP_IP>/get?ssid=<inputMessage>
            server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
              String inputMessage;
              int inputMessageFreq;
              String inputParam;
              // GET ssid value on <ESP_IP>/get?ssid=<inputMessage>
              if (request->hasParam(SSID)) {
                inputMessage = request->getParam(SSID)->value();
                if (inputMessage[0] != '\0')
                  {

                inputParam = SSID;

                preferences.begin("credentials", false);
                preferences.putString("ssid", inputMessage);
                Serial.println(inputParam);
                preferences.end();
              }
              }
              // GET mdp value on <ESP_IP>/get?mdp=<inputMessage>
              if (request->hasParam(MDP) ) {
                inputMessage = request->getParam(MDP)->value();
                if (inputMessage[0] != '\0')
                  {
                inputParam = inputParam+  MDP;



                preferences.begin("credentials", false);
                preferences.putString("password", inputMessage);
                Serial.println(inputParam);
                preferences.end();
                }

              }
              if (request->hasParam(USERID)) {
                inputMessage = request->getParam(USERID)->value();
                  if (inputMessage[0] != '\0')
                    {
                inputParam = inputParam+  USERID;



                  preferences.begin("credentials", false);
                preferences.putString("userid", inputMessage);
                Serial.println(inputParam);
                preferences.end();
              }
              }

              if (request->hasParam(FREQ)) {
                inputMessage = request->getParam(FREQ)->value();
                Serial.println(inputMessage);
                int x = atoi(inputMessage.c_str());

                  if (x >0)
                  {
                      printf("Converting : %d\n", x);
                    if (x <10)
                    {
                      x = 10;
                    }

                inputParam = inputParam+  FREQ;
                  preferences.begin("credentials", false);
                preferences.putUInt("freq", x);
                Serial.println(x);
                preferences.end();
              }
              }

              else {
                inputMessage = "No message sent";
                inputParam = "none";
              }
              Serial.println(inputMessage);

              request->send(200, "text/html",enregistre  );

            });



            server.begin();

            configured = true;

          }
          digitalWrite(ONBOARD_LED,HIGH);

        }
        client.loop();

      }
