/************ includes  *************/
#include <WiFi.h> 
#include <DHT.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>
#include "user_config.h"

/*** BlueTooth***/
#include "SimpleBLE.h" 

/* OverTheAir */
int OTAport = 8266;


/* Pins */
#define LUXPIN 34

Adafruit_BME280 bme;
SimpleBLE ble;

/* sensor Def */
float luxValue;
int LUX;
float calcLux;
float diffLux = 10;

float diffTemp = 0.2;
float tempValue;

float diffHum = 1;
float humValue;

float diffPres = 100;
float presValue;


WiFiClient espClient;
PubSubClient client(espClient);
char message_buff[100];
const int BUFFER_SIZE = 300;


/* Start Setup */

void setup(){

  Serial.begin(115200);

  pinMode(LUXPIN, INPUT);

  Serial.begin(115200);
  delay(10);

  ArduinoOTA.setPort(OTAport);

  ArduinoOTA.setHostname(SENSORNAME);

  

  Serial.println("Starting Node named " + String(SENSORNAME));

  setup_wifi();


  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 Sensor, check wiring!"); 
    while (1);
  }

  client.setServer(mqtt_server, mqtt_port);
 

  ArduinoOTA.onStart([]() {
    Serial.println("Starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total){
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error ==  OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IPess: ");
  Serial.println(WiFi.localIP());
  reconnect();

  ble.begin("ESP32 Sensor1");
  
}

/* Wifi */
void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


/* Send State */

void sendState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();

  root["lux"] = (String)LUX;
  root["temperature"] = (String)tempValue;
  root["humidity"] = (String)humValue;
  root["pressure"] = (String)presValue;

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  client.publish(data_state_topic, buffer, true);
  
}


/* Reconnect */
void reconnect() {
  // Loop until reconnected
  while(!client.connected()){
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(SENSORNAME, mqtt_user, mqtt_password)) {
      Serial.println("Connected");
      client.subscribe(data_set_topic);
    }else{
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.print(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
     }
  }
}

/* Start Check Sensor */
bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}
/* Start main loop */
void loop() {
  ArduinoOTA.handle();

  if(!client.connected()){
    //reconnect();
    software_Reset();
  }

  float newLuxValue = analogRead(LUXPIN);
  float newTempValue = bme.readTemperature();
  float newHumValue = bme.readHumidity();
  float newPresValue = bme.readPressure();
  
  client.loop();

 if (checkBoundSensor(newTempValue, tempValue, diffTemp)){ 
  tempValue = newTempValue;
  sendState();
 }

 if (checkBoundSensor(newHumValue, humValue, diffHum)){
  humValue = newHumValue;
  sendState();
 }

 if (checkBoundSensor(newPresValue, presValue, diffPres)){
  presValue = newPresValue;
  sendState();
 }

 if (checkBoundSensor(newLuxValue, LUX, diffLux)){
  LUX = newLuxValue;
  sendState();
 }
 

 
}

/****reset***/
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
Serial.print("resetting");
esp_restart(); 
}

