#include <Adafruit_Sensor.h> //Library for Adafruit sensors , we are using for DHT
#include <DHT_U.h> //DHT library which uses some func from Adafruit Sensor library
#include <ArduinoJson.h> //library for Parsing JSON
#include <PubSubClient.h> //library for MQTT
#include "rpcWiFi.h" //Wi-Fi library 
#include "TFT_eSPI.h" //TFT LCD library 
#include "Free_Fonts.h" //free fonts library 
#include <SoftwareSerial.h>;
#include "DHT.h"
#include <OneWire.h>                
#include <DallasTemperature.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Arduino.h>
#include <Wire.h>
#include "SHT31.h"
#include <DHT_U.h> //DHT library which uses some func from Adafruit Sensor library
#include <Adafruit_Sensor.h> //Library for Adafruit sensors , we are using for DHT


#define pin1 D2
#define pin2 D4
#define LED D3

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long interval = 6000;
unsigned long interval2 = 60000;
unsigned long interval3 = 60000;
unsigned long interval4 = 60000;
unsigned long interval5 = 60000;

unsigned long prevTime_T1 = millis();
unsigned long prevTime_T2 = millis();
unsigned long prevTime_T3 = millis(); 
unsigned long prevTime_T4 = millis(); 
unsigned long prevTime_T5 = millis();

TFT_eSPI tft; //initialize TFT LCD
TFT_eSprite spr = TFT_eSprite(&tft); //initialize sprite buffer

//DHT parameters
DHT dht(pin1, DHT11);

uint32_t delayMS;

//MQTT Credentials
const char* ssid = "Medina";//setting your ap ssid
const char* password = "belgorod25";//setting your ap psk
const char* mqttServer = "35.238.225.243"; //MQTT URL
const char* mqttUserName = "";  // MQTT username
const char* mqttPwd = "";  // MQTT password
const char* clientID = "username0001"; // client id username+0001
const char* topic = "Dar/Mal"; //publish topic

String msgStr = "";      // MQTT message buffer

float temp, hum;


//setting up wifi and mqtt client
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.println("Connecting to WiFi..");
    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.setFreeFont(FMB12);
    tft.setCursor((320 - tft.textWidth("Connecting to Wi-Fi.."))/2, 120);
    tft.print("Connecting to Wi-Fi..");
  Serial.println("Connecting to WiFi..");
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts > 30) {
         NVIC_SystemReset();
      }     
    }
    tft.fillScreen(TFT_BLACK);
    tft.setCursor((320 - tft.textWidth("Connected!"))/2, 120);
    tft.print("Connected..!");
    Serial.println("Connected to the WiFi network");
    Serial.print("IP Address: ");
    Serial.println (WiFi.localIP()); // prints out the device's IP address
  
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(clientID, mqttUserName, mqttPwd)) {
      Serial.println("MQTT connected");
      client.subscribe("api/request");
      Serial.println("Topic Subscribed");
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);  // wait 5sec and retry
    }

  }

}

//subscribe call back
void callback(char*topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.print("Message size :");
  Serial.println(length);
  Serial.println();
  Serial.println("-----------------------");

  StaticJsonDocument<256> doc; //read JSON data
  deserializeJson(doc, payload, length); //deserialise it
  JsonObject command = doc["command"]; //get the values of command parameter

  int command_parameters_led = command["parameters"]["led"]; //get value of led, which will be 1 or 0

  if (command_parameters_led == 1) {
    Serial.println("LED");
    digitalWrite(LED, HIGH);
  }
  else {
    digitalWrite(LED, LOW);
  }
}


void setup() {
  Serial.begin(115200);
  // Initialize device.
  dht.begin();

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  setup_wifi();

  client.setServer(mqttServer, 1883); //setting MQTT server
  client.setCallback(callback); //defining function which will be called when message is recieved.

}

void loop() {
  if (!client.connected()) { //if client is not connected
    reconnect(); //try to reconnect
  }
  client.loop();

  unsigned long currentTime = millis();

 if (currentTime - prevTime_T1 > interval) {
    readDht01();
    prevTime_T1 = currentTime;
    }
    
    

  }

  void readDht01(){
    temp = dht.readTemperature();
    hum = dht.readHumidity();
    Serial.print("Temperatura T0T1: ");
    Serial.print(temp);
    Serial.println(" ºC.");
    Serial.print("Humedad T0H1: ");
    Serial.print(hum);
    Serial.println(" %.");

    msgStr = "{\"action\": \"notification/insert\",\"deviceId\": \"s3s9TFhT9WbDsA0CxlWeAKuZykjcmO6PoxK6\",\"notification\":{\"notification\": \"temperature\",\"parameters\":{\"temp0\":" + String(temp) + ",\"humi0\":" + String(hum) + "}}}";
    byte arrSize = msgStr.length() + 1;
    char msg[arrSize];

    Serial.print("PUBLISH DATA:");
    Serial.println(msgStr);
    msgStr.toCharArray(msg, arrSize);
    client.publish(topic, msg);
    msgStr = "";
    delay(50);
    
    }
