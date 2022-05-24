#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoJson.h> //library for Parsing JSON
#include "rpcWiFi.h" //Wi-Fi library 
#include "TFT_eSPI.h" //TFT LCD library 
#include "Free_Fonts.h" //free fonts library 
#include <SoftwareSerial.h>;
#include "DHT.h"
#include <DallasTemperature.h>
#include <Arduino.h>
#include <Wire.h>
#include "SHT31.h"
#define ISR_SERVO_DEBUG             1
#include "SAMD_ISR_Servo.h"
#define USING_SAMD_TIMER            TIMER_TC3
#define MIN_MICROS        1000
#define MAX_MICROS        2000 

#define tierra0 A0 //13
//#define tierra1 A4 //22
#define pin1 D2 //16
#define pin2 D3 //18
#define nivel BCM8 // 24 para medir el nivel de agua
#define CANAL1 BCM4 //7  e1  ACTIVAR MOTOR
#define CANAL2 BCM0 //27  i1  CONTROL PIN 1 POR MOTOR VENTANA
#define CANAL3 BCM19 //35 i2  CONTROL PIN 2 POR MOTOR VENTANA
#define CANAL4 BCM1 //28      CONTROL DE CALEFACCION
#define CANAL5 BCM4 //7

#define SERVO_PIN_1 A1 //15
#define SERVO_PIN_2 A4 //22
#define SERVO_PIN_3 A7 //16

#define PWMOUT BCM17
#define pinLDR0 A5 //32
//#define pinLDR1 A7 //36
#define RX BCM14// Rx=10  //8
#define TX BCM15//TX=8    //10

int position;      // position in degrees
float temp0, hum0, temp1, hum1, temp2, hum2;
float co2;
float valorLDR0,valorLDR1;
float suelo0, suelo1;
const int motor_encender = 25; // ocaso valor minimo de luz para el cual haremos funcionar el sistema
const int temp_minima = 20; //rango de luz deseado (salida del sistema) éstos valores los debe asignar el usuario de acuerdo a su necesidad y situación.
const int temp_maxima = 24;
int ndispositivos = 0;
float tempC;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long interval = 3000;
unsigned long interval2 = 60000;

unsigned long prevTime_T1 = millis();
unsigned long prevTime_T2 = millis();
unsigned long prevTime_T3 = millis(); 
unsigned long prevTime_T4 = millis(); 
unsigned long prevTime_T5 = millis();
unsigned long prevTime_T6 = millis();
unsigned long prevTime_T7 = millis();
unsigned long prevTime_T8 = millis();
unsigned long prevTime_T9 = millis();
unsigned long prevTime_T10 = millis();

TFT_eSPI tft; //initialize TFT LCD
TFT_eSprite spr = TFT_eSprite(&tft); //initialize sprite buffer

//PINES PARA CONEXION -----------
DHT dht0(pin1, DHT11);
DHT dht1(pin2, DHT22);
SHT31 sht31 = SHT31();
SoftwareSerial myCo2(RX, TX); // RX=D10-io5-naranja, TX=D11-io23-cafe
byte request[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
unsigned char response[9];
uint32_t delayMS;

// Configuraciones del sistema----------------------
#define WLAN_SSID       "Medina" 
#define WLAN_PASS       "belgorod25" // OKM690wsx
#define MQTT_SERVER      "35.238.225.243" 
#define MQTT_SERVERPORT  1883
#define MQTT_USERNAME    ""
#define MQTT_KEY         ""
#define MQTT_PUBLIC_CO2      "DarMal/Karinskoe_T5/T5C0"
#define MQTT_PUBLIC_TEMP_0   "DarMal/Karinskoe_T5/T5T0" 
#define MQTT_PUBLIC_HUMI_0   "DarMal/Karinskoe_T5/T5H0" 
#define MQTT_PUBLIC_TEMP_1   "DarMal/Karinskoe_T5/T5T1"
#define MQTT_PUBLIC_HUMI_1   "DarMal/Karinskoe_T5/T5H1"
#define MQTT_PUBLIC_TEMP_2   "DarMal/Karinskoe_T5/T5T2"
#define MQTT_PUBLIC_HUMI_2   "DarMal/Karinskoe_T5/T5H2"
#define MQTT_PUBLIC_TIERRA_0 "DarMal/Karinskoe_T5/T5T2"
#define MQTT_PUBLIC_TIERRA_1 "DarMal/Karinskoe_T5/T5M1"
#define MQTT_PUBLIC_LUZ_0    "DarMal/Karinskoe_T5/T5L0"
#define MQTT_PUBLIC_LUZ_1    "DarMal/Karinskoe_T5/T5L1"
#define MQTT_PUBLIC_SERVO_0  "DarMal/Karinskoe_T5/T5S0"
#define MQTT_PUBLIC_SERVO_1  "DarMal/Karinskoe_T5/T5S1"
#define MQTT_PUBLIC_SERVO_2  "DarMal/Karinskoe_T5/T5S2"

// conexion y rutamiento de topics para los sensores--------
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, MQTT_SERVERPORT, MQTT_USERNAME, MQTT_USERNAME, MQTT_KEY);
Adafruit_MQTT_Publish co2_z19 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_CO2);
Adafruit_MQTT_Publish temp_0_hdt_0 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_TEMP_0);
Adafruit_MQTT_Publish hum_0_hdt_0 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_HUMI_0);
Adafruit_MQTT_Publish temp_1_hdt_1 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_TEMP_1);
Adafruit_MQTT_Publish hum_1_hdt_1 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_HUMI_1);
Adafruit_MQTT_Publish temp_2_hdt_2 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_TEMP_2);
Adafruit_MQTT_Publish hum_2_hdt_2 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_HUMI_2);
Adafruit_MQTT_Publish tierra_0 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_TIERRA_0);
Adafruit_MQTT_Publish tierra_1 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_TIERRA_1);
Adafruit_MQTT_Publish luz_0 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_LUZ_0);
Adafruit_MQTT_Publish luz_1 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_LUZ_1);
Adafruit_MQTT_Publish servo_0 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_SERVO_0);
Adafruit_MQTT_Publish servo_1 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_SERVO_1);
Adafruit_MQTT_Publish servo_2 = Adafruit_MQTT_Publish(&mqtt, MQTT_PUBLIC_SERVO_2);
//-----------------------------------------------------------------------------------------------
Adafruit_MQTT_Subscribe canal1 = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "DarMal/Karinskoe_T5/canal1/onoff");
Adafruit_MQTT_Subscribe canal2 = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "DarMal/Karinskoe_T5/canal2/onoff");
Adafruit_MQTT_Subscribe canal3 = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "DarMal/Karinskoe_T5/canal3/onoff");
Adafruit_MQTT_Subscribe canal4 = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "DarMal/Karinskoe_T5/canal4/onoff");
Adafruit_MQTT_Subscribe canal5 = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "DarMal/Karinskoe_T5/canal5/onoff");
Adafruit_MQTT_Subscribe slider = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "DarMal/Karinskoe_T5/canal6/slider");
void MQTT_connect();
typedef struct
{
  int     servoIndex;
  uint8_t servoPin;
} ISR_servo_t;
#define NUM_SERVOS            3
ISR_servo_t ISR_servo[] =
{
  { -1, SERVO_PIN_1 }, { -1, SERVO_PIN_2 }, { -1, SERVO_PIN_3 }
};
void setup() {
   pinMode(CANAL1, OUTPUT);
   pinMode(CANAL2, OUTPUT);
   pinMode(CANAL3, OUTPUT);
   pinMode(CANAL4, OUTPUT);
   pinMode(CANAL5, OUTPUT);
   pinMode(PWMOUT, OUTPUT);
   //Apagamos todos los relays
   digitalWrite(CANAL1, HIGH); 
   digitalWrite(CANAL2, HIGH); 
   digitalWrite(CANAL3, HIGH); 
   digitalWrite(CANAL4, HIGH);
   digitalWrite(CANAL5, HIGH); 
   digitalWrite(PWMOUT, HIGH);

   mqtt.subscribe(&canal1);
   mqtt.subscribe(&canal2);
   mqtt.subscribe(&canal3);
   mqtt.subscribe(&canal4);
   mqtt.subscribe(&canal5);
   mqtt.subscribe(&slider);
   pinMode(WIO_LIGHT, INPUT);
   pinMode(nivel, INPUT);//Configuro en nivel de agua como entrada


   Serial.begin(115200);
   Serial.println("*******Dary Malinovky*******");
   connectWiFi();
   Serial.print("RRSI: ");
   Serial.println(WiFi.RSSI());
   connectMQTT();
   dht0.begin();
   dht1.begin();
   sht31.begin();
   myCo2.begin(9600);
    //  ----------------LCD------------------------
  //LCD setup
  tft.begin(); //start TFT LCD
  tft.setRotation(3); //set TFT LCD rotation
  tft.fillScreen(TFT_BLACK); //fill background

  //header title 
  tft.fillRect(0,0,320,50,TFT_GREEN); //fill rectangle 
  tft.setFreeFont(FSB12); //set font type 
  tft.setTextColor(TFT_BLACK); //set text color
  tft.drawString("**DARY MALINOVKY**", 10, 20); //draw string 
    
  //text strings 
  tft.setTextColor(TFT_WHITE); 
  tft.setFreeFont(FS18); 
  tft.drawString("Temperature:", 10,60);
  tft.drawString("Humidity:", 10,110);
  tft.drawString("Co2:", 10,160);
  
  tft.setTextColor(TFT_GREEN); 
  tft.setFreeFont(FMB24);  
  tft.drawString("C",260,60);
  tft.drawString("%", 215,100);
  tft.drawString("PPM", 200,150);
  tft.drawFastHLine(0,140,320,TFT_GREEN); //draw horizontal line
  servoSetup();
} 
void loop() {
  unsigned long currentTime = millis();

  if (currentTime - prevTime_T1 > interval) {
    leerhum0();
    prevTime_T1 = currentTime;
    }
  if (currentTime - prevTime_T2 > interval) {
    leerhum1();
    prevTime_T2 = currentTime;
    }
  if (currentTime - prevTime_T3 > interval) {
    leerhum2();
    prevTime_T3 = currentTime;
    }
  if (currentTime - prevTime_T4 > interval) {
    LeerMoisture0();
    prevTime_T4 = currentTime;
    }
  if (currentTime - prevTime_T6 > interval) {
    LeerLuz_0();
    prevTime_T6 = currentTime;
    }
  if (currentTime - prevTime_T8 > interval) {
    leerco2();
    prevTime_T8 = currentTime;
    }
  if (currentTime - prevTime_T9 > interval) {
    NivelBonba();
    prevTime_T9 = currentTime;
    }
  if (currentTime - prevTime_T10 > interval) {
    Serial.println("______________________________________");
    prevTime_T10 = currentTime;
    }
  checkWifi();
  reconnect();
  suscribirCanal();
  servo();
}
void suscribirCanal() {
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    // Check if its the onoff button feed
    if (subscription == &canal1) {
      Serial.print(F("On-Off canal1: "));
      Serial.println((char *)canal1.lastread);
      
      if (strcmp((char *)canal1.lastread, "ON") == 0) {
        digitalWrite(CANAL1, LOW); 
      }
      if (strcmp((char *)canal1.lastread, "OFF") == 0) {
        digitalWrite(CANAL1, HIGH); 
      }
    }
    if (subscription == &canal2) {
      Serial.print(F("On-Off canal2: "));
      Serial.println((char *)canal2.lastread);
      
      if (strcmp((char *)canal2.lastread, "ON") == 0) {
        digitalWrite(CANAL2, LOW); 
      }
      if (strcmp((char *)canal2.lastread, "OFF") == 0) {
        digitalWrite(CANAL2, HIGH); 
      }
    }
    if (subscription == &canal3) {
      Serial.print(F("On-Off canal3: "));
      Serial.println((char *)canal3.lastread);
      
      if (strcmp((char *)canal3.lastread, "ON") == 0) {
        digitalWrite(CANAL3, LOW); 
      }
      if (strcmp((char *)canal3.lastread, "OFF") == 0) {
        digitalWrite(CANAL3, HIGH); 
      }
    }
    if (subscription == &canal4) {
      Serial.print(F("On-Off canal4: "));
      Serial.println((char *)canal4.lastread);
      
      if (strcmp((char *)canal4.lastread, "ON") == 0) {
        digitalWrite(CANAL4, LOW); 
      }
      if (strcmp((char *)canal4.lastread, "OFF") == 0) {
        digitalWrite(CANAL4, HIGH); 
      }
    }
    if (subscription == &canal5) {
      Serial.print(F("On-Off canal5: "));
      Serial.println((char *)canal5.lastread);
      
      if (strcmp((char *)canal5.lastread, "ON") == 0) {
        digitalWrite(CANAL5, LOW); 
      }
      if (strcmp((char *)canal4.lastread, "OFF") == 0) {
        digitalWrite(CANAL5, HIGH); 
      }
    }    
    // check if its the slider feed
    if (subscription == &slider) {
      Serial.print(F("Slider: "));
      Serial.println((char *)slider.lastread);
      uint16_t sliderval = atoi((char *)slider.lastread);  // convert to a number
      analogWrite(PWMOUT, sliderval);
    }
  }
  // ping the server to keep the mqtt connection alive
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
  }
void leerhum0() {  
  temp0 = sht31.getTemperature();
  hum0 = sht31.getHumidity();
  
  Serial.print("Temperatura T0T0: ");
  Serial.print(temp0);
  Serial.println(" ºC.");
  Serial.print("Humedad T0H0: ");
  Serial.print(hum0);
  Serial.println(" %.");
  temp_0_hdt_0.publish(temp0);
  hum_0_hdt_0.publish(hum0);
  //sprite buffer for temperature
  spr.createSprite(55, 40); //create buffer
  spr.fillSprite(TFT_BLACK); //fill background color of buffer
  spr.setFreeFont(FMB24); //set font type 
  spr.setTextColor(TFT_WHITE); //set text color
  spr.drawNumber(temp0, 0, 0); //display number 
  spr.pushSprite(200, 50); //push to LCD 
  spr.deleteSprite(); //clear buffer

  //sprite buffer for humidity 
  spr.createSprite(55, 40);
  spr.fillSprite(TFT_BLACK);
  spr.setFreeFont(FMB24);
  spr.setTextColor(TFT_WHITE);
  spr.drawNumber(hum0, 0, 0); //display number 
  spr.pushSprite(155, 100);
  spr.deleteSprite(); 
}
void leerhum1() { 
  temp1 = dht0.readTemperature();
  hum1 = dht0.readHumidity();
  Serial.print("Temperatura T1T1: ");
  Serial.print(temp1);
  Serial.println(" ºC.");
  Serial.print("Humedad T1H1: ");
  Serial.print(hum1);
  Serial.println(" %.");
  temp_1_hdt_1.publish(temp1);
  hum_1_hdt_1.publish(hum1);  
  if(temp1 >30){
    Serial.println("Canal3 ventana = ON :");
    digitalWrite(CANAL3, LOW);
    }else{
     Serial.println("Canal3 ventana = OFF :");
    digitalWrite(CANAL3, HIGH);
      }
}
void leerhum2() { 
    temp2 = dht1.readTemperature();
    hum2 = dht1.readHumidity();
    Serial.print("Temperatura T2T0: ");
    Serial.print(temp2);
    Serial.println(" ºC.");
    Serial.print("Humedad T2H0: ");
    Serial.print(hum2);
    Serial.println(" %.");
    temp_2_hdt_2.publish(temp2);
    hum_2_hdt_2.publish(hum2);  
}
void LeerMoisture0() {
    suelo0 = analogRead(tierra0);
    Serial.print("Soil Moisture _0  = " );
    Serial.println(suelo0);
    tierra_0.publish(suelo0);
    if(suelo0 >= 0 & suelo0 < 100){
        Serial.println("Sensor en suelo seco");
        Serial.println("On POMPA");
        digitalWrite(CANAL1, LOW);
  
    } else if(suelo0 > 100 & suelo0 <= 199){
        Serial.println("Sensor en suelo húmedo");
    }else if(suelo0 > 199){
        Serial.println("Sensor en agua");
        Serial.println("Off POMPA");
        digitalWrite(CANAL1, HIGH);
    }
}
void LeerLuz_0() { 
  valorLDR0 = analogRead(WIO_LIGHT);
  Serial.print("luz_0 = ");
  Serial.println(valorLDR0);
  luz_0.publish(valorLDR0);
  if(valorLDR0 < 150){
    Serial.println("Canal2 Luz = ON :");
    digitalWrite(CANAL2, LOW);
  }
  else{
   Serial.println("Canal2 luz = OFF :");
   digitalWrite(CANAL2, HIGH);
  }
}
void leerco2() { 
  myCo2.write(request, 9);
  myCo2.write((byte)0x00);

  memset(response, 0, 9);
  myCo2.readBytes(response, 9);
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc+=response[i];
  crc = 255 - crc;
  crc++;
  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    Serial.println("CRC error");
  } else {
    unsigned int HLconcentration = (unsigned int) response[2];
    unsigned int LLconcentration = (unsigned int) response[3];
    co2 = (256*HLconcentration) + LLconcentration;
    Serial.println(co2);
    for (i = 0; i < 9; i++) {
      Serial.print("0x");
      Serial.print(response[i],HEX);
      Serial.print("  ");
    }
    Serial.println("  ");
   co2_z19.publish(co2); //  Serial.write((byte) 0x00);

  }
  //sprite buffer for Co2 
  spr.createSprite(55, 40);
  spr.fillSprite(TFT_BLACK);
  spr.setFreeFont(FMB24);
  spr.setTextColor(TFT_WHITE);
  //spr.drawNumber(co2, 0, 0); 
  spr.drawNumber(co2, 0, 0, 4);
  spr.pushSprite(100, 160);
  spr.deleteSprite(); 
}  
void NivelBonba(){
  int SensorNivel = digitalRead(nivel); //Leo lo que marca el nivel de agua
  Serial.println("ESTADO DE TANQUE DE AGUA");
  Serial.println(SensorNivel);
  }
void connectWiFi() {
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
    WiFi.begin(WLAN_SSID, WLAN_PASS);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempts++;
        if (attempts > 30) {
         NVIC_SystemReset();
      }     
    }
    //tft.fillScreen(TFT_BLACK);
    //tft.setCursor((320 - tft.textWidth("Connected!"))/2, 120);
    //tft.print("Connected..!");
    Serial.println("Connected to the WiFi network");
    //Serial.print("IP Address: ");
    //Serial.println (WiFi.localIP()); // prints out the device's IP address
  
}
void checkWifi() {
  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("(void checkWifi) Reconnecting to WiFi...");
    //WiFi.disconnect();
    NVIC_SystemReset();
    previousMillis = currentMillis;
  }
}
void connectMQTT() {
  if (mqtt.connected())
    return;
  Serial.print("Connecting to MQTT... ");
  int attempts2 = 0;
  while (mqtt.connect() != 0) {
       delay(500);
       Serial.print(".");
       attempts2++;
       mqtt.disconnect();
       if (attempts2 > 30) {
         NVIC_SystemReset();
      }
  }
  Serial.println("MQTT Connected!");

}
void reconnect() {
 
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("test alive MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(3000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         NVIC_SystemReset();
       }
  }
}
void servoSetup() {  
  for (int index = 0; index < NUM_SERVOS; index++)
  {
    pinMode(ISR_servo[index].servoPin, OUTPUT);
    digitalWrite(ISR_servo[index].servoPin, LOW);
  }
  Serial.begin(115200);
  Serial.println(F("Servos ON "));
  // SAMD51 always uses TIMER_TC3
  SAMD_ISR_Servos.useTimer(USING_SAMD_TIMER);
  for (int index = 0; index < NUM_SERVOS; index++)
  {
    ISR_servo[index].servoIndex = SAMD_ISR_Servos.setupServo(ISR_servo[index].servoPin, MIN_MICROS, MAX_MICROS);
    if (ISR_servo[index].servoIndex != -1)
    {
      Serial.print(F("Setup OK Servo index = ")); 
      Serial.println(ISR_servo[index].servoIndex);
    }
    else
    {
      Serial.print(F("Setup Failed Servo index = ")); 
      Serial.println(ISR_servo[index].servoIndex);
    }
  }
  SAMD_ISR_Servos.setReadyToRun();
}
void printServoInfo(int indexServo){  

  Serial.print(F("Servo ID = "));
  Serial.print(indexServo);
  Serial.print(F(", act. pos. (deg) = "));
  Serial.print(SAMD_ISR_Servos.getPosition(ISR_servo[indexServo].servoIndex) );
  Serial.print(F(", pulseWidth (us) = "));
  Serial.println(SAMD_ISR_Servos.getPulseWidth(ISR_servo[indexServo].servoIndex));
}
void servo(){
  position = 0;
  //if(valorLDR0 < motor_encender){
 if(valorLDR0 > temp_minima && valorLDR0 < temp_maxima) {
  Serial.println("___________________________________________");
  Serial.println(F("****Cerrando ventanas****"));
  for (int index = 0; index < NUM_SERVOS; index++)
  {
    SAMD_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position );
    printServoInfo(index);
  }
  delay(5000);
  }
  //else{
  if(valorLDR0 > temp_minima && valorLDR0 > temp_maxima) {
  position = 90;
  Serial.println("___________________________________________");
  Serial.println(F("****Abriendo ventanas****"));

  for (int index = 0; index < NUM_SERVOS; index++)
  {
    SAMD_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position );
    printServoInfo(index);
  }
  delay(5000);
  }
  else{
  position = 0;
  Serial.println("___________________________________________");
  Serial.println(F("****CIERRE****"));

  for (int index = 0; index < NUM_SERVOS; index++)
  {
    SAMD_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position );
    printServoInfo(index);
  }
  delay(5000);
  }
}
