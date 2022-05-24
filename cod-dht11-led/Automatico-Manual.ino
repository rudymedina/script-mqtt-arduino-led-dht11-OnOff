//***AUTO/MANUAL SWITCH***//

int modeSwitchPin = 2;              // switch is connected to pin 2
int ledAutoPin = 10;
int ledManPin = 11;

int val;                        // variable for reading the pin status
int val2;                       // variable for reading the delayed status
int modeButtonState;                // variable to hold the button state
int Mode = 0;            

//***AUTO SPEED CONTROL***//

#include <dht.h>

#define dht_dpin 12

dht DHT;

#define motorControl 9


//***MANUAL SPEED CONTROL

int switchPin = 3;              // switch is connected to pin 3
int ledLowPin = 6;
int ledMedPin = 7;
int ledHighPin = 8;
int motorPin = 9;

int val3;                        // variable for reading the pin status
int val4;                       // variable for reading the delayed status
int manSpeedButtonState;                // variable to hold the button state
int State = 0;           




void setup()
{
  //***AUTO/MANUAL SWITCH SETUP***//

  pinMode(modeSwitchPin, INPUT);    // Set the switch pin as input
  pinMode(ledAutoPin, OUTPUT);
  pinMode(ledManPin, OUTPUT);
  modeButtonState = digitalRead(modeSwitchPin);   // read the initial state


  //***MANUAL SPEED CONTROL SETUP***//

  pinMode(switchPin, INPUT);
  pinMode(ledLowPin, OUTPUT);
  pinMode(ledMedPin, OUTPUT);
  pinMode(ledHighPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  manSpeedButtonState = digitalRead(switchPin);   // read the initial state

  //***AUTO SPEED CONTROL SETUP***//
  pinMode(motorControl, OUTPUT);
  analogWrite(motorControl, 255);

  Serial.begin(9600);
}

void loop()
{


  //***AUTO/MANUAL SWITCH LOOP***//

  val = digitalRead(modeSwitchPin);      // read input value and store it in val
  delay(10);                         // 10 milliseconds is a good amount of time
  val2 = digitalRead(modeSwitchPin);     // read the input again to check for bounces
  if (val == val2) {                 // make sure we got 2 consistant readings!
    if (val != modeButtonState) {          // the button state has changed!
      if (val == LOW) {                // check if the button is pressed
        if (Mode == 0) {
          Mode = 1;
        } else {
          if (Mode == 1) {
            Mode = 0;
          }
        }
      }
    }
  }
  modeButtonState = val;                 // save the new state in our variable


  if (Mode == 0)  //***FAN IS IN AUTOMATIC MODE***//
  {
    digitalWrite(ledAutoPin, HIGH);
    digitalWrite(ledManPin, LOW);
    Serial.println("Automatic Mode");

    digitalWrite(ledLowPin, LOW);
    digitalWrite(ledMedPin, LOW);
    digitalWrite(ledHighPin, LOW);


    //***AUTO SPEED CONTROL LOOP***//

    DHT.read11(dht_dpin);

    int temp = DHT.temperature;
    Serial.print("Temperature = ");
    Serial.println(DHT.temperature);
    delay(1000);

    if (temp >= 29)
    {
      digitalWrite(motorControl, 255);
      Serial.println("Fan Speed: 100%");
      delay(10);
    }

    else if (temp == 28)
    {
      digitalWrite(motorControl, 204);
      Serial.println("Fan Speed: 80%");
      delay(10);
    }

    else if (temp == 27)
    {
      digitalWrite(motorControl, 153);
      Serial.println("Fan Speed: 60%");
      delay(10);
    }

    else if (temp == 26)
    {
      digitalWrite(motorControl, 102);
      Serial.println("Fan Speed: 40%");
      delay(10);
    }

    else if (temp <= 25)
    {
      digitalWrite(motorControl, 51);
      Serial.println("Fan Speed: 20%");
      delay(10);
    }

  }

  if (Mode == 1)  //***FAN IS IN MANUAL MODE***//
  {
    digitalWrite(ledAutoPin, LOW);
    digitalWrite(ledManPin, HIGH);
    Serial.println("MANUAL MODE");


    //***MANUAL SPEED LOOP***//

    val3 = digitalRead(switchPin);      // read input value and store it in val
    delay(10);                         // 10 milliseconds is a good amount of time
    val4 = digitalRead(switchPin);     // read the input again to check for bounces
    if (val3 == val4)
    { // make sure we got 2 consistant readings!
      if (val3 != manSpeedButtonState)
      { // the button state has changed!
        if (val3 == LOW)
        { // check if the button is pressed
          if (State == 0)
          {
            State = 1;
          }
          else if (State == 1)
          {
            State = 2;
          }
          else if (State == 2)
          {
            State = 3;
          }
          else if (State == 3)
          {
            State = 0;
          }
        }
      }
    }
    manSpeedButtonState = val3;                 // save the new state in our variable


    // Now do whatever the lightMode indicates
    if (State == 0) { // all-off
      digitalWrite(ledLowPin, LOW);
      digitalWrite(ledMedPin, LOW);
      digitalWrite(ledHighPin, LOW);
      digitalWrite(motorPin, 0);
      Serial.println("Fan Off");
      delay(1000);
    }

    if (State == 1) {
      digitalWrite(ledLowPin, HIGH);
      digitalWrite(ledMedPin, LOW);
      digitalWrite(ledHighPin, LOW);
      digitalWrite(motorPin, 85);
      Serial.println("Low Speed");
      delay(1000);
    }
    if (State == 2)
    {
      digitalWrite(ledLowPin, LOW);
      digitalWrite(ledMedPin, HIGH);
      digitalWrite(ledHighPin, LOW);
      digitalWrite(motorPin, 170);
      Serial.println("Medium Speed");
      delay(1000);
    }
    if (State == 3)
    {
      digitalWrite(ledLowPin, LOW);
      digitalWrite(ledMedPin, LOW);
      digitalWrite(ledHighPin, HIGH);
      digitalWrite(motorPin, 255);
      Serial.println("High Speed");
      delay(1000);
    }
  }
}



/// otrooo codigooo

if (digitalRead(modeButton) == HIGH) //not pressed, let's say that's auto mode
{
  if (temperature >= 28)
    digitalWrite(ledPin, HIGH);
  else
    digitalWrite(ledPin, LOW);
}
else //manual mode
{
  if (digitalRead(ledButton) == HIGH) //not pressed, let's say that's led off
    digitalWrite(ledPin, LOW);
  else //pressed, on
    digitalWrite(ledPin, HIGH);
}
