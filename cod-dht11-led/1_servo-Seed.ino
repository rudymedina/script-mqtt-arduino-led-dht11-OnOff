#define ISR_SERVO_DEBUG             1
#include "SAMD_ISR_Servo.h"
#define USING_SAMD_TIMER            TIMER_TC3
#define MIN_MICROS        1000
#define MAX_MICROS        2000 

#define SERVO_PIN_1       A0

int position;      // position in degrees
void setup(){

    Serial.begin(115200);
    pinMode(SERVO_PIN_1, OUTPUT);
    digitalWrite(SERVO_PIN_1, LOW);
 
  Serial.println(F("Servos ON "));
  SAMD_ISR_Servos.setReadyToRun();
}
void loop()
{
  position = 0;
  Serial.println(F("Ventanas"));
  
 for (position = 0; position <= 180; position += 1)
  {
      SAMD_ISR_Servos.setPosition(SERVO_PIN_1,position);
      delay(15);                       // waits 15 ms for the servo to reach the position
    }
   for (position = 180; position <= 0; position -= 1)
  {
      SAMD_ISR_Servos.setPosition(SERVO_PIN_1,position);
      delay(15);                       // waits 15 ms for the servo to reach the position
    }
  
}
