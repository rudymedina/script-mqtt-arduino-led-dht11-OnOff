#define ISR_SERVO_DEBUG             1
#include "SAMD_ISR_Servo.h"
// TIMER_TC3 for SAMD51, TIMER_TC3/TIMER_TCC for SAMD21
#define USING_SAMD_TIMER            TIMER_TC3
// Published values for SG90 servos; adjust if needed
#define MIN_MICROS        1000
#define MAX_MICROS        2000 

#define SERVO_PIN_1       A0
#define SERVO_PIN_2       A1
#define SERVO_PIN_3       A2

int position;      // position in degrees

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

void setup(){
   Serial.begin(115200);
   servoSetup();
}

void loop() {
  servo();
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
  Serial.println(F("Ventanas 0 degree"));
  for (int index = 0; index < NUM_SERVOS; index++)
  {
    SAMD_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position );
    printServoInfo(index);
  }
  // waits 5s between test
  delay(5000);

  position = 90;
  Serial.println("___________________________________________");
  Serial.println(F("Abriendo ventanas"));

  for (int index = 0; index < NUM_SERVOS; index++)
  {
    SAMD_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position );
    printServoInfo(index);
  }
  
  // waits 5s between test
  delay(5000);
  Serial.println("___________________________________________");
  Serial.println(F("Cerrando Ventanas"));
  
  for (position = 0; position <= 180; position += 5)
  {
    for (int index = 0; index < NUM_SERVOS; index++)
    {
      SAMD_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position );
    }
    // waits 0.1s for the servo to reach the position
    delay(100);
  }
  // waits 5s between test
  delay(5000);
}
