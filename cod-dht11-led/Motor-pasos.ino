const int sensorPin = WIO_LIGHT; // LDR
const int motor_encender = 25; // ocaso valor minimo de luz para el cual haremos funcionar el sistema
const int temp_minima = 20; //rango de luz deseado (salida del sistema) éstos valores los debe asignar el usuario de acuerdo a su necesidad y situación.
const int temp_maxima = 24;

#define e1 BCM4  // Enable Pin for motor 1
#define i1 BCM0   // Control pin 1 for motor 1
#define i2 BCM19   // Control pin 2 for motor 1

// CANAL1   e1  ACTIVAR MOTOR
//CANAL2  i1  CONTROL PIN 1 POR MOTOR VENTANA
//CANAL3 i2  CONTROL PIN 2 POR MOTOR VENTANA
//CANAL4      CONTROL DE CALEFACCION
void setup()
{
 Serial.begin(115200);
}

void loop() {
 int valueAnalog = analogRead(sensorPin); //guardamos los valores observados por el LDR
 Serial.println(valueAnalog); //mostramos en pantalla la luz observada por el LDR
 if(valueAnalog < motor_encender) {
   digitalWrite(e1, HIGH); // Motor Activado
   digitalWrite(i1, HIGH);  //Motor Parado = Hay luz insuficiente
   digitalWrite(i2, HIGH);
 }
 if(valueAnalog > motor_encender && valueAnalog < temp_minima) {
   digitalWrite(e1, HIGH); // Motor Activado
   digitalWrite(i1, HIGH); // Gira adelante = Abre persiana
   digitalWrite(i2, LOW);
 }
 if(valueAnalog > temp_maxima) {
   digitalWrite(e1, HIGH); // Motor Activado
   digitalWrite(i1, LOW);  // Gira al revés = Cierra persiana
   digitalWrite(i2, HIGH);
 }
 if(valueAnalog > temp_minima && valueAnalog < temp_maxima) {
   digitalWrite(e1, HIGH); // Motor Activado
   digitalWrite(i1, HIGH);  //Motor Parado = Hay Luz ideal
   digitalWrite(i2, HIGH);
 }
}
