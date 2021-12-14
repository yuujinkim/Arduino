#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

#define _DUTY_MIN 1100
#define _DUTY_NUE 1450
#define _DUTY_MAX 1800

Servo myservo;
int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

  a = 84; //70;
  b = 333; //300;

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NUE);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);

  if(dist_cali < 255) myservo.writeMicroseconds(_DUTY_MAX);
  else myservo.writeMicroseconds(_DUTY_MIN);
}
