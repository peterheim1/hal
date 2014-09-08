

#define InA1            11                      // INA motor pin
#define InB1            8                       // INB motor pin
#define PWM1            10                       // PWM motor pin
#define InA2            7                      // INA motor pin
#define InB2            4                       // INB motor pin
#define PWM2            9                       // PWM motor pin


void setup() {
  Serial.begin(115200);

  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
}

void loop() {




}

void setM2Speed(int speed){
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
    
  analogWrite(PWM1,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  
  if (speed == 0)
  {
    digitalWrite(InA1,LOW);   // Make the motor coast no
    digitalWrite(InB1,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(InA1,HIGH);
    digitalWrite(InB1,LOW);
  }
  else
  {
    digitalWrite(InA1,LOW);
    digitalWrite(InB1,HIGH);
  }
}
 /// left motor
void setM1Speed(int speed){
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
    
  analogWrite(PWM2,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  
  if (speed == 0)
  {
    digitalWrite(InA2,LOW);   // Make the motor coast no
    digitalWrite(InB2,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(InA2,HIGH);
    digitalWrite(InB2,LOW);
  }
  else
  {
    digitalWrite(InA2,LOW);
    digitalWrite(InB2,HIGH);
  }
}

