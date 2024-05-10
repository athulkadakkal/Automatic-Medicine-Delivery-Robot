/*

On the Arduino Mega board, the SDA (Serial Data) and SCL (Serial Clock) pins are used for I2C communication. 
On the Mega, these pins are located as follows:

    SDA (Serial Data): Pin 20 (SDA) / Analog pin 4
    SCL (Serial Clock): Pin 21 (SCL) / Analog pin 5

/////////////////////////////////////////////////////////////////////////////////////////////////////

How To Connect TCS34725 RGB Sensor With Arduino mega
----------------------------------------------------

      1.Connect the GND pin on the color sensor module with Arduino
      2.Connect the Pin A4/20 on the Arduino to the SDA pin on the color sensor IC.
      3.Connect the pin A5/21 on the Arduino to the SCL pin on the color sensor IC
      4.Connect the Sensor pin labelled 3.3 V to the 3.3 V pin on the Arduino.  

/////////////////////////////////////////////////////////////////////////////////////////////////////

how you can connect three servo motors to an Arduino Mega:
----------------------------------------------------------

  1.Red Servo Signal: Digital pin 9
  2.Green Servo Signal: Digital pin 10
  3.Blue Servo Signal: Digital pin 11
  4.Power (VCC) for all servos: 5V pin on Arduino Mega
  5.Ground (GND) for all servos: Any GND pin on Arduino Mega

/////////////////////////////////////////////////////////////////////////////////////////////////////

l293d connection
----------------
  > 12v 
  > Gnd
  > 5v
  > motor 1 and motor 2 = m1 and m2
  > motor 3 and motor 4 = m3 and m4
  > in1 = arduino meag pin 24
  > in2 = 25
  > in3 = 26
  > in4 = 27

/////////////////////////////////////////////////////////////////////////////////////////////////////

line tracking sensor code
-------------------------
  > vcc = 5v
  > Gnd = Gnd
  > LEFT_SENSORPIN = 42
  > CENTER_SENSORPIN = 44
  > RIGHT_SENSORPIN = 46

////////////////////////////////////////////////////////////////////////////////////////////////////*/

  #include <Wire.h>
  #include "Adafruit_TCS34725.h"
  #include <Servo.h>
  
  // Define servo objects for red, green, and blue servos
  Servo redServo;
  Servo greenServo;
  Servo blueServo;

  // rgb led pin connection 
  #define redpin 3
  #define greenpin 5
  #define bluepin 6

  // connect the path tracking sensors to digital pins
  #define LEFT_SENSORPIN 42
  #define CENTER_SENSORPIN 44
  #define RIGHT_SENSORPIN 46

  int buzzerPin = 63;  // A9 is digital pin 63 on the Arduino Mega

  //connect l293d in1 - in4 pin to arduino
  int in1 = 24;
  int in2 = 25;
  int in3 = 38;
  int in4 = 39;

  int centerSensor,leftSensor,rightSensor;

  // Define pins for red, green, and blue servos
  int redServoPin = 9;
  int greenServoPin = 10;
  int blueServoPin = 11;

  #define commonAnode true

  byte gammatable[256];

  Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {

  Serial.begin(9600);

  
  if (tcs.begin()) {
    
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); 
  }
  pinMode(buzzerPin, OUTPUT);

  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

    pinMode(in1,OUTPUT);   
  pinMode(in2,OUTPUT);   
  pinMode(in3,OUTPUT);   
  pinMode(in4,OUTPUT);

  pinMode(LEFT_SENSORPIN,INPUT);
  pinMode(CENTER_SENSORPIN,INPUT);
  pinMode(RIGHT_SENSORPIN,INPUT);  


  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
  }

  // Attach each servo to its respective pin
  redServo.attach(redServoPin);
  greenServo.attach(greenServoPin);
  blueServo.attach(blueServoPin);


}

void loop() {

  moveServo(redServo, 0);
  moveServo(blueServo, 0);
  moveServo(greenServo, 0);
  
  // read input from path tracking sensors
   leftSensor=digitalRead(LEFT_SENSORPIN);
   centerSensor=digitalRead(CENTER_SENSORPIN);
   rightSensor=digitalRead(RIGHT_SENSORPIN);

  Serial.print(" Left : ");
  Serial.print(leftSensor);
  Serial.print(" Centre : ");
  Serial.print(centerSensor);
  Serial.print(" Right : ");
  Serial.print(rightSensor);
  Serial.println();
  delay(130);
 if (centerSensor == 1 && leftSensor == 1 && rightSensor == 1 ){
  digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    delay(130);
    }
    else if (centerSensor == 1 && leftSensor == 1 ){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);delay(70);
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    delay(70);
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    delay(70);
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    rgbsensor();
    }
    else if (centerSensor == 1 && rightSensor == 1 ){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);delay(70);
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    delay(70);
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    delay(70);
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    rgbsensor();
    }
  else if(centerSensor == 1 ){            //move  forward
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    delay(70);
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    rgbsensor();
  }
  
  // else if(t == 'B'){      //move reverse 
  //   digitalWrite(in1,LOW);
  //   digitalWrite(in2,HIGH);
  //   digitalWrite(in3,LOW);
  //   digitalWrite(in4,HIGH);
  // }
    
  else if(leftSensor == 1){      //turn left 
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,LOW);
    digitalWrite(in4,HIGH);delay(70);
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    rgbsensor();
  }
  
  else  if(rightSensor == 1){      //turn right 

    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);delay(70);
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    rgbsensor();
  }
  else 
  {
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
  }
  
 
}

void rgbsensor(){
  float red, green, blue;
  
  tcs.setInterrupt(false);  // turn on LED

  delay(60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);
  
  tcs.setInterrupt(true);  // turn off LED

  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));

  Serial.print("\n");

  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);

  if(int (red)>=130)
  {
    motorstop();

    moveServo(redServo, 50);
    delay(100);
    moveServo(redServo, 0);
  }
  else if(int (blue)>=120)
  {
    motorstop();

    moveServo(blueServo, 50);
    delay(1000);
    moveServo(blueServo, 0);
  }
  else if(int (green)>=125)
  {
    motorstop();

    moveServo(greenServo, 50);
    delay(1000);
    moveServo(greenServo, 0);
  }
   
}

// Function to move a servo to a specified position
void moveServo(Servo servo, int position) {
  servo.write(position);
  delay(500); // Delay to allow the servo to reach the desired position
}

void motorstop(){
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
    //centerSensor=0;
    //rightSensor=0;
    //leftSensor=0;
}
