
// Include libraries for servo
#include <Servo.h>
#include <Adafruit_ADXL345_U.h>
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);
Servo myservo;

// Include libraries for LCD display
#include <LiquidCrystal.h>

// LCD pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Define pins for ultrasonic sensors
#define trigPin 10
#define echoPin 13

// Include IR Remote Library by Ken Shirriff
#include <IRremote.h>

//Include All motor libraries
#include <Wire.h>
#include <ADXL345.h>
#include <math.h>
#include <Servo.h>

// Servo variable 
Servo xAxisServo;
Servo yAxisServo;
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

// Define sensor pin
const int RECV_PIN = 4;

// Define LED pin constants
const int redPin = 8;
const int yellowPin = 7;
const int greenPin = 3;
const int bluePin = 2;


// Define integer to remember toggle state
int togglestate = 0;

// Define IR Receiver and Results Objects
IRrecv irrecv(RECV_PIN);
decode_results results;


void setup() {
  myservo.attach(2); // Attach servo to pin 2
  Serial.begin(9600);
  if (!accel.begin())
    accel.setRange(ADXL345_RANGE_2_G);
  accel.setDataRate(ADXL345_DATARATE_50_HZ);
  myservo.attach(2);
}


// Run forever
void loop() {

  sensors_event_t event;
  accel.getEvent(&event);
  
  float xRawData = event.acceleration.x;
  float y = event.acceleration.y;
  Serial.print("x RawValue: ");
  Serial.print(xRawData);
  Serial.print("      ");
  int xAngle = (xRawData + 10.00) * (90.00 + 90.00) / (10.00 + 10.00) - 90.00;
  Serial.println(xAngle);
  myservo.write(xAngle+130);

  compute; 
}


/*
 * Ultrasonic Distance Sensor + Liquid Crystal Display (LCD)  
 */ 
void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  float duration, distance;
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.0344;
  
  if (distance >= 400 || distance <= 2){
    lcd.print("Out of range");
    delay(500);
  }
  else {
    lcd.print(distance);
    lcd.print(" cm");
    delay(500);
  }
  delay(500);
  lcd.clear();
}


void setup() {
  // Enable the IR Receiver
  irrecv.enableIRIn();
  
  Serial.begin(9600);
  adxl.powerOn();
  adxl.setRangeSetting(16);

  pinMode(4, INPUT_PULLUP);
  xAxisServo.attach(2);//zeroed at 130
  yAxisServo.attach(7);// zeroed at 55

  yAxisServo.write(55);
  xAxisServo.write(130);
   
  // Set LED pins as Outputs
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
}


void compute() {
 // reading = irrecv.decode(&results); 
 
  if (irrecv.decode(&results)) { // Results for when you press the button 

    switch (results.value) {

      case 0xFFA25D: //For the system to turn on (POWER BUTTON) #RED
        if (togglestate == 0) {
          digitalWrite(redPin, HIGH);      
          togglestate = 1;
          ///////// redPin = 1;  
        }
        else {
          digitalWrite(redPin, LOW);
          togglestate = 0;

        }
        break;

      case 0xFF02FD: // For the system to take shot (FUNCTION BUTTON) #BLUE
        if (togglestate == 0 && redPin == 1) {
          digitalWrite(yellowPin, HIGH);
          togglestate = 1;
          
/////////////////////////////////// For LED debbuging

/*int buttonState = digitalRead(4);
  int x, y, z;
  adxl.readXYZ(&x, &y, &z);


  double xyz[3];
  float ax, ay, az;
  adxl.getAcceleration(xyz);
  ax = xyz[0];
  ay = xyz[1];
  az = xyz[2];


  float x2 = pow(ax, 2);
  float y2 = pow(ay, 2);
  float z2 = pow(az, 2);

  float denominatorX = sqrt (y2 + z2);
  float denominatorY = sqrt (x2 + z2);

  float rightSideX =  ax / denominatorX;
  float leftSideY = ay / denominatorY;

  float radiansOfX = atan(rightSideX);
  float radiansOfY = atan(leftSideY);

  int angleOfX = radiansOfX * 180 / 3.14159265359;
  int angleOfY = radiansOfY * 180 / 3.14159265359;
  Serial.print("angle of x = ");
  Serial.print(angleOfX);
  Serial.print("       angle of y = ");
  Serial.println(angleOfY);

  if (buttonState == 0) {

    yAxisServo.write(55 - angleOfY);
    xAxisServo.write(130 + angleOfX);
    delay(100);
    */c
          
        }

        else {
          digitalWrite(yellowPin, LOW);
          togglestate = 0;
        }

        break;

      case 0xFFE21D: // For the system to calibrate (FUNCTION BUTTON) #green
        if (togglestate == 0) {
          digitalWrite(greenPin, HIGH);
          togglestate = 1;
        }

        else {
          digitalWrite(greenPin, LOW);
          togglestate = 0;
        }

        break;

      case 0xFF9867: // For the system to 'do something' (FUNCTION BUTTON) #blue
        if (togglestate == 0) {
          digitalWrite(bluePin, HIGH);
          togglestate = 1;
        }

        else {
          digitalWrite(bluePin, LOW);
          togglestate = 0;
        }

        break;

    }
    irrecv.resume();
  }
 }
