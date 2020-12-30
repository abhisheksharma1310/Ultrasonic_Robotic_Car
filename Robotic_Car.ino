//Codes for Self Controlled Robotic Car using Arduino
#include <Servo.h>        // Include Servo Library
#include <NewPing.h>      // Include Newping Library

// L293d Control Pins
const int LeftMotorForward = 4;
const int LeftMotorBackward = 5;
const int RightMotorForward = 6;
const int RightMotorBackward = 7;
const int LEDext = 1;
const int Buzzer = 0;


#define TRIGGER_PIN  A1  // Arduino pin to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A2  // Arduino pin to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 250 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 250cm.

Servo servo_motor;  // Servo's name
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

boolean goesForward = false;
int distance = 50;

void setup()
{
  // Set L293d Control Pins as Output
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(LEDext, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  
  servo_motor.attach(9);   // Attachs the servo on pin 9 to servo object.
  servo_motor.write(115);   // Set at 115 degrees. 
  delay(2000);              // Wait for 2s.
  distance = readPing();    // Get Ping Distance.
  delay(100);               // Wait for 100ms.
}

void loop() 
{  
  int distanceRight = 0;  //Initialize right side distance                                              
  int distanceLeft = 0;   //Initialize left side distance
  delay(50);

  if (distance <= 30)   //If distance of obstacle less than 30 cm from robot 
  {
    Stop();   //call stop function to stop the robot
    digitalWrite(LEDext, HIGH);   //Turn led ON
    digitalWrite(Buzzer, HIGH);   //Turn Buzzer ON
    delay(300);   //wait for 300ms
    moveBackward();   //call moveBackward function to move robot in backward direction
    
    delay(400);   //wait for 400ms
    Stop();   //call stop function to stop the robot
    
    delay(300);   //wait for 300ms
    distanceRight = lookRight();    //call lookRight function to save distance in distanceRight variable
    
    delay(300);   //wait for 300ms
    distanceLeft = lookLeft();    //call lookLeft function to save distance in distanceLeft variable
    
    delay(300);   //wait for 300ms

    if (distanceRight >= distanceLeft)    //If distance of right greater or equall to distance of left
    {
      turnRight();    //call function to turn right robot
      delay(300);   //wait for 300ms
      Stop();   //call stop function to stop robot
    }
    else //else
    {
      turnLeft();   //call function to turn left robot
      delay(300);   //wait for 300ms
      Stop();   //call stop function to stop robot
    }
  
  }
  else    //else
  {
    moveForward();    //call moveForward function to move robot in forward direction 
  }

    distance = readPing();    //call readPing function to calculate Distance
}

int lookRight()     // lookRight Function for Servo Motor
{  
  servo_motor.write(0);   //make servo position at 0 degree
  delay(500);   //wait for 500ms
  int distance = readPing();    //read distance
  delay(100);   //wait for 100ms
  servo_motor.write(90);    //make servo position 90 degree
  return distance;    //return distance whenever lookRight function is called
}

int lookLeft()      // lookLeft Function for Servo Motor 
{
  servo_motor.write(180);   //make servo position at 0 degree
  delay(500);   //wait for 500ms
  int distance = readPing();    //read distance
  delay(100);   //wait for 100ms
  servo_motor.write(90);    //make servo position 90 degree
  return distance;    //return distance whenever lookLeft function is called
}

int readPing()      // readPing Function for Ultrasonic Sensor.
{
  delay(100);                 // Wait 100ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int cm = sonar.ping_cm();   //Send ping, get ping distance in centimeters (cm).
  if (cm==0)
  {
    cm=250;
  }
  return cm;    //return distance whenever readPing function is called
}

void Stop()       // Stop Function for Motor Driver.
{
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward()    // Move Forward Function for Motor Driver.
{
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(RightMotorBackward, LOW);
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(LEDext, LOW);
    digitalWrite(Buzzer, LOW);
}

void moveBackward()   // Move Backward Function for Motor Driver.
{
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);
}

void turnRight()      // Turn Right Function for Motor Driver.
{
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
}

void turnLeft()       // Turn Left Function for Motor Driver.
{
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);
}
