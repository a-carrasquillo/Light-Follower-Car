/**
 * Light Follower Autonomous Car 
 * 
 * This project purpose is to build an autonomous car that follows the light     
 * of flashlight and detects when there is an object in its path.
*/

// Import the servo library so you can control the servo motor
#include <Servo.h>

// Constants

// L298N H-Bridge driving DC motor
// Microcontroller Unit (MCU) pulse width modulation (PWM) pin 6 to Enable 
// (EN) A on L298n board
const int ENA = 6;
// MCU digital pin 11 to Input (IN) 1 on L298n board
const int IN1 = 11;
// MCU digital pin 9 to IN2 on L298n board
const int IN2 = 9;
// MCU PWM pin 5 to ENB on L298n board
const int ENB = 5;
// MCU digital pin 8 to IN3 on L298n board
const int IN3 = 8;
// MCU digital pin 7 to IN4 on L298n board
const int IN4 = 7;

// Distance Sensor / Ultrasonic Sensor
// MCU digital pin 10 to trigger pin on ultrasonic sensor
const int TRIGGER = 10;
// MCU digital pin 12 to echo pin on ultrasonic sensor
const int ECHO = 12;
// Establish the minimum distance (cm) for object detection
// Note: This distance can be changed
const float MIN_DISTANCE = 25;

// MCU digital pin 4 to LED anode
const int LED = 4;
// MCU digital pin 2 to the buzzer anode
const int BUZZER = 2;

// Photoresistors
// Remember that photoresistors do not have polarity, so choose one leg 
// for connecting the Arduino's analog ports and the other one to the ground
// MCU analog pin 2 to left photoresistor
const int LEFT_PHOTO = A2;
// MCU analog pin 1 to middle photoresistor
const int MIDDLE_PHOTO = A1;
// MCU analog pin 3 to right photoresistor
const int RIGHT_PHOTO = A3;

// Note: The number used for comparison (1000) can be changed, this will affect
// the sensitivity of light change between the photoresistors. The greater the
// number, the greater should be the difference of light between the
// photoresistors to make a decision.
const int MINIMAL_DIFF_LIGHT = 1000;

// Global Variables
// Variable to hold the reads of the middle photoresistor
int middleSensorVal = 0;
// Variable to hold the minimal read of the middle photoresistor
int middleSensorMin = 1023;
// Variable to hold the maximal read of the middle photoresistor
int middleSensorMax = 0;

// Variable to hold the reads of the left photoresistor
int leftSensorVal = 0;
// Variable to hold the minimal read of the left photoresistor
int leftSensorMin = 1023;
// Variable to hold the maximal read of the left photoresistor
int leftSensorMax = 0;

// Variable to hold the reads of the right photoresistor
int rightSensorVal = 0;
// Variable to hold the minimal read of the right photoresistor
int rightSensorMin = 1023;
// Variable to hold the maximal read of the right photoresistor
int rightSensorMax = 0;

// Variable to indicate if there is an object in the car's path
bool object = false;
// Variable to indicate if the previous path of the car was the left
bool left = false;
// Variable to indicate if the previous path of the car was the middle
bool middle = false;
// Variable to indicate if the previous path of the car was the right
bool right = false;

// Create servo object to control the servo
Servo myServo;

void setup() {
  // Set all the L298n pin to output
  pinMode(ENA, OUTPUT); 
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Set the echo pin of the ultrasound as input
  pinMode(ECHO, INPUT);
  // Set the trigger pin of the ultrasound as output
  pinMode(TRIGGER,OUTPUT);
  
  // Set the led and the buzzer as output
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  
  // Set all the photoresistors as inputs
  pinMode(LEFT_PHOTO,INPUT);
  pinMode(MIDDLE_PHOTO,INPUT);
  pinMode(RIGHT_PHOTO,INPUT);
  
  // Indicate that the servo motor is attached to the MCU PWM pin 3
  myServo.attach(3);
  // Move the servo to its default position
  servoDefault();
  // Makes sure that all the motors are shut down
  motorStop();
  // Calibrate the three photoresistors
  photoCalibration();
  // Opens serial port, sets data rate to 9600 bps
  Serial.begin(9600);
  // Pauses the program for a second and a half
  delay(1500);
}

void loop() {
  // Read the photoresistors values
  readPhotoSensor();
  
  // Is the middle photoresistor receiving more light than the right and left one?
  if((middleSensorVal-leftSensorVal)>MINIMAL_DIFF_LIGHT and 
     (middleSensorVal-rightSensorVal)>MINIMAL_DIFF_LIGHT) {
    // Is the previous path the middle one? (middle == true)
    if(middle) {
      // Read the ultrasound to determine if there is an object and determine its
      // distance to the car
      float dist = readUltrasound();
      // Verify the distance to the object
      verifDist(dist);
      // Is not there an object close enough to the car? (!object == true)
      if(!object) {
        // Move the car forward
        goForward();
        // Indicates that the current path is not the left one
        left = false;
        // Indicates that the current path is the middle one
        middle = true;
        // Indicates that the current path is not the right one
        right = false;
      }
    } else { // The previous path is not the middle one
      // Stop the car
      motorStop();
      // Move the servo motor to its default position
      servoDefault();
      // Read the ultrasound to determine if there is an object and determine its
      // distance to the car
      float dist = readUltrasound();
      // Verify the distance to the object
      verifDist(dist);
      // Is not there an object close enough to the car? (!object == true)
      if(!object) {
        // Move the car forward
        goForward();
        // Indicates that the current path is not the left one
        left = false;
        // Indicates that the current path is the middle one
        middle = true;
        // Indicates that the current path is not the right one
        right = false;
      }
    }
  // Is the left photoresistor receiving more light than the middle and right one?
  } else if((leftSensorVal-middleSensorVal)>MINIMAL_DIFF_LIGHT  and 
          (leftSensorVal-rightSensorVal)>MINIMAL_DIFF_LIGHT) {
    // Is the previous path the left one? (left == true)   
    if(left) {
      // Read the ultrasound to determine if there is an object and determine its
      // distance to the car
      float dist = readUltrasound();
      // Verify the distance to the object
      verifDist(dist);
      // Is not there an object close enough to the car? (!object == true)
      if(!object) {
        // Move the car to the left side
        turnLeft();
        // Indicates that the current path is the left one
        left = true;
        // Indicates that the current path is not the middle one
        middle = false;
        // Indicates that the current path is not the right one
        right = false;
      }
    } else { // The previous path is not the left one
      // Stop the car
      motorStop();
      // Turn the servo motor to the left side
      servoTurnLeft();
      // Read the ultrasound to determine if there is an object and determine its
      // distance to the car
      float dist = readUltrasound();
      // Verify the distance to the object
      verifDist(dist);
      // Is not there an object close enough to the car? (!object == true)
      if(!object) {
        // Move the car to the left side
        turnLeft();
        // Indicates that the current path is the left one
        left = true;
        // Indicates that the current path is not the middle one
        middle = false;
        // Indicates that the current path is not the right one
        right = false;
      }
    }
  // Is the right photoresistor receiving more light than the middle and left one?
  } else if((rightSensorVal-middleSensorVal)>MINIMAL_DIFF_LIGHT and 
          (rightSensorVal-leftSensorVal)>MINIMAL_DIFF_LIGHT)  {
    // Is the previous path the right one? (right == true)   
    if(right) {
      // Read the ultrasound to determine if there is an object and determine its
      // distance to the car
      float dist = readUltrasound();
      // Verify the distance to the object
      verifDist(dist);
      // Is not there an object close enough to the car? (!object == true)
      if(!object) {
        // Turn the car to the right
        turnRight();
        // Indicates that the current path is not the left one
        left = false;
        // Indicates that the current path is not the middle one
        middle = false;
        // Indicates that the current path is the right one
        right = true;
      }
    } else {
      // Stop the car
      motorStop();
      // Turn the servo motor to the right
      servoTurnRight();
      // Read the ultrasound to determine if there is an object and determine its
      // distance to the car 
      float dist = readUltrasound();
      // Verify the distance to the object
      verifDist(dist);
      // Is not there an object close enough to the car? (!object == true)
      if(!object) {
        // Turn the car to the right
        turnRight();
        // Indicates that the current path is not the left one
        left = false;
        // Indicates that the current path is not the middle one
        middle = false;
        // Indicates that the current path is the right one
        right = true;
      }
    }
  } else { // All photoresistors' values are the same
    // Move the servo motor to its default position
    servoDefault();
    // Stop the car
    motorStop();
    // Read the ultrasound to determine if there is an object and determine its
    // distance to the car 
    float dist = readUltrasound();
    // Verify the distance to the object
    verifDist(dist);
    // Messages for debugging purpose
    Serial.print("Distance = ");
    Serial.println(dist);
    Serial.println("All the photoresistor are receiving the same amount of light");
  }
}

/**
 * Makes the car go forward
 */
void goForward() {
  // Message for debugging purpose
  Serial.println("Forward");
  // Turn on motor A
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Set speed to 100 out of possible range 0~255
  analogWrite(ENA, 100);
  // Turn on motor B
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  // Set speed to 100 out of possible range 0~255
  analogWrite(ENB, 100);
}

/**
 * Makes the car turn left
 */
void turnLeft() {
  // Message for debugging purpose
  Serial.println("Turn Left");
  // Turn on motor A
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Set speed to 150 out of possible range 0~255
  analogWrite(ENA, 150);
  // Turn on motor B
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  // Set speed to 150 out of possible range 0~255
  analogWrite(ENB, 150);
}

/**
 * Makes the car turn right
 */
void turnRight() {
  // Message for debugging purpose
  Serial.println("Turn Right");
  // Turn on motor A
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // Set speed to 150 out of possible range 0~255
  analogWrite(ENA, 150);
  // Turn on motor B
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  // Set speed to 150 out of possible range 0~255
  analogWrite(ENB, 150);
}

/**
 * Makes the car stop
 */
void motorStop() {
  // Message for debugging purpose
  Serial.println("Stop");
  // Turn off motor A
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  // Turn off motor B
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  // Shut down the enables
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  // Stop the program for 100 microseconds
  delayMicroseconds(100);
}

/**
 * Moves the servo until the ultrasonic sensor points forward
 * 
 * Note: This value may change depending on the servo and the mounting
 */
void servoDefault() {
  myServo.write(80);
}

/**
 * Moves the servo until the ultrasonic sensor points to the right
 * 
 * Note: This value may change depending on the servo and the mounting
 */
void servoTurnRight() {
  myServo.write(10);
}

/**
 * Moves the servo until the ultrasonic sensor points to the left
 * 
 * Note: This value may change depending on the servo and the mounting
 */
void servoTurnLeft() {
  myServo.write(160);
}

/**
 * Turns on the ultrasonic sensor to determine if there is an object 
 * and determine its distance to the car
 * 
 * @return Distance of the object to the car in centimeters
 */
float readUltrasound() {
  // Define the duration variable to hold the time that the sound wave
  // takes to return. Define the distance variable to hold the 
  // calculated distance to the object
  float duration, distance;
  // Make the ultrasonic sensor produce a sound wave
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(20);
  digitalWrite(TRIGGER, LOW);
  // Determine the time that the sound wave takes to return back
  duration = pulseIn(ECHO, HIGH);
  // Determine the distance to the object using the formula shown below
  // Note: 343 m/s is the velocity of the sound, which is equivalent
  // to 1/29.2 cm/microseconds, which is the value used in the formula
  // so you can get the distance in cm
  distance = (duration/2) * 1/29.2; 
  // Return the distance
  return distance;
}

/**
 * Calibrates the three photoresistors, so the car can work
 * in different environments
 */
void photoCalibration() {
  // Turn on the LED to indicate that the calibration process has begun
  digitalWrite(LED, HIGH);

  // Calibrate during five seconds
  while (millis() < 5000) {
    // Read the photoresistors
    middleSensorVal = analogRead(MIDDLE_PHOTO);
    leftSensorVal = analogRead(LEFT_PHOTO);
    rightSensorVal = analogRead(RIGHT_PHOTO);

    // Record the maximum value for the middle sensor
    if (middleSensorVal > middleSensorMax) {
      middleSensorMax = middleSensorVal;
    }

    // Record the minimum value for the middle sensor
    if (middleSensorVal < middleSensorMin) {
      middleSensorMin = middleSensorVal;
    }

    // Record the maximum value for the left sensor
    if (leftSensorVal > leftSensorMax) {
      leftSensorMax = leftSensorVal;
    }

    // Record the minimum value for the left sensor
    if (leftSensorVal < leftSensorMin) {
      leftSensorMin = leftSensorVal;
    }

    // Record the maximum value for the right sensor
    if (rightSensorVal > rightSensorMax) {
      rightSensorMax = rightSensorVal;
    }

    // Record the minimum value for the right sensor
    if (rightSensorVal < rightSensorMin) {
      rightSensorMin = rightSensorVal;
    }
  }

  // Indicate the end of the calibration process
  digitalWrite(LED, LOW);
}

/**
 * Reads the three photoresistors
 */
void readPhotoSensor() {
  // Read the middle sensor/photoresistor
  middleSensorVal = analogRead(MIDDLE_PHOTO);
  // Apply the calibration to the middle sensor reading
  middleSensorVal = map(middleSensorVal, middleSensorMin, middleSensorMax, 0, 255);
  // Message for debugging purpose
  Serial.print("Middle sensor:");
  Serial.println(middleSensorVal);
  
  // Read the left sensor
  leftSensorVal = analogRead(LEFT_PHOTO);
  // Apply the calibration to the left sensor reading
  leftSensorVal = map(leftSensorVal, leftSensorMin, leftSensorMax, 0, 255);
  // Message for debugging purpose
  Serial.print("Left sensor:");
  Serial.println(leftSensorVal);
  
  // Read the right sensor
  rightSensorVal = analogRead(RIGHT_PHOTO);
  // Apply the calibration to the right sensor reading
  rightSensorVal = map(rightSensorVal, rightSensorMin, rightSensorMax, 0, 255);
  // Message for debugging purpose
  Serial.print("Right sensor:");
  Serial.println(rightSensorVal);
  Serial.println("");

}

/**
 * Verify the distance to the object and determines if the car needs to stop
 * 
 * @param dist distance to the object
 */
void verifDist(float dist) {
  // Is the distance less or equal to the minimum distance?
  if(dist<=MIN_DISTANCE) {
    // Stop the car
    motorStop();
    // Indicate that there is an object
    object = true;
    // Turn on the LED and the buzzer to indicate that there is an object
    // in the path
    digitalWrite(LED, HIGH);
    digitalWrite(BUZZER, HIGH);
    // Message for debugging purpose
    Serial.println("Object in the middle");
  } else {
    // Indicate that there is not an object
    object = false;
    // Turn off the LED and the buzzer to indicate that there is not an object
    // in the path
    digitalWrite(LED, LOW);
    digitalWrite(BUZZER, LOW);
    // Message for debugging purpose
    Serial.println("There is no object in the middle");
  }
}
