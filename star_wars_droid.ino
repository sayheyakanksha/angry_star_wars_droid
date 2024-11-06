/***
Code Written: Akanksha
Imperal March Song Credits: Robson Couto, 2019
**/

#include <AFMotor.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// Define motors
AF_DCMotor motor1(1); // Front-left
AF_DCMotor motor2(2); // Front-right
AF_DCMotor motor3(3); // Back-left
AF_DCMotor motor4(4); // Back-right

// Bluetooth and Servo setup
SoftwareSerial BTSerial(2, 9); // RX, TX
Servo myServo;
const int servoPin = 10; // Servo pin

int servoPos = 100;        // Starting position of the servo
bool sweepDirection = true; // Direction of servo movement
unsigned long lastServoUpdate = 0;
const int servoInterval = 40; // Update servo every 40 ms

// Ultrasonic Sensor Pins
const int trigPin = A1;
const int echoPin = A2;
unsigned long lastDistanceCheck = 0;
const int distanceInterval = 100; // Check every 100 ms
const int detectionRange = 15; // Detection range in cm

// Smoothing Variables for Object Detection
int detectionCount = 0;
const int detectionThreshold = 3; // Number of consecutive detections needed

// Motor state variables
bool isMotorMoving = false;
char lastMotorCommand = 'S'; // 'S' for stopped

// Buzzer setup
const int buzzerPin = A0; // Buzzer pin

// Melody constants
#define NOTE_A4  440
#define NOTE_C5  523
#define NOTE_E5  659
#define NOTE_F4  349
#define NOTE_F5  698

// Song arrays and variables
int melody[] = {
  NOTE_A4,4, NOTE_A4,4, NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16,
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
  NOTE_E5,4, NOTE_E5,4, NOTE_E5,4, NOTE_F5,-8, NOTE_C5,16,
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
};
int tempo = 120;
int notes = sizeof(melody) / sizeof(melody[0]) / 2;
int wholenote = (60000 * 4) / tempo;
int divider = 0, noteDuration = 0;

void setup() {
  Serial.begin(38400);
  BTSerial.begin(38400);

  // Initialize motors
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor3.setSpeed(200);
  motor4.setSpeed(200);

  // Initialize pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize servo
  myServo.attach(servoPin);
  myServo.write(servoPos);

  Serial.println("Omniwheel Robot Ready for Commands and Object Detection");
}

void loop() {
  unsigned long currentMillis = millis();

  // Servo sweeping logic
  if (currentMillis - lastServoUpdate >= servoInterval) {
    sweepServo();
    lastServoUpdate = currentMillis;
  }

  // Handle Bluetooth commands
  if (BTSerial.available()) {
    char command = toupper(BTSerial.read());
    // Serial.print("Command received: ");
    // Serial.println(command);

    executeCommand(command); // Execute the motor command
  }

  // Object detection logic with smoothing
  if (currentMillis - lastDistanceCheck >= distanceInterval) {
    lastDistanceCheck = currentMillis;
    
    if (isObjectDetected()) {
      detectionCount++;
      if (detectionCount >= detectionThreshold) {
        stopMotors(); // Stop motors if object is detected
        isMotorMoving = false; // Indicate that motors are stopped
      }
    } else {
      detectionCount = 0; // Reset count if no object is detected
      if (!isMotorMoving && lastMotorCommand != 'S') {
        // Resume the last motor command if no object is detected and last command was a movement command
        executeCommand(lastMotorCommand); // Attempt to resume last movement
        isMotorMoving = true;
      }
    }
  }
}

// Function to execute motor commands
void executeCommand(char command) {
  if (command == 'Y') {
    stopMotors();               // Stop motors before playing sound
    honkBuzzer();                // Play honk sound
  } 
  else if (command == 'Z') {
    stopMotors();               // Stop motors before playing sound
    playImperialMarch();         // Play Imperial March
  } 
  else {
    switch (command) {
      case 'F': 
        moveForward(); 
        isMotorMoving = true; 
        lastMotorCommand = command;  // Store last movement command
        break;
      case 'B': 
        moveBackward(); 
        isMotorMoving = true; 
        lastMotorCommand = command;  // Store last movement command
        break;
      case 'L': 
        moveLeft(); 
        isMotorMoving = true; 
        lastMotorCommand = command;  // Store last movement command
        break;
      case 'R': 
        moveRight(); 
        isMotorMoving = true; 
        lastMotorCommand = command;  // Store last movement command
        break;
      case 'U': 
        moveDiagonal("UL"); 
        isMotorMoving = true; 
        lastMotorCommand = command;  // Store last movement command
        break;
      case 'V': 
        moveDiagonal("UR"); 
        isMotorMoving = true; 
        lastMotorCommand = command;  // Store last movement command
        break;
      case 'W': 
        moveDiagonal("LL"); 
        isMotorMoving = true; 
        lastMotorCommand = command;  // Store last movement command
        break;
      case 'X': 
        moveDiagonal("LR"); 
        isMotorMoving = true; 
        lastMotorCommand = command;  // Store last movement command
        break;
      case 'S': 
        stopMotors(); 
        isMotorMoving = false; 
        lastMotorCommand = 'S';  // Indicate that bot is stopped
        break;
      default: 
        Serial.println("Invalid Command"); 
        stopMotors(); 
        isMotorMoving = false; 
        lastMotorCommand = 'S';  // Indicate stop for invalid command
        break;
    }
  }
}


// Function to check if an object is within the detection range
bool isObjectDetected() {
  long duration;
  int distance;

  // Trigger ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo
  duration = pulseIn(echoPin, HIGH, 20000); // Timeout after 20 ms
  distance = duration * 0.034 / 2; // Convert to cm

  // Debugging
  //Serial.print("Distance: ");
  //Serial.print(distance);
  //Serial.println(" cm");

  return (distance > 0 && distance < detectionRange);
}

// Movement functions
void moveForward() {
  //Serial.println("Moving Forward");
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void moveBackward() {
  // Serial.println("Moving Backward");
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveLeft() {
  // Serial.println("Moving Left");
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
}

void moveRight() {
  // Serial.println("Moving Right");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
}

void moveDiagonal(String direction) {
  if (direction == "UL") {
    // Serial.println("Moving Upper Left");
    motor1.run(BACKWARD);
    motor2.run(RELEASE);
    motor3.run(BACKWARD);
    motor4.run(RELEASE);
  } else if (direction == "UR") {
    // Serial.println("Moving Upper Right");
    motor1.run(RELEASE);
    motor2.run(FORWARD);
    motor3.run(RELEASE);
    motor4.run(BACKWARD);
  } else if (direction == "LL") {
    // Serial.println("Moving Lower Left");
    motor1.run(RELEASE);
    motor2.run(BACKWARD);
    motor3.run(RELEASE);
    motor4.run(FORWARD);
  } else if (direction == "LR") {
    // Serial.println("Moving Lower Right");
    motor1.run(FORWARD);
    motor2.run(RELEASE);
    motor3.run(FORWARD);
    motor4.run(RELEASE);
  }
}

void stopMotors() {
  // Serial.println("Stopping Motors");
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
  isMotorMoving = false;
}

// Function for "beep-beep" honk sound
void honkBuzzer() {
  // Serial.println("Honking Buzzer");
  tone(buzzerPin, NOTE_A4); // Short beep
  delay(200);
  noTone(buzzerPin);
  delay(10); // Brief delay to stabilize Timer 2
  delay(200);
  
  tone(buzzerPin, NOTE_A4); // Long beep
  delay(600);
  noTone(buzzerPin);
  delay(10); // Brief delay to stabilize Timer 2
}

// Function to play Imperial March
void playImperialMarch() {
  // Serial.println("Playing Imperial March");
  int noteDuration = 0;

  for (int thisNote = 0; thisNote < notes * 2; thisNote += 2) {
    int divider = melody[thisNote + 1];
    noteDuration = (divider > 0) ? (wholenote / divider) : (wholenote / abs(divider) * 1.5);
    tone(buzzerPin, melody[thisNote], noteDuration * 0.9);
    delay(noteDuration);
    noTone(buzzerPin);
    delay(10); // Brief delay to stabilize Timer 2
  }
}

// Servo sweeping function
void sweepServo() {
  if (sweepDirection) {
    servoPos++;
    if (servoPos >= 160) {
      sweepDirection = false;
    }
  } else {
    servoPos--;
    if (servoPos <= 120) {
      sweepDirection = true;
    }
  }
  myServo.write(servoPos);
}
