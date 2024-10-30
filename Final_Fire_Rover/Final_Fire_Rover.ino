#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const int MOTOR_IN1 = 22; // Digital pin LIN1
const int MOTOR_IN2 = 19; // Digital pin LIN2

const int MOTOR_IN3 = 15;  // Digital pin RIN3
const int MOTOR_IN4 = 18; // Digital pin RIN4
const int RELAY_IN = 5;
const int flameSensorPin = 33; // Digital pin 
const int gasSensorPin = 35; // Analog pin 

// Ultrasonic sensor pins
const int TRIGGER_PIN = 16;  // Digital pin
const int ECHO_PIN = 17;      // Digital pin

const int TURN_TIME = 800;    // Time in milliseconds for turning

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32S-Bluetooth");
    pinMode(flameSensorPin, INPUT); 
    pinMode(gasSensorPin, INPUT); 
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_IN3, OUTPUT);
    pinMode(MOTOR_IN4, OUTPUT);
    pinMode(RELAY_IN, OUTPUT);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void loop() {
    if (SerialBT.available()) {
        char Direction = SerialBT.read(); 

        if (Direction == 'F') {
            moveForward();
        } else if (Direction == 'B') {
            moveBackward();
        } else if (Direction == 'L') {
            turnLeft();
        } else if (Direction == 'R') {
            turnRight();
        } else if (Direction == 'S') {
            stopBrushMotor();
        }
    } else {
        operateBrushMotor();
        navigateAndAvoidObstacles();

        int flameSensorValue = digitalRead(flameSensorPin);  // Read the value from the flame sensor
        int gasSensorValue = analogRead(gasSensorPin);  // Read the value from the gas sensor

        // Flame detection
        if (flameSensorValue == LOW) {  // Flame detected
            Serial.println("Flame detected!");
            stopBrushMotor();
            Serial.println("Turning relay on...");
            digitalWrite(RELAY_IN, LOW);  // Active-low relay: set to LOW to turn it ON
            delay(3000);  // Relay on for 10 seconds to extinguish fire
            digitalWrite(RELAY_IN, HIGH);  // Turn off relay after 10 seconds
        } else {
            Serial.println("Flame not detected.");  // Print message to the serial monitor
            digitalWrite(RELAY_IN, HIGH);  // Active-low relay: set to HIGH to turn it OFF
        }

        // Gas detection
        if (gasSensorValue > 400) {  // Assuming gas is detected if the value is higher than 400
            Serial.println("Gas detected! Sending hazard alarm...");
            SerialBT.println("Hazard Alarm: Gas detected!");
        }
    }
    
    delay(500);  // Delay to prevent rapid loop execution
}

void navigateAndAvoidObstacles() {
    int frontDistance = measureDistance(TRIGGER_PIN, ECHO_PIN);

    if (frontDistance < 0) {
        Serial.println("Error measuring distance");
        return;
    }
    
    if (frontDistance < 25) { // Threshold distance in centimeters
        Serial.println("Obstacle detected! Stopping brush motor and turning.");
        stopBrushMotor();
        delay(2000);
        turnLeft();  // Or turnRight(); depending on your preference
    } else {
        Serial.println("Path is clear. Continuing operation.");
        operateBrushMotor();
    }
}

int measureDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
    if (duration == 0) {
        Serial.println("Error: No echo received");
        return -1; // Error: No echo received
    }
    int distance = duration * 0.034 / 2;

    Serial.println(distance); // Print distance 
    return distance;  // Return the calculated distance
}

void operateBrushMotor() {
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 125);
    analogWrite(MOTOR_IN3, 0);
    analogWrite(MOTOR_IN4, 125);
}

void moveForward() {
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 125);
    analogWrite(MOTOR_IN3, 0);
    analogWrite(MOTOR_IN4, 125); 
}

void turnLeft() {
    analogWrite(MOTOR_IN1, 125);
    analogWrite(MOTOR_IN2, 0);
    analogWrite(MOTOR_IN3, 0);
    analogWrite(MOTOR_IN4, 125);
    delay(TURN_TIME);
    stopBrushMotor();
}

void moveBackward() {
    analogWrite(MOTOR_IN3, 125);
    analogWrite(MOTOR_IN1, 125);
    analogWrite(MOTOR_IN2, 0);
    analogWrite(MOTOR_IN4, 0);
}

void turnRight() {
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 125);
    analogWrite(MOTOR_IN3, 125);
    analogWrite(MOTOR_IN4, 0);
    delay(TURN_TIME);
}

void stopBrushMotor() {
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
    analogWrite(MOTOR_IN3, 0);
    analogWrite(MOTOR_IN4, 0);
}         