#define BLYNK_TEMPLATE_ID "TMPL3bJWQpdhr"
#define BLYNK_TEMPLATE_NAME "vision based robot"
#define BLYNK_AUTH_TOKEN "EsIw1fV5dn07ss6OtwS6KNOu1b4Bch-s"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Servo.h>

Servo cameraServo;  // Create servo object
const int SERVO_PIN = 2;  // Servo control pin
int currentServoAngle = 90;  // Initial position at 90 degrees

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "POCO X4 5G";
char pass[] = "1234567890";

// Motor control pins
const int IN1 = 4;
const int IN2 = 5;
const int IN3 = 12;
const int IN4 = 13;

// Speed control (PWM) pins
const int ENA = 14;
const int ENB = 15;

int currentSpeed = 128;

// Camera Servo Control (V7)
BLYNK_WRITE(V7) {
    int angle = param.asInt();
    if (angle >= 0 && angle <= 180) {
        currentServoAngle = angle;
        cameraServo.write(currentServoAngle);
    }
}

// Quick Camera Position Controls (V8)
BLYNK_WRITE(V8) {
    int position = param.asInt();
    switch(position) {
        case 1: // Left position
            currentServoAngle = 0;
            break;
        case 2: // Center position
            currentServoAngle = 90;
            break;
        case 3: // Right position
            currentServoAngle = 180;
            break;
    }
    cameraServo.write(currentServoAngle);
}

// Forward (V0)
BLYNK_WRITE(V0) {
    int state = param.asInt();
    if (state) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        stopMotors();
    }
    updateStatus();
}

// Left (V1)
BLYNK_WRITE(V1) {
    int state = param.asInt();
    if (state) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        stopMotors();
    }
    updateStatus();
}

// Stop (V2)
BLYNK_WRITE(V2) {
    stopMotors();
    updateStatus();
}

// Right (V3)
BLYNK_WRITE(V3) {
    int state = param.asInt();
    if (state) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    } else {
        stopMotors();
    }
    updateStatus();
}

// Backward (V4)
BLYNK_WRITE(V4) {
    int state = param.asInt();
    if (state) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    } else {
        stopMotors();
    }
    updateStatus();
}

// Speed Control (V5)
BLYNK_WRITE(V5) {
    currentSpeed = param.asInt();
    analogWrite(ENA, currentSpeed);
    analogWrite(ENB, currentSpeed);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void updateStatus() {
    bool isMoving = (digitalRead(IN1) || digitalRead(IN2) || 
                     digitalRead(IN3) || digitalRead(IN4));
    Blynk.virtualWrite(V6, isMoving ? 1 : 0);
}

void setup() {
    Serial.begin(115200);
    
    // Configure motor control pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Configure PWM pins
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    
    // Set initial speed
    analogWrite(ENA, currentSpeed);
    analogWrite(ENB, currentSpeed);
    
    // Initialize servo
    cameraServo.attach(SERVO_PIN);
    cameraServo.write(currentServoAngle);  // Set to initial position (90 degrees)
    
    Blynk.begin(auth, ssid, pass);
}

void loop() {
    Blynk.run();
}
