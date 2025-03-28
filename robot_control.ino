#define BLYNK_TEMPLATE_ID "TMPL3bJWQpdhr"
#define BLYNK_TEMPLATE_NAME "vision based robot"
#define BLYNK_AUTH_TOKEN "EsIw1fV5dn07ss6OtwS6KNOu1b4Bch-s"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>

Servo cameraServo;  // Create servo object
const int SERVO_PIN = 2;  // Servo control pin
int currentServoAngle = 90;  // Initial position at 90 degrees

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "realme C21Y";
char pass[] = "Keerthi@1707";

// Motor control pins
const int IN1 = 4;
const int IN2 = 5;
const int IN3 = 12;
const int IN4 = 13;

// Speed control (PWM) pins
const int ENA = 14;
const int ENB = 15;

int currentSpeed = 128;

// Ultrasonic sensor pins
const int TRIG_PIN = 18;
const int ECHO_PIN = 19;

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// Variables for GPS data
float latitude = 0.0;
float longitude = 0.0;
float speed_kmh = 0.0;

// Variables for ultrasonic sensor
long duration;
int distance_cm;
const int SAFE_DISTANCE = 30;  // Safe distance in centimeters

unsigned long lastGPSUpdate = 0;
const long GPS_UPDATE_INTERVAL = 500;  // Update GPS every 500ms
unsigned long lastUltrasonicUpdate = 0;
const long ULTRASONIC_UPDATE_INTERVAL = 100;  // Update ultrasonic every 100ms

// Static GPS coordinates
const float DEFAULT_LAT = 16.246477;
const float DEFAULT_LONG = 80.408726;
const float DEFAULT_SPEED = 0.0;

unsigned long gpsTimeout = 2000; // 2 second timeout
unsigned long lastValidGPS = 0;

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
        // Check distance before moving forward
        int current_distance = readUltrasonic();
        if (current_distance > SAFE_DISTANCE) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else {
            // Stop if obstacle detected
            stopMotors();
            // Notify through Blynk
            Blynk.virtualWrite(V12, "Obstacle detected!");
        }
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

int readUltrasonic() {
    // Clear the trigger pin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // Send 10μs pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Read the echo pin
    duration = pulseIn(ECHO_PIN, HIGH);
    
    // Calculate distance
    return duration * 0.034 / 2;  // Speed of sound / 2 (round trip)
}

// Function to update GPS data
void updateGPS() {
    bool validGPSData = false;
    
    if (gpsSerial.available() > 0) {
        while (gpsSerial.available() > 0) {
            if (gps.encode(gpsSerial.read())) {
                if (gps.location.isValid()) {
                    latitude = gps.location.lat();
                    longitude = gps.location.lng();
                    validGPSData = true;
                    lastValidGPS = millis();
                    
                    // Send GPS data to Blynk
                    Blynk.virtualWrite(V9, String(latitude, 6));
                    Blynk.virtualWrite(V10, String(longitude, 6));
                }
            }
        }
    }
    
    // Silently use default coordinates if needed
    if (!validGPSData && (millis() - lastValidGPS >= gpsTimeout)) {
        latitude = DEFAULT_LAT;
        longitude = DEFAULT_LONG;
        speed_kmh = DEFAULT_SPEED;
        
        Blynk.virtualWrite(V9, String(latitude, 6));
        Blynk.virtualWrite(V10, String(longitude, 6));
    }
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
    
    // Initialize GPS
    gpsSerial.begin(9600);
    
    // Initialize Ultrasonic pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void loop() {
    Blynk.run();
    
    // Update GPS data every 500ms
    if (millis() - lastGPSUpdate > GPS_UPDATE_INTERVAL) {
        updateGPS();
        lastGPSUpdate = millis();
    }
    
    // Update ultrasonic sensor readings every 100ms
    if (millis() - lastUltrasonicUpdate > ULTRASONIC_UPDATE_INTERVAL) {
        distance_cm = readUltrasonic();
        Blynk.virtualWrite(V12, distance_cm);  // Send distance to Blynk
        
        // Auto-stop if obstacle detected while moving forward
        if (distance_cm <= SAFE_DISTANCE && 
            digitalRead(IN1) == HIGH && digitalRead(IN3) == HIGH) {
            stopMotors();
            Blynk.virtualWrite(V11, "Emergency stop - Obstacle detected!");
        }
        
        lastUltrasonicUpdate = millis();
    }
}