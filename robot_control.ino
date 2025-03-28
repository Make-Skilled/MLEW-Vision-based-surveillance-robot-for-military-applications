#define BLYNK_TEMPLATE_ID "TMPL3ei0ExLSp"
#define BLYNK_TEMPLATE_NAME "vision robot"
#define BLYNK_AUTH_TOKEN "tXb_c-bePpPoSqBNZgrhQyrP061tylU6"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <esp_task_wdt.h> // For watchdog timer
#include <ThingSpeak.h>

WiFiClient client;

// ThingSpeak settings
int channel = 2896841;
const char* apikey = "VIHKX76IJ2IEXR87";

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
HardwareSerial gpsSerial(1); // Use Serial1 for GPS

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
unsigned long lastThingSpeakUpdate = 0;
const long THINGSPEAK_UPDATE_INTERVAL = 20000;  // Update ThingSpeak every 20 seconds

// Static GPS coordinates
const float DEFAULT_LAT = 16.246477;
const float DEFAULT_LONG = 80.408726;
const float DEFAULT_SPEED = 0.0;

unsigned long gpsTimeout = 2000; // 2 second timeout
unsigned long lastValidGPS = 0;

// WiFi and Blynk reconnection variables
unsigned long lastWiFiCheck = 0;
const long WIFI_CHECK_INTERVAL = 5000; // Check WiFi every 5 seconds

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
        int current_distance = readUltrasonic();
        if (current_distance > SAFE_DISTANCE) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else {
            stopMotors();
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
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    esp_task_wdt_reset(); // Reset watchdog during potentially long operation
    return duration * 0.034 / 2;
}

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
                    ThingSpeak.setField(3, String(latitude, 6));
                    ThingSpeak.setField(4, String(longitude, 6));
                }
            }
            esp_task_wdt_reset(); // Reset watchdog during GPS reading
        }
    }
    
    if (!validGPSData && (millis() - lastValidGPS >= gpsTimeout)) {
        latitude = DEFAULT_LAT;
        longitude = DEFAULT_LONG;
        speed_kmh = DEFAULT_SPEED;
        ThingSpeak.setField(3, String(latitude, 6));
        ThingSpeak.setField(4, String(longitude, 6));
    }
}

// Function to connect to WiFi with timeout
bool connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    Serial.print("Connecting to WiFi");
    
    unsigned long startAttemptTime = millis();
    const unsigned long timeout = 10000; // 10 seconds timeout
    
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - startAttemptTime >= timeout) {
            Serial.println("\nFailed to connect to WiFi within timeout!");
            return false;
        }
        delay(500);
        Serial.print(".");
        esp_task_wdt_reset(); // Reset watchdog during delay
    }
    Serial.println("\nConnected!");
    return true;
}

// Function to check and reconnect WiFi and Blynk
void checkConnections() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected! Attempting to reconnect...");
        connectWiFi();
    }
    
    if (WiFi.status() == WL_CONNECTED && !Blynk.connected()) {
        Serial.println("Blynk disconnected! Attempting to reconnect...");
        Blynk.connect();
    }
}

void setup() {
    Serial.begin(115200); // Increased baud rate for faster debugging

    // Initialize watchdog timer (10 seconds timeout)
    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL);

    // Attempt initial WiFi connection
    if (!connectWiFi()) {
        Serial.println("Initial WiFi connection failed. Continuing anyway...");
    }

    // Configure Blynk (non-blocking)
    Blynk.config(auth);
    if (WiFi.status() == WL_CONNECTED) {
        Blynk.connect();
    }

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
    cameraServo.write(currentServoAngle);
    
    // Initialize GPS with specific RX/TX pins
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
    
    // Initialize Ultrasonic pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Initialize ThingSpeak
    ThingSpeak.begin(client);
}

void loop() {
    // Reset watchdog timer
    esp_task_wdt_reset();

    // Run Blynk if connected
    if (Blynk.connected()) {
        Blynk.run();
    }

    // Check WiFi and Blynk connections periodically
    if (millis() - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
        checkConnections();
        lastWiFiCheck = millis();
    }
    
    // Update GPS data every 500ms
    if (millis() - lastGPSUpdate > GPS_UPDATE_INTERVAL) {
        updateGPS();
        lastGPSUpdate = millis();
    }
    
    // Update ultrasonic sensor readings every 100ms
    if (millis() - lastUltrasonicUpdate > ULTRASONIC_UPDATE_INTERVAL) {
        distance_cm = readUltrasonic();
        ThingSpeak.setField(1, distance_cm);
        
        if (distance_cm <= SAFE_DISTANCE && 
            digitalRead(IN1) == HIGH && digitalRead(IN3) == HIGH) {
            stopMotors();
            ThingSpeak.setField(2, 1);
        } else {
            ThingSpeak.setField(2, 0);
        }
        
        lastUltrasonicUpdate = millis();
    }

    // Send data to ThingSpeak every 20 seconds
    if (millis() - lastThingSpeakUpdate > THINGSPEAK_UPDATE_INTERVAL) {
        if (WiFi.status() == WL_CONNECTED) {
            int statusCode = ThingSpeak.writeFields(channel, apikey);
            if (statusCode == 200) {
                Serial.println("Data sent to ThingSpeak Channel ID: " + String(channel));
            } else {
                Serial.println("Failed to send data to ThingSpeak. Status code: " + String(statusCode));
            }
        }
        lastThingSpeakUpdate = millis();
    }

    // Small delay to prevent tight loops, but short enough to avoid watchdog issues
    delay(10);
}