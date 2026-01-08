#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>

// Pin Definitions for Arduino UNO
#define GSM_TX 3  // Connect to GSM RX
#define GSM_RX 2  // Connect to GSM TX

// Emergency Contact
#define EMERGENCY_NUMBER "+923079789986"

// Sensor Objects
MPU6050 mpu(Wire);
Adafruit_BMP280 bmp;
SoftwareSerial gsmSerial(GSM_RX, GSM_TX);

// Fall Detection Thresholds
#define FREEFALL_THRESHOLD 0.6      // g (acceleration magnitude during freefall)
#define IMPACT_THRESHOLD 2.0        // g (acceleration spike on impact)
#define HEIGHT_THRESHOLD 0.20       // meters (20cm)
#define GYRO_THRESHOLD 200          // degrees/second (rotation during fall)
#define STATIONARY_THRESHOLD 0.3    // g (lying still after fall)
#define STATIONARY_TIME 2000        // milliseconds to confirm inactivity

// State Variables
float initialAltitude = 0;
float currentAltitude = 0;
float minAltitude = 0;
bool freefallDetected = false;
bool impactDetected = false;
unsigned long impactTime = 0;
bool fallConfirmed = false;

// Timing
unsigned long lastUpdate = 0;
#define UPDATE_INTERVAL 50  // 50ms = 20Hz sampling

void setup() {
  Serial.begin(9600);
  gsmSerial.begin(9600);
  Wire.begin();
  
  Serial.println(F("=== Fall Detection System with GSM ==="));
  Serial.println(F("Emergency Number: +923079789986"));
  Serial.println(F("I2C: SDA->A4, SCL->A5"));
  Serial.println(F("GSM: TX->Pin2, RX->Pin3"));
  
  // Initialize MPU6050
  Serial.println(F("\nInitializing MPU6050..."));
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  
  if(status != 0) {
    Serial.println(F("MPU6050 FAILED!"));
    Serial.println(F("Check connections:"));
    Serial.println(F("  SDA -> A4"));
    Serial.println(F("  SCL -> A5"));
    Serial.println(F("  VCC -> 5V"));
    Serial.println(F("  GND -> GND"));
    while(1) {
      delay(1000);
    }
  }
  
  Serial.println(F("MPU6050 OK!"));
  Serial.println(F("Calibrating gyro, keep still..."));
  delay(1000);
  mpu.calcGyroOffsets();
  Serial.println(F("MPU6050 calibrated!"));
  
  // Initialize BMP280
  Serial.println(F("\nInitializing BMP280..."));
  if (!bmp.begin(0x76)) {
    Serial.println(F("BMP280 not at 0x76, trying 0x77..."));
    if (!bmp.begin(0x77)) {
      Serial.println(F("BMP280 FAILED!"));
      Serial.println(F("Check connections:"));
      Serial.println(F("  SDA -> A4"));
      Serial.println(F("  SCL -> A5"));
      Serial.println(F("  VCC -> 3.3V (NOT 5V!)"));
      Serial.println(F("  GND -> GND"));
      while(1) {
        delay(1000);
      }
    }
  }
  
  // BMP280 settings for fast response
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);
  
  Serial.println(F("BMP280 OK!"));
  
  // Initialize GSM Module
  Serial.println(F("\nInitializing GSM Module..."));
  delay(1000);
  
  gsmSerial.println("AT");
  delay(1000);
  if(gsmSerial.available()) {
    Serial.println(F("GSM Module responding!"));
    while(gsmSerial.available()) {
      Serial.write(gsmSerial.read());
    }
  } else {
    Serial.println(F("GSM Module not responding!"));
    Serial.println(F("Check connections:"));
    Serial.println(F("  GSM TX -> Arduino Pin 2"));
    Serial.println(F("  GSM RX -> Arduino Pin 3"));
    Serial.println(F("  GSM VCC -> 5V"));
    Serial.println(F("  GSM GND -> GND"));
  }
  
  // Set GSM to text mode
  gsmSerial.println("AT+CMGF=1");
  delay(1000);
  
  // Calibrate initial altitude
  Serial.println(F("\nCalibrating altitude..."));
  Serial.println(F("Keep device still for 2 seconds..."));
  delay(2000);
  initialAltitude = bmp.readAltitude(1013.25);
  minAltitude = initialAltitude;
  
  Serial.println(F("\n========================================"));
  Serial.println(F("       SYSTEM READY"));
  Serial.println(F("========================================"));
  Serial.print(F("Initial Altitude: "));
  Serial.print(initialAltitude, 3);
  Serial.println(F(" m"));
  Serial.print(F("Height Threshold: "));
  Serial.print(HEIGHT_THRESHOLD, 2);
  Serial.println(F(" m (20cm)"));
  Serial.print(F("Emergency Contact: "));
  Serial.println(EMERGENCY_NUMBER);
  Serial.println(F("\nDrop from 20cm+ to test!"));
  Serial.println(F("========================================\n"));
}

void loop() {
  // Update sensors at defined interval
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = millis();
    
    // Update MPU6050
    mpu.update();
    
    // Get acceleration magnitude (in g)
    float accX = mpu.getAccX();
    float accY = mpu.getAccY();
    float accZ = mpu.getAccZ();
    float accMagnitude = sqrt(accX*accX + accY*accY + accZ*accZ);
    
    // Get gyroscope magnitude (in deg/s)
    float gyroX = mpu.getGyroX();
    float gyroY = mpu.getGyroY();
    float gyroZ = mpu.getGyroZ();
    float gyroMagnitude = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ);
    
    // Get altitude
    currentAltitude = bmp.readAltitude(1013.25);
    float relativeAltitude = currentAltitude - initialAltitude;
    
    // Track minimum altitude during freefall
    if (accMagnitude < FREEFALL_THRESHOLD) {
      if (currentAltitude < minAltitude) {
        minAltitude = currentAltitude;
      }
    }
    
    // Print sensor data for monitoring
    Serial.print(F("Acc: "));
    Serial.print(accMagnitude, 2);
    Serial.print(F("g | Gyro: "));
    Serial.print(gyroMagnitude, 1);
    Serial.print(F("°/s | Alt: "));
    Serial.print(relativeAltitude, 3);
    Serial.print(F("m"));
    
    // STAGE 1: FREEFALL DETECTION
    if (!freefallDetected && accMagnitude < FREEFALL_THRESHOLD) {
      freefallDetected = true;
      minAltitude = currentAltitude;
      Serial.print(F(" >>> [FREEFALL!]"));
    }
    
    // STAGE 2: IMPACT DETECTION
    if (freefallDetected && !impactDetected && accMagnitude > IMPACT_THRESHOLD) {
      impactDetected = true;
      impactTime = millis();
      
      // Calculate fall height
      float fallHeight = abs(minAltitude - currentAltitude);
      
      Serial.print(F(" >>> [IMPACT! H="));
      Serial.print(fallHeight, 3);
      Serial.print(F("m]"));
      
      // Check if fall height exceeds threshold
      if (fallHeight >= HEIGHT_THRESHOLD) {
        Serial.print(F(" [OK: >=20cm]"));
      } else {
        Serial.print(F(" [Too low: <20cm]"));
      }
    }
    
    // STAGE 3: POST-IMPACT ANALYSIS
    if (impactDetected && !fallConfirmed) {
      unsigned long timeSinceImpact = millis() - impactTime;
      
      // Check if user is stationary after impact
      if (accMagnitude < (1.0 + STATIONARY_THRESHOLD) && 
          accMagnitude > (1.0 - STATIONARY_THRESHOLD) &&
          gyroMagnitude < GYRO_THRESHOLD / 2) {
        
        if (timeSinceImpact >= STATIONARY_TIME) {
          float fallHeight = abs(minAltitude - currentAltitude);
          
          if (fallHeight >= HEIGHT_THRESHOLD) {
            fallConfirmed = true;
            triggerFallAlert(fallHeight);
          } else {
            Serial.println(F("\n[FALSE: Height below 20cm threshold]"));
            Serial.print(F("  Measured: "));
            Serial.print(fallHeight, 3);
            Serial.print(F("m, Need: "));
            Serial.print(HEIGHT_THRESHOLD, 2);
            Serial.println(F("m"));
            resetFallDetection();
          }
        }
      } else {
        // User is moving, likely recovered - reset
        if (timeSinceImpact > 5000) {
          Serial.println(F("\n[FALSE: Device moved/recovered]"));
          resetFallDetection();
        }
      }
    }
    
    Serial.println();
  }
}

void triggerFallAlert(float height) {
  Serial.println(F("\n\n========================================"));
  Serial.println(F("    *** FALL DETECTED - EMERGENCY ***"));
  Serial.println(F("========================================"));
  Serial.print(F("Fall Height: "));
  Serial.print(height, 3);
  Serial.print(F(" m ("));
  Serial.print(height * 100, 1);
  Serial.println(F(" cm)"));
  Serial.print(F("Time: "));
  Serial.print(millis() / 1000);
  Serial.println(F(" seconds since startup"));
  Serial.println(F("\n>>> SENDING SMS ALERT <<<"));
  Serial.println(F("========================================\n"));
  
  // Send SMS Alert
  sendSMSAlert(height);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("SMS Alert Sent!"));
  Serial.println(F("System auto-resetting..."));
  Serial.println(F("========================================\n"));
  
  delay(2000);
  resetFallDetection();
}

void sendSMSAlert(float height) {
  Serial.print(F("Sending SMS to: "));
  Serial.println(EMERGENCY_NUMBER);
  
  // Set SMS format to text mode
  gsmSerial.println("AT+CMGF=1");
  delay(1000);
  
  // Set phone number
  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(EMERGENCY_NUMBER);
  gsmSerial.println("\"");
  delay(1000);
  
  // Compose message
  gsmSerial.print("EMERGENCY: Fall detected! Fall height: ");
  gsmSerial.print(height * 100, 1);
  gsmSerial.print(" cm. Immediate assistance may be required. Time: ");
  gsmSerial.print(millis() / 1000);
  gsmSerial.print(" sec");
  
  // Send message (Ctrl+Z)
  gsmSerial.write(26);
  delay(5000);
  
  // Print GSM response
  Serial.println(F("GSM Response:"));
  while(gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }
  Serial.println();
}

void resetFallDetection() {
  freefallDetected = false;
  impactDetected = false;
  fallConfirmed = false;
  initialAltitude = currentAltitude;
  minAltitude = currentAltitude;
  
  Serial.println(F(">>> System RESET - Ready for next detection <<<\n"));
}