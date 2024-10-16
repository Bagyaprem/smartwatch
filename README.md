# smartwatch
#its all about the pedometer accurate  steps count
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial ss(4, 3); // RX, TX for GPS module

int stepCount = 0;
bool lastStepDetected = false;
bool isCountingSteps = true;

void setup() {
  Serial.begin(115200);
  ss.begin(9600);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("MPU6050 Initialized.");
}

void loop() {
  // Read GPS data
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  // Check if the GPS speed is above 5 km/h
  float speedKmh = gps.speed.kmph();
  if (speedKmh > 5.0) {
    isCountingSteps = false; // Stop counting steps
    Serial.println("GPS speed above 5 km/h. Step counting stopped.");
  } else {
    isCountingSteps = true; // Resume counting steps if speed is below threshold
  }

  if (isCountingSteps) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate acceleration magnitude (sqrt of x^2 + y^2 + z^2)
    float accelerationMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                                       a.acceleration.y * a.acceleration.y +
                                       a.acceleration.z * a.acceleration.z);

    // Step detection based on threshold
    if (accelerationMagnitude > 1.5 && !lastStepDetected) {
      stepCount++;
      lastStepDetected = true;
      Serial.print("Step detected! Total steps: ");
      Serial.println(stepCount);
    } 
    if (accelerationMagnitude < 1.2) {
      lastStepDetected = false;  // Reset step detection flag
    }
  }

  delay(100); // Adjust this delay for sensitivity
}
