#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

Servo panServo;
Servo tiltServo;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

const int PAN_PIN = 9;
const int TILT_PIN = 10;

// Servo angle limits
const int PAN_MIN = 0;
const int PAN_MAX = 180;
const int TILT_MIN = 0;
const int TILT_MAX = 90;

// Current servo positions
int currentPanServo = 90;
int currentTiltServo = 45;

// Target position
float targetAzimuth = 0;
float targetAltitude = 45;

// PID control parameters
float kp_pan = 2.0;   // Proportional gain for pan
float ki_pan = 0.1;   // Integral gain for pan
float kd_pan = 0.5;   // Derivative gain for pan

float kp_tilt = 2.0;  // Proportional gain for tilt
float ki_tilt = 0.1;  // Integral gain for tilt
float kd_tilt = 0.5;  // Derivative gain for tilt

// PID variables
float errorSum_pan = 0;
float lastError_pan = 0;
float errorSum_tilt = 0;
float lastError_tilt = 0;

// Control loop timing
unsigned long lastUpdate = 0;
const int UPDATE_INTERVAL = 50; // ms

// Positioning tolerance
const float POSITION_TOLERANCE = 0.5; // degrees

// Serial buffer
String inputString = "";
boolean stringComplete = false;

// IMU calibration offset
float azimuthOffset = 0;
float altitudeOffset = 0;

void setup() {
  Serial.begin(9600);
  
  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("ERROR: No BNO055 detected. Check wiring!");
    while (1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  
  // Attach servos
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  
  // Move to home position
  moveToHome();
  
  Serial.println("Arduino Servo Controller with BNO055 Ready");
  Serial.println("Commands: MOVE:az,alt | HOME | STATUS | CALIBRATE | GET_ORIENTATION");
  
  // Display initial calibration status
  displayCalibrationStatus();
}

void loop() {
  // Process serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Run position control loop
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = millis();
    updatePosition();
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

void processCommand(String command) {
  command.trim();
  
  if (command.startsWith("MOVE:")) {
    String params = command.substring(5);
    int commaIndex = params.indexOf(',');
    
    if (commaIndex > 0) {
      targetAzimuth = params.substring(0, commaIndex).toFloat();
      targetAltitude = params.substring(commaIndex + 1).toFloat();
      
      // Reset PID
      errorSum_pan = 0;
      errorSum_tilt = 0;
      lastError_pan = 0;
      lastError_tilt = 0;
      
      Serial.print("OK: Target set - Az=");
      Serial.print(targetAzimuth, 2);
      Serial.print("° Alt=");
      Serial.print(targetAltitude, 2);
      Serial.println("°");
    }
  }
  else if (command == "HOME") {
    moveToHome();
  }
  else if (command == "STATUS") {
    sendStatus();
  }
  else if (command == "CALIBRATE") {
    calibrateIMU();
  }
  else if (command == "GET_ORIENTATION") {
    getOrientation();
  }
  else {
    Serial.println("ERROR: Unknown command");
  }
}

void updatePosition() {
  // Get current orientation from IMU
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  // Current azimuth (heading) and altitude (pitch)
  float currentAzimuth = orientationData.orientation.x - azimuthOffset;
  float currentAltitude = orientationData.orientation.y - altitudeOffset;
  
  // Normalize azimuth to 0-360
  if (currentAzimuth < 0) currentAzimuth += 360;
  if (currentAzimuth >= 360) currentAzimuth -= 360;
  
  // Calculate errors
  float error_pan = calculateAzimuthError(targetAzimuth, currentAzimuth);
  float error_tilt = targetAltitude - currentAltitude;
  
  // Check if we're close enough
  if (abs(error_pan) < POSITION_TOLERANCE && abs(error_tilt) < POSITION_TOLERANCE) {
    return; // Already at target
  }
  
  // PID control for pan
  errorSum_pan += error_pan;
  errorSum_pan = constrain(errorSum_pan, -100, 100); // Anti-windup
  float derivative_pan = error_pan - lastError_pan;
  float correction_pan = kp_pan * error_pan + ki_pan * errorSum_pan + kd_pan * derivative_pan;
  lastError_pan = error_pan;
  
  // PID control for tilt
  errorSum_tilt += error_tilt;
  errorSum_tilt = constrain(errorSum_tilt, -100, 100); // Anti-windup
  float derivative_tilt = error_tilt - lastError_tilt;
  float correction_tilt = kp_tilt * error_tilt + ki_tilt * errorSum_tilt + kd_tilt * derivative_tilt;
  lastError_tilt = error_tilt;
  
  // Apply corrections to servo positions
  currentPanServo += correction_pan;
  currentTiltServo += correction_tilt;
  
  // Constrain servo angles
  currentPanServo = constrain(currentPanServo, PAN_MIN, PAN_MAX);
  currentTiltServo = constrain(currentTiltServo, TILT_MIN, TILT_MAX);
  
  // Write to servos
  panServo.write(currentPanServo);
  tiltServo.write(currentTiltServo);
}

float calculateAzimuthError(float target, float current) {
  // Handle 0/360 degree wraparound
  float error = target - current;
  
  if (error > 180) {
    error -= 360;
  } else if (error < -180) {
    error += 360;
  }
  
  return error;
}

void moveToHome() {
  targetAzimuth = 0;
  targetAltitude = 45;
  currentPanServo = 90;
  currentTiltServo = 45;
  
  panServo.write(currentPanServo);
  tiltServo.write(currentTiltServo);
  
  errorSum_pan = 0;
  errorSum_tilt = 0;
  
  Serial.println("OK: Moving to home position");
}

void calibrateIMU() {
  Serial.println("Calibrating IMU offsets...");
  
  // Get current orientation
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  // Store current orientation as offset (assuming mount is at known position)
  azimuthOffset = orientationData.orientation.x;
  altitudeOffset = orientationData.orientation.y;
  
  Serial.print("OK: Calibration complete - Az offset=");
  Serial.print(azimuthOffset, 2);
  Serial.print("° Alt offset=");
  Serial.print(altitudeOffset, 2);
  Serial.println("°");
  
  displayCalibrationStatus();
}

void getOrientation() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  float azimuth = orientationData.orientation.x - azimuthOffset;
  float altitude = orientationData.orientation.y - altitudeOffset;
  
  if (azimuth < 0) azimuth += 360;
  
  Serial.print("ORIENTATION: Az=");
  Serial.print(azimuth, 2);
  Serial.print("° Alt=");
  Serial.print(altitude, 2);
  Serial.print("° Roll=");
  Serial.print(orientationData.orientation.z, 2);
  Serial.println("°");
}

void sendStatus() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  float currentAz = orientationData.orientation.x - azimuthOffset;
  float currentAlt = orientationData.orientation.y - altitudeOffset;
  
  if (currentAz < 0) currentAz += 360;
  
  Serial.print("STATUS: Target Az=");
  Serial.print(targetAzimuth, 2);
  Serial.print("° Alt=");
  Serial.print(targetAltitude, 2);
  Serial.print("° | Current Az=");
  Serial.print(currentAz, 2);
  Serial.print("° Alt=");
  Serial.print(currentAlt, 2);
  Serial.print("° | Servo Pan=");
  Serial.print(currentPanServo);
  Serial.print("° Tilt=");
  Serial.print(currentTiltServo);
  Serial.println("°");
}

void displayCalibrationStatus() {
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  Serial.println("BNO055 Calibration Status:");
  Serial.print("  System: "); Serial.print(system);
  Serial.print("  Gyro: "); Serial.print(gyro);
  Serial.print("  Accel: "); Serial.print(accel);
  Serial.print("  Mag: "); Serial.println(mag);
  Serial.println("  (3 = fully calibrated for each sensor)");
}
