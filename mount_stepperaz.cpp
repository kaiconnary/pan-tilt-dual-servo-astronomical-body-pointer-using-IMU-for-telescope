#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <AccelStepper.h>

Servo tiltServo;

AccelStepper panStepper(AccelStepper::DRIVER, 2, 3);  // STEP pin 2, DIR pin 3

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

const int STEPPER_STEP_PIN = 2;
const int STEPPER_DIR_PIN = 3;
const int STEPPER_ENABLE_PIN = 4;  
const int TILT_SERVO_PIN = 10;

// Stepper motor configuration
const int STEPS_PER_REVOLUTION = 200;  
const int MICROSTEPS = 16;             
const float GEAR_RATIO = 1.0;          
const float STEPS_PER_DEGREE = (STEPS_PER_REVOLUTION * MICROSTEPS * GEAR_RATIO) / 360.0;

const float MAX_SPEED = 2000.0;        
const float ACCELERATION = 1000.0;     

const int TILT_MIN = 0;
const int TILT_MAX = 90;

long currentPanSteps = 0;
int currentTiltServo = 45;

float targetAzimuth = 0;
float targetAltitude = 45;

float kp_pan = 8.0;    
float ki_pan = 0.2;    
float kd_pan = 1.0;    

float kp_tilt = 2.0;   
float ki_tilt = 0.1;   
float kd_tilt = 0.5;   

float errorSum_pan = 0;
float lastError_pan = 0;
float errorSum_tilt = 0;
float lastError_tilt = 0;

unsigned long lastUpdate = 0;
const int UPDATE_INTERVAL = 50; // ms

const float POSITION_TOLERANCE = 0.3; 

String inputString = "";
boolean stringComplete = false;

float azimuthOffset = 0;
float altitudeOffset = 0;

bool isMoving = false;

void setup() {
  Serial.begin(9600);
  if (!bno.begin()) {
    Serial.println("ERROR: No BNO055 detected. Check wiring!");
    while (1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, LOW);  
  
  panStepper.setMaxSpeed(MAX_SPEED);
  panStepper.setAcceleration(ACCELERATION);
  panStepper.setCurrentPosition(0); 
  
  tiltServo.attach(TILT_SERVO_PIN);
  
  moveToHome();
  
  Serial.println("Arduino Stepper/Servo Controller with BNO055 Ready");
  Serial.print("Stepper: ");
  Serial.print(STEPS_PER_DEGREE, 2);
  Serial.println(" steps/degree");
  Serial.println("Commands: MOVE:az,alt | HOME | STATUS | CALIBRATE | GET_ORIENTATION");
  
  displayCalibrationStatus();
}

void loop() {
  // Process serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  if (panStepper.distanceToGo() != 0) {
    panStepper.run();
    isMoving = true;
  } else {
    isMoving = false;
  }  
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
  else if (command == "ENABLE") {
    digitalWrite(STEPPER_ENABLE_PIN, LOW);
    Serial.println("OK: Stepper enabled");
  }
  else if (command == "DISABLE") {
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);
    Serial.println("OK: Stepper disabled");
  }
  else {
    Serial.println("ERROR: Unknown command");
  }
}

void updatePosition() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  float currentAzimuth = orientationData.orientation.x - azimuthOffset;
  float currentAltitude = orientationData.orientation.y - altitudeOffset;
  
  // Normalize azimuth to 0-360
  if (currentAzimuth < 0) currentAzimuth += 360;
  if (currentAzimuth >= 360) currentAzimuth -= 360;
  
  float error_pan = calculateAzimuthError(targetAzimuth, currentAzimuth);
  float error_tilt = targetAltitude - currentAltitude;
  
  if (abs(error_pan) < POSITION_TOLERANCE && abs(error_tilt) < POSITION_TOLERANCE) {
    if (panStepper.distanceToGo() == 0) {
      return; // Already at target and stopped
    }
  }
  errorSum_pan += error_pan;
  errorSum_pan = constrain(errorSum_pan, -100, 100); // Anti-windup
  float derivative_pan = error_pan - lastError_pan;
  float correction_pan = kp_pan * error_pan + ki_pan * errorSum_pan + kd_pan * derivative_pan;
  lastError_pan = error_pan;
  
  if (abs(error_pan) > POSITION_TOLERANCE) {
    long targetSteps = panStepper.currentPosition() + (long)(correction_pan * STEPS_PER_DEGREE);
    panStepper.moveTo(targetSteps);
  }
  errorSum_tilt += error_tilt;
  errorSum_tilt = constrain(errorSum_tilt, -100, 100); // Anti-windup
  float derivative_tilt = error_tilt - lastError_tilt;
  float correction_tilt = kp_tilt * error_tilt + ki_tilt * errorSum_tilt + kd_tilt * derivative_tilt;
  lastError_tilt = error_tilt;
  
  if (abs(error_tilt) > POSITION_TOLERANCE) {
    currentTiltServo += correction_tilt;
    currentTiltServo = constrain(currentTiltServo, TILT_MIN, TILT_MAX);
    tiltServo.write(currentTiltServo);
  }
}
float calculateAzimuthError(float target, float current) {
  float error = target - current;
  
  if (error > 180) {
    error -= 360;
  } else if (error < -180) {
    error += 360;
  }
  
  return error;
}

void moveToHome() {
  Serial.println("Moving to home position...");
  
  targetAzimuth = 0;
  targetAltitude = 45;
  
  panStepper.setCurrentPosition(0);
  currentPanSteps = 0;
  
  currentTiltServo = 45;
  tiltServo.write(currentTiltServo);
  
  errorSum_pan = 0;
  errorSum_tilt = 0;
  
  Serial.println("OK: Home position");
}

void calibrateIMU() {
  Serial.println("Calibrating IMU offsets...");
  
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
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
  
  float stepperAngle = panStepper.currentPosition() / STEPS_PER_DEGREE;
  
  Serial.print("STATUS: Target Az=");
  Serial.print(targetAzimuth, 2);
  Serial.print("° Alt=");
  Serial.print(targetAltitude, 2);
  Serial.print("° | Current Az=");
  Serial.print(currentAz, 2);
  Serial.print("° Alt=");
  Serial.print(currentAlt, 2);
  Serial.print("° | Stepper=");
  Serial.print(panStepper.currentPosition());
  Serial.print(" steps (");
  Serial.print(stepperAngle, 2);
  Serial.print("°) Servo=");
  Serial.print(currentTiltServo);
  Serial.print("° | Moving=");
  Serial.println(isMoving ? "YES" : "NO");
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
