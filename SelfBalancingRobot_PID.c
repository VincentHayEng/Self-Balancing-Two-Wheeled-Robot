/*============================================================================
    Single-Axis Two-Wheeled Self Balancing Robot
    The Kontrol Systers
    Justin D, Vincent H, and Jaideep V
============================================================================*/

/*============================================================================  
    Uses SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library
    https://github.com/sparkfun/SparkFun_Qwiic_6DoF_LSM6DSO_Arduino_Library
    Library licensed under MIT License by SparkFun Electronics
============================================================================*/

#include "SparkFunLSM6DSO.h"
#include "Wire.h"

LSM6DSO myIMU;

// Initialize motor pins
const int motorBackA = 4; // to IN1 for motor driver A
const int motorBackB = 5; // to IN2 for motor driver B
const int motorForwardA = 3; // to IN1 for motor driver A
const int motorForwardB = 6; // to In2 for motor driver B

// Motor factors
const float motorFactorA = 1.0;
const float motorFactorB = 1.0;

// Define PID variables
float kp  = 29;
float ki = 0.2;
float kd = 0.65;

// Integral constrain
float integralConstrain = 101;

// Defining reading biases
float gyroBias = 0.9;
float accelBias = 0.1;
float correctionBias = 0.99;

// Setting lowerPIDLimit
float lowerPIDLimit = 60;

// Initialize PID variables
float desiredTilt = 0;
float input = 0; // Calculated angle
float output = 0; // PID output
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

// Initializing IMU variables
float angleXGyro = 0;
float angleAccel = 0;
float angle = 0;

// Initialize gyro calibration bias variable
float gyroBiasX = 0;

// Initializing timing variable
unsigned long lastTime;

void setup()
{
  // Serial and Wire setup
  Serial.begin(9600);
  Wire.begin();
  delay(10);

  // Check if IMU is connected
  if (myIMU.begin()) Serial.println("IMU Ready");
  myIMU.initialize(BASIC_SETTINGS);

  // Calibrate X axis gyroscope
  calibrateGyro();

  lastTime = micros();

  // Set motor pins as output
  pinMode(motorBackA, OUTPUT);
  pinMode(motorBackB, OUTPUT);
  pinMode(motorForwardA, OUTPUT);
  pinMode(motorForwardB, OUTPUT);
}

void loop() 
{
  // Computing the change over time
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastTime) / 1e6;
  lastTime = currentTime;

  // IMU readings
  float zAccel = myIMU.readFloatAccelZ();
  float yAccel = myIMU.readFloatAccelY();
  float gyroXrate = myIMU.readFloatGyroX() - gyroBiasX;

  // Calculations for tilt angle
  angleXGyro += gyroXrate * deltaTime; // Integrating for angle reading
  angleAccel = atan2(yAccel, zAccel) * 180.0 / PI; // Converting angle found from acceleration to radians
  angleXGyro = angleXGyro * correctionBias + angleAccel * (1 - correctionBias); // Preventing drift from gyro readings
  angle = gyroBias * angleXGyro + accelBias * angleAccel; // Weighted summation for angle

  if ( zAccel > 0.999 && zAccel < 1.001 ) // If the Z acceleration detects the robot is upright, the angle is 0
  {
    angle = 0;
    input = angle;
  } 
  else // Else, angle is the calculated value
  {
    input = angle;
  }

  // Compute the PID
  computePID(deltaTime);

  // 
  controlMotors(output);

  // Printing for debugging and tuning
  Serial.print("Angleyz: "); Serial.print(angleAccel);
  Serial.print("\gyro: "); Serial.print(gyroXrate);
  Serial.print("\tAngle: "); Serial.print(angle);
  Serial.print("\tPID: "); Serial.println(output);
}

void calibrateGyro()
{
  // Initializing calibration variables
  // ROBOT MUST BE PERFECTLY UP RIGHT
  long numberOfSamples = 2000;
  float sum = 0;

  // Get samples
  for (int i = 0; i < numberOfSamples; i++)
  {
    // Add to sum
    sum += myIMU.readFloatGyroX();
    delay(1);
  }

  // Compute average reading
  gyroBiasX = sum / numberOfSamples;
  // Print the bias
  Serial.print("Gyro bias X = ");
  Serial.println(gyroBiasX);
}

void computePID(float deltaTime) 
{
  // Calculate error 
  error = desiredTilt - input;

  // Compute the integral
  integral += error * deltaTime;
  integral = constrain(integral, -integralConstrain, integralConstrain);

  // Compute derivative
  derivative = (error - lastError) / deltaTime;

  // Update lastError
  lastError = error;

  // Calculate PID output
  output = kp * error + ki * integral + kd * derivative;

  // Limit output to PWM min and max
  output = constrain(output, -255, 255);
}

void controlMotors(float pidOutput)
{
  if( pidOutput > lowerPIDLimit ) // If the robot is tilting forwards
  {
    analogWrite(motorForwardA, (int)pidOutput * motorFactorA);
    analogWrite(motorForwardB, (int)pidOutput * motorFactorB);
    digitalWrite(motorBackA, LOW);
    digitalWrite(motorBackB, LOW);
  }
  else if(pidOutput < -lowerPIDLimit) // If the robot is tilting backwards 
  { 
    analogWrite(motorBackA, (int)abs(pidOutput) * motorFactorA);
    analogWrite(motorBackB, (int)abs(pidOutput) * motorFactorB);
    digitalWrite(motorForwardA, LOW);
    digitalWrite(motorForwardB, LOW);
  }
  else // If the robot is balanced, disable all motors
  {  
    digitalWrite(motorForwardA, LOW);
    digitalWrite(motorForwardB, LOW);
    digitalWrite(motorBackA, LOW);
    digitalWrite(motorBackB, LOW);
  }
}

