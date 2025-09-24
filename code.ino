/**
 * EE396: Design Lab Project - Self Balancing Robot
 * * This code implements a PID controller to make a two-wheeled robot balance itself.
 * It uses an MPU6050 IMU to measure the tilt angle and an L293D motor driver
 * to control the motors.
 * * Institution: Indian Institute of Technology Guwahati
 * Supervised by: Dr. Chayan Bhawal
 * Developed by: Ramjas Sahu (220108048), Raghav Mour (220108047)
 */

#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Use Arduino Wire library for I2C communication
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// The minimum speed the motors will turn. Prevents stalling at low PWM values.
#define MIN_ABS_SPEED 30

// MPU6050 object
MPU6050 mpu;

// MPU control and status variables
bool dmpReady = false;      // True if DMP initialization was successful
uint8_t mpuIntStatus;       // Holds actual interrupt status byte from MPU
uint8_t devStatus;          // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;        // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;         // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];     // FIFO storage buffer

// Orientation and motion variables
Quaternion q;               // [w, x, y, z] quaternion container
VectorFloat gravity;        // [x, y, z] gravity vector
float ypr[3];               // [yaw, pitch, roll] container

// PID Controller variables
double originalSetpoint = 172.50; // The desired angle for the robot to be stable (found experimentally)
double setpoint = originalSetpoint;
double input, output;       // 'input' is the actual angle, 'output' is the motor correction

// PID tuning parameters. These values will need to be adjusted for your specific robot.
// Note: The project report indicates final tuned values of Kp=21, Kd=490.
// These values below are from the initial code provided.
double Kp = 60;
double Kd = 2.2;
double Ki = 270;

// Create PID controller object
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Motor speed balancing factors (if one motor is slightly faster than the other)
double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.5;

// Motor Controller Pin Definitions
int ENA = 5;  // PWM Speed Control for Motor A
int IN1 = 6;  // Direction Control 1 for Motor A
int IN2 = 7;  // Direction Control 2 for Motor A
int IN3 = 9;  // Direction Control 1 for Motor B
int IN4 = 8;  // Direction Control 2 for Motor B
int ENB = 10; // PWM Speed Control for Motor B

// Create Motor Controller object
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// MPU interrupt flag
volatile bool mpuInterrupt = false; 

// Interrupt Service Routine (ISR) to set the interrupt flag
void dmpDataReady() {
    mpuInterrupt = true;
}

// SETUP FUNCTION - Runs once when the Arduino starts
void setup() {
    // Initialize I2C communication
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

    // Initialize MPU6050
    mpu.initialize();

    // Initialize the Digital Motion Processor (DMP)
    devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here; these are specific to your MPU6050
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    // Check if DMP initialization was successful
    if (devStatus == 0) {
        // Turn on the DMP
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection on pin 2
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set dmpReady flag so the main loop() knows it's okay to use it
        dmpReady = true;

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        // Configure the PID controller
        pid.SetMode(AUTOMATIC);     // Turn the PID on
        pid.SetSampleTime(10);      // Set PID calculation interval to 10ms
        pid.SetOutputLimits(-255, 255); // Set output range to match PWM range
    } else {
        // DMP Initialization failed
        Serial.begin(9600);
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

// MAIN LOOP - Runs repeatedly
void loop() {
    // If DMP is not ready, don't do anything
    if (!dmpReady) return;

    // Wait for MPU interrupt or for the FIFO buffer to have enough data
    while (!mpuInterrupt && fifoCount < packetSize) {
        // While waiting for MPU data, keep the PID running to control motors
        pid.Compute();
        motorController.move(output, MIN_ABS_SPEED);
    }

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for FIFO overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // Reset FIFO to recover
        mpu.resetFIFO();
        // Serial.println(F("FIFO overflow!")); // Uncomment for debugging
    } else if (mpuIntStatus & 0x02) { // Check if data is ready
        // Wait for correct packet size
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Adjust fifoCount for any remaining bytes
        fifoCount -= packetSize;

        // Get the Yaw, Pitch, and Roll values from the DMP data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert pitch from radians to degrees and scale it for the PID input
        // The "+ 180" is to map the angle to a positive range that works with the setpoint
        input = ypr[1] * 180 / M_PI + 180;
    }
}
