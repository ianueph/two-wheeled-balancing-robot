#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "QuickPID.h"

/*---
---
---
---
MPU 6050 Declarations and Variables
---
---
---
---*/ 

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*-Packet structure for InvenSense teapot demo-*/ 
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high

/*---
---
---
---
PID Control Declarations and Variables
---
---
---
---*/ 

//Define Variables we'll be connecting to
float Setpoint, Input, Output;
float Kp = 6, Ki = .5, Kd = 2.5;

//Specify PID links
QuickPID myPID(&Input, &Output, &Setpoint);

/*---
---
---
---
Motor Controller Declarations and Variables
---
---
---
---*/ 

byte leftDIO = 2;
byte leftPWM = 5;
byte rightDIO = 4;
byte rightPWM = 6;

void DMPDataReady() {
  MPUInterrupt = true;
}

void setupMPU6050() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Serial.begin(115200); //115200 is required for Teapot Demo output
  while (!Serial);

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  /*Wait for Serial input*/
  Serial.println(F("\nSend any character to begin: "));
  while (Serial.available() && Serial.read()); // Empty buffer
  while (!Serial.available());                 // Wait for data
  while (Serial.available() && Serial.read()); // Empty buffer again

  /*Print Calibrations*/
  mpu.CalibrateAccel(20);
  mpu.CalibrateGyro(20);
  mpu.PrintActiveOffsets();

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(-13.00000);
  mpu.setYGyroOffset(42.00000);
  mpu.setZGyroOffset(16.00000);
  mpu.setXAccelOffset(-1084.00000);
  mpu.setYAccelOffset(-344.00000);
  mpu.setZAccelOffset(1708.00000);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
  pinMode(LED_BUILTIN, OUTPUT);
}

void setupPID() {
  Input = 0;
  Setpoint = 0;

  //apply PID gains
  myPID.SetTunings(Kp, Ki, Kd);

  //turn the PID on
  myPID.SetMode(myPID.Control::automatic);
  myPID.SetOutputLimits(-255, 255);
}

void setupMotorController() {
  pinMode(leftDIO, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightDIO, OUTPUT);
  pinMode(rightPWM, OUTPUT);
}

float readPitch() {
  float pitch;
  float yaw;
  float roll;
    
  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      yaw = ypr[0] * 180/M_PI;
      pitch = ypr[1] * 180/M_PI;
      roll = ypr[2] * 180/M_PI;

      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    #endif
    /* Blink LED to indicate activity */
  blinkState = !blinkState;
  digitalWrite(LED_BUILTIN, blinkState);
  }

  return pitch;
}

float computePID() {
  Input = readPitch();
  myPID.Compute();
  return Output;
}

void controlDCMotors(int PIDvalue) {
  float rate = abs(PIDvalue);
  float direction = (PIDvalue >= 0);

  digitalWrite(leftDIO, direction);
  analogWrite(leftPWM, rate);
  digitalWrite(rightDIO, direction);
  analogWrite(rightPWM, rate);

  Serial.print("direction:\t");
  Serial.print(direction ? "Forward " : "Backward");
  Serial.print("\trate:\t");
  Serial.println(rate);
}

void setup() {
  setupMPU6050();
  setupPID();
}

void loop() {
  if (!DMPReady) return; // Stop the program if DMP programming fails.
  computePID();
  controlDCMotors(Output);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}