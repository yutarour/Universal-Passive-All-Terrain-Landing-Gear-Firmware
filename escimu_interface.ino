#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

#define ESC_PWM 10

Servo ESC;
MPU6050 mpu(0x68,&Wire1);
float ypr[3];  // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
int const INTERRUPT_PIN = 3;

// Velocity calculation variables
float zVelocity = 0.0;       // Integrated velocity in Z direction (m/s)
float prevAccelZ = 0.0;      // Previous Z acceleration for filtering
unsigned long prevTime = 0;  // Previous time for integration
bool firstSample = true;     // Flag for first sample

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer
VectorFloat gravity;     // Gravity vector
VectorInt16 aa;          // Accelerometer measurements
VectorInt16 aaReal;      // Gravity-free accelerometer measurements
Quaternion q;

unsigned long calibrationStartTime;
int calibrationStep = 0;
uint8_t dataPacket[6] = { '$', 0, 0, 0, '\r', '\n' };

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;  // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

void setMotorSpeed(int speed) {
  // Constrain speed to valid range (1000-2000 microseconds)
  speed = constrain(speed, 1000, 2000);
  ESC.writeMicroseconds(speed);
}

void setup() {
  Serial.begin(921600);

  // Initialize ESC
  ESC.attach(ESC_PWM, 1000, 2000);
  setMotorSpeed(1500);  // Stop motor
  delay(500);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire1.begin();
  Wire1.setClock(400000);
  Wire1.setTimeout(3000);
#endif

  while (!Serial)
    ;
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }

  mpu.dmpInitialize();
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'c') {  //cal routine
      if (devStatus == 0) {
        Serial.println("$ Starting calibration...");
        calibrationStartTime = millis();
        calibrationStep = 1;

        Serial.println("$ Calibrating accelerometer (6 positions required)...");
        mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050

        Serial.println("$ Accelerometer calibration complete!");
        Serial.println("$ Calibrating gyroscope...");
        calibrationStep = 2;
        mpu.CalibrateGyro(6);

        Serial.println("$ Gyroscope calibration complete!");
        Serial.println("$ These are the Active offsets: ");
        mpu.PrintActiveOffsets();

        Serial.println("$ Enabling DMP...");  //Turning ON DMP
        calibrationStep = 3;
        mpu.setDMPEnabled(true);

        /*Enable Arduino interrupt detection*/
        Serial.print(F("$ Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        MPUIntStatus = mpu.getIntStatus();

        /* Set the DMP Ready flag */
        Serial.println(F("$ DMP ready! Waiting for first interrupt..."));
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        calibrationStep = 0;  // Done
      }
    } else if (input == 'l') {
      mpu.dmpGetCurrentFIFOPacket(FIFOBuffer);
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Serial.print("$ ");
      Serial.print(ypr[0] * 180 / M_PI);  // Yaw in degrees
      Serial.print(",");
      Serial.print(ypr[1] * 180 / M_PI);  // Pitch in degrees
      Serial.print(",");
      Serial.print(ypr[2] * 180 / M_PI);  // Roll in degrees
      Serial.println();
    }

    else if (input == 's') {  // Start motor with speed
      while (Serial.available() < 1)
        ;  // Wait for speed data
      int speedPercent = Serial.parseInt();
      int pwmValue = map(speedPercent, 0, 100, 1500, 1900);
      setMotorSpeed(pwmValue);
      Serial.print("$Motor start: ");
      Serial.print(speedPercent);
      Serial.println("%");
    } else if (input == 'r') {  // Reverse motor with speed
      while (Serial.available() < 1)
        ;  // Wait for speed data
      int speedPercent = Serial.parseInt();
      int pwmValue = map(speedPercent, 0, 100, 1500, 1200);
      setMotorSpeed(pwmValue);
      Serial.print("$Motor Start reverse: ");
      Serial.print(speedPercent);
      Serial.println("%");
    } else if (input == 'm') {  // Start measurements
      Serial.println("$ Starting Measurements");
      bool measuring = true;
      zVelocity = 0.0;  // Reset velocity when starting measurements
      firstSample = true;

      while (measuring) {
        if (Serial.available() > 0) {
          char input = Serial.read();
          if (input == 'q') measuring = false;
        }
        if (!DMPReady) {
          Serial.println("$ ERROR DMP STOP");
        }
        if (MPUInterrupt) {
          MPUInterrupt = false;
          unsigned long currentTime = millis();

          mpu.dmpGetCurrentFIFOPacket(FIFOBuffer);
          mpu.dmpGetQuaternion(&q, FIFOBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

          // Get linear acceleration (without gravity)
          mpu.dmpGetAccel(&aa, FIFOBuffer);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

          // Calculate Z velocity (convert from mg to m/s^2 and integrate)
          float accelZ = aaReal.z / 1000.0 * 9.81;  // Convert to m/s^2

          if (!firstSample) {
            float deltaTime = (currentTime - prevTime) / 1000.0;  // Convert to seconds
            zVelocity += accelZ * deltaTime;                      // Integrate acceleration to get velocity

            // Simple high-pass filter to remove drift (adjust 0.98 factor as needed)
            zVelocity *= 0.98;
          } else {
            firstSample = false;
          }

          prevTime = currentTime;

          // Print all data
          Serial.print("$ ");
          Serial.print(ypr[0] * 180 / M_PI);  // Yaw in degrees
          Serial.print(",");
          Serial.print(ypr[1] * 180 / M_PI);  // Pitch in degrees
          Serial.print(",");
          Serial.print(ypr[2] * 180 / M_PI);  // Roll in degrees
          Serial.print(",");
          Serial.print(zVelocity);  // Z-axis velocity in m/s
          Serial.println();
        }
      }
      Serial.println("$ Stop measure");
    } else if (input == 'x') {  // Stop motor
      setMotorSpeed(1500);
      Serial.println("$ Motor stopped");
    } else if (input == 'z') {  // Reset Z velocity
      zVelocity = 0.0;
      Serial.println("$ Z velocity reset to 0");
    }
  }
}