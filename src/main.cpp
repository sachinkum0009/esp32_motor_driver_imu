#include <Arduino.h>

// #include <MPU6050.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define EARTH_GRAVITY_MS2 9.80665  // m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gg;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif



// put function declarations here:
// int myFunction(int, int);

void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200); // Initialize the Serial interface with baud rate 115200
  Serial.println("Intialized Serial Monitor"); // Print "Hello" to the Serial monitor

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(27, 26);
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  Serial.println("Wire started"); // Print "Hello" to the Serial monitor

    mpu.initialize();
  Serial.println("MPU initialized"); // Print "Hello" to the Serial monitor
    devStatus = mpu.dmpInitialize();
    Serial.print("devStatus: ");
    Serial.println(devStatus);
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-156);
    mpu.setYGyroOffset(-11);
    mpu.setZGyroOffset(-14);
    mpu.setXAccelOffset(-3699);
    mpu.setYAccelOffset(-2519);
    mpu.setZAccelOffset(1391); // 1688 factory default for my test chip
  Serial.println("MPU initialized"); // Print "Hello" to the Serial monitor

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {

        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }


  

  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("in the loop"); // Print "in the loop" to the Serial monitor

  /**
     * IMU Sensor Values
     */
    if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetGyro(&gg, fifoBuffer);
        mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
        Serial.print("ggWorld\t");
        Serial.print(ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD);
        Serial.print("\t");
        Serial.print(ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD);
        Serial.print("\t");
        Serial.println(ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD);

        Serial.print("Quaternion:\t");
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.println(q.z);

        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("Yaw Pitch Roll:\t");
        Serial.print(ypr[0] * RAD_TO_DEG);
        Serial.print("\t");
        Serial.print(ypr[1] * RAD_TO_DEG);
        Serial.print("\t");
        Serial.println(ypr[2] * RAD_TO_DEG);
    

        // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    }
    else {
        // Wait for all data to be available
        Serial.println("Waiting for dumpready");
    }
  delay(300); // Delay for 300 milliseconds
}

// put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }