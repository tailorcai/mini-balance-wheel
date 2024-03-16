#include <Arduino.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "flex_log.h"

// const uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

typedef void (*intProc)(void);

class MPU6050_Entity {
public:
//BEGIN==========MPU6050==================================================
  bool dmpReady;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  int pinInt;

  // packet structure for InvenSense teapot demo
  MPU6050 mpu;
  float Angle_Balance,Gyro_Balance;           //平衡倾角 平衡陀螺仪 转向陀螺仪
//END==========MPU6050==================================================    
    MPU6050_Entity():dmpReady(false) {

    }
    void initialize(int interruptPin, intProc proc ) {   
      pinInt = interruptPin;
      
      Flex_Log& _logger = Flex_Log::instance();
      _logger.debug( "Starting MPU6050");
      mpu.initialize(); 

      // verify connection
      _logger.debug("Testing device connections...");
      _logger.debug(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

      // wait for ready
      // Serial.println("\nSend any character to begin DMP programming and demo: ");
      // while (Serial.available() && Serial.read()); // empty buffer
      // while (!Serial.available());                 // wait for data
      // while (Serial.available() && Serial.read()); // empty buffer again

      // load and configure the DMP
      _logger.debug("Initializing DMP...");
      devStatus = mpu.dmpInitialize();
      // mpu.setRate(2);
      // Serial.println("rate:");Serial.println( mpu.getRate());
      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(14);
      mpu.setYGyroOffset(36);
      mpu.setZGyroOffset(5);
      mpu.setZAccelOffset(802); // 1688 factory default for my test chip

      // make sure it worked (returns 0 if so)
      if (devStatus == 0) {
          // Calibration Time: generate offsets and calibrate our MPU6050
          mpu.CalibrateAccel(6);
          mpu.CalibrateGyro(6);
          mpu.PrintActiveOffsets();
          // turn on the DMP, now that it's ready
          _logger.debug("Enabling DMP...");
          mpu.setDMPEnabled(true);

          // enable Arduino interrupt detection
          _logger.debug("Enabling interrupt detection (Arduino external interrupt " + String(digitalPinToInterrupt(pinInt)) + ")...");
          attachInterrupt(digitalPinToInterrupt(pinInt), proc, RISING);
          mpuIntStatus = mpu.getIntStatus();

          // set our DMP Ready flag so the main loop() function knows it's okay to use it
          _logger.debug("DMP ready! Waiting for first interrupt...");
          dmpReady = true;

          // get expected DMP packet size for later comparison
          packetSize = mpu.dmpGetFIFOPacketSize();
      } else {
          // ERROR!
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          _logger.debug("DMP Initialization failed (code "+ String(devStatus)+")" );
      }
    }

    void get_Angle() {
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        //#ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            // Serial.print("quat\t");
            // Serial.print(q.w);
            // Serial.print("\t");
            // Serial.print(q.x);
            // Serial.print("\t");
            // Serial.print(q.y);
            // Serial.print("\t");
            // Serial.println(q.z);

            // mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // Serial.print("ypr\t");
            // Serial.print(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);        

            Angle_Balance = ypr[1] * 180/M_PI;    
            VectorInt16 gyro;
            mpu.dmpGetGyro(&gyro, fifoBuffer);
            Gyro_Balance = -gyro.y;

            // Serial.print( Angle_Balance );
            // Serial.print( "\t" );
            // Serial.println( Gyro_Balance );

      }

  }
};