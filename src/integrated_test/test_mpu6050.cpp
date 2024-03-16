// 采集MPU6050数据，发送到上位机
#include <Arduino.h>
#include "common.h"
#include "flex_log.h"
#include "mpu6050_dmp.h"

TaskHandle_t th_p[1];

Flex_Log& _logger = Flex_Log::instance();

void cmdCallback(void*cmd) {
    Serial.print( "cmd: ");
    Serial.println( (const char*) cmd );
}
void HostTask(void *args) {
    _logger.run( cmdCallback );
}

MPU6050_Entity  entity_MPU6050;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
volatile u_long mpuIntTimestamp = 0;
void dmpDataReady() {
    mpuInterrupt = true;
    // u_long m = micros();
    // u_long dt = m - mpuIntTimestamp;
    // if( mpuIntTimestamp ) _logger.log( String("mpu6050: ") + dt );
    mpuIntTimestamp = micros();
}

// measure interrupt accuracy and send interval to monitor
#define OUTPUT_READABLE_YAWPITCHROLL
void loop() {
    if( entity_MPU6050.dmpReady && mpuInterrupt ) {
        // clear interrupt flag
        mpuInterrupt = false;

        // 计算中断的时间间隔，检查是否均匀
        static u_long last_tm = 0;
        u_long dt = 0;
        if( last_tm ) {
            dt = mpuIntTimestamp - last_tm;
            // _logger.log( String("mpu6050: ") + dt );
        }
        last_tm = mpuIntTimestamp;

            dt = micros();
            entity_MPU6050.get_Angle();
            dt = micros() - dt;
            char buf[200];
            sprintf(buf, "chs: %4.2f,%4.2f,%d\n",entity_MPU6050.Angle_Balance,entity_MPU6050.Gyro_Balance, dt);
            _logger.debug(buf);
            // Serial.println( dt);
//         uint8_t fifoBuffer[64];

//   // orientation/motion vars
//   Quaternion q;           // [w, x, y, z]         quaternion container
//   VectorFloat gravity;    // [x, y, z]            gravity vector
//   float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector        
//         if (entity_MPU6050.mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

//             #ifdef OUTPUT_READABLE_YAWPITCHROLL
//                 // display Euler angles in degrees
//                 entity_MPU6050.mpu.dmpGetQuaternion(&q, fifoBuffer);
//                 entity_MPU6050.mpu.dmpGetGravity(&gravity, &q);
//                 entity_MPU6050.mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//                 Serial.print("ypr\t");
//                 Serial.print(ypr[0] * 180/M_PI);
//                 Serial.print("\t");
//                 Serial.print(ypr[1] * 180/M_PI);
//                 Serial.print("\t");
//                 Serial.println(ypr[2] * 180/M_PI);
//                 // delay(50);
//             float Angle_Balance = ypr[1] * 180/M_PI;    
//             VectorInt16 gyro;
//             entity_MPU6050.mpu.dmpGetGyro(&gyro, fifoBuffer);
//             float Gyro_Balance = -gyro.y;                
//             #endif
//         }        
    }
    delay(1);
}


void setup() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);

  WiFi.begin("LINJIA","050208linjia");
  
  while( WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi begin ..");
    delay(200);
  }

  if( _logger.begin( host_ip )) {
    xTaskCreatePinnedToCore(HostTask, "HostTask", 4096, NULL, 3, &th_p[0], 0); 
  }

  pinMode(INTERRUPT_PIN, INPUT);
  entity_MPU6050.initialize( INTERRUPT_PIN , dmpDataReady );

}