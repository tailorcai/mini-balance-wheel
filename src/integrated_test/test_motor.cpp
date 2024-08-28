// 电机控制和速度检测
#include <Arduino.h>
#include "common.h"
#include "flex_log.h"
#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 
#include "at8236_esp32.h" 
#include "motor_adj.h"

TaskHandle_t th_p[1];

Flex_Log& _logger = Flex_Log::instance();


MotorWithAdj motorL(19,21,1,5000,8,1, 0);
MotorWithAdj motorR(23,22,1,5000,8,2, 0);
ESP32Encoder encoderL, encoderR;

int8_t PIN_ENCODER_L[] = {17,16};
int8_t PIN_ENCODER_R[] = {2,4};

int lspeed, rspeed;
void cmdCallback(void*cmd) {
    Serial.print( "cmd: ");
    Serial.println( (const char*) cmd );
    if( 2 == sscanf((const char*)cmd,"%d,%d", &lspeed,&rspeed)) {
        motorL.drive( lspeed );
        motorR.drive( rspeed );
        Serial.println(" speed set ok!");
    }
    else {
        Serial.println(" speed set fail!");
    }
}

void HostTask(void *args) {
    _logger.run( cmdCallback );
}

void loop() {
    int l = encoderL.getCount();
    encoderL.clearCount();
    int r = encoderR.getCount();
    encoderR.clearCount();

    _logger.log(String("chs:") + lspeed+","+rspeed+","+ l + "," + r );
    delay(100);
}


void setup() {
  Serial.begin(115200);

  WiFi.begin("LINJIA","050208linjia");
  
  while( WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi begin ..");
    delay(200);
  }

  if( _logger.begin(host_ip)) {
    xTaskCreatePinnedToCore(HostTask, "HostTask", 4096, NULL, 3, &th_p[0], 0); 
  }
  else {
    Serial.println("FATAL ERROR: connect to monitor failed");
  }

  encoderL.attachFullQuad( PIN_ENCODER_L[0], PIN_ENCODER_L[1]);
  encoderL.setCount(0);

  encoderR.attachFullQuad( PIN_ENCODER_R[0], PIN_ENCODER_R[1]);
  encoderR.setCount(0);

        // motorL.drive( lspeed );
        // motorR.drive( rspeed );
}