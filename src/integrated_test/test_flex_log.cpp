// 测试上位机通讯各项功能
#include <Arduino.h>
#include "flex_log.h"

TaskHandle_t th_p[1];

Flex_Log& _logger = Flex_Log::instance();

void cmdCallback(void*cmd) {
    Serial.print( "cmd: ");
    Serial.println( (const char*) cmd );
}
void Core0task(void *args) {
    _logger.run( cmdCallback );
}

void setup() {
    Serial.begin(115200);

    WiFi.begin("LINJIA","050208linjia");
  
    while( WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi begin ..");
        delay(200);
    }

    if( _logger.begin( "192.168.0.248")) {
        xTaskCreatePinnedToCore(Core0task, "Core0task", 4096, NULL, 3, &th_p[0], 0); 
    }
}


void loop() {
    static int cnt = 0;
    _logger.log( "loop " + String(cnt++));
    delay(100);
}