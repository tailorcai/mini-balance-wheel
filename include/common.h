#pragma once

#define PIN_SDA     19        // ESP32 default 21 
#define PIN_SCL     21        // ESP32 default 22

#define INTERRUPT_PIN 18

const char * host_ip = "192.168.0.248";

enum ERROR_TYPES { NO_ERROR, ERROR_PAUSE, ERROR_FLOP, ERROR_6050 };
