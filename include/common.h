#pragma once

#define PIN_SDA     26        // ESP32 default 21 
#define PIN_SCL     27        // ESP32 default 22

#define INTERRUPT_PIN 25

const char * host_ip = "192.168.0.172";

enum ERROR_TYPES { NO_ERROR, ERROR_PAUSE, ERROR_FLOP, ERROR_6050 };
