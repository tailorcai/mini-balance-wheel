#ifndef __MCPWM_MOTOR__
#define __MCPWM_MOTOR__

#include <Arduino.h>
#include <stdio.h>

#include "esp_system.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

class McpwmMotor
{
private:
    /* 电机速度，单位mm */
    int16_t speeds[4]{0, 0};
    bool mMotorAttached[4]{false, false, false, false};

public:
    McpwmMotor() = default;
    ~McpwmMotor() = default;
    void attachMotor(uint8_t id, uint8_t gpioIn1, uint8_t gpioIn2);
    void stopMotor(int8_t motorId = -1);
    void updateMotorSpeed(int8_t id, int16_t pwmValue);
};

#endif // __MCPWM_MOTOR__