#ifndef __HMI_H
#define __HMI_H

#include "main.h"

typedef struct
{
    const char *name;
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState pin_state;
} buttonLED;

typedef struct
{
    const char *name;
    GPIO_TypeDef *port;
    uint16_t Pin_grip;
    GPIO_PinState Pin_grip_state;
    uint16_t Pin_x;
    // uint32_t Pin_x_pos;
    uint16_t Pin_y;
    // uint32_t Pin_y_pos;
} joystick;

extern buttonLED greenLED;
extern buttonLED redLED;
extern buttonLED homeButton;
extern buttonLED runTestButton;
extern buttonLED autoManButton;
extern joystick controlJoystick;

// Required function prototypes external to hmi HAL files
void HMI_Init(void);
void readDigitalPinState(buttonLED butLED);

#endif /* __HMI_H */