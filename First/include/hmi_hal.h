#ifndef __HMI_H
#define __HMI_H

#include "main.h"
#include <string.h>

typedef struct
{
    const char *name;
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState pin_state;
    uint32_t mode;
    uint32_t pull;
    uint32_t speed;
} buttonLED;

typedef struct
{
    const char *name;
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t mode;
    uint32_t pull;
    uint32_t channel;
} Pot;

// typedef struct // UPDATE W/ ABOVE
// {
//     const char *name;
//     GPIO_TypeDef *port;
//     uint16_t Pin_grip;
//     GPIO_PinState Pin_grip_state;
//     uint16_t Pin_x;
//     // uint32_t Pin_x_pos;
//     uint16_t Pin_y;
//     // uint32_t Pin_y_pos;
// } joystick;

extern buttonLED greenLED;
extern buttonLED redLED;
extern buttonLED activeLED;
extern buttonLED homeButton;
extern buttonLED runTestButton;
extern buttonLED autoManButton;
extern buttonLED gripButton;
extern Pot xPot;
extern Pot yPot;
extern Pot zPot;

// Required function prototypes external to hmi HAL files
void HMI_Init(void);
void changeLEDState(buttonLED butLED, const char *ledMode);
void buttonDebug(void);
uint32_t Read_Pot(Pot *pot);
uint32_t Read_ADC_Value(uint32_t channel);

#endif /* __HMI_H */