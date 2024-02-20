#ifndef __LIMIT_SWITCH_H
#define __LIMIT_SWITCH_H

#include "main.h"

typedef struct
{
    GPIO_TypeDef *port; // GPIO Port
    uint16_t theta1Pin; // GPIO Pin
    uint16_t theta2Pin; // GPIO Pin
    uint16_t thetazPin; // GPIO Pin
} LimitSwitch;

extern LimitSwitch limitSwitches;

void Limit_Switch_Init(void);

#endif /* __LIMIT_SWITCH_H */