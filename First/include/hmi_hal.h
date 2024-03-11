#ifndef __HMI_HH
#define __HMI_H

#include "main.h"

typedef struct
{
    const char *name;
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState pin_state;
} buttonLED;

extern buttonLED greenLED;
extern buttonLED redLED;
extern buttonLED homeButton;
extern buttonLED runTestButton;
extern buttonLED autoManButton;
// Add reset button if req'd

// Required function prototypes external to hmi HAL files

#endif /* __HMI_H */