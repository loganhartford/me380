#include "stm32f4xx.h"  // Include the STM32F4xx HAL library
#include "stm32f4xx_hal.h"


// Function prototypes

// TIM handle declaration

int main(void) {
    // Initialize the HAL Library
    HAL_Init();

    // Main program logic
    while (1) {
       
    }
}


void SysTick_Handler(void) {
    HAL_IncTick();
}