#include "main.h"
#include <string.h>

#define GPIOA_BASE_ADDR 0x40020000

void Button_Init() {

    // 1. Enable GPIO_A clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 2. Set PA0 as input
    uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
    *GPIOA_MODER &= ~(0b11 << 0);
}

#define GPIOD_BASE_ADDR 0x40020C00 // Get addr from  Data sheet <- Memory mapping

void Led_Init() {
    // 1. Enable GPIO_D clock
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // 2. Set PD12, PD13, PD14 and PD15 as outputs - Get PD numbers from schematic
    uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_BASE_ADDR + 0x00); 
    *GPIOD_MODER &= (0b11111111 << 24); // Clear
    *GPIOD_MODER |= (0b01010101 << 24); // Set
}

int GREEN_LED   = 12;
int ORANGE_LED  = 13;
int RED_LED     = 14;
int BLUE_LED    = 15;
int ON          = 1;
int OFF         = 0;

void Led_Ctrl(int LED, int OnOff) {
    uint32_t* GPIOD_ODR = (uint32_t*)(GPIOD_BASE_ADDR + 0x14); 

    if (OnOff == 1)
        *GPIOD_ODR |=  (0b1 << LED);
    else 
        *GPIOD_ODR &= ~(0b1 << LED);
}

// 4 - Check if the button is pressed or not
int Button_Status() {
    uint32_t* GPIOA_IDR = (uint32_t*)(GPIOA_BASE_ADDR + 0x10);
    return (*GPIOA_IDR >> 0) & 1;
}

#define EXTI_BASE_ADDR 0x40013C00
#define ISER_BASE_ADDR 0xE000E100 // From ref manual - M4

void Button_Interrupt_Int() {

  // Config EXTI to send interrupt to NVIC when detect rising of failling edge

  // select trigger for EXTI0 is rising
  uint32_t* EXTI_RTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x08);
  *EXTI_RTSR |= (0b1 << 0);

  // select trigger for EXTI0 is falling
  uint32_t* EXTI_FTSR = (uint32_t*)(EXTI_BASE_ADDR + 0x0C);
  *EXTI_FTSR |= (0b1 << 0);

  // masking
  uint32_t* EXTI_IMR = (uint32_t*)(EXTI_BASE_ADDR + 0x00);
  *EXTI_IMR |= (0b1 << 0);

  // NVIC accepts interrupt from EXTI0
  /*
    From vectore table in ref manual - stm32, we know position of EXTI0 interrupt is 6
  */
  uint32_t* ISER0 = (uint32_t*)(ISER_BASE_ADDR + 0x00);
  *ISER0 |= (0b1 << 6);

}

void EXTI0_IRQHandler() {

  Led_Ctrl(GREEN_LED, Button_Status());

  // Clear interrupt flag to exit handler function
  uint32_t* EXTI_PR = (uint32_t*)(EXTI_BASE_ADDR + 0x14);
  *EXTI_PR |= (0b1 << 0);
}

int main() {

//HAL_Init();
  Button_Init();
  Led_Init();
  Button_Interrupt_Int();




  while(1) {
    //  Led_Ctrl(RED_LED, ON);
    //  HAL_Delay(1000);
    //  Led_Ctrl(RED_LED, OFF);
    //  HAL_Delay(1000);
  }

  return 0;
}
