#include "main.h"
#include <string.h>
// 1 - Config button
/*
    PA0 -> PA15 are controlled by GPIOA

    From schematic, we know PA0 is USER & Wake-up Button

    From data sheet, we know GPIOA_BASE_ADDR is at 0x40020000

    From ref manual - stm32, we know 
        GPIOA_MODER 
            -- offset addr is 0
            -- 00 is input mode
*/


#define GPIOA_BASE_ADDR 0x40020000 

void Button_Init() {

    // 1. Enable GPIO_A clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 2. Set PA0 as input
    uint32_t* GPIOA_MODER = (uint32_t*)(GPIOA_BASE_ADDR + 0x00);
    *GPIOA_MODER &= ~(0b11 << 0); 
}

// 2 - Config LED
/*
    PA0 -> PA15 are controlled by GPIOA

    From schematic, we know PD12, PD13, PD14, and PD15 are LED_GREEN, ORANGE_GREEN, RED_GREEN, and BLUE_GREEN respectively

    From data sheet, we know GPIOD_BASE_ADDR is at 0x40020C00

    From ref manual - stm32, we know 
        GPIOD_MODER 
            -- offset addr is 0
            -- 01 is output mode
*/
#define GPIOD_BASE_ADDR 0x40020C00 // Get addr from  Data sheet <- Memory mapping

void Led_Init() {
    // 1. Enable GPIO_D clock
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // 2. Set PD12, PD13, PD14 and PD15 as outputs - Get PD numbers from schematic
    uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_BASE_ADDR + 0x00); 
    *GPIOD_MODER &= (0b11111111 << 24); // Clear
    *GPIOD_MODER |= (0b01010101 << 24); // Set
}

// 3 - Control LED
/*
    From ref manual - stm32, we know 
        GPIOD_ODR 
            -- offset addr is 14
            -- Output data register  
*/
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

// 5 - Config button interrupt
/*
  EXTI - External Interrupt

  From data sheet, we know GPIOA_BASE_ADDR is at 0x40013C0

  From ref manual - stm32, we know 

  Why EXTI0????

*/
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

  Led_Ctrl(RED_LED, Button_Status());

  // Clear interrupt flag to exit handler function
  uint32_t* EXTI_PR = (uint32_t*)(EXTI_BASE_ADDR + 0x14);
  *EXTI_PR |= (0b1 << 0);
}

/*

  GOOD UNTIL THIS PART

*/

// 6. Config UART
#define GPIOB_BASE_ADDR 0x40020400
#define USART1_BASE_ADDR 0x40011000

void uart_init()
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	uint32_t* GPIOB_MODER = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
	uint32_t* GPIOB_AFRL  = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
	uint32_t* USART_BRR  = (uint32_t*)(USART1_BASE_ADDR + 0x08);
	uint32_t* USART_CR1  = (uint32_t*)(USART1_BASE_ADDR + 0x0C);


	*GPIOB_MODER &= ~(0b1111 << 12); // CLEAR
	*GPIOB_MODER |= (0b1010 << 12);

	*GPIOB_AFRL	&= ~(0xff << 24);
	*GPIOB_AFRL	|= (0b01110111 << 24);

	// set baud rate ~ 9600 -> UARTDIV = 104.16667 -> mantissa = 104 & fraction = 0.16667 * 16 = 3
	*USART_BRR &= ~(0xffff << 0);
	*USART_BRR |= (3 << 0);
	*USART_BRR |= (104 << 4);

	// data frame
	*USART_CR1 |= (0b1 << 10); // Enable parity
	*USART_CR1 |= (0b1 << 12); // 9 bits length

	// enable UART
	*USART_CR1 |= (0b1 << 13);

  // transmiter, receiver
	*USART_CR1 |= (0b11 << 2);
}

void uart_send_one_byte(char data)
{
	uint32_t* USART_SR = (uint32_t*)(USART1_BASE_ADDR + 0x00);
	uint32_t* USART_DR = (uint32_t*)(USART1_BASE_ADDR + 0x04);
	// wait for TXE == 1
	while (((*USART_SR >> 7) & 1) == 0);

	*USART_DR = data;

	while (((*USART_SR >> 6) & 1) == 0);
}

void uart_send_string(char* str)
{

	// Get size of string
	int size;
	size = strlen(str);

	for (int i = 0; i < size; i++)
		uart_send_one_byte(str[i]);
}

char uart_receive_one_byte()
{
	uint32_t* USART_SR = (uint32_t*)(USART1_BASE_ADDR + 0x00);
	uint32_t* USART_DR = (uint32_t*)(USART1_BASE_ADDR + 0x04);
	// wait for RxNE == 1
	while (((*USART_SR >> 5) & 1) == 0);
	return *USART_DR;
}

/*
khi nhận được uart rx thì UART tại ra interrupt signal để giử xuống NVIC
NVIC chấp nhận interrupt signal của UART

    + Set bit RXNEIE (Enable bit 5 của CR1)
    + UART RX interrupt ở position bao nhiêu? -> Xem vector table -> REFERENCE MANUAL
        --> position 37
    + Mở ARM lên xem NESTED VECTOR INTERRUPT TABLE
        --> bit 5 của ISER1 (0xE000E100 + 4) 
*/

void Uart_Interrupt_Init() {

  uint32_t* USART_CR1 = (uint32_t*)(USART1_BASE_ADDR + 0x0C);
  *USART_CR1 |= (0b1 << 5);

  uint32_t* ISER1 = (uint32_t*)(ISER_BASE_ADDR + 0x04);
  *ISER1 |= (0b1 << 5);
  
}

void USART1_IRQHandler() {
	char data = uart_receive_one_byte();
	if(data == 'x')
    Led_Ctrl(ORANGE_LED, ON);
	else if(data == 'o')
    Led_Ctrl(ORANGE_LED, OFF);
}

int main() {

  HAL_Init();
  Button_Init();
  Led_Init();
  Button_Interrupt_Int();
  uart_init();
  Uart_Interrupt_Init()

  while(1) {
    Led_Ctrl(GREEN_LED, ON);
    HAL_Delay(1000);
    Led_Ctrl(GREEN_LED, OFF);
    HAL_Delay(1000);
    uart_send_string("HELLO\n");


  }

  return 0;
}