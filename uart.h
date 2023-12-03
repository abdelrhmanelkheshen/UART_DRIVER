

/******************************************************************************
 *
 * [FILE NAME]: <UART.h>
 *
 * [AUTHOR]: <Abdelrhman Hesham>
 *
 * [DATE CREATED]: <15/11/2023>
 *
 * [DESCRIPTION]: <Header file for UART >
 *
 *******************************************************************************/



#ifndef UART_H
#define UART_H

#include "std_types.h"


#define RCGCUART_REG  (*(volatile unsigned long*)0x400FE618)
#define PORTA_BASE_ADDRESS 0x40004000
#define PORTB_BASE_ADDRESS 0x40005000
#define PORTC_BASE_ADDRESS 0x40006000
#define PORTD_BASE_ADDRESS 0x40007000
#define PORTE_BASE_ADDRESS 0x40024000
#define PORTF_BASE_ADDRESS 0x40025000
#define AFSEL_REG_OFFSET 0x420
#define DATA_REG_OFFSET 0x00
#define CTL_REG_OFFSET 0x52C
#define DEN_REG_OFFSET 0x51C
#define DIR_REG_OFFSET 0x400
#define PUR_REG_OFFSET 0x510
#define AMSEL_REG_OFFSET 0x528


#define UART0_BASE_ADDRESS 0x4000C000
#define UART1_BASE_ADDRESS 0x4000D000
#define UART2_BASE_ADDRESS 0x4000E000
#define UART3_BASE_ADDRESS 0x4000F000
#define UART4_BASE_ADDRESS 0x40010000
#define UART5_BASE_ADDRESS 0x40011000
#define UART6_BASE_ADDRESS 0x40012000
#define UART7_BASE_ADDRESS 0x40013000


#define UART_DATA_REG_OFFSET 0x00
#define UART_RECIEVE_STATUS_REG_OFFSET 0x004
#define UART_FLAG_RATE_REG_OFFSET 0x018
#define UART_LOW_POWER_REG_OFFSET 0x020
#define UART_INTEGER_BAUD_RATE_REG_OFFSET 0x024
#define UART_FRACTIONAL_BAUD_RATE_REG_OFFSET 0x028
#define UART_LINE_CTRL_REG_OFFSET 0x02C
#define UART_CTRL_REG_OFFSET 0x030
#define UART_FIFO_REG_OFFSET 0x034
#define UART_CLOCK_SOURCE_REG_OFFSET 0xFC8



#define UART_CTL_ENABLE_BIT 0
#define UART_INTEGER_BAUD_RATE_MASK 0xFFFF0000 
#define UART_FRACTIONAL_BAUD_RATE_MASK 0xFFFFFFE0 
#define UART_CLOCK_SOURCE_MASK 0xFFFFFFF0
#define UART_LINE_CTRL_SOURCE_MASK 0xFFFFFF0F
#define UART_CLK_FREQUENCY_MASK 0x00
#define PORT_A      0
#define PIN_0       0 

#define PORT_B 1
#define PIN_0 0
#define PIN_1 1

#define PORT_C 2
#define PIN_0 0


#define PIN0_PIN1_CTRL_MASK 0xFFFFFF00
#define PIN4_PIN5_CTRL_MASK 0xFF00FFFF
#define PIN6_PIN7_CTRL_MASK 0x00FFFFFF

#define UART_PIN0_PIN1_CTRL_MASK 0x00000011
#define UART_PIN4_PIN5_CTRL_MASK 0x00110000
#define UART_PIN6_PIN7_CTRL_MASK 0x11000000

#define UART_RECIEVE_MODE    (uint8)0x01U 
#define UART_TRANSMIT_MODE    (uint8)0x01U 


#define LENGTH_LINE_CTRL_REG_START_BIT 5 
#define INTDIV_IBRD_REG_START_BIT     0
#define FLOAT_DIV_FBRD_REG_START_BIT     0

#define TRANSMIT_FLAG_BIT 5
#define RECIEVE_FLAG_BIT 4

#define FIRST_BIT 0 

#define PORTA_SYSCLK_MASK (0x00000001)
#define PORTB_SYSCLK_MASK (0x00000002)
#define PORTC_SYSCLK_MASK (0x00000004)
#define PORTD_SYSCLK_MASK (0x00000008)
#define PORTE_SYSCLK_MASK (0x00000010)
#define PORTF_SYSCLK_MASK (0x00000020)



/************************************************************************************
* Name: DataSize 
* Description: Defining New type as an enum to represent number of data bits
************************************************************************************/


typedef enum 
{
  five_bits_data,
  six_bits_data,
  seven_bits_data,
  eight_bits_data
}DataSize;

/************************************************************************************

* Name:uart_configuration
* Parameters: uint32 baud rate
  uint32 CLK 
  uint8 uart number   
  uint8 port number 
  uint8 transmit pin number
  uint8 receive pin number 
  DataSize length
 

* Description: uart configuration .  
************************************************************************************/

typedef struct
{
  uint32 baud_rate;
  uint32 clk ; 
  uint8 uart_num ;
  uint8 port_num;
  uint8 transmit_pin_num;
  uint8 receive_pin_num;
  DataSize length;
}uart_configuration ;



/************************************************************************************
* Service Name: uartInit
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): Pointer to Structure
* Parameters (out): None
* Return value: None
* Description: Intialize Uart
************************************************************************************/


void uartInit(uart_configuration* ptr2config);



/************************************************************************************
* Service Name: UART_sendByte
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): uint8 Data
* Parameters (out): None
* Return value: None
* Description: API to Send data 
************************************************************************************/
void UART_sendByte(const uint8 data);


/************************************************************************************
* Service Name: uart_RecieveByte
* Sync/Async: Synchronous
* Reentrancy: reentrant
* Parameters (in): None
* Parameters (out): uint8 Data
* Return value: uint8
* Description: API to recieve/read data  
************************************************************************************/


uint8 uart_RecieveByte(void);














#endif