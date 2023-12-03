


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





#include "uart.h"
#include "Common_Macros.h"
#include "tm4c123gh6pm.h"


volatile uint32* uart_ptr = NULL_PTR ;
volatile uint32* port_ptr = NULL_PTR ;

  void uartInit(uart_configuration* ptr2config)
{
  uint32 portCLK;
  switch(ptr2config->uart_num)
  {
  case 0:
    uart_ptr = (volatile uint32*)UART0_BASE_ADDRESS;
    break;
  case 1:
    uart_ptr = (volatile uint32*)UART1_BASE_ADDRESS;
    break;
  case 2:
    uart_ptr = (volatile uint32*)UART2_BASE_ADDRESS;
    break;
  case 3:
    uart_ptr = (volatile uint32*)UART3_BASE_ADDRESS;
    break;
  case 4:
    uart_ptr = (volatile uint32*)UART4_BASE_ADDRESS;
    break;
  case 5:
    uart_ptr = (volatile uint32*)UART5_BASE_ADDRESS;
    break;
  case 6:
    uart_ptr = (volatile uint32*)UART6_BASE_ADDRESS;
    break;
  case 7:
    uart_ptr = (volatile uint32*)UART7_BASE_ADDRESS;
    break;
 }
  switch(ptr2config->port_num)
  {
    case 0:
    portCLK =PORTA_SYSCLK_MASK ; 
    break;
  case 1:
    portCLK = PORTB_SYSCLK_MASK ; 
    break;
  case 2:
    portCLK = PORTC_SYSCLK_MASK;
    break;
  case 3:
    portCLK = PORTD_SYSCLK_MASK;
    break;
  case 4:
    portCLK = PORTE_SYSCLK_MASK;
    break;
  case 5:
    portCLK = PORTF_SYSCLK_MASK;
    break;
  }
  SET_BIT(SYSCTL_RCGCUART_R,ptr2config->uart_num) ;
  SET_BIT(SYSCTL_RCGCGPIO_R,ptr2config->port_num) ;
  while((SYSCTL_PRGPIO_R & portCLK) == LOGIC_LOW);
  CLEAR_BIT(*(volatile uint32*)((volatile uint8 *)uart_ptr + UART_CTRL_REG_OFFSET),FIRST_BIT);
  switch(ptr2config->port_num)
  {
  case 0:
    port_ptr = (volatile uint32*)(PORTA_BASE_ADDRESS);
    break;
  case 1:
    port_ptr = (volatile uint32*)(PORTB_BASE_ADDRESS);
    break;
  case 2:
    port_ptr = (volatile uint32*)(PORTC_BASE_ADDRESS);
    break;
  case 3:
    port_ptr = (volatile uint32*)(PORTD_BASE_ADDRESS);
    break;
  case 4:
    port_ptr = (volatile uint32*)(PORTE_BASE_ADDRESS);
    break;
  case 5:
    port_ptr = (volatile uint32*)(PORTF_BASE_ADDRESS);
    break;
  }
  SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_ptr + DEN_REG_OFFSET),ptr2config->transmit_pin_num);
  SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_ptr + DEN_REG_OFFSET),ptr2config->receive_pin_num);
  SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_ptr + AFSEL_REG_OFFSET),ptr2config->transmit_pin_num);
  SET_BIT(*(volatile uint32 *)((volatile uint8 *)port_ptr + AFSEL_REG_OFFSET),ptr2config->receive_pin_num);
  
  switch(ptr2config->receive_pin_num)
  {
   case 0:
  *(volatile uint32 *)((volatile uint8 *)port_ptr + CTL_REG_OFFSET) = ((*(volatile uint32 *)((volatile uint8 *)port_ptr + CTL_REG_OFFSET)) & (PIN0_PIN1_CTRL_MASK))\
    
  | (UART_PIN0_PIN1_CTRL_MASK);
  break;
  case 4:
    *(volatile uint32 *)((volatile uint8 *)port_ptr + CTL_REG_OFFSET) = ((*(volatile uint32 *)((volatile uint8 *)port_ptr + CTL_REG_OFFSET)) & (PIN4_PIN5_CTRL_MASK))\
    
  | (UART_PIN4_PIN5_CTRL_MASK);
  break;
    case 6:
    *(volatile uint32 *)((volatile uint8 *)port_ptr + CTL_REG_OFFSET) = ((*(volatile uint32 *)((volatile uint8 *)port_ptr + CTL_REG_OFFSET)) & (PIN6_PIN7_CTRL_MASK))\
    
  | (UART_PIN6_PIN7_CTRL_MASK);
  break;
  }
  float32 val = (float32)(ptr2config->clk)/(float32)(16*ptr2config->baud_rate) ; 
  uint32 int_val = (uint32) val ; 
  float32 float_val = (float32)val - (float32)int_val  ; 
  
  uint32 float_reg = (uint32)(float_val*64.0 + 0.5) ; 
  *(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_INTEGER_BAUD_RATE_REG_OFFSET) = ((*(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_INTEGER_BAUD_RATE_REG_OFFSET)) & \
    (UART_INTEGER_BAUD_RATE_MASK)) | (int_val << INTDIV_IBRD_REG_START_BIT);
  
  
  
  *(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_FRACTIONAL_BAUD_RATE_REG_OFFSET) = ((*(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_FRACTIONAL_BAUD_RATE_REG_OFFSET)) & \
    (UART_FRACTIONAL_BAUD_RATE_MASK)) | (float_reg << FLOAT_DIV_FBRD_REG_START_BIT);
  
  *(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_CLOCK_SOURCE_REG_OFFSET) = ((*(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_CLOCK_SOURCE_REG_OFFSET)) & \
    (UART_CLOCK_SOURCE_MASK)) | (UART_CLK_FREQUENCY_MASK);
  
  SET_BIT(*(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_CTRL_REG_OFFSET),UART_CTL_ENABLE_BIT);

  
  *(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_LINE_CTRL_REG_OFFSET) = ((*(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_LINE_CTRL_REG_OFFSET)) & \
    (UART_LINE_CTRL_SOURCE_MASK)) | (ptr2config->length << LENGTH_LINE_CTRL_REG_START_BIT);
  

}


void UART_sendByte(const uint8 data)
{
   while(((*(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_FLAG_RATE_REG_OFFSET)) & (1 << TRANSMIT_FLAG_BIT)) != LOGIC_LOW);
  
  *(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_DATA_REG_OFFSET) = data;
}

uint8 uart_RecieveByte(void)
{
  uint8 recieved_byte;
  while(((*(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_FLAG_RATE_REG_OFFSET)) & (1 << RECIEVE_FLAG_BIT)) != LOGIC_LOW);
  recieved_byte =  *(volatile uint32 *)((volatile uint8 *)uart_ptr + UART_DATA_REG_OFFSET);
  return recieved_byte;
}

