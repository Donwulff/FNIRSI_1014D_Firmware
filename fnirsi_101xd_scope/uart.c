//----------------------------------------------------------------------------------------------------------------------------------

#include "ccu_control.h"
#include "gpio_control.h"
#include "uart.h"
#include "variables.h"
#include "timer.h"

//One request may be outstanding. Retry a silent controller after 20ms without
//holding up acquisition; pace empty replies even in the modal input waits.
#define UART_POLL_INTERVAL_MS 1
#define UART_REPLY_TIMEOUT_MS 20
static uint8 uart1_poll_pending;
static uint32 uart1_poll_ticks;

//----------------------------------------------------------------------------------------------------------------------------------
//The connections for the UART are on:
//  PA2:  UART1_RX
//  PA3:  UART1_TX
//----------------------------------------------------------------------------------------------------------------------------------

void uart1_init(void)
{
  //PORTA is also used for the communication with the clock synthesizer IC and is setup elsewhere too, so no direct write here
  //After reset the pins are disabled and set to 7. Anding with 5 sets it to the needed value of 5
  *PORTA_CFG0_REG &= 0xFFFF55FF;
//  *PORTA_CFG0_REG |= 0x00005500;

  //Enable UART1 clock
  *CCU_BUS_CLK_GATE2 |= CCU_BCRG2_UART1_EN;

  //De-assert the reset on UART1
  *CCU_BUS_SOFT_RST2 |= CCU_BSRR2_UART1_RST;

  //No modem control used
  *UART1_MC_REG = 0;

  //Enable access to the divisor latch register. Probably no need to or here, just set the bit
  *UART1_LC_REG |= UART_LCR_DLAB;

  //set divisor latch LSB 24/25 * 0x51
  *UART1_DLL_REG = 0x4E;

  //divisor latch MSB left at default 0
  *UART1_DLM_REG = 0;

  //Setup the UART for working with 8 bit data, 1 stop bit and no parity
  *UART1_LC_REG = UART_LCR_WLEN8;

  uart1_poll_pending = 0;
  uart1_poll_ticks = timer0_get_ticks() - UART_REPLY_TIMEOUT_MS;
}

//----------------------------------------------------------------------------------------------------------------------------------

uint8 uart1_receive_data(void)
{
  uint32 status = *UART1_LS_REG;
  uint32 now = timer0_get_ticks();
  uint8 data;

  //Consume a reply before considering a retry, including replies arriving after
  //a long acquisition or modal screen. Never overwrite an unprocessed key.
  if(status & UART_LSR_DR)
  {
    data = *UART1_RX_REG;
    uart1_poll_pending = 0;
    if(status & (UART_LSR_PE | UART_LSR_FE | UART_LSR_BI))
      return 0;
    return data;
  }

  if(uart1_poll_pending && (uint32)(now - uart1_poll_ticks) < UART_REPLY_TIMEOUT_MS)
    return 0;

  if((uint32)(now - uart1_poll_ticks) >= UART_POLL_INTERVAL_MS &&
     (status & UART_LSR_TEMT))
  {
    *UART1_TX_REG = 0xFF;
    uart1_poll_ticks = now;
    uart1_poll_pending = 1;
  }

  return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------

uint8 uart1_get_user_input(void)
{
  //Check if polling the user interface is needed
  //When a command has been received but not processed yet it should be skipped
  if(toprocesscommand == 0)
  {
    //Set the received command in the user interface data to be processed
    toprocesscommand = uart1_receive_data();
  }

  //Return the current active command as a flag to signal actual input has been received
  return(toprocesscommand);
}

//----------------------------------------------------------------------------------------------------------------------------------

void uart1_wait_for_user_input(void)
{
  //Modal confirmation screens deliberately wait for input. The UART transaction
  //itself is still nonblocking and retries if the controller temporarily vanishes.
  while((lastreceivedcommand = uart1_receive_data()) == 0);
}

//----------------------------------------------------------------------------------------------------------------------------------
