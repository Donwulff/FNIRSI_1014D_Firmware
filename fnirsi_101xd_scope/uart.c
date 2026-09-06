//----------------------------------------------------------------------------------------------------------------------------------

#include "ccu_control.h"
#include "gpio_control.h"
#include "uart.h"
#include "uart_poll.h"
#include "timer.h"
#include "variables.h"

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
}

//----------------------------------------------------------------------------------------------------------------------------------

static UART1POLL uart1_poll;
static uint8 uart1_last_action;

static void uart1_kick(void)
{
  uint8 action;

  action = uart1_poll_next(&uart1_poll, *UART1_LS_REG, timer0_get_ticks());
  if(action == UART1_POLL_TX_FF)
  {
    *UART1_TX_REG = 0xFF;
  }
}

uint8 uart1_receive_data(void)
{
  uint8 action;
  uint8 byte;

  //One 0xFF / one-byte reply. Never issue a second poll while a reply is outstanding.
  action = uart1_poll_next(&uart1_poll, *UART1_LS_REG, timer0_get_ticks());
  uart1_last_action = action;

  if(action == UART1_POLL_TX_FF)
  {
    *UART1_TX_REG = 0xFF;
    return(0);
  }

  if(action == UART1_POLL_READ_RX)
  {
    byte = (uint8)*UART1_RX_REG;
    //Start the next poll now so the GD32 reply overlaps acquire/display, and
    //so sm_handle can drain queued rotary detents in the same frame.
    uart1_kick();
    return(byte);
  }

  return(0);
}

uint8 uart1_collect_next_command(uint32 timeout_ms)
{
  uint32 t0 = timer0_get_ticks();
  uint8 byte;

  while((timer0_get_ticks() - t0) < timeout_ms)
  {
    byte = uart1_receive_data();
    if(uart1_last_action == UART1_POLL_READ_RX)
    {
      toprocesscommand = byte;
      return(byte);
    }
  }

  return(0);
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
  //Wait for the user to push a button or rotate a dial on the front panel of the scope
  while((lastreceivedcommand = uart1_receive_data()) == 0);
}

//----------------------------------------------------------------------------------------------------------------------------------
