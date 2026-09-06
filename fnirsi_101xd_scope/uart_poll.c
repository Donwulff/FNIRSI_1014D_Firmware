//----------------------------------------------------------------------------------------------------------------------------------

#include "uart_poll.h"
#include "uart.h"

//----------------------------------------------------------------------------------------------------------------------------------

uint8 uart1_poll_next(UART1POLL *p, uint32 lsr, uint32 now)
{
  if(p->state == UART1_POLL_WAIT_RX)
  {
    if(lsr & UART_LSR_DR)
    {
      p->state = UART1_POLL_IDLE;
      return UART1_POLL_READ_RX;
    }

    if((now - p->wait_start) >= UART1_POLL_TIMEOUT)
    {
      p->state = UART1_POLL_IDLE;
    }

    return UART1_POLL_NOP;
  }

  //A late reply after WAIT_RX timed out is still sitting in RX. Take it
  //before starting another 0xFF or the next poll would pair with a stale byte.
  if(lsr & UART_LSR_DR)
  {
    return UART1_POLL_READ_RX;
  }

  if(lsr & UART_LSR_TEMT)
  {
    p->state = UART1_POLL_WAIT_RX;
    p->wait_start = now;
    return UART1_POLL_TX_FF;
  }

  return UART1_POLL_NOP;
}

//----------------------------------------------------------------------------------------------------------------------------------
