//----------------------------------------------------------------------------------------------------------------------------------

#ifndef UART_POLL_H
#define UART_POLL_H

#include "types.h"

//----------------------------------------------------------------------------------------------------------------------------------

#define UART1_POLL_IDLE        0
#define UART1_POLL_WAIT_RX     1

//timer0 ticks (1 ms). Bound a missing key-controller reply so WAIT_RX cannot stick forever.
#define UART1_POLL_TIMEOUT     20

#define UART1_POLL_NOP         0
#define UART1_POLL_TX_FF       1
#define UART1_POLL_READ_RX     2

//----------------------------------------------------------------------------------------------------------------------------------

typedef struct
{
  uint8  state;
  uint32 wait_start;
} UART1POLL;

//Advance the 0xFF poll/response state machine from LSR + time. No MMIO.
//TX_FF: caller must write 0xFF to UART1 TX.
//READ_RX: caller must read UART1 RX and return that byte.
//NOP: no register access this step (return 0 to the key layer).
uint8 uart1_poll_next(UART1POLL *p, uint32 lsr, uint32 now);

//----------------------------------------------------------------------------------------------------------------------------------

#endif /* UART_POLL_H */
