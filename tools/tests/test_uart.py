"""Exercise the firmware UART poll with a silent, delayed and recovering controller."""

from pathlib import Path
import subprocess
import tempfile
import unittest

from test_review_fixes import SCOPE, function


class UartTests(unittest.TestCase):
    def test_request_reply_poll(self):
        source = (SCOPE / "uart.c").read_text()
        state = source[source.index("#define UART_POLL_INTERVAL_MS"):source.index("void uart1_init")]
        program = r'''
#include <assert.h>
#include "uart.h"
static volatile uint32 uart_status, received, transmitted;
static uint32 ticks;
static uint8 toprocesscommand;
#undef UART1_LS_REG
#undef UART1_RX_REG
#undef UART1_TX_REG
#define UART1_LS_REG (&uart_status)
#define UART1_RX_REG (&received)
#define UART1_TX_REG (&transmitted)
uint32 timer0_get_ticks(void) { return ticks; }
'''
        program += state + function("uart.c", "uart1_receive_data")
        program += function("uart.c", "uart1_get_user_input")
        program += r'''
int main(void)
{
  uart1_poll_ticks = 0xffffffe0U;
  //Neither a stuck transmitter nor a silent controller can block a call.
  for(ticks = 0; ticks < 100; ticks++) assert(uart1_get_user_input() == 0);
  assert(!transmitted);
  uart_status = UART_LSR_TEMT;
  assert(uart1_get_user_input() == 0 && transmitted == 0xff);
  transmitted = 0;
  for(ticks = 101; ticks < 120; ticks++) assert(!uart1_get_user_input());
  assert(!transmitted);
  assert(!uart1_get_user_input() && transmitted == 0xff);
  //A late reply wins over a retry; the next event stays in RX until dispatched.
  ticks = 1000;
  transmitted = 0;
  received = 8;
  uart_status |= UART_LSR_DR;
  assert(uart1_get_user_input() == 8 && !transmitted);
  received = 12;
  assert(uart1_get_user_input() == 8);
  toprocesscommand = 0;
  assert(uart1_get_user_input() == 12);
  //Damaged bytes are consumed without dispatch; the next poll recovers.
  toprocesscommand = 0;
  uart_status |= UART_LSR_FE;
  assert(!uart1_get_user_input());
  uart_status = UART_LSR_TEMT;
  assert(!uart1_get_user_input() && transmitted == 0xff);
  //Timer wrap preserves the timeout and one-millisecond idle pacing.
  uart1_poll_ticks = 0xfffffff8U;
  uart1_poll_pending = 1;
  transmitted = 0;
  ticks = 11;
  assert(!uart1_get_user_input() && !transmitted);
  ticks = 12;
  assert(!uart1_get_user_input() && transmitted == 0xff);
  uart_status |= UART_LSR_DR;
  received = 0;
  assert(!uart1_get_user_input());
  uart_status = UART_LSR_TEMT;
  transmitted = 0;
  assert(!uart1_get_user_input() && !transmitted);
  ticks++;
  assert(!uart1_get_user_input() && transmitted == 0xff);
  return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix="fnirsi-uart-") as temporary:
            path = Path(temporary)
            (path / "test.c").write_text(program)
            subprocess.run(["gcc", "-O1", "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
                            "-I" + str(SCOPE), str(path / "test.c"), "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], timeout=5, check=True)
