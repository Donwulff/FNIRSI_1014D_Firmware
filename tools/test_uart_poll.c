#include <stdio.h>
#include <string.h>
#include "../fnirsi_101xd_scope/types.h"
#include "../fnirsi_101xd_scope/uart.h"
#include "../fnirsi_101xd_scope/uart_poll.h"

static int failures = 0;

static void expect_eq(const char *name, uint32 got, uint32 want)
{
  if(got != want)
  {
    printf("FAIL %s: got %u want %u\n", name, got, want);
    failures++;
  }
}

int main(void)
{
  UART1POLL p;
  uint8 a;

  memset(&p, 0, sizeof(p));
  a = uart1_poll_next(&p, UART_LSR_TEMT, 100);
  expect_eq("idle+TEMT action", a, UART1_POLL_TX_FF);
  expect_eq("idle+TEMT state", p.state, UART1_POLL_WAIT_RX);
  expect_eq("idle+TEMT wait_start", p.wait_start, 100);

  a = uart1_poll_next(&p, UART_LSR_DR | UART_LSR_TEMT, 101);
  expect_eq("wait+DR action", a, UART1_POLL_READ_RX);
  expect_eq("wait+DR state", p.state, UART1_POLL_IDLE);

  a = uart1_poll_next(&p, UART_LSR_TEMT, 101);
  expect_eq("kick after read action", a, UART1_POLL_TX_FF);
  expect_eq("kick after read state", p.state, UART1_POLL_WAIT_RX);

  memset(&p, 0, sizeof(p));
  p.state = UART1_POLL_WAIT_RX;
  p.wait_start = 50;
  a = uart1_poll_next(&p, 0, 55);
  expect_eq("wait no DR action", a, UART1_POLL_NOP);
  expect_eq("wait no DR state", p.state, UART1_POLL_WAIT_RX);

  a = uart1_poll_next(&p, 0, 50 + UART1_POLL_TIMEOUT);
  expect_eq("wait timeout action", a, UART1_POLL_NOP);
  expect_eq("wait timeout state", p.state, UART1_POLL_IDLE);

  memset(&p, 0, sizeof(p));
  a = uart1_poll_next(&p, 0, 0);
  expect_eq("idle no TEMT action", a, UART1_POLL_NOP);
  expect_eq("idle no TEMT state", p.state, UART1_POLL_IDLE);

  memset(&p, 0, sizeof(p));
  p.state = UART1_POLL_WAIT_RX;
  p.wait_start = 0;
  a = uart1_poll_next(&p, UART_LSR_TEMT, 1);
  expect_eq("no second TX while waiting", a, UART1_POLL_NOP);
  expect_eq("still WAIT_RX", p.state, UART1_POLL_WAIT_RX);

  memset(&p, 0, sizeof(p));
  p.state = UART1_POLL_WAIT_RX;
  p.wait_start = 0xFFFFFFF0u;
  a = uart1_poll_next(&p, 0, 0xFFFFFFF0u + UART1_POLL_TIMEOUT);
  expect_eq("tick wrap timeout action", a, UART1_POLL_NOP);
  expect_eq("tick wrap timeout state", p.state, UART1_POLL_IDLE);

  memset(&p, 0, sizeof(p));
  p.state = UART1_POLL_IDLE;
  a = uart1_poll_next(&p, UART_LSR_DR | UART_LSR_TEMT, 200);
  expect_eq("idle late DR action", a, UART1_POLL_READ_RX);
  expect_eq("idle late DR state", p.state, UART1_POLL_IDLE);

  if(failures)
  {
    printf("%d failure(s)\n", failures);
    return 1;
  }
  printf("ok\n");
  return 0;
}
