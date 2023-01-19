//----------------------------------------------------------------------------------------------------------------------------------

#ifndef PORT_A_H
#define PORT_A_H

//----------------------------------------------------------------------------------------------------------------------------------

#include "types.h"

//----------------------------------------------------------------------------------------------------------------------------------

//Port A registers
#define PORT_A_CFG_REG          ((volatile uint32 *)(0x01C20800))
#define PORT_A_DATA_REG         ((volatile uint32 *)(0x01C20810))

//----------------------------------------------------------------------------------------------------------------------------------

#define CG_DEVICE_ADDR_WRITE    0xC0

#define CG_CLK_DIS              0x00000003

#define CG_CLK0_CTRL            0x00000010
#define CG_CLK1_CTRL            0x00000011
#define CG_CLK2_CTRL            0x00000012

#define CG_MSNA_P3              0x0000001B
#define CG_MSNA_P1B             0x0000001D

#define CG_MS0_P3               0x0000002B
#define CG_MS0_DIV              0x0000002C

#define CG_MS1_P3               0x00000033
#define CG_MS1_P1B              0x00000035

#define CG_PLL_RST              0x000000B1

#define CG_CLOCK_DELAY          10
#define CG_DATA_DELAY           10

//----------------------------------------------------------------------------------------------------------------------------------

#define UART1_RX_REG           ((volatile uint32 *)(0x01C25400+0x00))
#define UART1_TX_REG           ((volatile uint32 *)(0x01C25400+0x00))
#define UART1_DLL_REG          ((volatile uint32 *)(0x01C25400+0x00))
#define UART1_DLM_REG          ((volatile uint32 *)(0x01C25400+0x04))
#define UART1_IER_REG          ((volatile uint32 *)(0x01C25400+0x04))
#define UART_IER_RDI           0x01
#define UART1_FCR_REG          ((volatile uint32 *)(0x01C25400+0x08))
#define UART_FCR_ENABLE_FIFO   0x01
#define UART_FCR_CLEAR_RCVR    0x02
#define UART_FCR_CLEAR_XMIT    0x04
#define UART_FCR_T_TRIG_11     0x30
#define UART1_LCR_REG          ((volatile uint32 *)(0x01C25400+0x0C))
#define UART_LCR_DLAB          0x80
#define UART_LCR_WLEN8         0x03
#define UART1_MCR_REG          ((volatile uint32 *)(0x01C25400+0x10))
#define UART1_LSR_REG          ((volatile uint32 *)(0x01C25400+0x14))
#define UART1_MSR_REG          ((volatile uint32 *)(0x01C25400+0x18))
#define UART1_USR_REG          ((volatile uint32 *)(0x01C25400+0x7C))
#define UART1_DBG_DLL_REG      ((volatile uint32 *)(0x01C25400+0xB0))

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------

void uart1_setup(void);

void uart1_handler(void);

void cg_i2c_setup(void);

void cg_i2c_send_data(uint16 reg_addr, uint8 *buffer, uint32 size);

void cg_i2c_send_start(void);
void cg_i2c_send_stop(void);

void cg_i2c_clock_ack(void);
void cg_i2c_clock_nack(void);

void cg_i2c_send_byte(uint8 data);

void cg_delay(uint32 usec);



//----------------------------------------------------------------------------------------------------------------------------------

#endif /* PORT_A_H */
