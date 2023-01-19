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

#define GD_KEY_RUN_STOP 0x01
#define GD_KEY_AUTO 0x02
#define GD_KEY_MENU 0x03
#define GD_KEY_S_PIC 0x04
#define GD_KEY_S_WAV 0x05
#define GD_KEY_H_CUR 0x06
#define GD_KEY_V_CUR 0x07
#define GD_KEY_NAV_LEFT 0x08
#define GD_KEY_NAV_UP 0x09
#define GD_KEY_NAV_OK 0x0A
#define GD_KEY_NAV_DOWN 0x0B
#define GD_KEY_NAV_RIGHT 0x0C
#define GD_KEY_MOVE_SPEED 0x0D
#define GD_KEY_CH1 0x0E
#define GD_KEY_CONF_CH1 0x0F
#define GD_KEY_CH2 0x10
#define GD_KEY_CONF_CH2 0x11
#define GD_KEY_ORIG 0x12
#define GD_KEY_TRIG_MODE 0x13
#define GD_KEY_TRIG_EDGE 0x14
#define GD_KEY_TRIG_CHX 0x15
#define GD_KEY_TRIG_50P 0x16
#define GD_KEY_F1 0x17
#define GD_KEY_F2 0x18
#define GD_KEY_F3 0x19
#define GD_KEY_F4 0x1A
#define GD_KEY_F5 0x1B
#define GD_KEY_F6 0x1C
#define GD_KEY_GEN 0x1D
#define GD_KEY_NEXT 0x1E
#define GD_KEY_LAST 0x1F
#define GD_KEY_DEL 0x20
#define GD_KEY_SEE_ALL 0x21
#define GD_KEY_SEL 0x22
#define GD_TRIM_SEL_ADD 0x23
#define GD_TRIM_SEL_SUB 0x24
#define GD_TRIM_X_CH1_SUB 0x25
#define GD_TRIM_X_CH1_ADD 0x26
#define GD_TRIM_Y_CH2_SUB 0x27
#define GD_TRIM_Y_CH2_ADD 0x28
#define GD_TRIM_ORIG_SUB 0x29
#define GD_TRIM_ORIG_ADD 0x2A
#define GD_TRIM_TRIG_LEVEL_SUB 0x2B
#define GD_TRIM_TRIG_LEVEL_ADD 0x2C
#define GD_TRIM_SCALE_CH1_ADD 0x2D
#define GD_TRIM_SCALE_CH1_SUB 0x2E
#define GD_TRIM_SCALE_CH2_ADD 0x2F
#define GD_TRIM_SCALE_CH2_SUB 0x30
#define GD_TRIM_TIME_SUB 0x31
#define GD_TRIM_TIME_ADD 0x32

#define GD_OFF 0xC8

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
