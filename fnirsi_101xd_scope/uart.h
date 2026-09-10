//----------------------------------------------------------------------------------------------------------------------------------

#ifndef UART_H
#define UART_H

//----------------------------------------------------------------------------------------------------------------------------------

#include "types.h"

//----------------------------------------------------------------------------------------------------------------------------------

#define UART1_RX_REG           ((volatile uint32 *)(0x01C25400))
#define UART1_TX_REG           ((volatile uint32 *)(0x01C25400))
#define UART1_DLL_REG          ((volatile uint32 *)(0x01C25400))
#define UART1_DLM_REG          ((volatile uint32 *)(0x01C25404))
#define UART1_IE_REG           ((volatile uint32 *)(0x01C25404))
#define UART1_FC_REG           ((volatile uint32 *)(0x01C25408))
#define UART1_LC_REG           ((volatile uint32 *)(0x01C2540C))
#define UART1_MC_REG           ((volatile uint32 *)(0x01C25410))
#define UART1_LS_REG           ((volatile uint32 *)(0x01C25414))
#define UART1_MS_REG           ((volatile uint32 *)(0x01C25418))
#define UART1_S_REG            ((volatile uint32 *)(0x01C2547C))
#define UART1_DBG_DLL_REG      ((volatile uint32 *)(0x01C254B0))

//----------------------------------------------------------------------------------------------------------------------------------

#define UART_IER_RDI           0x00000001

//----------------------------------------------------------------------------------------------------------------------------------

#define UART_LSR_DR            0x00000001
#define UART_LSR_OE            0x00000002
#define UART_LSR_PE            0x00000004
#define UART_LSR_FE            0x00000008
#define UART_LSR_BI            0x00000010
#define UART_LSR_THRE          0x00000020
#define UART_LSR_TEMT          0x00000040
#define UART_LSR_FIFOERR       0x00000080

//----------------------------------------------------------------------------------------------------------------------------------

#define UART_SR_BUSY           0x00000001
#define UART_SR_TFNF           0x00000002
#define UART_SR_TFE            0x00000004
#define UART_SR_RFNE           0x00000008
#define UART_SR_RFF            0x00000010

//----------------------------------------------------------------------------------------------------------------------------------

#define UART_FCR_ENABLE_FIFO   0x00000001
#define UART_FCR_CLEAR_RCVR    0x00000002
#define UART_FCR_CLEAR_XMIT    0x00000004
#define UART_FCR_T_TRIG_11     0x00000030

//----------------------------------------------------------------------------------------------------------------------------------

#define UART_LCR_DLAB          0x00000080

#define UART_LCR_WLEN8         0x00000003

//----------------------------------------------------------------------------------------------------------------------------------
//1014D key codes (returned by uart1_get_user_input)
//Documentation only — dispatch uses the matching UIC_BUTTON_*/UIC_ROTARY_* codes in statemachine.h.
//0x08=RIGHT / 0x0C=LEFT per pecostm32's confirmed mapping (the original port_a labels had these two swapped).
//----------------------------------------------------------------------------------------------------------------------------------

#define GD_KEY_RUN_STOP 0x01
#define GD_KEY_AUTO 0x02
#define GD_KEY_MENU 0x03
#define GD_KEY_S_PIC 0x04
#define GD_KEY_S_WAV 0x05
#define GD_KEY_H_CUR 0x06
#define GD_KEY_V_CUR 0x07
#define GD_KEY_NAV_RIGHT 0x08
#define GD_KEY_NAV_UP 0x09
#define GD_KEY_NAV_OK 0x0A
#define GD_KEY_NAV_DOWN 0x0B
#define GD_KEY_NAV_LEFT 0x0C
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

void uart1_init(void);

//Requests data from the user interface controller and returns it to the caller
uint8 uart1_receive_data(void);

//Requests data from the user interface controller when no previous command is set and sets it in the toprocesscommand variable
uint8 uart1_get_user_input(void);

//Waits for user input and sets the received command in the lastreceivedcommand variable
void uart1_wait_for_user_input(void);

//After a reply, uart1_receive_data() already sent the next 0xFF. Spin up to
//timeout_ms for that reply. Returns the byte (and sets toprocesscommand); 0
//means idle or timeout so the caller should stop draining.
uint8 uart1_collect_next_command(uint32 timeout_ms);


//----------------------------------------------------------------------------------------------------------------------------------

#endif /* UART_H */

//----------------------------------------------------------------------------------------------------------------------------------
