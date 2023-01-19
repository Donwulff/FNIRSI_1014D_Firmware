//----------------------------------------------------------------------------------------------------------------------------------
//The clock generator on the scope is not connected to a true I2C interface of the processor, so bit banging is used
//
//The connections are:
//  PA0:  SDA
//  PA1:  SCL
//  PA2:  UART1_RX
//  PA3:  UART1_TX
//
//1013D has touch-I2C on the UART RX/TX pins, so the pin assignments are different
//
//----------------------------------------------------------------------------------------------------------------------------------

#include "variables.h"
#include "port_a.h"
#include "ccu_control.h"
#include "display_lib.h"
#include "scope_functions.h"
#include "fpga_control.h"
#include "statemachine.h"
#include "timer.h"

//----------------------------------------------------------------------------------------------------------------------------------
//ClockGen could use register file from ClockBuilder Pro, but currently there are only few registers set, so setting them separately

void cg_i2c_setup(void)
{
  uint8 command;

  //Make sure all SCL and SDA are at a high level and RX/TX  are at a low level before enabling as output
  *PORT_A_DATA_REG = 0x00000003;

  //Setup all port A pins
  *PORT_A_CFG_REG = 0xFFFF5511;

  // 0x03 3 Output Enable Control -- Disable all clock outputs
  command = 0xFF;
  cg_i2c_send_data(CG_CLK_DIS, &command, 1);

  // Power down clocks - we will rewrite these shortly.
  command = 0x80;
  cg_i2c_send_data(CG_CLK0_CTRL, &command, 1);
  cg_i2c_send_data(CG_CLK1_CTRL, &command, 1);
  cg_i2c_send_data(CG_CLK2_CTRL, &command, 1);

  // PLL setup -- 200MHz && 50MHz config

  // 0x0F 15 PLL Input Source left on default

  // 0x10 16 CLK0 Control -- MS0_INT=Integer |  PLLA | CLK0_SRC=MultiSynth0 | CLK0_IDRV=8mA
  command = 0x4F;
  cg_i2c_send_data(CG_CLK0_CTRL, &command, 1);

  // 0x11 17 CLK1 Control -- PLLA | CLK1_SRC=MultiSynth1 | CLK1_IDRV=8mA
  command = 0x0F;
  cg_i2c_send_data(CG_CLK1_CTRL, &command, 1);

  // 0x12 18 CLK2 Control -- CLK2_PDN | CLK2_SRC=MultiSynth1 | CLK2_IDRV=2mA powered down
  command = 0x8C;
  cg_i2c_send_data(CG_CLK2_CTRL, &command, 1);

  // MultiSync PLLA Divider

  // 0x1B 27 MSNA_P3[7:0] -- No zero denominator
  command = 0x01;
  cg_i2c_send_data(CG_MSNA_P3, &command, 1);

  // 0x1d 29 MSNA_P1[15:8] -- ( Integer part - 4 ) * 128
  command = 0x0E; // 800 MHz max
  cg_i2c_send_data(CG_MSNA_P1B, &command, 1);

  // MultiSynth0 -- 200MHz configuration

  // 0x2B 43 MS0_P3[7:0] -- No zero denominator
  command = 0x01;
  cg_i2c_send_data(CG_MS0_P3, &command, 1);

  // 0x2C 44 R0_DIV[2:0] -- MS0_DIVBY4 for 200MHz clock
  command = 0x0C;
  cg_i2c_send_data(CG_MS0_DIV, &command, 1);

  // MultiSynth1 -- 50MHz original

  // 0x33 51 MS1_P3[7:0] -- No zero denominator
  command = 0x01;
  cg_i2c_send_data(CG_MS1_P3, &command, 1);

  // 0x35 53 MS1_P1[15:8] -- Integer part * 128 - 4
  command = 0x06; // 50MHz
//  command = 0x05; // 66MHz
//  command = 0x04; // 75MHz
//  command = 0x03; // 80MHz
  cg_i2c_send_data(CG_MS1_P1B, &command, 1);



  // 0xB1 177 PLL_RST
  command = 0xAC;
  cg_i2c_send_data(CG_PLL_RST, &command, 1);

  // Enable clock outputs
  command = 0xFC;
  cg_i2c_send_data(CG_CLK_DIS, &command, 1);

}

//----------------------------------------------------------------------------------------------------------------------------------


void cg_i2c_send_data(uint16 reg_addr, uint8 *buffer, uint32 size)
{
  //Start a communication sequence
  cg_i2c_send_start();

  //Send the device address for writing
  cg_i2c_send_byte(CG_DEVICE_ADDR_WRITE);

  //Send the register address
  cg_i2c_send_byte(reg_addr);

  //Send the data bytes
  while(size)
  {
    //Send a byte
    cg_i2c_send_byte(*buffer);

    //Point to the next one
    buffer++;

    //One byte done
    size--;
  }

  //Stop the communication sequence
  cg_i2c_send_stop();
}

//----------------------------------------------------------------------------------------------------------------------------------

void cg_i2c_send_start(void)
{
  //Setup all port A pins
  *PORT_A_CFG_REG = 0xFFFF5511;

  //Make SDA high
  *PORT_A_DATA_REG |= 0x00000001;

  //Wait for a while
  cg_delay(CG_DATA_DELAY);

  //Make SCL high
  *PORT_A_DATA_REG |= 0x00000002;

  //Wait for a while
  cg_delay(CG_CLOCK_DELAY);

  //Make SDA low
  *PORT_A_DATA_REG &= 0x0000000E;

  //Wait for a while
  cg_delay(CG_DATA_DELAY);

  //Make SCL low
  *PORT_A_DATA_REG &= 0x0000000D;

  //Wait for a while
  cg_delay(CG_DATA_DELAY);
}

//----------------------------------------------------------------------------------------------------------------------------------

void cg_i2c_send_stop(void)
{
  *PORT_A_CFG_REG = 0xFFFF5511;

  //Make SDA low
  *PORT_A_DATA_REG &= 0x0000000E;

  //Wait for a while
  cg_delay(CG_DATA_DELAY);

  //Make SCL high
  *PORT_A_DATA_REG |= 0x00000002;

  //Wait for a while
  cg_delay(CG_CLOCK_DELAY);

  //Make SDA high
  *PORT_A_DATA_REG |= 0x00000001;

  //Wait for a while
  cg_delay(CG_DATA_DELAY);
}

//----------------------------------------------------------------------------------------------------------------------------------

void cg_i2c_send_byte(uint8 data)
{
  int i;

  //Make sure INT is set as input and the rest of the signals as output
  *PORT_A_CFG_REG = 0xFFFF5511;

  //Send 8 bits
  for(i=0;i<8;i++)
  {
    //Check if bit to send is high or low
    if(data & 0x80)
    {
      //Make SDA high
      *PORT_A_DATA_REG |= 0x00000001;
    }
    else
    {
      //Make SDA low
      *PORT_A_DATA_REG &= 0x0000000E;
    }

    //Wait for a while
    cg_delay(CG_DATA_DELAY);

    //Make SCL high
    *PORT_A_DATA_REG |= 0x00000002;

    //Wait for a while
    cg_delay(CG_CLOCK_DELAY);

    //Make SCL low
    *PORT_A_DATA_REG &= 0x0000000D;

    //Wait for a while
    cg_delay(CG_DATA_DELAY);

    //Select the next bit to send
    data <<= 1;
  }

  //Clock the ack bit
  cg_i2c_clock_ack();
}

//----------------------------------------------------------------------------------------------------------------------------------

void cg_i2c_clock_ack(void)
{
  //Make sure INT is set as input and the rest of the signals as output
  *PORT_A_CFG_REG = 0xFFFF5511;

  //Make SDA low
  *PORT_A_DATA_REG &= 0x0000000E;

  //Wait for a while
  cg_delay(CG_DATA_DELAY);

  //Make SCL high
  *PORT_A_DATA_REG |= 0x00000002;

  //Wait for a while
  cg_delay(CG_CLOCK_DELAY);

  //Make SCL low
  *PORT_A_DATA_REG &= 0x0000000D;

  //Wait for a while
  cg_delay(CG_DATA_DELAY);
}

//----------------------------------------------------------------------------------------------------------------------------------
//A count of 4 is approximately 3uS when running on 600MHz with cache enabled

void cg_delay(uint32 usec)
#if 0
{
  int i;

  for(i=0;i<usec;i++);
}
#else
{
  //Lower then 64 does not work properly, because the panel fails to hold the new configuration when coming from the original code
  unsigned int loops = usec * 90;

  __asm__ __volatile__ ("1:\n" "subs %0, %1, #1\n"  "bne 1b":"=r"(loops):"0"(loops));
}
#endif

//----------------------------------------------------------------------------------------------------------------------------------

void uart1_setup(void)
{
  //Make sure UART1 RX/TX is configured as UART
  *PORT_A_CFG_REG = 0xFFFF5511;

  //enable UART1 clock
  *CCU_BUS_CLK_GATE2 |= 1<<21;

  //reset UART1
  *CCU_BUS_SOFT_RST2 &= ~(1<<21);

  //wait for reset to complete
  cg_delay(100);

  //de-assert reset on UART1
  *CCU_BUS_SOFT_RST2 |= 1<<21;

  //enable Receive Data Available interrupt
  *UART1_IER_REG = UART_IER_RDI;

  //setup FIFO
  *UART1_FCR_REG = UART_FCR_ENABLE_FIFO | UART_FCR_T_TRIG_11 |
                   UART_FCR_CLEAR_XMIT | UART_FCR_CLEAR_RCVR;

  //just make sure it's the default
  *UART1_MCR_REG = 0;

  //access divisor latch register
  *UART1_LCR_REG |= UART_LCR_DLAB;

  //set divisor latch lsb to 0x51 = 81 like original
  //*UART1_DLL_REG = 0x51;

  //set divisor latch lsb 24/25 * 0x51
  *UART1_DLL_REG = 0x4E;

  //divisor latch msb left at default 0
  *UART1_DLM_REG = 0;

  //divisor latch diasable
  *UART1_LCR_REG &= 0xFFFFFF7F;

  //try to mimic original code setting 8 bit data
  *UART1_LCR_REG = ( (*UART1_LCR_REG & 0xFFFFFFE0) | UART_LCR_WLEN8 );

   //wait 8 bit send times
   cg_delay(1000);
}

//----------------------------------------------------------------------------------------------------------------------------------

void display_stage( uint8 stage ) {
#ifdef DEBUG
  display_set_fg_color(0x00FFFFFF);
  display_fill_rect(80, 0, 80, 38);
  display_set_fg_color(0x00000000);
  display_decimal( 80, 10, stage );
#endif /* DEBUG */
}

//----------------------------------------------------------------------------------------------------------------------------------

void uart1_handler(void)
{
  //wait for room in transmit FIFO
  while(!(*UART1_USR_REG & 0x2));

  //write byte into the FIFO
  *UART1_TX_REG = 0xFF;

  //wait for the byte to be transfered
  while(!(*UART1_USR_REG & 0x2));

  //wait for data in receive FIFO
//  while(!(*UART1_USR_REG & 0x8));

  uint8 val = *UART1_RX_REG;
#ifdef DEBUG
  //display read data
  static uint8 sval = 0;
  if(val) {
    sval=val;
    display_set_fg_color(0x00FFFFFF);
    display_fill_rect(0, 0, 80, 38);
    display_set_fg_color(0x00000000);
  }
  display_decimal(10, 10, sval);

  static uint8 slsr = 0;
  uint8 lsr = *UART1_LSR_REG;
  if(lsr) {
    slsr = lsr;
  }
  display_hex(30, 0, 1, slsr);

  static uint8 smsr = 0;
  uint8 msr = *UART1_MSR_REG;
  if(msr) {
    smsr = lsr;
  }
  display_hex(30, 10, 1, smsr);

  display_hex(30, 20, 1, *UART1_DBG_DLL_REG);
#endif /* DEBUG */

  if( GD_KEY_AUTO == val ) {
    scope_do_auto_setup();
  }

  if( GD_KEY_MENU == val ) {
    scope_setup_usb_screen();
  }

  if( GD_KEY_CONF_CH1 == val ) {
    //Enable the channel
    scopesettings.channel1.magnification = (scopesettings.channel1.magnification + 1) % 3;

    //Display this
    scope_channel_settings(&scopesettings.channel1, 0);
  }

  if( GD_KEY_CONF_CH2 == val ) {
    //Enable the channel
    scopesettings.channel2.magnification = (scopesettings.channel2.magnification + 1) % 3;

    //Display this
    scope_channel_settings(&scopesettings.channel2, 0);
  }

  if( GD_KEY_TRIG_MODE == val ) {
    display_stage( 0 );

    scopesettings.triggermode = 0; // (scopesettings.triggermode ^ 2) % 3;

    display_stage( 1 );
    //Set the new mode in the hardware
    fpga_set_trigger_mode();

    display_stage( 2 );
    //Make sure the scope is running
    scopesettings.runstate = 0;

    display_stage( 3 );
    //Show this on the screen
    scope_run_stop_text();

    //Match the volts per division settings for both channels
    //Is needed when vertical zoom has been used
    display_stage( 4 );
    match_volt_per_div_settings(&scopesettings.channel1);
    display_stage( 5 );
    match_volt_per_div_settings(&scopesettings.channel2);

    display_stage( 6 );
  }

  // Okay this starts to be where code should be unified...
  if( GD_KEY_TRIG_CHX == val ) {
    //Set the channel 1 as trigger source
    scopesettings.triggerchannel = scopesettings.triggerchannel ? 0 : 1;

    display_stage( scopesettings.triggerchannel );

    //Update the FPGA
    fpga_set_trigger_channel();

    //Set the trigger vertical position to match channel 1 trace position
    scope_calculate_trigger_vertical_position();

    //Display this
    scope_trigger_channel_select();
  }

  // Should use a time counter somewhere...
  static uint16 traceadj = 1;
  static uint8 tracecmd = 0;
  if( val >= 37 && val <= 40 ) {
    if(val != tracecmd) {
      traceadj = 0;
    }
    traceadj++;
  }
  tracecmd = val;

  if( GD_TRIM_X_CH1_SUB == val ) {
    //Update the current position
    scopesettings.channel1.traceposition -= traceadj;

    //Write it to the FPGA
    fpga_set_channel_offset(&scopesettings.channel1);

    display_stage( 10 );
  }

  if( GD_TRIM_X_CH1_ADD == val ) {
    //Update the current position
    scopesettings.channel1.traceposition += traceadj;

    //Write it to the FPGA
    fpga_set_channel_offset(&scopesettings.channel1);

    display_stage( 11 );
  }

  if( GD_TRIM_Y_CH2_SUB == val ) {
    //Update the current position
    scopesettings.channel2.traceposition -= traceadj;

    //Write it to the FPGA
    fpga_set_channel_offset(&scopesettings.channel2);

    display_stage( traceadj );
  }

  if( GD_TRIM_Y_CH2_ADD == val ) {
    //Update the current position
    scopesettings.channel2.traceposition += traceadj;

    //Write it to the FPGA
    fpga_set_channel_offset(&scopesettings.channel2);

    display_stage( traceadj );
  }

  if( 0 == scopesettings.triggerchannel ) {
    if( GD_TRIM_TRIG_LEVEL_SUB == val ) {
      //Update the current position
      scopesettings.triggerlevel--;

      //Write it to the FPGA
//      fpga_set_channel_offset(&scopesettings.channel1);

      display_stage( traceadj );
    }

    if( GD_TRIM_TRIG_LEVEL_ADD == val ) {
      //Update the current position
      scopesettings.triggerlevel++;

      //Write it to the FPGA
//      fpga_set_channel_offset(&scopesettings.channel1);

      display_stage( traceadj );
    }
  } else {
    if( GD_TRIM_TRIG_LEVEL_SUB == val ) {
      //Update the current position
      scopesettings.triggerlevel--;

      //Write it to the FPGA
//      fpga_set_channel_offset(&scopesettings.channel2);

      display_stage( traceadj );
    }

    if( GD_TRIM_TRIG_LEVEL_ADD == val ) {
      //Update the current position
      scopesettings.triggerlevel++;

      //Write it to the FPGA
//      fpga_set_channel_offset(&scopesettings.channel2);

      display_stage( 13 );
    }
  }

  if( GD_TRIM_SCALE_CH1_ADD == val || GD_TRIM_SCALE_CH1_SUB == val ) {
    if( GD_TRIM_SCALE_CH1_ADD == val ) {
      //Step up to the next setting. (Lowering the setting)
      scopesettings.channel1.displayvoltperdiv++;
    } else {
      //Step up to the next setting. (Lowering the setting)
      scopesettings.channel1.displayvoltperdiv--;
    }

    //Show the change on the screen
    scope_channel_settings(&scopesettings.channel1, 0);

    //Only update the FPGA in run mode
    //For waveform view mode the stop state is forced and can't be changed
    if(scopesettings.runstate == 0)
    {
      //Copy the display setting to the sample setting
      scopesettings.channel1.samplevoltperdiv = scopesettings.channel1.displayvoltperdiv;

      //Set the volts per div for this channel
      fpga_set_channel_voltperdiv(&scopesettings.channel1);
      //Since the DC offset is influenced set that too
      fpga_set_channel_offset(&scopesettings.channel1);

      //Wait 50ms to allow the circuit to settle
      timer0_delay(50);
    }
  }

  if( GD_TRIM_SCALE_CH2_ADD == val || GD_TRIM_SCALE_CH2_SUB == val ) {
    if( GD_TRIM_SCALE_CH2_ADD == val ) {
      //Step up to the next setting. (Lowering the setting)
      scopesettings.channel2.displayvoltperdiv++;
    } else {
      //Step up to the next setting. (Lowering the setting)
      scopesettings.channel2.displayvoltperdiv--;
    }

    //Show the change on the screen
    scope_channel_settings(&scopesettings.channel2, 1);

    //Only update the FPGA in run mode
    //For waveform view mode the stop state is forced and can't be changed
    if(scopesettings.runstate == 0)
    {
      //Copy the display setting to the sample setting
      scopesettings.channel2.samplevoltperdiv = scopesettings.channel2.displayvoltperdiv;

      //Set the volts per div for this channel
      fpga_set_channel_voltperdiv(&scopesettings.channel2);
      //Since the DC offset is influenced set that too
      fpga_set_channel_offset(&scopesettings.channel2);

      //Wait 50ms to allow the circuit to settle
      timer0_delay(50);
    }
  }

  if( GD_TRIM_TIME_ADD == val ) {
    if( scopesettings.samplerate < 16 ) {
      scopesettings.samplerate++;
      fpga_set_sample_rate( scopesettings.samplerate );
      scope_acqusition_settings( 0 );
    }

    if( scopesettings.samplerate > 0 ) {
      scopesettings.timeperdiv--;
      scope_acqusition_settings( 0 );
    }
  }

  if( GD_TRIM_TIME_SUB == val ) {
    if( scopesettings.samplerate > 0 ) {
      scopesettings.samplerate--;
      fpga_set_sample_rate( scopesettings.samplerate );
      scope_acqusition_settings( 0 );
    }

    if( scopesettings.timeperdiv < 23 ) {
      scopesettings.timeperdiv++;
      scope_acqusition_settings( 0 );
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
