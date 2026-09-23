//----------------------------------------------------------------------------------------------------------------------------------

#include "bl_fpga_control.h"

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_init(void)
{
  //First set pin high in data register to avoid spikes when changing from input to output
  FPGA_CLK_INIT();

  //Initialize the three control lines for output
  FPGA_CTRL_INIT();
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_write_cmd(uint8 command)
{
  //Set the control lines for writing a command
  FPGA_CMD_WRITE();

  //Set the bus for writing
  FPGA_BUS_DIR_OUT();

  //Write the data to the bus
  FPGA_SET_DATA(command);

  //Clock the data into the FPGA
  FPGA_PULSE_CLK();
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_write_byte(uint8 data)
{
  //Set the control lines for writing a command
  FPGA_DATA_WRITE();

  //Set the bus for writing
  FPGA_BUS_DIR_OUT();

  //Write the data to the bus
  FPGA_SET_DATA(data);

  //Clock the data into the FPGA
  FPGA_PULSE_CLK();
}

//----------------------------------------------------------------------------------------------------------------------------------

uint16 fpga_read_short(void)
{
  uint16 data;

  //Set the bus for reading
  FPGA_BUS_DIR_IN();

  //Set the control lines for reading a command
  FPGA_DATA_READ();

  //Clock the data to the output of the FPGA
  FPGA_PULSE_CLK();

  //Get the msb
  data = FPGA_GET_DATA() << 8;

  //Clock the data to the output of the FPGA
  FPGA_PULSE_CLK();

  //Get the lsb
  data |= FPGA_GET_DATA();

  //Read the data
  return(data);
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_backlight_brightness(uint8 brightness)
{
  fpga_write_cmd(0x38);
  fpga_write_byte(brightness);
}

//----------------------------------------------------------------------------------------------------------------------------------

uint16 fpga_get_version(void)
{
  fpga_write_cmd(0x06);

  return(fpga_read_short());
}

//----------------------------------------------------------------------------------------------------------------------------------

uint16 fpga_check_ready(void)
{
  int i;
  uint32 attempt;
  uint16 version;

  //Both 1014D bitstreams are supported. No response must not block FEL recovery.
  for(attempt = 0; attempt < FPGA_READY_ATTEMPTS; attempt++)
  {
    version = fpga_get_version();

    if((version == FPGA_VERSION_STOCK) || (version == FPGA_VERSION_CUSTOM_AL3))
    {
      return(version);
    }

    //if not just wait a bit and try it again
    //At 600MHz CPU_CLK 1000 = ~200uS
    for(i=0;i<1000;i++)
    {
      __asm__ ("nop");
    }
  }

  return(0);
}

//----------------------------------------------------------------------------------------------------------------------------------
