//----------------------------------------------------------------------------------------------------------------------------------
//The touch panel on the scope is not connected to a true I2C interface, so bit banging is used
//
//The connections are:
//  PA0:  RESET
//  PA1:  INT
//  PA2:  SDA
//  PA3:  SCL
//
//The touch panel uses a GT911 controller and has a special startup sequence to set the I2C device address
//
//----------------------------------------------------------------------------------------------------------------------------------

#include "variables.h"
#include "touchpanel.h"

//----------------------------------------------------------------------------------------------------------------------------------

#define TOUCH_CONFIG_ADDRESS       (uint8 *)0x81BFFC1C

//----------------------------------------------------------------------------------------------------------------------------------
//Touch panel configuration for the GT9157 set to 800x480 resolution

#ifdef USE_TP_CONFIG
#define USE_LR_CONFIG

#ifdef USE_LR_CONFIG
uint8 tp_config_data[] =
{
  0xFF, 0x20, 0x03, 0xE0, 0x01, 0x0A, 0xFD, 0x00, 0x01, 0x08, 0x28, 0x08, 0x5A, 0x3C, 0x03, 0x05,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x1A, 0x1E, 0x14, 0x87, 0x29, 0x0A, 0x75, 0x77,
  0xB2, 0x04, 0x00, 0x00, 0x00, 0x9A, 0x01, 0x11, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x50, 0xA0, 0x94, 0xD5, 0x02, 0x08, 0x00, 0x00, 0x04, 0xA1, 0x55, 0x00, 0x8F,
  0x62, 0x00, 0x7F, 0x71, 0x00, 0x73, 0x82, 0x00, 0x69, 0x95, 0x00, 0x69, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12, 0x14, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x04, 0x06, 0x08, 0x0A, 0x0C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x24, 0x26, 0x28, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01
};
#else
uint8 tp_config_data[] =
{
  0xFF, 0x20, 0x03, 0xE0, 0x01, 0x0A, 0xFD, 0x00, 0x01, 0x08, 0x28, 0x08, 0x5A, 0x3C, 0x03, 0x05,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x1A, 0x1E, 0x14, 0x8B, 0x2A, 0x0C, 0x75, 0x77,
  0xB2, 0x04, 0x00, 0x00, 0x00, 0x9A, 0x01, 0x11, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x50, 0xA0, 0x94, 0xD5, 0x02, 0x08, 0x00, 0x00, 0x04, 0xA1, 0x55, 0x00, 0x8F,
  0x62, 0x00, 0x7F, 0x71, 0x00, 0x73, 0x82, 0x00, 0x69, 0x95, 0x00, 0x69, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x18, 0x16, 0x14, 0x12, 0x10, 0x0E, 0x0C, 0x0A, 0x08, 0x06, 0x04, 0x02, 0xFF, 0xFF, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x22,
  0x21, 0x20, 0x1F, 0x1E, 0x1D, 0x1C, 0x18, 0x16, 0x13, 0x12, 0x10, 0x0F, 0x0C, 0x0A, 0x08, 0x06,
  0x04, 0x02, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01, 
};
#endif
#endif

//----------------------------------------------------------------------------------------------------------------------------------

void tp_i2c_setup(void)
{
}

//----------------------------------------------------------------------------------------------------------------------------------

void tp_i2c_wait_for_touch_release(void)
{
}

//----------------------------------------------------------------------------------------------------------------------------------

void tp_i2c_read_status(void)
{
}

//----------------------------------------------------------------------------------------------------------------------------------

void tp_i2c_send_data(uint16 reg_addr, uint8 *buffer, uint32 size)
{
}

//----------------------------------------------------------------------------------------------------------------------------------

void tp_i2c_read_data(uint16 reg_addr, uint8 *buffer, uint32 size)
{
}

//----------------------------------------------------------------------------------------------------------------------------------

void tp_i2c_send_start(void)
{
}

//----------------------------------------------------------------------------------------------------------------------------------

void tp_i2c_send_stop(void)
{
}

//----------------------------------------------------------------------------------------------------------------------------------

void tp_i2c_clock_ack(void)
{
}

//----------------------------------------------------------------------------------------------------------------------------------

void tp_i2c_clock_nack(void)
{
}

//----------------------------------------------------------------------------------------------------------------------------------

void tp_i2c_send_byte(uint8 data)
{
}

//----------------------------------------------------------------------------------------------------------------------------------

uint8 tp_i2c_read_byte(void)
{
  return(0);
}

//----------------------------------------------------------------------------------------------------------------------------------
//A count of 4 is approximately 3uS when running on 600MHz with cache enabled

void tp_delay(uint32 usec)
#if 0
{
}
#else
{
}
#endif
  
//----------------------------------------------------------------------------------------------------------------------------------
