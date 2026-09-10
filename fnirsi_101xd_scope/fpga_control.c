//----------------------------------------------------------------------------------------------------------------------------------

#include "fpga_control.h"
#include "touchpanel.h"
#include "timer.h"
#include "variables.h"
#include "port_config.h"
#if PORT_1014D
#include "uart.h"
#endif

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

uint8 fpga_read_cmd(void)
{
  //Set the bus for reading
  FPGA_BUS_DIR_IN();
  
  //Set the control lines for reading a command
  FPGA_CMD_READ();

  //Clock the data to the output of the FPGA
  FPGA_PULSE_CLK();
 
  //Read the data
  return(FPGA_GET_DATA());
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

uint8 fpga_read_byte(void)
{
  //Set the bus for reading
  FPGA_BUS_DIR_IN();
  
  //Set the control lines for reading a command
  FPGA_DATA_READ();

  //Clock the data to the output of the FPGA
  FPGA_PULSE_CLK();
 
  //Read the data
  return(FPGA_GET_DATA());
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_write_short(uint16 data)
{
  //Set the control lines for writing a command
  FPGA_DATA_WRITE();

  //Set the bus for writing
  FPGA_BUS_DIR_OUT();
  
  //Write the msb to the bus  
  FPGA_SET_DATA(data >> 8);
  
  //Clock the data into the FPGA
  FPGA_PULSE_CLK();
  
  //Write the lsb to the bus  
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

void fpga_write_int(uint32 data)
{
  //Set the control lines for writing a command
  FPGA_DATA_WRITE();

  //Set the bus for writing
  FPGA_BUS_DIR_OUT();
  
  //Write the msb to the bus  
  FPGA_SET_DATA(data >> 24);
  
  //Clock the data into the FPGA
  FPGA_PULSE_CLK();

  //Write the mlsb to the bus  
  FPGA_SET_DATA(data >> 16);
  
  //Clock the data into the FPGA
  FPGA_PULSE_CLK();
  
  //Write the lmsb to the bus  
  FPGA_SET_DATA(data >> 8);
  
  //Clock the data into the FPGA
  FPGA_PULSE_CLK();
  
  //Write the lsb to the bus  
  FPGA_SET_DATA(data);
  
  //Clock the data into the FPGA
  FPGA_PULSE_CLK();
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_backlight_brightness(uint16 brightness)
{
  fpga_write_cmd(0x38);
  fpga_write_short(brightness);
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_translated_brightness(void)
{
  uint16 data;
  
  //Translate the 0 - 100 brightness value into data for the FPGA
  data = (scopesettings.screenbrightness * 560) + 4000;
  
  //Write it to the FPGA
  fpga_set_backlight_brightness(data);
}

//----------------------------------------------------------------------------------------------------------------------------------

uint16 fpga_get_version(void)
{
  uint16  fw_version;
  fpgasettings.fw_FPGA = 0;
  
  fpga_write_cmd(0x06);
  fw_version = fpga_read_short();
  
  if(fw_version == 0x1432) fpgasettings.fw_FPGA = 1;  //fnirsi fw in FPGA
  if(fw_version == 0x1532) fpgasettings.fw_FPGA = 2;  //PECO fw in FPGA AL3
  if(fw_version == 0x1632) fpgasettings.fw_FPGA = 3;  //PECO fw in FPGA EF2

  return(fw_version);
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_check_ready(void)
{
  int i;
  
  //Check if the FPGA is up and running by reading the version number
  while(fpga_get_version() != 0x1432)
  {
    //if not just wait a bit and try it again
    //At 600MHz CPU_CLK 1000 = ~200uS
    for(i=0;i<1000;i++)
    {
      __asm__ ("nop");
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_enable_system(void)
{
  //This command is only ever set with 1 as data and this function is only called at setup and check hardware
  fpga_write_cmd(0x04);
  fpga_write_byte(0x01);
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_on_off_generator(void)
{
  if (fpgasettings.gen_enable == 1)
  {
    fpga_set_generator_freq(fpgasettings.gen_freq);
    fpga_set_generator_duty(fpgasettings.gen_duty);
    
    // Send the command for switch output
    fpga_write_cmd(0x50);

    // Switch output to on
    fpga_write_int(1);
  }
  else
  { // To be safe, we will turn off signal generation.
    fpga_set_generator_freq(100); //100Hz
    fpga_set_generator_duty(0);   //duty 0%
    
    // Send the command for switch output
    fpga_write_cmd(0x50);

    // Switch output off to tristate
    fpga_write_int(0);
  }
}

//------------------------------------------------------------------------------

void fpga_set_generator_freq(uint32 freq_hz)
{
  //Fout = (phase_inc * Fclk) / 2^32
  //phase_inc = (Fout * 2^32) / Fclk
  //phase_inc = (Fout * 4294967296) / 50000000

  uint64 t = (uint64)freq_hz * 4294967296ULL;

  // Send the command for settings frequency
  fpga_write_cmd(0x51);

  // Set frequency - send divider (32-bit)
  fpga_write_int((t / 50000000ULL));

}

//------------------------------------------------------------------------------

void fpga_set_generator_duty(uint8 duty_pct) //50 is 50%
{
    // threshold = duty_pct * (2^32 - 1) / 100
    uint64 d = (uint64)duty_pct * 0xFFFFFFFFULL;

    //Send the command for settings duty cycle
    fpga_write_cmd(0x52);

    //Set duty cycle
    fpga_write_int((uint32)(d / 100ULL));
}
  
//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_channel_enable(PCHANNELSETTINGS settings)
{
  //Send the command for channel enable to the FPGA
  fpga_write_cmd(settings->enablecommand);
  
  //Write the needed data based on the setting
  if(settings->enable == 0)
  {
    //Disable the channel
    fpga_write_byte(0x00);
  }
  else
  {
    //Enable the channel
    fpga_write_byte(0x01);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_channel_coupling(PCHANNELSETTINGS settings)
{
  //Send the command for channel 1 coupling to the FPGA
  fpga_write_cmd(settings->couplingcommand);
  
  //Write the needed data based on the setting
  if(settings->coupling == 0)
  {
    //Set the DC coupling for the channel
    fpga_write_byte(0x01);
  }
  else
  {
    //Set the AC coupling for the channel
    fpga_write_byte(0x00);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_channel_voltperdiv(PCHANNELSETTINGS settings)
{
  register uint32 setting = settings->samplevoltperdiv;
  
  //Send the command for channel 1 volts per div to the FPGA
  fpga_write_cmd(settings->voltperdivcommand);
  
  //Check if setting in range of what is allowed
  if(setting > 5)
    setting = 5;
  
  //Write it to the FPGA
  fpga_write_byte(setting);
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_channel_offset(PCHANNELSETTINGS settings)
{
  //uint32 offset;
    
  //Send the command for channel DC offset to the FPGA scopesettings.channel1.traceposition
  fpga_write_cmd(settings->offsetcommand);

  //Write the center offset data for this channel and volt per div setting
  //fpga_write_short(settings->dc_calibration_offset[settings->samplevoltperdiv];
  fpga_write_short(settings->dc_calibration_offset[settings->samplevoltperdiv]+(settings->dcoffset));
  //fpga_write_short(settings->dc_calibration_offset[settings->samplevoltperdiv]-(settings->dcoffset));
  //offset = (signal_adjusters[0] * (settings->dcoffset)/2) / signal_adjusters[settings->samplevoltperdiv];
  
  //offset = (multiply * (settings->traceposition)) / signal_adjusters[settings->displayvoltperdiv];
  
  //fpga_write_short(settings->dc_calibration_offset[settings->samplevoltperdiv]-(offset));
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_sample_rate(uint32 samplerate)
{
  //Translate the time per div setting to what the FPGA needs
  //22-11-2021 Since the long time base settings have been removed and the system does things differently a translation is needed here
  //Old time base was 0 for 50S/div to 29 for 10nS/div
  //This is no longer the case. 0 means 200mS/div but needs the setting for 50mS/div. Same for 100mS/div. Simplest solution is a translation table
  
  //This most likely needs to be based on the acquisition speed and no longer the time per div setting, but that needs further investigation
  
  //Make sure the setting is in range of the table
  if(samplerate < (sizeof(sample_rate_settings) / sizeof(uint32)))
  {
    //Write the command to the FPGA
    fpga_write_cmd(0x0D);

    //Write the time base clock setting to the FPGA
    fpga_write_int(sample_rate_settings[samplerate]);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_trigger_channel(void)
{
  //Send the command for selecting the trigger channel to the FPGA
  fpga_write_cmd(0x15);
  
  //Write the needed data based on the setting
  if(scopesettings.triggerchannel == 0)
  {
    //Set channel 1 as trigger input
    fpga_write_byte(0x00);
  }
  else
  {
    //Set channel 2 as trigger input
    fpga_write_byte(0x01);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_trigger_edge(void)
{
  //Send the command for selecting the trigger channel to the FPGA
  fpga_write_cmd(0x16);
  
  //Write the needed data based on the setting
  if(scopesettings.triggeredge == 0)
  {
    //Set trigger edge to rising
    fpga_write_byte(0x00);
  }
  else
  {
    //Set trigger edge to falling
    fpga_write_byte(0x01);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_swap_trigger_channel(void)
{
  //Check if channel 1 is disabled or channel 2 is enabled
  if((scopesettings.channel1.enable == 0) || (scopesettings.channel2.enable))
  {
    //Check if both channels are disabled
    if(scopesettings.channel2.enable == 0)
    {
      //If so do nothing
      return;
    }
    
    //Else check if channel 1 is enabled
    if(scopesettings.channel1.enable)
    {
      //Assume the correct setting is already in the FPGA and do nothing
      return;
    }
    
    //Only channel 2 enabled so set it as source
    scopesettings.triggerchannel = 1;
  }
  else
  {
    //Only channel 1 enabled so set that as source
    scopesettings.triggerchannel = 0;
  }
  
  //Write it the new setting to the FPGA
  fpga_set_trigger_channel();
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_trigger_level(void)
{
  uint32 voltperdiv;
  uint32 traceposition;
  int32 dcoffset;
  int32  level;
  
  //The trace position on screen equals to ADC reading 128
  //The trigger vertical position is relative to the trace position.
  //The screen position values are volt/div related, so conversion to ADC level is needed
  
  //Check which channel is used for triggering
  if(scopesettings.triggerchannel == 0)
  {
    //Channel 1, so use its data
    voltperdiv    = scopesettings.channel1.samplevoltperdiv;
    traceposition = scopesettings.channel1.traceposition;
    //dcoffset = scopesettings.channel1.dcoffset/2;
    //dcoffset = (scopesettings.channel1.dcoffset*100)/220;//195
    dcoffset = (scopesettings.channel1.dcoffset*100)/scopesettings.channel1.dc_shift_size;
    scopesettings.channel1.triggerverticalposition=scopesettings.triggerverticalposition;
  }
  else
  {
    //Channel 2, so use its data
    voltperdiv    = scopesettings.channel2.samplevoltperdiv;
    traceposition = scopesettings.channel2.traceposition;
    //dcoffset = scopesettings.channel2.dcoffset/2;
    //dcoffset = (scopesettings.channel2.dcoffset*100)/220;//230
    dcoffset = (scopesettings.channel2.dcoffset*100)/scopesettings.channel2.dc_shift_size;
    scopesettings.channel2.triggerverticalposition=scopesettings.triggerverticalposition;
  }
  
  //The difference between the two positions determines the level offset on 128, but it needs to be scaled back first
  //level = ((((int32)scopesettings.triggerverticalposition - (int32)traceposition) * 4194304) / signal_adjusters[voltperdiv]) + 128;
  //level = ((((int32)scopesettings.triggerverticalposition - (int32)traceposition) * 2100000) / signal_adjusters[voltperdiv]) + 130;
  //level = ((((int32)scopesettings.triggerverticalposition - (int32)traceposition) * 2097152) / signal_adjusters[voltperdiv]) + 128;
  
    level = ((((int32)(scopesettings.triggerverticalposition - dcoffset) - (int32)traceposition) * 2097152) / signal_adjusters[voltperdiv]) + 128;
  //level = scopesettings.channel1.center-255+traceposition+scopesettings.triggerverticalposition;

  //Set the new level in the settings
  scopesettings.triggerlevel = level;

  //Limit on extremes
  if(level < 0)
  {
    level = 0;
  }
  else if(level > 255)
  {
    level = 255;
  }
  
  //Send the command for setting the trigger level to the FPGA
  fpga_write_cmd(0x17);
  
  //Write the actual level to the FPGA
  fpga_write_byte(level);
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_trigger_mode(void)
{
  //Send the command for selecting the trigger mode to the FPGA
  fpga_write_cmd(0x1A);
  
  //Write the needed data based on the setting
  if(scopesettings.triggermode == 0)
  {
    //Set trigger mode to auto
    fpga_write_byte(0x00);
  }
  else
  {
    //Set trigger mode to single or normal
    fpga_write_byte(0x01);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------
//short time base
void fpga_set_time_base(uint32 timebase) //len pre rezim auto, urcuje max cas po ktory sa odoberaju vzorky
{
  //Make sure setting is in range
  if(timebase < (sizeof(timebase_settings) / sizeof(uint32)))
  {
    //Write the command to set the short time base data to the FPGA
    fpga_write_cmd(0x0E);
    
    //Table settings ranges from setting 0 (200mS/div) to 23 (5nS/div)
    if(fpgasettings.fw_FPGA == 1)
    {
    fpga_write_int(timebase_settings[timebase]);
    }
      else fpga_write_int(scopesettings.samplecount);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_set_long_timebase(uint32 timebase)
{
  //Send the data, which is always 2000
  //fpga_write_int(2000);//0x07d0 is 2000
  
  //Make sure setting is in range
  if(timebase < (sizeof(timebase_settings) / sizeof(uint32)))
  {
    //Send the command for long time base to the FPGA
    //fpga_write_cmd(0x0D);
  
    //Table settings ranges from setting 0 (200mS/div) to 23 (5nS/div)
    //fpga_write_int(timebase_settings[timebase]);
  }
 
  //The command 0x0D sets the rate with which the ADC is clocked. The 0x07D0 means 100KSa/s.
  //With the command 0x28 the mode is selected. Needs to be set to 0x01 for the roll mode sampling. Not sure if it is needed for every read of the sampl
  
  //Disable trigger system???
  fpga_write_cmd(0x0F);
  fpga_write_byte(0x01);

#if PORT_1014D
  if(fpgasettings.fw_FPGA == 1)
  {
    //Stock roll-entry "reset": 0x0D <- 0x27 then the run rate + mode select
    //(pecostm32-RE "FPGA explained".txt:218-219; seen at switch-to-roll and after touch).
    //The 0x27 write is the least-understood byte here -- first thing to drop on bench trouble.
    fpga_write_cmd(0x0D);
    fpga_write_int(0x00000027);

    fpga_arm_long_timebase_cycle();
  }
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
//Roll-mode per-read-cycle arm (F25's other half). The stock FPGA is fed 0x0D (rate) + 0x28/0x01
//before EVERY 0x24/0x26 read cycle, not just at mode entry: pecostm32-RE "FPGA explained".txt:202
//and the 100mS_50S-div capture lines 63-82 (0x0D 0x000007D0 -> 0x28 0x01 -> 0x24 x10 -> 0x26 x10,
//repeating). Stock writes 0x000007D0 for ALL roll timebases (table lines 205-214) and paces the
//reads with a timer, exactly like scope_get_long_timebase_data's timeout loop does.
//Stock fw only: PECO FPGAs (fw_FPGA 2/3) have their own long-timebase handling. 1014D only:
//the 1013D variant stays at Atlan4's shipped behavior (no hardware to test -- policy 2026-08-21).
//Applied 2026-08-21, hw-verify pending (REVIEW-2026-08-21.md follow-up).
void fpga_arm_long_timebase_cycle(void)
{
#if PORT_1014D
  if(fpgasettings.fw_FPGA == 1)
  {
    //ADC clock rate for roll mode: stock always 0x000007D0 here (fpga_write_int is MSB-first,
    //matching the capture byte order 0x00 0x00 0x07 0xD0)
    fpga_write_cmd(0x0D);
    fpga_write_int(0x000007D0);

    //Mode select: 0x01 = roll (counterpart of the 0x28/0x00 fast select in fpga_do_conversion)
    fpga_write_cmd(0x28);
    fpga_write_byte(0x01);
  }
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------

#if PORT_1014D
static uint8 fpga_abort_on_key;
#endif

void fpga_conversion_abort_on_key(uint8 enable)
{
#if PORT_1014D
  fpga_abort_on_key = enable;
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_do_conversion(void)
{ 
  scopesettings.conversion_done = 0;    //flag for conversion, 0-start
#if PORT_1014D
  //Auto never waits — keep the banner latched. Normal/Single wait from arm to edge.
  if(scopesettings.triggermode)
  {
    display_triggered = 0;
  }
#endif
  
  //Check if sampling with trigger system enabled
  if(scopesettings.samplemode == 1)
  {
    //Enable trigger system???
    fpga_write_cmd(0x0F);
    fpga_write_byte(0x00);
  }
  else
  {
    //Disable the trigger circuit??
    fpga_write_cmd(0x0F);
    fpga_write_byte(0x01);
  }

  //Set the FPGA for dual ADC mode + short time base mode. Atlan4 commented these out on a wrong
  //"0x28 not support" guess (see line ~603), but pecostm32-RE's bus captures prove the STOCK 1014D
  //FPGA is fed 0x28 every conversion -- FPGA explained.txt:76 ("mode select. 0x00 for all time base
  //below 100mS, 0x01 for 100mS and up"), confirmed in every *_signal_data_read_sequence.txt at fast
  //timebases (0x28/0x00). pecostm32's firmware -- clean, no interleave comp, on this exact stock unit
  //-- sends 0x29/0x01 then 0x28/0x00 right here. Omitting them left our dual-ADC interleave in a
  //default, unconfigured mode: the leading remaining suspect for our oversized sawtooth. Stock FPGA
  //only (fw_FPGA==1); the PECO fw_FPGA>1 paths keep their own 0x0B/0x0C setup below. (PORT_AUDIT.md F25)
  //The #if PORT_1014D deliberately leaves the 1013D variant at Atlan4's shipped behavior, even though
  //pecostm32's 1013D upstream sends 0x28/0x00 unconditionally right here -- no 1013D hardware to test
  //on (policy 2026-08-21). Candidate fix if a real 1013D shows the <=200ns/div interleave sawtooth.
#if PORT_1014D
  if(fpgasettings.fw_FPGA == 1)
  {
    //Dual ADC mode (pecostm32 sends this; not present in the stock-firmware capture, harmless on stock)
    fpga_write_cmd(0x29);
    fpga_write_byte(0x01);

    //Short time base mode (0x00 = fast / <100mS; roll / >=100mS would use 0x01)
    fpga_write_cmd(0x28);
    fpga_write_byte(0x00);
  }
#endif

  //Reset the sample system
  fpga_write_cmd(0x01);
  fpga_write_byte(0x01);
  
  //Send check on ready command
  fpga_write_cmd(0x05);
  
  //Wait for the flag to become 1 (bounded: on the 1014D FPGA this can never assert if the sample
  //clock/handshake differs, which would hang the whole main loop here before it reaches input
  //handling. Time out and continue so the scope stays responsive instead of freezing.)
  //1014D acquire-only: UART poll after 20 ms, abort the arm. Do not call sm_handle here —
  //that writes FPGA commands and the next fpga_read_byte is then not the 0x05 flag.
  {
    uint32 fpga_ready_timeout = 0;
#if PORT_1014D
    uint8 aborted = 0;
    uint32 t0 = timer0_get_ticks();
#endif
    while((fpga_read_byte() & 1) == 0)
    {
      if(++fpga_ready_timeout > 2000000) break;
#if PORT_1014D
      if(fpga_abort_on_key && ((timer0_get_ticks() - t0) > 20) && uart1_get_user_input())
      {
        scopesettings.display_data_done = 1;
        aborted = 1;
        break;
      }
#endif
    }

#if PORT_1014D
    if(!aborted)
#endif
    {
      //Test again to make sure it was no glitch. pecostm32 double-reads the reset-ready flag here
      //("make sure it was no glitch") and Atlan4 had commented it out. With the 0x28 mode-select now
      //engaging the proper fast dual-ADC capture, a premature ready read surfaces as occasional noise
      //peaks (was masked by the sawtooth before F25). Restored -- bounded like the first wait so we
      //keep Atlan4's anti-hang timeout instead of pecostm32's unbounded spin. (PORT_AUDIT.md F26)
      fpga_ready_timeout = 0;
      while((fpga_read_byte() & 1) == 0)
      {
        if(++fpga_ready_timeout > 2000000) break;
#if PORT_1014D
        if(fpga_abort_on_key && ((timer0_get_ticks() - t0) > 20) && uart1_get_user_input())
        {
          scopesettings.display_data_done = 1;
          aborted = 1;
          break;
        }
#endif
      }
    }
#if PORT_1014D
    speed_note_t05_ms(timer0_get_ticks() - t0);
#endif
  }

  //posielat ked je sytem v resete, aby neprepisal udaje zachodu
  
  if(fpgasettings.fw_FPGA > 1) 
  {
    //Write the command to set the pretrigger_samples to the FPGA
    fpga_write_cmd(0x0B);
    fpga_write_short(fpgasettings.settriggerpoint/2);

    //Write the command to set the total_samples to the FPGA
    fpga_write_cmd(0x0C);
    fpga_write_short(fpgasettings.totalsamples/2);

    //set time to automode
    fpga_set_time_base(scopesettings.timeperdiv);
  }
  
  //Done with reset
  fpga_write_cmd(0x01);
  fpga_write_byte(0x00);
}
//----------------------------------------------------------------------------------------------------------------------------------

uint8 fpga_done_conversion(void)
{
  //Send check on triggered or buffer full command to the FPGA
  fpga_write_cmd(0x0A);
  
  //Check if sampling with trigger system enabled
  if(scopesettings.samplemode == 1)
  {
    if (((fpga_read_byte() & 1) == 1))
    {
        //Disable trigger system???
        fpga_write_cmd(0x0F);
        fpga_write_byte(0x01);
        scopesettings.conversion_done = 1;
    }
  }
  else
  {
    //Check for the FPGA to signal triggered or buffer full
    if (((fpga_read_byte() & 1) == 1)) scopesettings.conversion_done = 1;
  }
  
  //Scan the touch panel
#if PORT_1014D
  //1014D: no touch panel, and tp_i2c_read_status() rewrites PORT_A_CFG_REG to the touch I2C config
  //(0xFFFF1101) — tearing PA2/PA3 out of UART mode every acquisition and jamming the key-controller
  //lines. Skip it so the UART key input keeps working.
#else
  tp_i2c_read_status();
  if (havetouch == 1) scopesettings.display_data_done = 1;
#endif
      
  return (scopesettings.conversion_done);
}

//----------------------------------------------------------------------------------------------------------------------------------

uint16 fpga_prepare_for_transfer(void)
{
  uint32 data;
  
  //Send the command for getting some data from the FPGA
  fpga_write_cmd(0x14);
  
  //Just read a short, and with 0x0FFF and add 2
  //Have to see if the add 2 is needed
  //data = (fpga_read_short() & 0x0FFF) + 2;    
  data = (fpga_read_short() & 0x1FFF);  //change 11.12.2025
 
  return(data & 0x1FFF);
}

//----------------------------------------------------------------------------------------------------------------------------------

//See https://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Binary_numeral_system_.28base_2.29
uint32 isqrt(uint32 n) {
  uint32 x = n;
  uint32 c = 0;
  uint32 d = 1 << 30;

  while (d > n) d >>= 2;

  while (d != 0) {
    if (x >= c + d) {
      x -= c + d;
      c = (c >> 1) + d;
    } else {
      c >>= 1;
    }
    d >>= 2;
  }
  return c;
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_read_sample_data(PCHANNELSETTINGS settings, uint32 triggerpoint)
{
  uint32 threshold;
  
  //Clear min and max and average for new calculations
  settings->min     = 0x7FFFFFFF;
  settings->max     = 0;
  settings->average = 0;
  settings->rms     = 0;
  
  settings->minDC     = 0x7FFFFFFF;
  settings->maxDC     = 0;
  
  //Send command 0x1F to the FPGA followed by the translated data returned from command 0x14
  fpga_write_cmd(0x1F);
  fpga_write_short(triggerpoint);
  
  //Send a command for getting ADC1 trace data from the FPGA
  fpga_write_cmd(settings->adc1command);
  
  //Signal no checking on the first ADC when compensating
  settings->checkfirstadc = 0;
  
  //Setup for reading the first ADC
  settings->compensation = settings->adc1compensation;
  settings->buffer = &settings->tracebuffer[1];
  
  //Read the data for the first ADC. Samples start on the second location, skipping every other sample
  fpga_read_adc_data(settings);

  //Save the calculated measurements
  settings->adc1rawaverage = settings->rawaverage;
  

  //Calculate the signal center for frequency determination
  //Calculate the signal center for autosetup dc shift  
  settings->centerDC = (settings->maxDC + settings->minDC) / 2;
  
  //Calculate the peak to peak value for autosetup dc shift
  settings->peakpeakDC = settings->maxDC - settings->minDC;
  
    //Calculate the signal center for frequency determination
  //settings->center = (settings->max + settings->min) / 2;
  
  //Calculate a zero crossing threshold value
  threshold = ((settings->maxDC - settings->minDC) / 10) + 2;
  
  //Set levels for detecting zero crossings
  settings->highlevel = settings->centerDC + threshold;
  settings->lowlevel  = settings->centerDC - threshold;
  
  
  /*
  //Calculate the signal center for frequency determination
  settings->center = (settings->max + settings->min) / 2;
  
  //Calculate a zero crossing threshold value
  threshold = ((settings->max - settings->min) / 10) + 2;
  
  //Set levels for detecting zero crossings
  settings->highlevel = settings->center + threshold;
  settings->lowlevel  = settings->center - threshold;
  */
  
  //Initialize the frequency determination
  settings->zerocrossings   = 0;
  settings->lowsamplecount  = 0;
  settings->lowdivider      = 0;
  settings->highsamplecount = 0;
  settings->highdivider     = 0;
  
  settings->previousindex = 0;
  
  //See in which state the signal starts
  if(settings->tracebuffer[1] > settings->highlevel)
  {
    //Above high it is a one
    settings->state = 1;
  }
  else
  {
    //Below start with a zero
    settings->state = 0;
  }
  
  //Send command 0x1F to the FPGA followed by the translated data returned from command 0x14
  fpga_write_cmd(0x1F);
  fpga_write_short(triggerpoint);
  
  //Send a command for getting ADC2 trace data from the FPGA
  fpga_write_cmd(settings->adc2command);

  //Signal to check the readings of the first ADC on being zero when compensating
  settings->checkfirstadc = 1;

  //Setup for reading the first ADC
  settings->compensation = settings->adc2compensation;
  settings->buffer = &settings->tracebuffer[0];
  
  //Read the data for the second ADC. Samples start on the first location, skipping every other sample
  fpga_read_adc_data(settings);

  //Save the calculated measurements
  settings->adc2rawaverage = settings->rawaverage;
  
  //Preserve the pre-division compensated sample sum so ui_display_vavg can recover the
  //oversampling resolution -- dividing to a whole ADC code here throws it away (PORT_AUDIT F28)
  settings->averagesum = settings->average;

  //Calculate the overall average
  settings->average /= scopesettings.samplecount;

  //Calculate the RMS
  settings->rms = isqrt(settings->rms / scopesettings.samplecount);

  //Calculate the peak to peak value
  settings->peakpeak = settings->max - settings->min;
  
  //Calculate the signal center
  settings->center = (settings->max + settings->min) / 2;
  
  //===================================
  /*
  //Calculate the frequency if possible
#define FP_SHIFT 20

if(settings->zerocrossings > 2 &&
   settings->highdivider &&
   settings->lowdivider)
{
  settings->frequencyvalid = 1;

  // Average HIGH time (Q16.16 samples)
  settings->hightime =
    ((uint64)settings->highsamplecount << FP_SHIFT)
    / settings->highdivider;

  // Average LOW time (Q16.16 samples)
  settings->lowtime =
    ((uint64)settings->lowsamplecount << FP_SHIFT)
    / settings->lowdivider;

  // FULL PERIOD = 2 × (HIGH + LOW)
  settings->periodtime =
    (settings->hightime + settings->lowtime) << 1;

  // Frequency
  settings->frequency =
    ((uint64)freq_calc_data[scopesettings.samplerate].sample_rate << FP_SHIFT)
    / settings->periodtime;
}
else
{
  settings->frequencyvalid = 0;
}

*/
  
  //=============================
/*
    // --------------------------------------------------------------
     // FREQUENCY + TIME CALCULATION
     // --------------------------------------------------------------
    if ((settings->zerocrossings >= 4) && (settings->highdivider > 0) && (settings->lowdivider > 0))
    {
        // Average HIGH and LOW time in SAMPLES 
        settings->hightime = settings->highsamplecount / settings->highdivider;

        settings->lowtime = settings->lowsamplecount / settings->lowdivider;

        settings->periodtime = settings->hightime + settings->lowtime;

        if (settings->periodtime > 0)
        {
            settings->frequency = (freq_calc_data[scopesettings.samplerate].sample_rate / settings->periodtime) / 2;

            settings->frequencyvalid = 1;
        }
        else
        {
            settings->frequencyvalid = 0;
        }
    }
    else
    {
        settings->frequencyvalid = 0;
    }
  
uint32 sr = freq_calc_data[scopesettings.samplerate].sample_rate;

settings->hightime =
    (uint64)settings->hightime * 1000000ULL / sr;

settings->lowtime =
    (uint64)settings->lowtime * 1000000ULL / sr;

        settings->periodtime = settings->hightime + settings->lowtime;
*/

//}

  
  
  
//=========================================
  

   //Calculate the frequency if possible
  if(settings->zerocrossings > 2)
  {
    //Signal valid frequency determination possible
    settings->frequencyvalid = 1;
    

                     //Calculate the average high time of the signal expressed in samples
    settings->hightime = (((uint64)settings->highsamplecount << 16) / settings->highdivider) * 2;
    
    //Calculate the average low time of the signal expressed in samples
    settings->lowtime = (((uint64)settings->lowsamplecount << 16) / settings->lowdivider) * 2;

    //With the Si5351 override the ADCs run above the nominal rate and the sample-domain times
    //shrink accordingly; rescale so the nominal-rate conversions stay true (the frequency below
    //and the time readouts downstream; duty is a ratio and unaffected) -- PORT_AUDIT F33
    if(sampling_clock_scale != 1.0)
    {
      settings->hightime = (uint32)((double)settings->hightime / sampling_clock_scale + 0.5);
      settings->lowtime  = (uint32)((double)settings->lowtime  / sampling_clock_scale + 0.5);
    }

    //Calculate the period time expressed in samples
    settings->periodtime = settings->hightime + settings->lowtime;
 
      //Calculate the frequency
    settings->frequency = ((uint64)freq_calc_data[scopesettings.samplerate].sample_rate << 16) / settings->periodtime;
 
 
/*
    settings->hightime = (settings->highsamplecount * 2) / settings->highdivider;

    settings->lowtime = (settings->lowsamplecount * 2) / settings->lowdivider;

    settings->periodtime = settings->hightime + settings->lowtime;

    settings->frequency = (uint64)freq_calc_data[scopesettings.samplerate].sample_rate / settings->periodtime;

*/
    
    //Calculate the frequency
   // settings->frequency = ((uint64)freq_calc_data[scopesettings.samplerate].sample_rate << 20) / settings->periodtime;
/*
     
            settings->hightime = settings->highsamplecount / settings->highdivider;

        settings->lowtime = settings->lowsamplecount / settings->lowdivider;

        settings->periodtime = settings->hightime + settings->lowtime;


            settings->frequency = (freq_calc_data[scopesettings.samplerate].sample_rate / settings->periodtime) / 2;
 */
            
/*
 
                 //Calculate the average high time of the signal expressed in samples
    settings->hightime = (((uint64)settings->highsamplecount << 20) / settings->highdivider) * 2;
    
    //Calculate the average low time of the signal expressed in samples
    settings->lowtime = (((uint64)settings->lowsamplecount << 20) / settings->lowdivider) * 2;
    
    //Calculate the period time expressed in samples
    settings->periodtime = settings->hightime + settings->lowtime;
 */
            
  }
  else
  {
    //Signal no valid frequency determination possible
    settings->frequencyvalid = 0;
  }
  
}

//----------------------------------------------------------------------------------------------------------------------------------

void fpga_read_adc_data(PCHANNELSETTINGS settings)
{
  register int32  sample;
  register uint32 count;
  register uint32 sum = 0;
  
  //static uint64 global_sample_counter = 0;
  
  //Set the bus for reading
  FPGA_BUS_DIR_IN();
  
  //Set the control lines for reading a command
  FPGA_DATA_READ();
  
  //Set the number of samples to read
  count = scopesettings.nofsamples;
  //count = fpgasettings.totalsamples/2;
  
  //Read the data as long as there is count
  while(count)
  {
    //Clock the data to the output of the FPGA
    FPGA_PULSE_CLK();

    //Read the data
    sample = FPGA_GET_DATA();
    
    //Sum the raw data for ADC difference calibration
    sum += sample;

    //Compensate the value for ADC inequality (interleave sawtooth correction).
    //Applied unconditionally at every rate, matching pecostm32's read path (his firmware
    //is clean on this hardware and applies comp at all rates -- PORT_AUDIT.md F23). The old
    //samplerate==0 gate left the sawtooth uncorrected on every timescale except the fastest
    //("both/always" on the bench). If a hedgehog/ripple appears at slower rates instead, our
    //FPGA read does NOT return distinct interleaved samples there and this must be reverted.
    sample += settings->compensation;

    //Check if sample became negative
    if(sample < 0)
    {
      //Keep it on zero if so
      sample = 0;
    }
    
    //Check if sample over its max
    if(sample > 255)
    {
      //Keep it on max if so
      sample = 255;
    }
      
 //*****************************************************************************   
    //Check if sample overload ADC
    if (sample == 0)    settings->ADoverload |= 1;
    if (sample == 255)  settings->ADoverload |= 254;    //=255
 //*****************************************************************************   
    
    //Check if busy with second ADC data (matches pecostm32: gated on checkfirstadc only)
    if(settings->checkfirstadc)
    {
      //When ADC1 compensation is positive, ADC1 bottoms out on this value
      //Near the top ADC2 reaches the top value first, so same method is needed
      if(settings->adc1compensation > 0)
      {
        //So when the compensated ADC2 sample is below the ADC1 compensation value the reading needs to be matched
        if(sample < settings->adc1compensation)
        {
          //Match the two readings when within compensation range
          settings->buffer[1] = sample;
        }
        //Or when the compensated ADC1 sample is above max value ADC2 can reach the reading also needs to be matched
        else if(settings->buffer[1] > (255 + settings->adc2compensation))
        {
          //Use the compensated ADC1 sample in that case
          sample = settings->buffer[1];
        }
      }
      //When ADC1 compensation is negative, ADC2 bottoms out on it's compensation value
      else if(settings->adc1compensation < 0)
      {
        //So when the compensated ADC1 sample is below the ADC2 compensation value the reading needs to be matched
        if(settings->buffer[1] < settings->adc2compensation)
        {
          //Use the compensated ADC1 sample in that case
          sample = settings->buffer[1];
        }
        //Or when the compensated ADC2 sample is above max value ADC1 can reach the reading also needs to be matched
        else if(sample > (255 + settings->adc1compensation))
        {
          //Match the two readings when within compensation range
          settings->buffer[1] = sample;
        }
      }
    }
    
    if (settings->invert) sample = 255-sample;  //Invert channel settings 1-signal inverted 
    
    //Store the data
    *settings->buffer = sample;
    
//-*****************************************************************************    
    
        //Get the minimum value of the samples
    if(sample < settings->minDC)
    {
      //Keep the lowest
      settings->minDC = sample;
    }
    
    //Get the maximum value of the samples
    if(sample > settings->maxDC)
    {
      //Keep the highest
      settings->maxDC = sample;
    }
  //************** 
    /* ------------------------------------------------------------------
 * Robust zero-crossing + frequency + time+ / time- measurement
 * Time is measured ONLY in samples using a monotonic counter
 * ------------------------------------------------------------------ */


    /* --------------------------------------------------------------
     * GLOBAL MONOTONIC SAMPLE COUNTER
     * -------------------------------------------------------------- */
    
    //global_sample_counter++;

    /* --------------------------------------------------------------
     * ZERO-CROSSING / TIME MEASUREMENT
     * -------------------------------------------------------------- */
    /*
    if (settings->checkfirstadc)
    {
        uint64 idx = global_sample_counter;   // TRUE TIME IN SAMPLES

        // Rising edge: LOW -> HIGH 
        if (sample > settings->highlevel)
        {
            if (settings->state == 0)
            {
                settings->state = 1;

                if (settings->zerocrossings > 0)
                {
                    uint64 dt = idx - settings->previousindex;
                    settings->lowsamplecount += dt;
                    settings->lowdivider++;
                }

                settings->previousindex = idx;
                settings->zerocrossings++;
            }
        }
        // Falling edge: HIGH -> LOW 
        else if (sample < settings->lowlevel)
        {
            if (settings->state == 1)
            {
                settings->state = 0;

                if (settings->zerocrossings > 0)
                {
                    uint64 dt = idx - settings->previousindex;
                    settings->highsamplecount += dt;
                    settings->highdivider++;
                }

                settings->previousindex = idx;
                settings->zerocrossings++;
            }
        }
    }

*/

    //======================
        // ---- zero-crossing / frequency detection (robust pre veľké nofsamples) ---- 
    
    if (settings->checkfirstadc)
    {
        // compute a stable up-counting sample index (0 .. nofsamples-1) 
        uint64 idx = (scopesettings.nofsamples - count); // up-counter 

        // Use current sample (already compensated/inverted) 
        if (sample > settings->highlevel)
        {
            // Rising edge: Low -> High 
            if (settings->state == 0)
            {
                settings->state = 1;

                if (settings->zerocrossings == 0)
                {
                    // first crossing: just record the index 
                    settings->previousindex = idx;
                }
                else
                {
                    // interval that was LOW 
                    uint64 dt = idx - settings->previousindex;
                    settings->lowsamplecount += dt;
                    settings->lowdivider++;
                    settings->previousindex = idx;
                }

                settings->zerocrossings++;
            }
        }
        else if (sample < settings->lowlevel)
        {
            // Falling edge: High -> Low 
            if (settings->state == 1)
            {
                settings->state = 0;

                if (settings->zerocrossings == 0)
                {
                    settings->previousindex = idx;
                }
                else
                {
                    // interval that was HIGH 
                    uint64 dt = idx - settings->previousindex;
                    settings->highsamplecount += dt;
                    settings->highdivider++;
                    settings->previousindex = idx;
                }

                settings->zerocrossings++;
            }
        }
    }

    
    //=====================
    /*
    //Check if busy with second ADC data to see if frequency determination can be done
    if(settings->checkfirstadc)
    {
      //Check against the high level
      if(sample > settings->highlevel)
      {
        //If above, check if in low state
        if(settings->state == 0)
        {
          //If so flip the state
          settings->state = 1;
          
          //Check if first zero crossing detected
          if(settings->zerocrossings == 0)
          {
            //Set the previous index if so
            settings->previousindex = count;
          }
          else
          {
            //Calculate the total number of samples in the low parts of the signal
            settings->lowsamplecount += settings->previousindex - count;
            
            //Add one to the low divider for average calculation
            settings->lowdivider++;
            
            //Set the new previous index
            settings->previousindex = count;
          }

          //Found a zero crossing
          settings->zerocrossings++;
        }
      }
      //Check against the low level
      else if(sample < settings->lowlevel)
      {
        //If below check if in high state
        if(settings->state == 1)
        {
          //If so flip the state
          settings->state = 0;

          //Check if first zero crossing detected
          if(settings->zerocrossings == 0)
          {
            //Set the previous index if so
            settings->previousindex = count;
          }
          else
          {
            //Calculate the total number of samples in the high parts of the signal
            settings->highsamplecount += settings->previousindex - count;
            
            //Add one to the low divider for average calculation
            settings->highdivider++;
            
            //Set the new previous index
            settings->previousindex = count;
          }
          
          //Found a zero crossing
          settings->zerocrossings++;
        }
      }
    }  
    */
  //***************  
    //For dc shift in DC mode  
    //385 korekcia nameranej hodnoty (avg=0 pri DC s offsetom aj AC mode), vecie cislo-nizsia zobrazena hodnota
    //sample+=((settings->dcoffset*100)/385); //386 = -200mV, 385 = +100mV
    sample+=((settings->dcoffset*100)/(settings->dc_shift_value)); //386 = -200mV, 385 = +100mV
    
    //Get the minimum value of the samples
    if(sample < settings->min)
    {
      //Keep the lowest
      settings->min = sample;
    }
    
    //Get the maximum value of the samples
    if(sample > settings->max)
    {
      //Keep the highest
      settings->max = sample;
    }
    
    //Add the samples for average calculation
    settings->average += sample;

	//Add squares of the samples for RMS calculation
	settings->rms += (sample - 128) * (sample - 128);
     
    //Skip a sample to allow for ADC2 data to be placed into
    settings->buffer += 2;
    //fpga fifo RAM    
    //settings->buffer += 1;
         
    //One read done
    count--;
  }
 
  //Calculate the raw average
  settings->rawaverage = sum / scopesettings.nofsamples;
}

//----------------------------------------------------------------------------------------------------------------------------------
//Raw ring readout for the acquisition probe (FPGA_NOTES.md §capture geometry): set the FPGA
//read pointer with 0x1F and clock out 'count' bytes from one ADC read command, without any
//compensation, clipping or statistics. Reading past the ~4096 sample ring shows the wrap.
//Bus protocol mirrors fpga_read_adc_data; the next fpga_write_cmd restores the bus direction.

void fpga_dump_ring(uint8 command, uint16 readpointer, uint8 *buffer, uint32 count)
{
  //Set the read start address in the FPGA
  fpga_write_cmd(0x1F);
  fpga_write_short(readpointer);

  //Request the ADC sample data
  fpga_write_cmd(command);

  //Set the bus for reading data
  FPGA_BUS_DIR_IN();
  FPGA_DATA_READ();

  while(count--)
  {
    //Clock the next sample to the output of the FPGA and store it as is
    FPGA_PULSE_CLK();
    *buffer++ = FPGA_GET_DATA();
  }
}

//----------------------------------------------------------------------------------------------------------------------------------

uint16 fpga_average_trace_data(PCHANNELSETTINGS settings)
{
  //Read ten bytes of trace data and average them
  uint16 data = 0;
  int16  datatmp = 0;
  uint32 count = 10;
  
  //register uint32 setting = settings->samplevoltperdiv;
  
  //Send a command for getting trace data from the FPGA
  fpga_write_cmd(settings->averagecommand);
  
  //Set the bus for reading
  FPGA_BUS_DIR_IN();
  
  //Set the control lines for reading a command
  FPGA_DATA_READ();

  //Read the data as long as there is count
  while(count)
  {
    //Clock the data to the output of the FPGA
    FPGA_PULSE_CLK();
 
    //Read the data CH1 or CH2
    //No interleave compensation here: the roll-mode 0x24/0x26 averages mix both ADCs so a
    //per-ADC comp is meaningless, and the old 'timeperdiv >= 29' gate could never be true
    //on this long-timebase-only path anyway (dead code dropped, REVIEW-2026-08-21)
    datatmp = FPGA_GET_DATA();

    //Check limits
    if (datatmp > 255)  datatmp = 255;
    if (datatmp < 0)    datatmp = 0;


    data += (uint16)datatmp;

    //One read done
    count--;
  }

  //Calculate the average and return it
  //data = data / 10;
  //data = (data * multiply) >> VOLTAGE_SHIFTER;
  return(data / 10);
}

uint16 fpga_average_trace_data_long(PCHANNELSETTINGS settings)
{
  //Read ten bytes of trace data and average them
  uint16 data = 0;
  int16 datatmp = 0;
  uint32 count = 100;
  
  //register uint32 setting = settings->samplevoltperdiv;
  
  //Send a command for getting trace data from the FPGA
  fpga_write_cmd(settings->averagecommand);
  
  
  //Set the bus for reading
  FPGA_BUS_DIR_IN();
  
  //Set the control lines for reading a command
  FPGA_DATA_READ();

  //Read the data as long as there is count
  while(count)
  {
    //Clock the data to the output of the FPGA
    FPGA_PULSE_CLK();
 
    //Read the data CH1 or CH2
    //No interleave compensation: same reasoning as fpga_average_trace_data above
    datatmp = FPGA_GET_DATA();

    //Check limits
    if (datatmp > 255)  datatmp = 255;
    if (datatmp < 0)    datatmp = 0;


    data += (uint16)datatmp;

    //One read done
    count--;
  }
  
  //Calculate the average and return it
  //data = data / 10;
  //data = (data * multiply) >> VOLTAGE_SHIFTER;
    
  return(data);
}


//----------------------------------------------------------------------------------------------------------------------------------
/*
void fpga_set_battery_level(void)
{
  //Send the command for setting the battery level to the FPGA??
  fpga_write_cmd(0x3C);
  
  //Write the data
  fpga_write_short(32431);
}
*/
//----------------------------------------------------------------------------------------------------------------------------------

void fpga_delay(uint32 usec)
{
  uint32 loops = usec * 54;

  __asm__ __volatile__ ("1:\n" "subs %0, %1, #1\n"  "bne 1b":"=r"(loops):"0"(loops));
}

//----------------------------------------------------------------------------------------------------------------------------------
