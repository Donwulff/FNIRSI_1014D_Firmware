//----------------------------------------------------------------------------------------------------------------------------------

#include "types.h"

#include "menu.h"
#include "test.h"
#include "ccu_control.h"
#include "spi_control.h"
#include "timer.h"
#include "interrupt.h"
#include "display_control.h"
#include "fpga_control.h"
#include "touchpanel.h"
#include "port_config.h"
#include "clock_synthesizer.h"
#include "uart.h"
#include "power_and_battery.h"
#include "DS3231.h"

#include "fnirsi_101xd_scope.h"
#include "display_lib.h"
#include "scope_functions.h"
#include "statemachine.h"
#if PORT_1014D
#include "menu_1014d.h"
#endif

#include "sd_card_interface.h"
#include "ff.h"

#include "usb_interface.h"
#include "cdc_class.h"
#include "PC_interface.h"

#include "arm32.h"
#include "mmu.h"

#include "variables.h"

#include <string.h>

//----------------------------------------------------------------------------------------------------------------------------------

extern IRQHANDLERFUNCION interrupthandlers[];

//----------------------------------------------------------------------------------------------------------------------------------

int main(void)
{
  //Initialize data in BSS section
  memset(&BSS_START, 0, &BSS_END - &BSS_START);

  //Initialize the clock system
  sys_clock_init();

  //Identity MMU so the D-cache C bit actually applies (ARM926 data cacheability
  //comes from the page tables). Framebuffer and sunxi MMIO stay NCNB.
  mmu_setup();
  arm32_icache_enable();
  arm32_dcache_enable();

  //Clear the interrupt variables
  memset(interrupthandlers, 0, 256);

  //Setup timer interrupt
  timer0_setup();

  //Setup power monitoring
  power_interrupt_init();

  //Setup battery monitoring
  battery_check_init();

  //Enable interrupts only once. In the original code it is done on more then one location.
  arm32_interrupt_enable();

  //Initialize SPI for flash (PORT C + SPI0)
  sys_spi_flash_init();

#if PORT_1014D
  //1014D: program the external Si5351 clock generator (PA0/PA1) BEFORE the FPGA, so the FPGA/ADC
  //has its sample clock and the FPGA-driven backlight PWM is stable (otherwise the display flashes).
  if (sampling_clock_p1b == 0)
    sampling_clock_p1b = 0x06;     // stock 50 MHz default
  if (sampling_clock_scale == 0.0)
    sampling_clock_scale = 1.0;
  clock_synthesizer_setup();
#endif

  //Initialize FPGA (PORT E)
  fpga_init();
  
  //Read version firmware in FPGA
  fpga_get_version();

  //Turn off the display brightness
  fpga_set_backlight_brightness(0x0000);

  //Initialize display (PORT D + DEBE)
  sys_init_display(SCREEN_WIDTH, SCREEN_HEIGHT, (uint16 *)maindisplaybuffer);

  //Setup the display library for the scope hardware
  scope_setup_display_lib();
  
  //Setup and check SD card on file system being present
  if(f_mount(&fs, "0", 1))
  {
    //Show SD card error message on failure
    //Set max screen brightness
    fpga_set_backlight_brightness(0xEA60);

    //Clear the display
    display_set_fg_color(BLACK_COLOR);
    display_fill_rect(0, 0, 800, 480);

    //Display the message in red
    display_set_fg_color(RED_COLOR);
    display_set_font(&font_3);
    display_text(30, 50, "SD ERROR");

    //On error just hang
    while(1);
  }
 
  //load picture startup logo
  //load_picture("scope.bmp");

  //Setup the touch panel interface
#if PORT_1014D
  //1014D has no touch panel; PA0/PA1 are the clock-gen I2C (already set up by cg_i2c_setup()).
  //Running tp_i2c_setup() here would bit-bang GT911 init bytes onto the Si5351 lines and could
  //corrupt the clock, so it is deliberately skipped. (touchpanel.c stays compiled for the RTC.)
#else
  tp_i2c_setup();
#endif

#ifndef USE_TP_CONFIG
#ifdef SAVE_TP_CONFIG
  //Save the touch panel configuration to be save
  save_tp_config();
#endif
#endif

  //Load configuration data from FLASH
  scope_load_configuration_data();
  //if (dev_mode)
  //Setup the USB interface
  usb_device_init();
  
  //Serial active sent HELLO message
  //if(USB_CH340==1) {SendString("Fnirsi 1013D"); ch340Send('\n');}
  
  //ch340Send('A');
 
  //Enable or disable the channels based on the scope loaded settings
  fpga_set_channel_enable(&scopesettings.channel1);
  fpga_set_channel_enable(&scopesettings.channel2);

  //Set the volts per div for each channel based on the loaded scope settings
  fpga_set_channel_voltperdiv(&scopesettings.channel1);
  fpga_set_channel_voltperdiv(&scopesettings.channel2);

  //Set the channels AC or DC coupling based on the loaded scope settings
  fpga_set_channel_coupling(&scopesettings.channel1);
  fpga_set_channel_coupling(&scopesettings.channel2);
  
  //Enable something in the FPGA
  fpga_enable_system();
  
  fpga_set_sample_rate(scopesettings.samplerate);       //Short time base  
          //Setup the trigger system in the FPGA based on the loaded scope settings
  if(scopesettings.long_mode)  fpga_set_long_timebase(scopesettings.timeperdiv);
    else fpga_set_time_base(scopesettings.timeperdiv);  //Send the time base command for the longer settings
      //Set the sample rate that belongs to the selected time per div setting
    //scopesettings.samplerate = time_per_div_sample_rate[scopesettings.timeperdiv];  

  fpga_set_trigger_channel();
  fpga_set_trigger_edge();
  fpga_set_trigger_level();
  fpga_set_trigger_mode();

  //Set the trace offsets in the FPGA
  fpga_set_channel_offset(&scopesettings.channel1);
  fpga_set_channel_offset(&scopesettings.channel2);
  
  fpga_on_off_generator();
  //fpga_set_generator_freq(103456);  //in Hz
  //fpga_set_generator_duty(70);

  //Some initialization of the FPGA??. Data written with command 0x3C
  //fpga_set_battery_level();      //Only called here and in hardware check
  
  //Set screen brightness
  fpga_set_translated_brightness();
  
  //load picture startup logo
  load_picture("scope.bmp");
  //if (!dev_mode) timer0_delay(3000);//2000

#if PORT_A_KEYDEBUG
  //PROOF-OF-LIFE: full-screen magenta flash held 2s
  display_set_screen_buffer((uint16 *)maindisplaybuffer);
  display_set_fg_color(0x00FF00FF);
  display_fill_rect(0, 0, 800, 480);
  display_set_fg_color(0x0000FF00);
  display_set_font(&font_5);
  display_text(120, 210, "PORT_A 1014D BUILD RUNNING");
  timer0_delay(2000);
#endif

  //View message if load a default configuration in case of settings
#if PORT_1014D
  //1014D: skip restore screen (no touch panel, would hang waiting for touch)
  restore = 0;
#else
  if (restore) scope_setup_restore_screen();
#endif
  
  //Set battery level to max for next calculations
  scopesettings.batterychargelevel = 20;    
  
  //Set the user value screen brightness
  scope_set_maxlight(scopesettings.maxlight);       //1 set max, 0 user value
  
  //Setup the main parts of the screen
#if PORT_1014D
  ui_setup_main_screen();
#else
  scope_setup_main_screen();
#endif
  DBG_STAGE(2);   //main screen chrome drawn

  //Modification on 09-03-2022. Begin
  //Clear the sample memory
  //memset(channel1tracebuffer, 128, sizeof(channel1tracebuffer));
  //memset(channel2tracebuffer, 128, sizeof(channel2tracebuffer));
  
  memset(channel1tracebufferAVG, 128, sizeof(channel1tracebufferAVG));
  memset(channel2tracebufferAVG, 128, sizeof(channel2tracebufferAVG));
   
  //Set the number of samples in use
//  scopesettings.nofsamples  = SAMPLES_PER_ADC;
  //scopesettings.samplecount = SAMPLE_COUNT;
  
  scopesettings.samplecount = fpgasettings.totalsamples;//SAMPLE_COUNT;
  scopesettings.nofsamples = fpgasettings.totalsamples/2;
  
  //fpgasettings.settriggerpoint = 1500;
  //fpgasettings.totalsamples = 3000;//16382;
  extern uint32 a,b,c,d,e,f;
  f = fpgasettings.totalsamples;//16382;
   
  //fpga_set_generator_freq(103456);  //in Hz
  //fpga_set_generator_duty(70);
  
  //fpgasettings.gen_enable =  1;   //1 is enable
  //fpgasettings.gen_freq = 1000;   //1kHz     //int32 !!!
  //fpgasettings.gen_duty =   50;   //50%
  
  //Show initial trace data. When in NORMAL or SINGLE mode the display needs to be drawn because otherwise if there is no signal it remains black
  //if set long time base, no drawn.
  //if(!scopesettings.long_mode) scope_display_trace_data();
  //else scope_preset_long_mode();
  
  //scopesettings.display_data_done = 1
  
  //Switch to RUN
  //scopesettings.runstate = 1;//ok
  //scopesettings.display_data_done = 1;
  
  //scopesettings.batterychargelevel = 20;    //set battery level to max for next calculations
  
  //Flag READY to start convert
  scopesettings.display_data_done = 1;
  scopesettings.conversion_done = 0;
  havetouch = 0;
  
  scope_preset_values();
  scope_calculate_sample_range_properties();  // Ensure disp_xrange + trigger_position_min/max are valid for the loaded timebase before sm_init/rotary input. Fixes initial "stuck at extreme offset" on 1014D (min/max were zero until first timebase change).
  DBG_STAGE(3);   //scope_preset_values done

  ini_SysParam();
  DBG_STAGE(4);   //ini_SysParam done

  //Wait until the analog part start
  timer0_delay(50);
  
  //only test and set calibration value, on setings menu
  //dc_shift_cal=1;
  
  //timerH=1000;
  
  //scope_open_measures_menu();
  //while(1);

  //scope_open_keyboard_menu();

#if PORT_1014D
  uart1_init();
  sm_init();
#endif
  DBG_STAGE(5);   //uart1_setup done, about to enter main loop

#if PORT_A_KEYDEBUG
  uint16 dbg_fpgaver = fpga_get_version();
#endif

  //Monitor the battery, process and display trace data and handle user input until power is switched off
  while(1)
  {
#if PORT_A_KEYDEBUG
    //Diagnostic overlay in right column (heartbeat, FPGA version, key code)
    {
      static uint32 hb = 0;
      display_set_screen_buffer((uint16 *)maindisplaybuffer);  //draw to the VISIBLE framebuffer
      display_set_font(&font_3);
      display_set_fg_color(0x00000000);
      display_fill_rect(732, 300, 68, 150);
      display_set_fg_color(0x0000FF00);
      display_decimal(734, 302, ++hb);            //heartbeat: must increment
      display_hex(734, 322, 4, dbg_fpgaver);      //FPGA version (expect 1432)
      display_decimal(734, 342, fpgasettings.fw_FPGA); //fw path (expect 1)
      display_decimal(734, 362, toprocesscommand); //last non-zero key code (0 = nothing received)
      display_hex(734, 382, 2, 0);                //LSR placeholder
      display_hex(734, 402, 4, 0);                //cfg placeholder
      display_hex(734, 422, 2, 0);                //DLL placeholder
    }
#endif
    //Handle the touch panel input
    //touch_handler();
    
    //usb_ch340_send_text("CH340 mode ready\r\n");
    
    //Monitor the battery status
    battery_check_status();
    DBG_STAGE(6);   //battery check done (loop is alive)

    //Handle the touch panel input
    //touch_handler();
    
    
 //-****************************************************************************   
    //scope_test_longbase_value();
  
    //scope_test_ADoffset_value();    //view values for ad offset min max center p2p
    //scope_test_calibration_value();
    
    //scope_test_trigerposition_value();
    
    //Serial active sent HELLO message
    //if(USB_CH340==1) {SendString("Fnirsi 1013D"); ch340Send('\n');}

/*
      int usb_ready = 0;
        if (usb_device_state == USB_STATE_ENABLED && !usb_ready)
    {
        usb_ready = 1;
        usb_ch340_send_text("#OK,CONNECTED\n");
        usb_ch340_send_text("V1.3.0C MOD V9B4\n");
    }
 */
 
    
    //if ((USB_CH340 == 1) && !dev_mode) usb_CDC_send_text("Fnirsi 1013D CH340 mode ready\r\n");
    
    //usb_ch340_process_rx();
    
    //scope_test_TP();
    
/*
      scope_open_keyboard_menu(5,10);
    while(1){
 
  Ahandle_generator_menu_touch();
   }
 */
      
    
    
    
/*
       scope_open_generator_menu();  
    while(1){

      gen_freq_select();
    timer0_delay(200);
    
    };
 */
      
    
  //-***************************************************************************
  //Select the mode 1-Long time base or 0-short time base
  if(scopesettings.long_mode)
    {
        scope_get_long_timebase_data();
    }
    else
    {   //Long memory mode, trigger is the beginning buffer
        //----------------------long memory mode --------------------------------------------------------
        if(scopesettings.long_memory)
        {
           
            
        }
        //--------------------- short mode --------------------
        else
        //Standard memory mode, trigger is the middle buffer
        {
            scopesettings.samplemode = 1;//1
            
            
            //Go through the trace data and display the trace data
            DBG_STAGE(7);   //about to acquire
            scope_acquire_trace_data();
            DBG_STAGE(8);   //acquire returned
        }
    }
    
    //--------------------------------------------------------------------------
    
    //Check developer mode and show DEV_mode on the screen
    if (dev_mode)
    {
      display_set_fg_color(RED_COLOR);
      display_set_font(&font_2);
      display_text(670, 50, "DEV mode");
      
      scope_test_trigerposition_value();
      
          //usb_CDC_receive_callback();
    }
    
   extern uint32 a,b,c,d,e,f;
//fpgasettings.totalsamples = f;//16382;
    //if (e==1) {timer0_delay(500);scope_save_view_item_file(VIEW_TYPE_PICTURE); e=0;}
    
    
    //--------------------------------------------------------------------------

    //Handle user input
    DBG_STAGE(9);   //trace pipeline done, about to poll keys
#if PORT_1014D
    sm_handle_user_input();
    //In roll mode scope_get_long_timebase_data() draws the trace itself; running the
    //standard renderer as well made the two repaint over each other every pass (F21
    //mechanism 1, REVIEW-2026-08-21 follow-up). Overlay menus are not composited during
    //roll -- the roll sweep's own blit would wipe them anyway.
    //Full-frame draws (blit + speed overlay) are capped to 60 Hz; acquire and keys
    //still run every pass. Roll crawl is per-sample and is not gated here.
    if(ui_display_frame_due())
    {
        if(scopesettings.long_mode)
        {
            ui_draw_roll_chrome();
        }
        else if(enabletracedisplay || ui_menu_composite_active())
        {
            scope_display_trace_data();
        }
        ui_draw_speed_overlay();
    }
#else
    //1013D: handle the touch panel input
    touch_handler();
#endif

    //--------------------------------------------------------------------------
    
#if !PORT_1014D
    //READ and show RTC time
    //1013D only: readtime() is a stub on the 1014D, and a config sector written by a
    //1013D with the RTC enabled would otherwise leave a 1014D black-filling this rect
    //over the P14 top bar every 500 ticks with no on-device way to turn it off
    if (!(timerRTC)&&(onoffRTC))
    {
      timerRTC=500;
      //Reading time and date of DS3231
      readtime();
      //Set color the background
      display_set_fg_color(BLACK_COLOR);
      //Fill the time background //x , y , sirka, vyska
      display_fill_rect(TIME_TEXT_XPOS, TIME_TEXT_YPOS, TIME_TEXT_WIDTH, TIME_TEXT_HEIGHT);
      //Show time on the screen
      display_set_fg_color(WHITE_COLOR);
      display_set_font(&font_2);
      display_text(TIME_TEXT_XPOS, TIME_TEXT_YPOS, buffertime);
    }
#endif
    
    //--------------------------------------------------------------------------
  }//end While(1);
}//end main
//----------------------------------------------------------------------------------------------------------------------------------
