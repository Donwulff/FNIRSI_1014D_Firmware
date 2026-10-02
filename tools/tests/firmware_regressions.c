#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "variables.h"
#include "statemachine.h"
#include "scope_functions.h"
#include "menu_1014d.h"
#include "fpga_control.h"
#include "display_lib.h"
#include "clock_synthesizer.h"
#include "sd_card_interface.h"
#include "timer.h"
#include "test.h"
#include "uart.h"

static uint32 startup_config;
#undef STARTUP_CONFIG_ADDRESS
#define STARTUP_CONFIG_ADDRESS (&startup_config)

FONTDATA font_0, font_1;
static uint16 disk[2048][256];
static int disk_writes, write_failure, corrupt_write, read_failure = -1;
static int32 shown;
static uint8 command, trigger_disable;
static uint32 arms, triggered_arms, fail_arm, done_calls, readouts, dumps;
static uint32 ticks, tick_step = 1000, long_timebase_calls;
static uint16 *draw_buffer;
static uint32 visible_lines, scratch_lines, blits, composites, overlay;
static uint32 legacy_grids, port_grids, last_line_x, acquisition_draws;
static uint32 draw_channel, conversion_pending;
static uint8 adc_values[2];
static uint16 displayed_points[2][698];
static uint32 scan_head_x;
static int short_write, close_failure, opens, closes, saved, timed_out, missing_average;

int32 sd_card_read(uint32 sector, uint32 blocks, uint8 *buffer)
{
  assert(blocks == 1 && sector < 2048);
  if((int)sector == read_failure) return -1;
  memcpy(buffer, disk[sector], 512);
  return SD_OK;
}

int32 sd_card_write(uint32 sector, uint32 blocks, uint8 *buffer)
{
  assert(blocks == 1);
#if !PORT_1014D
  if(sector == DISPLAY_CONFIG_SECTOR) return SD_OK;
#endif
  assert(sector == SETTINGS_SECTOR || sector == INPUT_CALIBRATION_SECTOR);
  disk_writes++;
  if(write_failure) return -1;
  memcpy(disk[sector], buffer, 512);
  if(corrupt_write) disk[sector][10] ^= 1;
  return SD_OK;
}

void scope_load_input_calibration_data(void) {}
void scope_load_ALLREF_file(void) {}
void scope_calculate_sample_range_properties(void) {}
void scope_check_long_trigger(void) {}
void scope_run_stop_text(void) {}
void scope_run_stop_button(int mode) {}
void scope_draw_grid(void) { legacy_grids++; }
void ui_draw_grid(void) { port_grids++; }
void scope_display_trace_data(void) { acquisition_draws++; }
void scope_process_trigger(uint32 count) {}
void scope_do_50_percent_trigger_setup(void) {}
void fpga_set_trigger_level(void) {}
void scope_draw_pointers(void) {}
void scope_draw_time_cursors(void) {}
void scope_draw_volt_cursors(void) {}
void scope_display_cursor_measurements(void) {}
void timer0_delay(uint32 ms) {}
uint32 timer0_get_ticks(void) { ticks += tick_step; return ticks; }
void display_set_fg_color(uint32 c)
{
  if(c == CHANNEL1_COLOR) draw_channel = 0;
  if(c == CHANNEL2_COLOR) draw_channel = 1;
}
void display_fill_rect(uint32 x, uint32 y, uint32 w, uint32 h)
{
  memset(displayed_points, 0, sizeof(displayed_points));
}
void display_set_font(PFONTDATA f) {}
void display_decimal(uint32 x, uint32 y, int32 v) {}
void display_text(uint32 x, uint32 y, const char *s)
{
  if(strstr(s, "Saved acqprobe")) saved++;
  if(strstr(s, "ACQUISITION TIMEOUT")) timed_out++;
  if(strcmp(s, "- - -") == 0) missing_average++;
}
void display_set_screen_buffer(uint16 *p) { draw_buffer = p; }
void display_set_source_buffer(uint16 *p) {}
void display_copy_rect_to_screen(uint32 x, uint32 y, uint32 w, uint32 h)
{
  assert(draw_buffer == (uint16 *)maindisplaybuffer);
  assert(x == 2 && y == 48 && w == 705 && h == 432);
  blits++;
}
void display_draw_line(uint32 x, uint32 y, uint32 x2, uint32 y2)
{
  assert(x >= 7 && x2 <= 704);
  assert(x <= x2);
  if(scan_head_x) assert(!(x <= scan_head_x && x2 > scan_head_x));
  displayed_points[draw_channel][x2 - 7] = y2 + 1;
  last_line_x = x2;
  if(draw_buffer == (uint16 *)maindisplaybuffer) visible_lines++;
  else scratch_lines++;
}
int32 scope_get_y_sample(PCHANNELSETTINGS s, int32 i)
{
  assert(i >= 0 && i < 3000);
  return s->tracebuffer[i];
}
uint32 ui_menu_composite_active(void) { return overlay; }
void ui_draw_outline(void) {}
void ui_display_trigger_settings(void) {}
void ui_display_waiting_triggered_text(uint32 state) {}
void ui_draw_pointers(void) {}
void ui_display_cursors(void) {}
void ui_redraw_active_menu(void)
{
  assert(draw_buffer == displaybuffertmp);
  if(overlay) composites++;
}
void ui_update_measurements(void) {}
void ui_print_value(uint32 y, int32 v, uint32 s, char *d, uint32 sign, int32 res) { shown = v; }
static void show_clock_test_status(uint8 p, uint32 s, uint32 k, uint8 b, int32 r, int y) {}
void fpga_arm_long_timebase_cycle(void) {}
uint16 fpga_average_trace_data(PCHANNELSETTINGS s)
{
  readouts++;
  return adc_values[s == &scopesettings.channel2];
}
void fpga_write_cmd(uint8 c) { command = c; }
void fpga_write_byte(uint8 b)
{
  if(command == 0x0f) trigger_disable = b;
  if(command == 0x01 && b == 0) { arms++; if(!trigger_disable) triggered_arms++; }
}
uint8 fpga_read_byte(void) { return 1; }
void fpga_write_short(uint16 s) {}
void fpga_set_channel_enable(PCHANNELSETTINGS s) {}
void fpga_set_channel_voltperdiv(PCHANNELSETTINGS s) {}
void fpga_set_channel_offset(PCHANNELSETTINGS s) {}
void fpga_set_backlight_brightness(uint16 b) {}
void fpga_set_sample_rate(uint32 r) {}
void fpga_set_time_base(uint32 t) {}
void fpga_set_long_timebase(uint32 t) { long_timebase_calls++; }
void fpga_set_trigger_mode(void) {}
uint8 fpga_done_conversion(void)
{
  done_calls++;
  return !conversion_pending && (!fail_arm || arms < fail_arm);
}
#if !PORT_1014D
void fpga_do_conversion(void) { arms++; }
#endif
void fpga_read_sample_data(PCHANNELSETTINGS s, uint32 t) { readouts++; }
uint16 fpga_prepare_for_transfer(void) { return 123; }
void fpga_dump_ring(uint8 c, uint16 p, uint8 *b, uint32 n) { dumps++; }
uint32 measure_high_rate_artifact(PCHANNELSETTINGS s, uint32 *p, uint32 a) { *p = 0; return 0; }
double clock_synthesizer_apply_sampling_clock(uint8 p)
{ sampling_clock_p1b = p; sampling_clock_scale = 8.0 / (p + 2); return sampling_clock_scale; }
void uart1_wait_for_user_input(void) {}
FRESULT f_open(FIL *fp, const TCHAR *name, BYTE mode) { opens++; return FR_OK; }
FRESULT f_write(FIL *fp, const void *buf, UINT bytes, UINT *written)
{ *written = short_write ? 0 : bytes; return FR_OK; }
FRESULT f_close(FIL *fp) { closes++; return closes == close_failure ? FR_DISK_ERR : FR_OK; }

#include "firmware_functions.inc"

static void reset(void)
{
  memset(&scopesettings, 0, sizeof(scopesettings));
  scope_reset_config_data();
  scopesettings.channel1.tracebuffer = (uint8 *)channel1tracebuffer;
  scopesettings.channel2.tracebuffer = (uint8 *)channel2tracebuffer;
  scopesettings.channel1.color = CHANNEL1_COLOR;
  scopesettings.channel2.color = CHANNEL2_COLOR;
  scopesettings.samplecount = 3000;
  scopesettings.nofsamples = 1500;
  fpgasettings.fw_FPGA = 1;
  arms = triggered_arms = fail_arm = done_calls = readouts = dumps = 0;
  opens = closes = close_failure = short_write = saved = timed_out = 0;
  ticks = 0;
  tick_step = 1000;
  legacy_grids = port_grids = acquisition_draws = 0;
  visible_lines = scratch_lines = blits = composites = overlay = last_line_x = 0;
  conversion_pending = scan_head_x = 0;
  adc_values[0] = adc_values[1] = 128;
#if PORT_1014D
  disp_long_mode = 0;
#endif
}

static void test_acquisition_rendering(void)
{
  reset();
  scopesettings.runstate = RUN_STATE_RUNNING;
  scopesettings.display_data_done = 1;
  touchstate = 0;
  scope_acquire_trace_data();
  assert(arms == 1 && readouts == 2);
#if PORT_1014D
  assert(acquisition_draws == 0); //The main loop owns the frame after key handling.
#else
  assert(acquisition_draws == 1); //Retain the original 1013D rendering contract.
#endif
}

static void test_movespeed(void)
{
  reset();
  assert(scopesettings.movespeed == MOVE_SPEED_FAST);
#if !PORT_1014D
  assert(scopesettings.movespeed == 0);
  scopesettings.movespeed ^= 1;
  assert(scopesettings.movespeed == 1);
  scopesettings.movespeed ^= 1;
#endif
  for(int speed = 0; speed <= 1; speed++)
  {
    scopesettings.movespeed = speed;
    scope_save_config_data();
    scopesettings.movespeed = 99;
    scope_restore_config_data();
    assert(scopesettings.movespeed == (speed ? 1 : MOVE_SPEED_FAST));
    memset(viewfilesetupdata, 0, sizeof(viewfilesetupdata));
    viewfilesetupdata[OTHER_SETTING_OFFSET] = speed;
    scope_restore_setup_from_file();
    assert(scopesettings.movespeed == (speed ? 1 : MOVE_SPEED_FAST));
  }
}

#if PORT_1014D
static void checksum(uint16 *buffer, int begin, int end)
{
  uint32 sum = 0;
  for(int i = begin; i < end; i++) sum += buffer[i];
  buffer[0] = sum >> 16;
  buffer[1] = sum;
}

static void test_storage(void)
{
  memset(disk, 0, sizeof(disk));
  uint8 *mbr = (uint8 *)disk[0];
  mbr[510] = 0x55; mbr[511] = 0xAA;
  mbr[450] = 0x0C; mbr[455] = 8; mbr[458] = 1;
  scope_save_config_data();
  memcpy(disk[LEGACY_SETTINGS_SECTOR], settingsworkbuffer, 512);
  for(int i = 8; i < 60; i++) disk[LEGACY_INPUT_CALIBRATION_SECTOR][i] = 4000;
  checksum(disk[LEGACY_INPUT_CALIBRATION_SECTOR], 8, 60);
  assert(scope_prepare_config_storage());
  assert(disk_writes == 2);
  assert(memcmp(disk[SETTINGS_SECTOR], disk[LEGACY_SETTINGS_SECTOR], 512) == 0);
  assert(memcmp(disk[INPUT_CALIBRATION_SECTOR], disk[LEGACY_INPUT_CALIBRATION_SECTOR], 512) == 0);
  assert(scope_prepare_config_storage());
  assert(disk_writes == 2); //Destination data wins on repeated boots.
  memset(disk[SETTINGS_SECTOR], 0, 512);
  write_failure = 1;
  assert(!scope_prepare_config_storage());
  write_failure = 0; corrupt_write = 1;
  assert(!scope_prepare_config_storage());
  corrupt_write = 0;
  assert(scope_prepare_config_storage());
  read_failure = SETTINGS_SECTOR;
  assert(!scope_prepare_config_storage());
  read_failure = -1;
  memset(disk[SETTINGS_SECTOR], 0, 512);
  disk[LEGACY_SETTINGS_SECTOR][20] ^= 1;
  int before = disk_writes;
  assert(scope_prepare_config_storage());
  assert(disk_writes == before); //Never migrate corrupt legacy program bytes.
  mbr[455] = 1;
  assert(!scope_prepare_config_storage());
  scope_save_configuration_data();
  assert(disk_writes == before); //Never write inside an early partition.
  mbr[455] = 8; mbr[450] = 0xEE;
  assert(!scope_prepare_config_storage());
  mbr[450] = 0x0C; mbr[510] = 0;
  assert(!scope_prepare_config_storage());
  mbr[510] = 0x55;
  scope_save_configuration_data();
  assert(disk_writes == before + 1);
}

static void start_roll(uint32 timebase)
{
  reset();
  tick_step = 0;
  scopesettings.runstate = RUN_STATE_RUNNING;
  scopesettings.long_mode = 1;
  scopesettings.timeperdiv = timebase;
  enablesampling = enabletracedisplay = 1;
  overlay = 0;
  scope_preset_values();
  assert(draw_buffer == (uint16 *)maindisplaybuffer);
}

static uint32 displayed_point_count(uint32 channel)
{
  uint32 count = 0;
  for(uint32 column = 0; column < 698; column++)
    if(displayed_points[channel][column]) count++;
  return count;
}

static void test_roll(void)
{
  //The horizontal scale must remain 50 pixels/div even if a frame misses many
  //sample deadlines. Test every supported roll scale with fractional intervals.
  const uint32 division_ms[] = {50000, 20000, 10000, 5000, 2000, 1000, 500};
  for(uint32 timebase = 0; timebase < 7; timebase++)
  {
    start_roll(timebase);
    uint32 pixel_ms = division_ms[timebase] / 50;
    scope_get_long_timebase_data();
    assert(scopesettings.lastx == 7 && scopesettings.count == 1);
    ticks += 23 * pixel_ms + pixel_ms / 2;
    scope_get_long_timebase_data();
    assert(scopesettings.lastx == 30 && scopesettings.count == 2);
    ticks += pixel_ms / 2;
    scope_get_long_timebase_data();
    assert(scopesettings.lastx == 31 && scopesettings.count == 3);
    assert(readouts == 6); //Three actual observations, no synthetic catch-up reads.
    scope_display_long_trace_data();
    assert(last_line_x == 31 && port_grids == 1 && legacy_grids == 0);
    assert(visible_lines == 0 && blits == 1);

    ticks = 698 * pixel_ms;
    scope_get_long_timebase_data();
    scope_display_long_trace_data();
    assert(scopesettings.lastx == 7 && scopesettings.count == 4);
    assert(displayed_point_count(0) == 3 && last_line_x == 31);
    ticks += (3 * 698 + 45) * pixel_ms;
    scope_get_long_timebase_data();
    scope_display_long_trace_data();
    assert(scopesettings.lastx == 52 && displayed_point_count(0) == 1);
  }

  //Screen and sample-buffer wraps are independent. Repainting must use valid
  //indices and retain the preceding sweep ahead of the scan.
  start_roll(6);
  for(int i = 0; i < 6100; i++)
  {
    ticks = i * 10;
    scope_get_long_timebase_data();
    uint32 previous = scratch_lines;
    scope_display_long_trace_data();
    assert(scratch_lines > previous);
    assert(last_line_x == (i < 698 ? 7 + i : 704));
    assert(displayed_points[0][i % 698] == 129);
  }
  assert(blits == 6100 && port_grids == 6100 && legacy_grids == 0);
  assert(visible_lines == 0 && scopesettings.count == 100);

  //Full-screen views suppress sampling/rendering, while overlay menus composite
  //over the trace. Returning from a full-screen view does not include paused time.
  enablesampling = enabletracedisplay = 0;
  uint32 previous = readouts;
  uint32 previous_x = scopesettings.lastx;
  scope_get_long_timebase_data();
  scope_display_long_trace_data();
  assert(readouts == previous && blits == 6100);
  ticks += 60000;
  overlay = 1;
  scope_get_long_timebase_data();
  scope_display_long_trace_data();
  assert(readouts == previous + 2 && composites == 1 && blits == 6101);
  assert(scopesettings.lastx == previous_x + 1);
  overlay = 0;
  enablesampling = enabletracedisplay = 1;

  //A stopped trace can still be repainted after scratch is reused by a menu.
  scopesettings.runstate = RUN_STATE_STOPPED;
  scope_get_long_timebase_data();
  previous = readouts;
  previous_x = scopesettings.lastx;
  ticks += 60000;
  scope_display_long_trace_data();
  assert(readouts == previous && displayed_points[0][previous_x - 7] == 129);
  scopesettings.runstate = RUN_STATE_RUNNING;
  scope_get_long_timebase_data();
  assert(scopesettings.lastx == previous_x + 1);

  //Reset after a timebase change must discard the old sweep and clock origin.
  scopesettings.timeperdiv = 0;
  scope_preset_values();
  scope_get_long_timebase_data();
  scope_display_long_trace_data();
  assert(scopesettings.lastx == 7 && displayed_point_count(0) == 1);
  previous = readouts;
  ticks += 999;
  scope_get_long_timebase_data();
  assert(readouts == previous);
  ticks++;
  scope_get_long_timebase_data();
  assert(readouts == previous + 2 && scopesettings.lastx == 8);

  //Unsigned tick subtraction also handles the 49-day millisecond timer wrap.
  start_roll(6);
  ticks = 0xFFFFFFF0;
  scope_get_long_timebase_data();
  ticks += 40;
  scope_get_long_timebase_data();
  assert(scopesettings.lastx == 11 && scopesettings.count == 2);

  //Waiting for a trigger does not consume the capture time. SINGLE finishes by
  //elapsed time even with very sparse loop passes, and NORMAL rearms its sweep.
  for(uint32 mode = 1; mode <= 2; mode++)
  {
    start_roll(6);
    scopesettings.triggermode = mode;
    scope_preset_values();
    ticks = 100000;
    scope_get_long_timebase_data();
    assert(readouts == 0 && scopesettings.count == 0);
    triggerlong = 1; //Hardware-trigger mock remains otherwise idle.
    scope_get_long_timebase_data();
    assert(scopesettings.lastx == 7);
    ticks += 29990;
    scope_get_long_timebase_data();
    assert(roll_capture_pixels == 3000 && scopesettings.count == 2);
    previous = readouts;
    ticks += 10;
    scope_get_long_timebase_data();
    assert(readouts == previous && triggerlong == 0);
    if(mode == 1)
      assert(scopesettings.runstate == RUN_STATE_STOPPED);
    else
      assert(scopesettings.runstate == RUN_STATE_RUNNING && roll_capture_pixels == 0);
    scope_display_long_trace_data();
    assert(displayed_point_count(0) == 1 && displayed_points[0][2999 % 698] == 129);
    if(mode == 2)
    {
      //Keep the completed capture throughout trigger waiting and replace it
      //progressively once another trigger arrives.
      ticks += 10000;
      scope_get_long_timebase_data();
      assert(readouts == previous);
      triggerlong = 1;
      adc_values[0] = 140;
      scope_get_long_timebase_data();
      scan_head_x = 7;
      scope_display_long_trace_data();
      scan_head_x = 0;
      assert(displayed_points[0][0] == 141);
      assert(displayed_points[0][2999 % 698] == 129);
    }

    //A loop delayed past the entire capture must finish without backdating a
    //new observation into the expired capture.
    start_roll(6);
    scopesettings.triggermode = mode;
    scope_preset_values();
    triggerlong = 1;
    scope_get_long_timebase_data();
    previous = readouts;
    ticks += 60000;
    scope_get_long_timebase_data();
    assert(readouts == previous && triggerlong == 0);
    assert(scopesettings.runstate == (mode == 1 ? RUN_STATE_STOPPED : RUN_STATE_RUNNING));
  }
}

static void test_roll_retention(void)
{
  start_roll(6);
  adc_values[0] = 90;
  adc_values[1] = 110;
  for(uint32 i = 0; i < 698; i++)
  {
    ticks = i * 10;
    scope_get_long_timebase_data();
  }
  scope_display_long_trace_data();
  assert(displayed_point_count(0) == 698 && displayed_point_count(1) == 698);

  adc_values[0] = 170;
  adc_values[1] = 190;
  ticks += 10;
  scope_get_long_timebase_data();
  scan_head_x = 7;
  scope_display_long_trace_data();
  assert(displayed_points[0][0] == 171 && displayed_points[1][0] == 191);
  for(uint32 i = 1; i < 698; i++)
  {
    assert(displayed_points[0][i] == 91);
    assert(displayed_points[1][i] == 111);
  }

  //A slow frame expires the passed columns but keeps the untouched tail. The
  //renderer may interpolate between actual new observations, never across sweeps.
  ticks += 230;
  scope_get_long_timebase_data();
  scan_head_x = 30;
  scope_display_long_trace_data();
  assert(displayed_point_count(0) == 676);
  assert(displayed_points[0][0] == 171 && displayed_points[0][23] == 171);
  for(uint32 i = 1; i < 23; i++) assert(displayed_points[0][i] == 0);
  assert(displayed_points[0][24] == 91 && displayed_points[0][697] == 91);

  //The display cache must survive capture-buffer reuse, STOP and overlay redraw.
  memset(channel1tracebuffer, 128, sizeof(channel1tracebuffer));
  memset(channel2tracebuffer, 128, sizeof(channel2tracebuffer));
  scopesettings.runstate = RUN_STATE_STOPPED;
  scope_get_long_timebase_data();
  overlay = 1;
  scope_display_long_trace_data();
  assert(displayed_points[0][0] == 171 && displayed_points[0][697] == 91);
  assert(displayed_points[1][0] == 191 && displayed_points[1][697] == 111);
  overlay = 0;

  //Leaving roll for 200 ms/div must retain the roll display while the FPGA is
  //busy. Only an actual completed readout switches display ownership to sweep.
  scopesettings.runstate = RUN_STATE_RUNNING;
  scopesettings.long_mode = 0;
  scopesettings.timeperdiv = 11;
  scopesettings.display_data_done = 1;
  scope_preset_values();
  conversion_pending = 1;
  uint32 previous = readouts;
  scope_acquire_trace_data();
  assert(disp_long_mode == 1 && readouts == previous);
  scope_display_long_trace_data();
  assert(displayed_points[0][0] == 171 && displayed_points[0][697] == 91);
  scope_acquire_trace_data();
  assert(disp_long_mode == 1 && arms == 1);
  conversion_pending = 0;
  scope_acquire_trace_data();
  assert(disp_long_mode == 0 && readouts == previous + 2);
  scan_head_x = 0;

  //A disabled channel must not inherit unmeasured samples on a later sweep.
  start_roll(6);
  scopesettings.channel2.enable = 0;
  scope_get_long_timebase_data();
  scopesettings.channel2.enable = 1;
  ticks += 10;
  scope_get_long_timebase_data();
  scope_display_long_trace_data();
  assert(displayed_point_count(0) == 2 && displayed_point_count(1) == 1);
  assert(displayed_points[1][0] == 0 && displayed_points[1][1] == 129);

  //NORMAL can finish partway across the screen. Its old break must survive
  //ahead of the next scan, as well as the break after the new sample at x=7.
  start_roll(6);
  scopesettings.triggermode = 2;
  scope_preset_values();
  triggerlong = 1;
  for(uint32 i = 0; i < 3000; i++)
  {
    ticks = i * 10;
    scope_get_long_timebase_data();
  }
  scan_head_x = scopesettings.lastx;
  ticks += 10;
  scope_get_long_timebase_data();
  scope_display_long_trace_data();
  assert(displayed_point_count(0) == 698);
  triggerlong = 1;
  adc_values[0] = 190;
  scope_get_long_timebase_data();
  scope_display_long_trace_data();
  assert(displayed_point_count(0) == 698 && displayed_points[0][0] == 191);
  scan_head_x = 0;
}

static void test_average(void)
{
  reset();
  scopesettings.samplecount = 0;
  shown = 12345;
  ui_display_vavg(0, &scopesettings.channel1);
  assert(shown == 12345 && missing_average == 1);
  scopesettings.samplecount = 3000;
  scopesettings.channel1.average = 128;
  scopesettings.channel1.averagesum = 128 * 3000;
  scopesettings.channel1.averagecount = 3000;
  memset(channel1tracebuffer, 128, 3000);
  memset(channel2tracebuffer, 130, 3000);
  ui_display_vavg(0, &scopesettings.channel1);
  assert(shown == 0);
  ui_prepare_setup_for_file();
  scopesettings.channel1.averagesum = 150 * 8000;
  scopesettings.channel1.averagecount = scopesettings.samplecount = 8000;
  assert(ui_check_waveform_file() == 0);
  ui_restore_setup_from_file();
  ui_display_vavg(0, &scopesettings.channel1);
  assert(shown == 0 && scopesettings.samplecount == 3000);
  assert(scopesettings.channel2.averagesum == 130 * 3000);
  assert(scopesettings.channel2.averagecount == 3000);
}

static void test_clock(void)
{
  for(int trigger = 0; trigger < 3; trigger++)
  {
    reset();
    scopesettings.samplemode = 1;
    scopesettings.triggermode = trigger;
    scopesettings.timeperdiv = 20;
    auto_detect_max_clean_sampling_clock();
    assert(arms == 40 && triggered_arms == 0);
    assert(scopesettings.samplemode == 1 && scopesettings.triggermode == trigger);
    assert(scopesettings.timeperdiv == 20 && trigger_disable == 0);
  }
  reset();
  scopesettings.long_mode = 1;
  scopesettings.samplemode = 0;
  scopesettings.triggermode = 2;
  fail_arm = 1;
  auto_detect_max_clean_sampling_clock();
  assert(arms == 1 && long_timebase_calls == 1);
  assert(scopesettings.samplemode == 0 && scopesettings.triggermode == 2);
  assert(trigger_disable == 1);
}

static void test_probe(void)
{
  //tick_step=1000 permits three captures in each rate phase, then the dump.
  for(int failure = 1; failure <= 7; failure += 3)
  {
    reset();
    scopesettings.triggermode = 2;
    scopesettings.samplemode = 0;
    fail_arm = failure;
    scope_do_acquisition_probe();
    assert(timed_out == 1 && opens == 0 && dumps == 0);
    assert(scopesettings.samplemode == 0 && scopesettings.triggermode == 2);
    assert(scopesettings.display_data_done == 1);
  }
  reset();
  scope_do_acquisition_probe();
  assert(saved == 1 && opens == 2 && closes == 2 && dumps == 5);
  reset();
  short_write = 1;
  scope_do_acquisition_probe();
  assert(saved == 0 && closes == 2);
  for(int failure = 1; failure <= 2; failure++)
  {
    reset();
    close_failure = failure;
    scope_do_acquisition_probe();
    assert(saved == 0 && closes == 2);
  }
  reset();
  fpgasettings.fw_FPGA = 2;
  scope_do_acquisition_probe();
  assert(arms == 0 && opens == 0 && dumps == 0);
}
#endif

int main(void)
{
  test_movespeed();
  test_acquisition_rendering();
#if PORT_1014D
  test_storage();
  test_roll();
  test_roll_retention();
  test_average();
  test_clock();
  test_probe();
#endif
  printf("Firmware regressions passed (PORT_1014D=%d)\n", PORT_1014D);
  return 0;
}
