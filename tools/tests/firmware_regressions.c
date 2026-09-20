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
void scope_draw_grid(void) {}
void scope_draw_pointers(void) {}
void scope_draw_time_cursors(void) {}
void scope_draw_volt_cursors(void) {}
void scope_display_cursor_measurements(void) {}
void timer0_delay(uint32 ms) {}
uint32 timer0_get_ticks(void) { ticks += tick_step; return ticks; }
void display_set_fg_color(uint32 c) {}
void display_fill_rect(uint32 x, uint32 y, uint32 w, uint32 h) {}
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
  assert(x >= 6 && x2 <= 704);
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
uint16 fpga_average_trace_data(PCHANNELSETTINGS s) { readouts++; return 128; }
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
uint8 fpga_done_conversion(void) { done_calls++; return !fail_arm || arms < fail_arm; }
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
  scopesettings.samplecount = 3000;
  scopesettings.nofsamples = 1500;
  fpgasettings.fw_FPGA = 1;
  arms = triggered_arms = fail_arm = done_calls = readouts = dumps = 0;
  opens = closes = close_failure = short_write = saved = timed_out = 0;
  ticks = 0;
  tick_step = 1000;
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

static void test_roll(void)
{
  reset();
  scopesettings.runstate = 1;
  scopesettings.long_mode = 1;
  scopesettings.timeperdiv = 6;
  enablesampling = enabletracedisplay = 1;
  scope_preset_values();
  assert(draw_buffer == (uint16 *)maindisplaybuffer);
  for(int i = 0; i < 6100; i++)
  {
    scope_get_long_timebase_data();
    uint32 previous = scratch_lines;
    scope_display_long_trace_data();
    assert(scratch_lines > previous);
  }
  assert(blits == 6100 && visible_lines == 0); //First sweep, screen wrap and buffer wrap.
  enablesampling = enabletracedisplay = 0;
  uint32 previous = readouts;
  scope_get_long_timebase_data();
  scope_display_long_trace_data();
  assert(readouts == previous && blits == 6100);
  overlay = 1;
  scope_get_long_timebase_data();
  scope_display_long_trace_data();
  assert(readouts == previous + 2 && composites == 1 && blits == 6101);
  overlay = 0;
  enablesampling = enabletracedisplay = 1;
  scopesettings.timeperdiv = 0;
  tick_step = 0;
  ticks = previoustimerticks + 999;
  previous = readouts;
  scope_get_long_timebase_data();
  assert(readouts == previous);
  ticks++;
  scope_get_long_timebase_data();
  assert(readouts == previous + 2);
  previoustimerticks = 0xFFFFFF00;
  ticks = previoustimerticks + 1000;
  scope_get_long_timebase_data();
  assert(readouts == previous + 4); //Timer wrap.
  scopesettings.count = 3000;
  scopesettings.triggermode = 1;
  ticks += 1000;
  scope_get_long_timebase_data();
  assert(scopesettings.runstate == RUN_STATE_STOPPED);
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
#if PORT_1014D
  test_storage();
  test_roll();
  test_average();
  test_clock();
  test_probe();
#endif
  printf("Firmware regressions passed (PORT_1014D=%d)\n", PORT_1014D);
  return 0;
}
