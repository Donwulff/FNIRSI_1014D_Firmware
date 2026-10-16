"""Final zero correction must use compensated sums and verify the physical result."""

from pathlib import Path
import subprocess
import tempfile
import unittest

from test_review_fixes import SCOPE, function


class CalibrationTests(unittest.TestCase):
    def test_final_zero_and_failure_restore(self):
        source = (SCOPE / "scope_functions.c").read_text()
        functions = source[source.index("static int32 caldbg_hdiff"):source.index("void uart1_wait_for_user_input(void);")]
        functions += source[source.index("#define HIGH_DC_OFFSET"):source.index("#if PORT_1014D", source.index("#define HIGH_DC_OFFSET"))]
        for name in ("interleave_set_centre", "interleave_runtime_triggerpoint", "interleave_settle",
                     "calibration_read_frame", "calibration_verify_zero", "scope_do_channel_calibration",
                     "scope_do_baseline_calibration"):
            functions += function("scope_functions.c", name)
        program = r'''
#include <assert.h>
#include <string.h>
#include "variables.h"
#include "scope_functions.h"
#include "fpga_control.h"
#include "display_lib.h"
#include "timer.h"
#include "menu.h"
FONTDATA font_1, font_3;
static uint32 ticks, reads, armed, fail_capture, bad_count, fixed_residual;
static uint32 fail_channel, hardware_rate, hardware_time;
static double dac_centre = 860, slope = 3.85;
static double adc_difference[2];
void display_set_fg_color(uint32 c) {}
void display_set_font(PFONTDATA p) {}
void display_text(uint32 x, uint32 y, const char *s) {}
void display_decimal(uint32 x, uint32 y, int32 v) {}
void display_fill_rect(uint32 x, uint32 y, uint32 w, uint32 h) {}
void uart1_wait_for_user_input(void) {}
void scope_trigger_settings(int m) {}
void scope_acqusition_settings(int m) {}
void scope_channel_settings(PCHANNELSETTINGS s, int m) {}
void fpga_set_trigger_mode(void) {}
void fpga_write_cmd(uint8 c) {}
void fpga_write_byte(uint8 b) {}
void fpga_set_channel_voltperdiv(PCHANNELSETTINGS s) {}
void fpga_set_channel_offset(PCHANNELSETTINGS s) {}
void fpga_set_sample_rate(uint32 s) { hardware_rate = s; }
void fpga_set_time_base(uint32 t) { hardware_time = t; }
void fpga_set_long_timebase(uint32 t) { hardware_time = t; }
void timer0_delay(uint32 d) { ticks += d; }
uint32 timer0_get_ticks(void) { return ticks++; }
void fpga_do_conversion(void) { armed++; }
uint8 fpga_done_conversion(void) { return !fail_capture; }
uint16 fpga_prepare_for_transfer(void) { return 900; }
uint32 measure_high_rate_artifact(PCHANNELSETTINGS s, uint32 *p, uint32 a) { *p = 0; return 0; }
void fpga_read_sample_data(PCHANNELSETTINGS s, uint32 t)
{
  reads++;
  int ch = s->adc1command == 0x20 ? 0 : 1;
  double raw = 128 + (dac_centre + s->samplevoltperdiv * 2 -
                      s->dc_calibration_offset[s->samplevoltperdiv]) / slope;
  double difference = adc_difference[ch];
  s->adc1rawaverage = raw;
  s->adc2rawaverage = raw + difference;
  double mean = raw + (difference + s->adc1compensation + s->adc2compensation) / 2;
  if(fixed_residual || (fail_channel == ch + 1 && scopesettings.samplecount == 3000))
    mean = 130;
  s->averagesum = mean * scopesettings.samplecount + 0.5;
  s->averagecount = bad_count ? 0 : scopesettings.samplecount;
}
'''
        program += functions
        program += r'''
static void prepare(void)
{
  memset(&scopesettings, 0, sizeof(scopesettings));
  memset(&calibrationsettings, 0, sizeof(calibrationsettings));
  reads = armed = ticks = fail_capture = bad_count = fixed_residual = fail_channel = 0;
  dac_centre = 860;
  slope = 3.85;
  adc_difference[0] = 1; adc_difference[1] = -1;
  scopesettings.samplecount = 3000;
  scopesettings.nofsamples = 1500;
  scopesettings.samplerate = 0;
  scopesettings.timeperdiv = 32;
  fpgasettings.totalsamples = 3000;
  fpgasettings.fw_FPGA = 1;
  calibrationsettings.adc1command = 0x20;
  calibrationsettings.adc1compensation = 2;
  calibrationsettings.adc2compensation = -3;
  for(int v = 0; v < 6; v++)
  {
    sampleratedcoffsetstep[0][v] = sampleratedcoffsetstep[1][v] = slope * (1 << 20);
    calibrationsettings.dc_calibration_offset[v] = 860 + v * 2;
  }
}
int main(void)
{
  //The integer average would read 128, but the compensated mean is +0.779.
  prepare();
  calibrationsettings.dc_calibration_offset[0] = 857;
  assert(calibration_verify_zero(&calibrationsettings, 0));
  assert(calibrationsettings.dc_calibration_offset[0] == 860);
  assert(reads == 24 && caldbg_zero[0][0] == 0);
  //Signed rounding corrects an equal negative residual in the other direction.
  prepare();
  calibrationsettings.dc_calibration_offset[0] = 863;
  assert(calibration_verify_zero(&calibrationsettings, 0));
  assert(calibrationsettings.dc_calibration_offset[0] == 860 && reads == 24);
  assert(calibrationsettings.adc1compensation == 2 && calibrationsettings.adc2compensation == -3);
  prepare();
  assert(calibration_verify_zero(&calibrationsettings, 0) && reads == 12);
  //An unresponsive DAC cannot pass on the predicted correction alone.
  prepare(); fixed_residual = 1;
  assert(!calibration_verify_zero(&calibrationsettings, 0) && reads == 48);
  prepare(); bad_count = 1;
  assert(!calibration_verify_zero(&calibrationsettings, 0) && reads == 1);
  prepare(); fail_capture = 1; ticks = 0xfffffff0U;
  assert(!calibration_verify_zero(&calibrationsettings, 0) && reads == 0);
  prepare(); sampleratedcoffsetstep[0][0] = sampleratedcoffsetstep[1][0] = 0;
  assert(!calibration_verify_zero(&calibrationsettings, 0));
  prepare(); calibrationsettings.dc_calibration_offset[0] = 810;
  assert(!calibration_verify_zero(&calibrationsettings, 0));
  //Run the whole calibration for both channels/ranges, using actual compensated
  //sums from a synthetic analog transfer, including odd interleave differences.
  prepare();
  adc_difference[0] = 5; adc_difference[1] = -5;
  assert(scope_do_baseline_calibration());
  for(int v = 0; v < 6; v++)
  {
    assert(caldbg_zero[0][v] >= -500 && caldbg_zero[0][v] <= 500);
    assert(caldbg_zero[1][v] >= -500 && caldbg_zero[1][v] <= 500);
  }
  assert(scopesettings.channel1.dc_calibration_offset[6] == scopesettings.channel1.dc_calibration_offset[5]);
  assert(hardware_rate == 0 && hardware_time == 32);
  assert(scopesettings.samplecount == 3000 && scopesettings.nofsamples == 1500);
  assert(scopesettings.channel1.adc1compensation == 2 && scopesettings.channel1.adc2compensation == -3);
  assert(scopesettings.channel2.adc1compensation == -2 && scopesettings.channel2.adc2compensation == 3);
  //If CH2's final check fails, both channels keep their original calibration.
  prepare(); fail_channel = 2;
  adc_difference[0] = 5; adc_difference[1] = -5;
  for(int v = 0; v < 7; v++)
  {
    scopesettings.channel1.dc_calibration_offset[v] = 901;
    scopesettings.channel2.dc_calibration_offset[v] = 902;
  }
  scopesettings.channel1.adc1compensation = 7;
  assert(!scope_do_baseline_calibration());
  for(int v = 0; v < 7; v++)
  {
    assert(scopesettings.channel1.dc_calibration_offset[v] == 901);
    assert(scopesettings.channel2.dc_calibration_offset[v] == 902);
  }
  assert(scopesettings.channel1.adc1compensation == 7);
  assert(hardware_rate == 0 && hardware_time == 32 && scopesettings.samplecount == 3000);
  prepare(); scopesettings.long_mode = 1; scopesettings.timeperdiv = 6;
  adc_difference[0] = 5; adc_difference[1] = -5;
  assert(scope_do_baseline_calibration());
  assert(scopesettings.long_mode && hardware_time == 6 && hardware_rate == 0);
  return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix="fnirsi-calibration-") as temporary:
            path = Path(temporary)
            (path / "test.c").write_text(program)
            subprocess.run(["gcc", "-O1", "-fcommon", "-ffunction-sections", "-fdata-sections",
                            "-fsanitize=undefined", "-fno-sanitize-recover=undefined",
                            "-I" + str(SCOPE), str(path / "test.c"), str(SCOPE / "variables.c"),
                            "-Wl,--gc-sections", "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], timeout=10, check=True)
