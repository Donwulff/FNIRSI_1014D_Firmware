#include <assert.h>
#include <stdio.h>
#include "bl_fpga_control.h"
#include "bl-uart.h"

static uint16 reply;
static unsigned int version_reads;
static unsigned int ready_after;
static uint8 keys[8];
static unsigned int key_count;
static unsigned int key_reads;
static unsigned int menu_lines;

uint16 fpga_get_version(void)
{
  version_reads++;
  return(version_reads > ready_after ? reply : 0xFFFF);
}

uint8 uart1_receive_data(void)
{
  assert(key_reads < key_count);
  return(keys[key_reads++]);
}

void display_text(uint32 x, uint32 y, char *text)
{
  assert(x < 800 && y < 480);
  assert(text[0] != 0);
  menu_lines++;
}

#include "bootloader_functions.inc"

static void check_ready(uint16 version, unsigned int delay, uint16 expected)
{
  reply = version;
  ready_after = delay;
  version_reads = 0;
  assert(fpga_check_ready() == expected);
  assert(version_reads == (expected ? delay + 1 : FPGA_READY_ATTEMPTS));
}

int main(void)
{
  uint16 versions[] = {FPGA_VERSION_STOCK, FPGA_VERSION_CUSTOM_AL3};
  uint8 choices[] = {UIC_BUTTON_F1, UIC_BUTTON_F2, UIC_BUTTON_F3};
  unsigned int v, choice;

  for(v = 0; v < 2; v++)
  {
    check_ready(versions[v], 0, versions[v]);
    check_ready(versions[v], 5, versions[v]);
    check_ready(versions[v], FPGA_READY_ATTEMPTS - 1, versions[v]);
    check_ready(versions[v], FPGA_READY_ATTEMPTS, 0);

    key_reads = menu_lines = 0;
    key_count = 1;
    keys[0] = 49;
    assert(select_boot_source(versions[v]) == 0);
    assert(key_reads == 1 && menu_lines == 0);

    for(choice = 0; choice < 3; choice++)
    {
      key_reads = menu_lines = 0;
      key_count = 4;
      keys[0] = 0;
      keys[1] = 0;
      keys[2] = 49;
      keys[3] = choices[choice];
      assert(select_boot_source(versions[v]) == choice);
      assert(key_reads == key_count && menu_lines == 3);
    }
  }

  check_ready(0, 0, 0);
  check_ready(0xFFFF, 0, 0);
  check_ready(0x1632, 0, 0);
  check_ready(0x1234, 0, 0);
  key_reads = key_count = menu_lines = 0;
  assert(select_boot_source(0) == 2);
  assert(key_reads == 0 && menu_lines == 0);
  puts("Bootloader regressions passed (stock/custom/delayed/missing FPGA, F1/F2/F3)");
  return(0);
}
