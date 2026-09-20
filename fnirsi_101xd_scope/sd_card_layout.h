#ifndef SD_CARD_LAYOUT_H
#define SD_CARD_LAYOUT_H

#include "port_config.h"

#define SD_BOOT_SECTOR                  16
#define SD_MIN_PARTITION_SECTOR       2048
#define LEGACY_INPUT_CALIBRATION_SECTOR 708
#define LEGACY_SETTINGS_SECTOR          709
#define DISPLAY_CONFIG_SECTOR           710

#if PORT_1014D
//The loader reads the app contiguously from sector 80. Keep persistent data
//at the end of the reserved first MiB, outside the packed boot image.
#define SCOPE_START_SECTOR               80
#define INPUT_CALIBRATION_SECTOR       2046
#define SETTINGS_SECTOR                2047
#else
#define SCOPE_START_SECTOR              750
#define INPUT_CALIBRATION_SECTOR        708
#define SETTINGS_SECTOR                 709
#endif

#endif
