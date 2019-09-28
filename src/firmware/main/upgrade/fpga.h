#ifndef UPGRADE_FPGA_H
#define UPGRADE_FPGA_H

#include <stdbool.h>
#include <stdint.h>

bool upgrade_fpga_check(void);
bool upgrade_fpga_send(void);
uint32_t upgrade_fpga_build_id(void);

#endif
