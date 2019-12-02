#include <stdbool.h>
#include <stdint.h>

#include "i2c.h"

#define CAM_IP 0x21
#ifndef CAMERA_H
#define CAMERA_H

typedef struct
{
    uint8_t reg;
    bool val;
    uint8_t position;
    uint8_t final_reg_val;
} cam_setting_t;


bool camera_init(cam_setting_t*, unsigned int);
bool camera_write2register(uint8_t, uint8_t);
uint8_t camera_read_reg(uint8_t);

#endif
