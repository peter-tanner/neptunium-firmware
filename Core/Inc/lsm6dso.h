
#ifndef __LSM6DSO_H
#define __LSM6DSO_H

#include "lsm6dsox_reg.h"
#include "utils.h"
#include "main.h"

void lsm6dsox_read_data_drdy_handler(void);
void lsm6dsox_read_data_drdy_init(void);

#endif