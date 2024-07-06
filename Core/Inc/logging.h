#ifndef __LOGGING_H
#define __LOGGING_H

#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include <string.h>

void StartLoggingTask(void const *argument);
#endif