#ifndef __USB_PRINTF
#define __USB_PRINTF

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <integer.h>
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum logging_interfaces
    {
        IF_USB = 0b01,
        IF_LORA = 0b10,
    } logging_interfaces;

    void logging(const char *format, logging_interfaces logging_interfaces, ...);

#ifdef __cplusplus
}
#endif

#endif