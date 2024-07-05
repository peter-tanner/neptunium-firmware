#include <stdint.h>

#define FLOAT_CHAR_BUFF_SIZE 10

char *float_to_char(float x, char *p);
uint8_t print_float(float x);

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define HALT_BKPT()                   \
    {                                 \
        __asm__ __volatile__("BKPT"); \
        while (1)                     \
        {                             \
        }                             \
    }
