#include "logging.h"

#define CHUNK_SIZE 64
// Write string constants which have a size over one block (64)
void cdc_write_safe(char buf[], size_t size)
{
    char *chunk = buf;
    for (size_t i = 0; i + CHUNK_SIZE < size; i += CHUNK_SIZE)
    {
        tud_cdc_write(chunk, CHUNK_SIZE);
        tud_cdc_write_flush();
        chunk += CHUNK_SIZE;
    }
    tud_cdc_write(chunk, strlen(chunk));
    tud_cdc_write_flush();
}

void logging(const char *format, logging_interfaces logging_interfaces, ...)
{
    if (logging_interfaces == 0)
        return;

    va_list args;
    va_start(args, format);

    // Determine the size of the formatted string
    va_list args_copy;
    va_copy(args_copy, args);
    int length = vsnprintf(NULL, 0, format, args_copy) + 1; // +1 for null terminator
    va_end(args_copy);

    // Allocate memory for the buffer
    char *buffer = (char *)malloc(length);
    if (!buffer)
    {
        // Handle allocation failure
        return;
    }

    // Format the string into the buffer
    vsnprintf(buffer, length, format, args);

    // Transmit the buffer over USB
    if (logging_interfaces & IF_USB)
        cdc_write_safe(buffer, strlen(buffer));

    // Transmit over LoRa
    if (logging_interfaces & IF_LORA)
        // TODO:
        __NOP();

    // Free the allocated memory
    free(buffer);

    va_end(args);
}