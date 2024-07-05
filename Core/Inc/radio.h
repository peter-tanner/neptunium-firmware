#ifndef __RADIO_H
#define __RADIO_H

#include "config.h"
#include "tusb.h"
#include "utils.h"
#include "main.h"
#include "sx126x.h"
#include "sx126x_hal.h"

#define F_915MHZ 915000000 // MHz
#define FREQUENCY F_915MHZ

#define RADIO_RX_TIMEOUT 120000 // ms

#define TX_PWR 22 // dBm

#define RAMP_TIME SX126X_RAMP_40_US // us

#define EN_RX 0b01
#define EN_TX 0b10

void lora_direction();
void lora_setup(uint8_t enable_reg);
void lora_handle_irq(void);

/**
 * Radio data transfer - write
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
                                     const uint8_t *data, const uint16_t data_length);

/**
 * Radio data transfer - read
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,
                                    uint8_t *data, const uint16_t data_length);

/**
 * Reset the radio
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_reset(const void *context);

/**
 * Wake the radio up.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_wakeup(const void *context);

#endif