/*
 ******************************************************************************
 * @file    multi_read_fifo.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to get data from sensor FIFO
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MET001V1
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 * - DISCOVERY_SPC584B + STEVAL-MET001V1
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * NUCLEO_STM32F411RE - Host side: UART(COM) to USB bridge
 *                    - I2C(Default) / SPI(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

// #define STEVAL_MKI109V3  /* little endian */
// #define NUCLEO_F411RE    /* little endian */
// #define SPC584B_DIS      /* big endian */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

/* Includes ------------------------------------------------------------------*/
#include "lps22hb.h"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint32_t data_raw_pressure;
static int16_t data_raw_temperature;
static float pressure_hPa;
static float temperature_degC;
static float altitude;
static uint8_t whoamI, rst;
static stmdev_ctx_t dev_ctx;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

void lps22hb_multi_read_fifo_handler(void)
{
    uint8_t reg;
    /* Read output only if fifo watermark set */
    lps22hb_int_source_t interrupt_source;
    lps22hb_int_source_get(&dev_ctx, &interrupt_source);

    if (1)
    {
        lps22hb_fifo_fth_flag_get(&dev_ctx, &reg);

        if (reg)
        {
            /* Read FIFO watermark */
            lps22hb_fifo_data_level_get(&dev_ctx, &reg);

            while (reg--)
            {
                memset(&data_raw_pressure, 0x00, sizeof(int32_t));
                lps22hb_pressure_raw_get(&dev_ctx, &data_raw_pressure);
                pressure_hPa = lps22hb_from_lsb_to_hpa(data_raw_pressure);

                memset(&data_raw_temperature, 0x00, sizeof(int16_t));
                lps22hb_temperature_raw_get(&dev_ctx, &data_raw_temperature);
                altitude = lps22hb_from_lsb_to_altitude(data_raw_pressure);
                temperature_degC = lps22hb_from_lsb_to_degc(data_raw_temperature);
            }
        }
        __NOP();
    }
}

/* Main Example --------------------------------------------------------------*/
void lps22hb_multi_read_fifo_init(void)
{
    /* Initialize mems driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &hspi2;
    /* Check device ID */
    lps22hb_device_id_get(&dev_ctx, &whoamI);

    if (whoamI != LPS22HB_ID)
    {
        while (1)
        {
            /* manage here device not found */
        }
    }

    /* Restore default configuration */
    lps22hb_reset_set(&dev_ctx, PROPERTY_ENABLE);

    do
    {
        lps22hb_reset_get(&dev_ctx, &rst);
    } while (rst);

    lps22hb_auto_add_inc_set(&dev_ctx, PROPERTY_ENABLE);
    /* Enable Block Data Update */
    lps22hb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set FIFO watermark to 16 samples */
    lps22hb_fifo_watermark_set(&dev_ctx, 16);
    /* Set FIFO mode */
    lps22hb_fifo_mode_set(&dev_ctx, LPS22HB_DYNAMIC_STREAM_MODE);
    /* Enable FIFO */
    lps22hb_fifo_set(&dev_ctx, PROPERTY_ENABLE);
    /* Can be set FIFO watermark status on INT_DRDY pin */
    // lps22hb_fifo_threshold_on_int_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lps22hb_data_rate_set(&dev_ctx, LPS22HB_ODR_10_Hz);

    lps22hb_fifo_ovr_on_int_set(&dev_ctx, PROPERTY_ENABLE);
    lps22hb_fifo_full_on_int_set(&dev_ctx, PROPERTY_ENABLE);
    lps22hb_fifo_threshold_on_int_set(&dev_ctx, PROPERTY_ENABLE);
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef ok = HAL_OK;
    ok |= HAL_SPI_Transmit(handle, &reg, 1, 1000);
    ok |= HAL_SPI_Transmit(handle, (uint8_t *)bufp, len, 1000);
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
    return ok;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    reg |= 0x80; // Read flag
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef ok = HAL_OK;
    ok |= HAL_SPI_Transmit(handle, &reg, 1, 1000);
    ok |= HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET);
    return ok;
}
