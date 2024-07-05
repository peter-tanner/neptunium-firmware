#include "radio.h"

void lora_direction()
{
}

void check_status(volatile uint8_t status)
{
    if (SX126X_STATUS_OK != status)
        HALT_BKPT();
}

struct sx126x_ctx
{
} ctx;

void lora_rx_data(void);

uint8_t rx_buf[256]; // MAXIMUM RECEIVE SIZE OF 256 FOR LORA

void lora_setup(uint8_t enable_reg)
{
    if (!enable_reg)
        return;

    // 1. If not in STDBY_RC mode, then go to this mode with the command SetStandby(...)
    sx126x_chip_status_t radio_status;
    if (sx126x_get_status(&ctx, &radio_status) == SX126X_STATUS_OK)
    {
        check_status(sx126x_set_standby(&ctx, SX126X_STANDBY_CFG_RC));
    }
    else
        HALT_BKPT();

    volatile uint32_t number = 0;
    sx126x_get_random_numbers(&ctx, &number, 1);

    // 2. Define the protocol (LoRa® or FSK) with the command SetPacketType(...)
    check_status(sx126x_set_pkt_type(&ctx, SX126X_PKT_TYPE_LORA));

    // 3. Define the RF frequency with the command SetRfFrequency(...)
    check_status(sx126x_set_rf_freq(&ctx, FREQUENCY));

    // ENABLE DC-DC CONVERTER
    check_status(sx126x_set_reg_mode(&ctx, SX126X_REG_MODE_DCDC));

    if (enable_reg & EN_TX)
    {
        // 4. Define the Power Amplifier configuration with the command SetPaConfig(...)
        // FROM DATASHEET:
        // Table 13-21: PA Operating Modes with Optimal Settings
        // Mode Output Power paDutyCycle hpMax deviceSel paLut Value in SetTxParams1
        // SX1262 +22 dBm 0x04 0x07 0x00 0x01 +22 dBm
        // Note:
        // These changes make the use of nominal power either sub-optimal or unachievable.
        // Caution!
        // The following restrictions must be observed to avoid voltage overstress on the PA, exceeding the maximum ratings
        // may cause irreversible damage to the device:
        // • For SX1261 at synthesis frequency above 400 MHz, paDutyCycle should not be higher than 0x07.
        // • For SX1261 at synthesis frequency below 400 MHz, paDutyCycle should not be higher than 0x04.
        // • For SX1262, paDutyCycle should not be higher than 0x04.
        sx126x_pa_cfg_params_t pa_params = {
            .pa_duty_cycle = 0x04,
            .hp_max = 0x07,
            .device_sel = 0x00,
            .pa_lut = 0x01};
        check_status(sx126x_set_pa_cfg(&ctx, &pa_params));

        // 5. Define output power and ramping time with the command SetTxParams(...)
        sx126x_set_tx_params(&ctx, TX_PWR, RAMP_TIME);
    }
    if (enable_reg & EN_RX)
    {
        // ENABLE RX BOOSTED MODE
        check_status(sx126x_cfg_rx_boosted(&ctx, true));
    }

    // 6. Define where the data payload will be stored with the command SetBufferBaseAddress(...)
    check_status(sx126x_set_buffer_base_address(&ctx, 0x00, 0x00));

    // ~~7. Send the payload to the data buffer with the command WriteBuffer(...)~~

    // 8. Define the modulation parameter according to the chosen protocol with the command SetModulationParams(...)1
    sx126x_mod_params_lora_t lora_mod_params = {
        .sf = SX126X_LORA_SF7,
        .bw = SX126X_LORA_BW_125,
        .cr = SX126X_LORA_CR_4_5,
        .ldro = 0x00};
    check_status(sx126x_set_lora_mod_params(&ctx, &lora_mod_params));

    // 9. Define the frame format to be used with the command SetPacketParams(...)2
    // USED FOR RECEIVING - CALLED AGAIN WITH PROPER LENGTH DURING TX
    sx126x_pkt_params_lora_t lora_pkt_params = {
        .preamble_len_in_symb = 8,
        .header_type = SX126X_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes = 0xFF,
        .crc_is_on = true,
        .invert_iq_is_on = false};
    check_status(sx126x_set_lora_pkt_params(&ctx, &lora_pkt_params));

    // 10. Configure DIO and IRQ: use the command SetDioIrqParams(...) to select TxDone IRQ and map this IRQ to a DIO (DIO1, DIO2 or DIO3)
    // 7. Configure DIO and irq: use the command SetDioIrqParams(...) to select the IRQ RxDone and map this IRQ to a DIO (DIO1 or DIO2 or DIO3), set IRQ Timeout as well.
    // ENABLE TCXO SOURCE ON DIO3
    // TODO: CHECK IF THIS REQUIRES EXTERNAL 3.3V OR IF THE XTAL IS BUILT INTO THE MODULE).
    // TIMEOUT = DELAY * 15.625 μs = 1.5625 ms
    check_status(sx126x_set_dio3_as_tcxo_ctrl(&ctx, SX126X_TCXO_CTRL_3_3V, 100));
    // ENABLE RF SWITCH ON DIO2
    check_status(sx126x_set_dio2_as_rf_sw_ctrl(&ctx, true));
    check_status(sx126x_set_tx(&ctx, 1000));
    check_status(sx126x_set_rx(&ctx, 1000));
    check_status(sx126x_set_tx(&ctx, 1000));
    check_status(sx126x_set_rx(&ctx, 1000));
    // ENABLE IRQ ON DIO1
    uint16_t irq_mask = SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERROR;
    irq_mask |= (enable_reg & EN_RX ? SX126X_IRQ_RX_DONE : 0);
    irq_mask |= (enable_reg & EN_TX ? SX126X_IRQ_TX_DONE : 0);
    check_status(sx126x_set_dio_irq_params(&ctx, irq_mask,
                                           irq_mask, SX126X_IRQ_NONE, SX126X_IRQ_NONE));

    // 11. Define Sync Word value: use the command WriteReg(...) to write the value of the register via direct register access
    check_status(sx126x_set_lora_sync_word(&ctx, 0x12));

    sx126x_errors_mask_t errors = 0;
    // IMAGE CALIBRATION
    // When the 32 MHz clock is coming from a TCXO, the calibration will fail
    // and the user should request a complete calibration after calling the
    // function SetDIO3AsTcxoCtrl(...).
    // Table 9-2: Image Calibration Over the ISM Bands
    // Frequency Band [MHz] Freq1 Freq2
    // 902 - 928 0xE1 (default) 0xE9 (default)
    do
    {
        check_status(sx126x_cal_img(&ctx, 0xE1, 0xE9));
        check_status(sx126x_cal(&ctx, SX126X_CAL_ALL));
        sx126x_get_device_errors(&ctx, &errors);
    } while (errors);

    // 12. Set the circuit in transmitter mode to start transmission with the command SetTx(). Use the parameter to enable Timeout
    // 9. Set the circuit in reception mode: use the command SetRx(). Set the parameter to enable timeout or continuous mode

    // 13. Wait for the IRQ TxDone or Timeout: once the packet has been sent the chip goes automatically to STDBY_RC mode
    // 10. Wait for IRQ RxDone2 or Timeout: the chip will stay in Rx and look for a new packet if the continuous mode is selected otherwise it will goes to STDBY_RC mode.

    // 14. Clear the IRQ TxDone flag
    // 11. In case of the IRQ RxDone, check the status to ensure CRC is correct: use the command GetIrqStatus()

    // CHECK FOR ERRORS FIXME: it fails
}

void lora_handle_irq(void)
{
    sx126x_irq_mask_t irq_mask;
    if (sx126x_get_irq_status(&ctx, &irq_mask) == SX126X_STATUS_OK)
    {
        sx126x_clear_irq_status(&ctx, irq_mask);
        if (irq_mask & SX126X_IRQ_TX_DONE)
            __NOP();
        if (irq_mask & SX126X_IRQ_RX_DONE)
            lora_rx_data();
        if (irq_mask & SX126X_IRQ_TIMEOUT)
            __NOP();
        if (irq_mask & SX126X_IRQ_CRC_ERROR)
            __NOP();
    }
}

void lora_rx_data(void)
{
    // TODO: Should the chip go into standby mode here?
    sx126x_set_standby(&ctx, SX126X_STANDBY_CFG_RC);

    sx126x_pkt_status_lora_t pkt_status;
    sx126x_get_lora_pkt_status(&ctx, &pkt_status); // TODO: Engineering logs

    sx126x_rx_buffer_status_t buf_status;
    sx126x_get_rx_buffer_status(&ctx, &buf_status);
    if (buf_status.pld_len_in_bytes > 0)
    {
        sx126x_read_buffer(&ctx, buf_status.buffer_start_pointer, rx_buf, buf_status.pld_len_in_bytes);
        tud_cdc_write(rx_buf, buf_status.pld_len_in_bytes);
    }
}

void lora_tx_data(int8_t power, char *data, uint8_t length)
{
    // TODO: Should the chip go into standby mode here?
    sx126x_set_standby(&ctx, SX126X_STANDBY_CFG_RC);

    check_status(sx126x_set_buffer_base_address(&ctx, 0x00, 0x00));

    sx126x_pkt_params_lora_t lora_pkt_params = {
        .preamble_len_in_symb = 8,
        .header_type = SX126X_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes = length,
        .crc_is_on = true,
        .invert_iq_is_on = false};
    check_status(sx126x_set_lora_pkt_params(&ctx, &lora_pkt_params));

    check_status(sx126x_write_buffer(&ctx, 0, (uint8_t *)data, length));

    check_status(sx126x_set_tx(&ctx, SX126X_MAX_TIMEOUT_IN_MS));
}

void lora_rx_mode(void)
{
    check_status(sx126x_set_rx(&ctx, RADIO_RX_TIMEOUT));
    // TODO: check and handle errors
}

#define SPI_TIMEOUT 1000

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
                                     const uint8_t *data, const uint16_t data_length)
{
    HAL_StatusTypeDef ok = HAL_OK;
    HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
    ok |= HAL_SPI_Transmit(&hspi3, (uint8_t *)command, command_length, SPI_TIMEOUT);
    if (data != NULL)
        ok |= HAL_SPI_Transmit(&hspi3, (uint8_t *)data, data_length, SPI_TIMEOUT);
    HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
    return ok;
}

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
                                    uint8_t *data, const uint16_t data_length)
{
    HAL_StatusTypeDef ok = HAL_OK;
    HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
    ok |= HAL_SPI_Transmit(&hspi3, (uint8_t *)command, command_length, SPI_TIMEOUT);
    ok |= HAL_SPI_Receive(&hspi3, data, data_length, SPI_TIMEOUT);
    HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
    return ok;
}

/**
 * Reset the radio
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 *
 * @warning NOT IMPLEMENTED
 */
sx126x_hal_status_t sx126x_hal_reset(const void *context)
{
    // NOT IMPLEMENTED
}

/**
 * Wake the radio up.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 *
 * @warning NOT IMPLEMENTED
 */
sx126x_hal_status_t sx126x_hal_wakeup(const void *context)
{
    // NOT IMPLEMENTED
}