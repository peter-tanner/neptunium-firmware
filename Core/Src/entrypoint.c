#include "entrypoint.h"
#include "logging.h"
#include "fatfs.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
// #include "lsm6dsox.h"
#include "lps22hb.h"
// #include "hdc2010.h"

extern osMessageQId usbRxQueueHandle;

extern osThreadId loggingTaskHandle;

extern osThreadId lsm6dsoIntTaskHandle;
extern osThreadId lps22hbIntTaskHandle;

volatile uint8_t usb_connected = 0;
volatile uint8_t disk_initialized = USBD_FAIL;

/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
void StartEntrypointTask(void const *argument)
{
#if 0
    lsm6ds3tr_c_read_data_drdy_init();
    lps22hb_multi_read_fifo_init();
    hdc2010_init();

    // FIXME: USE CORRECT PRIORITY.
    osThreadDef(lsm6dsoIntTask, lsm6ds3tr_c_read_data_drdy_handler, osPriorityIdle, 0, 128);
    lsm6dsoIntTaskHandle = osThreadCreate(osThread(lsm6dsoIntTask), NULL);

    osThreadDef(lps22hbIntTask, lps22hb_multi_read_fifo_handler, osPriorityIdle, 0, 128);
    lps22hbIntTaskHandle = osThreadCreate(osThread(lps22hbIntTask), NULL);

    for (;;)
    {
        osDelay(1);
    }
#endif
#if 1

    // 3 CASES
    // 1. SYSTEM IS CONNECTED AS TELEMETRY GROUND STATION
    // 2. SYSTEM IS CONNECTED TO A COMPUTER AS A USB MASS STORAGE CLASS DEVICE
    // 3. SYSTEM IS LOGGING INFORMATION TO THE eMMC
    // TODO: WAIT FOR USER INPUT TO DECIDE WHETHER TO CHOOSE CASE 1 OR 2, IF TIMEOUT AFTER 60 SECONDS ACT AS LOGGING DEVICE.
    // OR TODO: USE USB COMPOSITE DEVICE

    // CURRENT CONFIGURATION - ONLY USE AS MASS STORAGE DEVICE.2

    // CHECK IF USB IS CONNECTED AND DISABLE FATFS IF CONNECTED.

#define BOOT_TEXT "\r\nBooted in interactive mode" \
                  "\r\nSelect an option:"          \
                  "\r\n1 - Ground station"         \
                  "\r\n2 - USB Drive"              \
                  "\r\nAny other key - Flight computer (Default)\r\n"

#define SELECTED_DRIVE_TEXT "\r\nSelected USB Drive mode. Please wait..."
#define SELECTED_GND_TEXT "\r\nSelected Ground station mode."
#define SELECTED_FLIGHT_TEXT "\r\nSelected Flight computer mode."

    osEvent connected_event = osSignalWait(0x01, 30000);
    uint8_t selected_option = '0';
    if (connected_event.status == osEventSignal)
    {
        osDelay(1000); // DELAY FOR SERIAL CONNECTION TO COMPLETE.
        uint8_t rx_buf[10];
        uint32_t rx_len;
        while (CDC_Transmit_FS(BOOT_TEXT, sizeof(BOOT_TEXT)) != USBD_OK)
            ;
        osEvent option_event = osMessageGet(usbRxQueueHandle, osWaitForever);
        if (option_event.status == osEventMessage)
            selected_option = (uint8_t)option_event.value.v;
    }

    switch (selected_option)
    {
    case '1':
        CDC_Transmit_FS(SELECTED_GND_TEXT, sizeof(SELECTED_GND_TEXT));
        break;
    case '2':
        USER_Driver.disk_initialize(0);
        disk_initialized = USBD_OK;
        CDC_Transmit_FS(SELECTED_DRIVE_TEXT, sizeof(SELECTED_DRIVE_TEXT));
        break;
    default:
        CDC_Transmit_FS(SELECTED_FLIGHT_TEXT, sizeof(SELECTED_FLIGHT_TEXT));
        osThreadDef(loggingTask, StartLoggingTask, osPriorityIdle, 0, 2048);
        loggingTaskHandle = osThreadCreate(osThread(loggingTask), NULL);
        break;
    }

    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
#endif
}
