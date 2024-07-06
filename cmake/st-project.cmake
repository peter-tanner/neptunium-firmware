# THIS FILE IS AUTOMATICALLY GENERATED. DO NOT EDIT.
# BASED ON c:\Users\Peter\Documents\GITHUB\neptunium-master

function(add_st_target_properties TARGET_NAME)

target_compile_definitions(
    ${TARGET_NAME} PRIVATE
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:ASM>>:DEBUG>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:DEBUG>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:USE_HAL_DRIVER>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:STM32F302xB>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:STM32F302xC>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:DEBUG>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:USE_HAL_DRIVER>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:STM32F302xB>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:USE_HAL_DRIVER>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:STM32F302xB>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:STM32F302xC>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:USE_HAL_DRIVER>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:STM32F302xB>"
)

target_include_directories(
    ${TARGET_NAME} PRIVATE
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Core\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F3xx_HAL_Driver\\Inc\\Legacy>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F3xx_HAL_Driver\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Device\\ST\\STM32F3xx\\Include>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Include>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\lsm6dsox-pid>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\lps22hb-pid>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\ubx_parser>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\sx126x_driver\\src>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\tinyusb\\src>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/FATFS_15\\Target>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/FATFS_15\\App>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\Third_Party\\FatFs_15\\src>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\Third_Party\\FreeRTOS\\Source\\include>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\Third_Party\\FreeRTOS\\Source\\CMSIS_RTOS>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\Third_Party\\FreeRTOS\\Source\\portable\\GCC\\ARM_CM4F>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\App>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\Class\\CDC_ACM\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\Class\\MSC\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\Class\\COMPOSITE\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\Core\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\Target>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\App>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\Target>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Class\\CDC\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Core\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F3xx_HAL_Driver\\Inc\\Legacy>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F3xx_HAL_Driver\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Device\\ST\\STM32F3xx\\Include>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Include>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\App>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\Target>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Inc>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Class\\CDC\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Core\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F3xx_HAL_Driver\\Inc\\Legacy>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F3xx_HAL_Driver\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Device\\ST\\STM32F3xx\\Include>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Include>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/FATFS_15\\Target>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/FATFS_15\\App>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\Third_Party\\FatFs_15\\src>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\Third_Party\\FreeRTOS\\Source\\include>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\Third_Party\\FreeRTOS\\Source\\CMSIS_RTOS>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\Third_Party\\FreeRTOS\\Source\\portable\\GCC\\ARM_CM4F>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\App>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\Class\\CDC_ACM\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\Class\\MSC\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\Class\\COMPOSITE\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\Core\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE\\Target>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_COMPOSITE>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\App>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\Target>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Class\\CDC\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Core\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F3xx_HAL_Driver\\Inc\\Legacy>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\STM32F3xx_HAL_Driver\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Device\\ST\\STM32F3xx\\Include>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Drivers\\CMSIS\\Include>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\App>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/USB_DEVICE\\Target>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Inc>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:${PROJECT_SOURCE_DIR}/Middlewares\\ST\\STM32_USB_Device_Library\\Class\\CDC\\Inc>"
)

target_compile_options(
    ${TARGET_NAME} PRIVATE
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:ASM>>:-g3>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:-g3>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:-g3>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:ASM>>:-g0>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:-g0>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:-g0>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:-Og>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:-Os>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:-Os>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:C>>:>"
    "$<$<AND:$<CONFIG:Debug>,$<COMPILE_LANGUAGE:CXX>>:>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:C>>:>"
    "$<$<AND:$<NOT:$<CONFIG:Debug>>,$<COMPILE_LANGUAGE:CXX>>:>"
    "$<$<CONFIG:Debug>:-mcpu=cortex-m4>"
    "$<$<CONFIG:Debug>:-mfpu=fpv4-sp-d16>"
    "$<$<CONFIG:Debug>:-mfloat-abi=hard>"
    "$<$<NOT:$<CONFIG:Debug>>:-mcpu=cortex-m4>"
    "$<$<NOT:$<CONFIG:Debug>>:-mfpu=fpv4-sp-d16>"
    "$<$<NOT:$<CONFIG:Debug>>:-mfloat-abi=hard>"
)

target_link_libraries(
    ${TARGET_NAME} PRIVATE
)

target_link_directories(
    ${TARGET_NAME} PRIVATE
)

target_link_options(
    ${TARGET_NAME} PRIVATE
    "$<$<CONFIG:Debug>:-mcpu=cortex-m4>"
    "$<$<CONFIG:Debug>:-mfpu=fpv4-sp-d16>"
    "$<$<CONFIG:Debug>:-mfloat-abi=hard>"
    "$<$<NOT:$<CONFIG:Debug>>:-mcpu=cortex-m4>"
    "$<$<NOT:$<CONFIG:Debug>>:-mfpu=fpv4-sp-d16>"
    "$<$<NOT:$<CONFIG:Debug>>:-mfloat-abi=hard>"
    -T
    "$<$<CONFIG:Debug>:${PROJECT_SOURCE_DIR}/STM32F302CBTX_FLASH.ld>"
    "$<$<NOT:$<CONFIG:Debug>>:${PROJECT_SOURCE_DIR}/STM32F302CBTX_FLASH.ld>"
)

target_sources(
    ${TARGET_NAME} PRIVATE
    "FATFS_15\\App\\fatfs.c"
    "FATFS_15\\Target\\user_diskio_spi.c"
    "FATFS_15\\Target\\user_diskio.c"
    "Core\\Src\\entrypoint.c"
    "Core\\Src\\freertos.c"
    "Core\\Src\\logging.c"
    "Core\\Src\\lps22hb.c"
    "Core\\Src\\lsm6dso.c"
    "Core\\Src\\main.c"
    "Core\\Src\\radio.c"
    "Core\\Src\\stm32f3xx_hal_msp.c"
    "Core\\Src\\stm32f3xx_hal_timebase_tim.c"
    "Core\\Src\\stm32f3xx_it.c"
    "Core\\Src\\syscalls.c"
    "Core\\Src\\sysmem.c"
    "Core\\Src\\system_stm32f3xx.c"
    "Core\\Src\\utils.c"
    "Core\\Startup\\startup_stm32f302c8tx.s"
    "Middlewares\\ST\\STM32_USB_Device_Library\\Class\\CDC\\Src\\usbd_cdc.c"
    "Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Src\\usbd_core.c"
    "Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Src\\usbd_ctlreq.c"
    "Middlewares\\ST\\STM32_USB_Device_Library\\Core\\Src\\usbd_ioreq.c"
    "Middlewares\\Third_Party\\FatFs_15\\src\\diskio.c"
    "Middlewares\\Third_Party\\FatFs_15\\src\\ff_gen_drv.c"
    "Middlewares\\Third_Party\\FatFs_15\\src\\ff.c"
    "Middlewares\\Third_Party\\FatFs_15\\src\\ffsystem.c"
    "Middlewares\\Third_Party\\FatFs_15\\src\\ffunicode.c"
    "Middlewares\\Third_Party\\FatFs_15\\src\\option\\ccsbcs.c"
    "Middlewares\\Third_Party\\FatFs_15\\src\\option\\syscall.c"
    "Middlewares\\Third_Party\\FreeRTOS\\Source\\CMSIS_RTOS\\cmsis_os.c"
    "Middlewares\\Third_Party\\FreeRTOS\\Source\\croutine.c"
    "Middlewares\\Third_Party\\FreeRTOS\\Source\\event_groups.c"
    "Middlewares\\Third_Party\\FreeRTOS\\Source\\list.c"
    "Middlewares\\Third_Party\\FreeRTOS\\Source\\portable\\GCC\\ARM_CM4F\\port.c"
    "Middlewares\\Third_Party\\FreeRTOS\\Source\\portable\\MemMang\\heap_4.c"
    "Middlewares\\Third_Party\\FreeRTOS\\Source\\queue.c"
    "Middlewares\\Third_Party\\FreeRTOS\\Source\\stream_buffer.c"
    "Middlewares\\Third_Party\\FreeRTOS\\Source\\tasks.c"
    "Middlewares\\Third_Party\\FreeRTOS\\Source\\timers.c"
    "Drivers\\lps22hb-pid\\lps22hb_reg.c"
    "Drivers\\lsm6dsox-pid\\lsm6dsox_reg.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_adc_ex.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_adc.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_can.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_cortex.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_dma.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_exti.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_flash_ex.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_flash.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_gpio.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_i2c_ex.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_i2c.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_pcd_ex.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_pcd.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_pwr_ex.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_pwr.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_rcc_ex.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_rcc.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_spi_ex.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_spi.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_tim_ex.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_tim.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_uart_ex.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal_uart.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_hal.c"
    "Drivers\\STM32F3xx_HAL_Driver\\Src\\stm32f3xx_ll_usb.c"
    "Drivers\\sx126x_driver\\src\\lr_fhss_mac.c"
    "Drivers\\sx126x_driver\\src\\sx126x_driver_version.c"
    "Drivers\\sx126x_driver\\src\\sx126x_lr_fhss.c"
    "Drivers\\sx126x_driver\\src\\sx126x.c"
    "Drivers\\ubx_parser\\neo_m8_conversion.c"
    "Drivers\\ubx_parser\\neo_m8_gps.c"
    "Drivers\\ubx_parser\\neo_m8_ubx_checksum.c"
    "Drivers\\ubx_parser\\ringbuffer_char.c"
    "USB_DEVICE\\App\\usb_device.c"
    "USB_DEVICE\\App\\usbd_cdc_if.c"
    "USB_DEVICE\\App\\usbd_desc.c"
    "USB_DEVICE\\Target\\usbd_conf.c"
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
    $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.hex
)

add_custom_command(
    TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
    $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.bin
)

endfunction()