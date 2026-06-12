# Select the example that should be built.
if(CONFIG_EXAMPLE_BUILD_WS2812)
    set(project_path "examples/ws2812/")
elseif(CONFIG_EXAMPLE_BUILD_SD)
    set(project_path "examples/sd/")
elseif(CONFIG_EXAMPLE_BUILD_ADC)
    set(project_path "examples/adc/")
elseif(CONFIG_EXAMPLE_BUILD_RS485_CAN)
    set(project_path "examples/rs485_can/")
endif()

idf_component_register(SRC_DIRS ${project_path}
                       INCLUDE_DIRS ${project_path}
                       REQUIRES driver
                                led_strip
                                lilygo_device_driver
                                esp_adc
                                esp_driver_gpio
                                esp_driver_sdspi
                                esp_driver_spi
                                esp_driver_twai
                                esp_driver_uart
                                esp_driver_usb_serial_jtag
                                esp_timer
                                fatfs
                                sdmmc)
