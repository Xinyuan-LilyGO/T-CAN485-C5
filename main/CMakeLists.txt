# Select the example that should be built.
if(CONFIG_EXAMPLE_BUILD_WS2812)
    set(project_path "examples/ws2812/")
elseif(CONFIG_EXAMPLE_BUILD_SD)
    set(project_path "examples/sd/")
elseif(CONFIG_EXAMPLE_BUILD_ADC)
    set(project_path "examples/adc/")
elseif(CONFIG_EXAMPLE_BUILD_RS485_CAN)
    set(project_path "examples/rs485_can/")
elseif(CONFIG_EXAMPLE_BUILD_GENERAL_TEST)
    set(project_path "examples/general_test/")
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
                                esp_event
                                esp_http_server
                                esp_netif
                                esp_wifi
                                esp_timer
                                fatfs
                                lwip
                                nvs_flash
                                sdmmc)
