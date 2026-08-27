# Select the example that should be built.
if(CONFIG_EXAMPLE_BUILD_WS2812)
    set(project_path "examples/ws2812/")
elseif(CONFIG_EXAMPLE_BUILD_SD)
    set(project_path "examples/sd/")
elseif(CONFIG_EXAMPLE_BUILD_ADC_DIGITAL_INPUT)
    set(project_path "examples/adc_digital_input/")
elseif(CONFIG_EXAMPLE_BUILD_RS485_CAN)
    set(project_path "examples/rs485_can/")
elseif(CONFIG_EXAMPLE_BUILD_ETHERNET_IPERF)
    set(project_path "examples/ethernet_iperf/")
elseif(CONFIG_EXAMPLE_BUILD_GENERAL_TEST)
    set(project_path "examples/general_test/")
endif()

idf_component_register(SRC_DIRS ${project_path}
                       INCLUDE_DIRS ${project_path}
                       REQUIRES driver
                                console
                                led_strip
                                lilygo_device_driver
                                esp_adc
                                esp_driver_gpio
                                esp_driver_sdspi
                                esp_driver_spi
                                esp_driver_twai
                                esp_driver_uart
                                esp_driver_usb_serial_jtag
                                esp_eth
                                esp_event
                                esp_http_server
                                esp_netif
                                esp_wifi
                                esp_timer
                                fatfs
                                lwip
                                nvs_flash
                                sdmmc)

if(CONFIG_EXAMPLE_BUILD_GENERAL_TEST)
    target_link_options(${COMPONENT_LIB}
                        INTERFACE "-Wl,--wrap=sdmmc_init_spi_crc")
endif()
