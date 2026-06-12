# Select the example that should be built.
if(CONFIG_EXAMPLE_BUILD_WS2812)
    set(project_path "examples/ws2812/")
endif()

idf_component_register(SRC_DIRS ${project_path}
                       INCLUDE_DIRS ${project_path}
                       REQUIRES led_strip lilygo_device_driver)
