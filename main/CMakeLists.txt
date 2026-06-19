idf_build_get_property(target IDF_TARGET)
if(target STREQUAL "esp32s3")
    set(SERIAL_DRIVER_COMPONENT esp_driver_usb_serial_jtag)
else()
    set(SERIAL_DRIVER_COMPONENT esp_driver_uart)
endif()

idf_component_register(
    SRCS
        "app_main.c"
        "drivers/encoder_pcnt.c"
        "drivers/joystick_adc.c"
        "drivers/motor_pwm.c"
        "protocol/json_command.c"
        "serial/serial_io.c"
        "telemetry/json_telemetry.c"
    INCLUDE_DIRS
        "."
        "drivers"
        "protocol"
        "serial"
        "telemetry"
    REQUIRES
        esp_adc
        esp_driver_mcpwm
        esp_driver_pcnt
        ${SERIAL_DRIVER_COMPONENT}
        esp_timer
        json
)
