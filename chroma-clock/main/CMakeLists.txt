idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES "bme280" "config" "ds3231" "hd44780" "http" "sntp" "ticker" "wifi" "ws2812b"
)
