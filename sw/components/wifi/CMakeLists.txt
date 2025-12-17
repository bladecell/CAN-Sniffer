idf_component_register(
    SRCS "wifi.cpp"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_wifi nvs_flash
)