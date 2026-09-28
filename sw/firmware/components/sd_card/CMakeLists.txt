idf_component_register(
    SRCS "sd_card.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver fatfs freertos sdmmc
    PRIV_REQUIRES json
)
