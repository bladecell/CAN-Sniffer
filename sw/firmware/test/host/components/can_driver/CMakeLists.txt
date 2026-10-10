# Host stand-in for the real can_driver (TWAI hardware). Only the types the OBD2 parsing code touches.
idf_component_register(INCLUDE_DIRS "include" REQUIRES freertos)
