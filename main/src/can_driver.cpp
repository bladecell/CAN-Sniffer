#include "can_driver.hpp"


CanDriver::~CanDriver() {
    (void)deinit();
}

esp_err_t CanDriver::init() {
    if(isInitialized()) {
        return ESP_OK;
    }

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));
    ESP_ERROR_CHECK(twai_node_enable(node_hdl));
    initialized = true;
    return ESP_OK;
}

esp_err_t CanDriver::deinit() {
    ESP_ERROR_CHECK(twai_node_delete(node_hdl));
    return ESP_OK;
}

twai_node_status_t CanDriver::getStatus() {
    twai_node_status_t status{};
    ESP_ERROR_CHECK(twai_node_get_info(node_hdl, &status, &node_record));
    return status;
}