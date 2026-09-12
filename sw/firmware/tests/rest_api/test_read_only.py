import pytest


pytestmark = pytest.mark.integration


def get_json(api, path):
    response = api.request("GET", path)
    assert response.status_code == 200, response.text
    assert "application/json" in response.headers.get("Content-Type", "")
    return response.json()


def assert_collection(payload):
    assert isinstance(payload, dict)
    assert isinstance(payload.get("data"), list)
    assert isinstance(payload.get("count"), int)
    assert payload["count"] == len(payload["data"])


def test_index(api, device_reachable):
    response = api.request("GET", "/")
    assert response.status_code == 200
    assert response.content
    assert "text/html" in response.headers.get("Content-Type", "")


def test_system_schema(api, device_reachable):
    payload = get_json(api, "/api/v1/system")
    required = {"app_version", "uptime_s", "restart_reason", "mac", "state", "battery_voltage",
                "sd_card_detected", "component_status"}
    assert required <= payload.keys()
    assert isinstance(payload["app_version"], str)
    assert isinstance(payload["component_status"], list)
    for component in payload["component_status"]:
        assert isinstance(component, dict)
        assert isinstance(component.get("name"), str)
        assert isinstance(component.get("status"), str)


def test_can_bus_schema(api, device_reachable):
    payload = get_json(api, "/api/v1/can_bus")
    assert {"state", "bitrate", "rs_mode", "debug_mode", "initialized", "bus_connected", "node_status"} <= payload.keys()
    assert isinstance(payload["state"], str)
    assert isinstance(payload["node_status"], dict)
    assert {"twai_error_state", "tx_error_count", "rx_error_count"} <= payload["node_status"].keys()


def test_obd2_schema(api, device_reachable):
    payload = get_json(api, "/api/v1/obd2")
    assert {"continuous_running", "pid_initialized", "pid_def_count", "pid_data_count", "supported_pids"} <= payload.keys()
    assert isinstance(payload["supported_pids"], dict)
    assert isinstance(payload["supported_pids"].get("groups"), list)
    assert isinstance(payload["supported_pids"].get("count"), int)


@pytest.mark.parametrize("path", ["/api/v1/pid_def", "/api/v1/pid_data"])
def test_pid_collection_schema(api, device_reachable, path):
    assert_collection(get_json(api, path))


def test_dtc_schema(api, device_reachable):
    payload = get_json(api, "/api/v1/dtc")
    assert isinstance(payload, dict)
    assert isinstance(payload.get("dtcs"), list)
    assert isinstance(payload.get("count"), int)
    assert payload["count"] == len(payload["dtcs"])
    assert isinstance(payload.get("status"), str)


def test_settings_schema(api, device_reachable):
    payload = get_json(api, "/api/v1/settings")
    assert isinstance(payload, list)
    for section in payload:
        assert isinstance(section, dict)
        assert isinstance(section.get("name"), str)
        assert isinstance(section.get("settings"), dict)


def test_sd_info_schema(sd_info):
    required = {"name", "mount_path", "capacity", "used_space_mb", "max_freq_mhz", "is_sdio", "is_mmc",
                "is_mounted", "is_present"}
    assert required <= sd_info.keys()
    assert isinstance(sd_info["is_mounted"], bool)
    assert isinstance(sd_info["is_present"], bool)


def test_sd_root_tree_schema(api, mounted_sd):
    payload = get_json(api, "/api/v1/sd_card/tree")
    assert payload.get("status") == "success"
    assert payload.get("path") == "/"
    assert isinstance(payload.get("children"), list)
    for child in payload["children"]:
        assert isinstance(child, dict)
        if "children" in child:
            assert isinstance(child.get("path"), str)
            assert isinstance(child["children"], list)
        else:
            assert isinstance(child.get("name"), str)
            assert child.get("type") == "file"
            assert isinstance(child.get("size"), (int, float))
