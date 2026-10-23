"""Offline canonical ELF/partition check. Never opens or runs a device."""
import json
import pathlib
import re
import subprocess
import sys


def function_body(text, name):
    match = re.search(r"^[0-9a-f]+ <" + re.escape(name) + r">:\n(.*?)(?=\n\n|\Z)", text, re.M | re.S)
    if not match:
        raise ValueError(f"missing function: {name}")
    return match.group(1)


def verify_disassembly(text):
    start = function_body(text, "start_sim_task(CanDriver*)")
    wrapper = function_body(text, "__wrap_xTaskCreatePinnedToCore")
    if not re.search(r"\bcall\w*\s+[^\n]*<__wrap_xTaskCreatePinnedToCore>", start):
        raise ValueError("production simulator does not call task wrapper")
    direct = re.search(r"\bcall\w*\s+[^\n]*<xTaskCreatePinnedToCore>", wrapper)
    indirect = re.search(r"\bl32r\s+(a\d+),[^\n]*<xTaskCreatePinnedToCore>[^\n]*\n[^\n]*\bcallx8\s+\1\b", wrapper)
    if not direct and not indirect:
        raise ValueError("wrapper does not call the real SDK creator")
    if "<legacyTaskCreationShouldFail()>" not in wrapper:
        raise ValueError("original OBD task injection missing from combined wrapper")


def main():
    build = pathlib.Path(sys.argv[1]).resolve()
    cache = (build / "CMakeCache.txt").read_text()
    objdump = re.search(r"^CMAKE_OBJDUMP:FILEPATH=(.+)$", cache, re.M).group(1)
    nm = re.search(r"^CMAKE_NM:FILEPATH=(.+)$", cache, re.M).group(1)
    elf = build / "obd_can_lifecycle_tests.elf"
    disassembly = subprocess.check_output([objdump, "-d", "-C", str(elf)], text=True)
    verify_disassembly(disassembly)
    symbols = subprocess.check_output([nm, "-C", str(elf)], text=True)
    for symbol in ("start_sim_task(CanDriver*)", "stop_sim_task()", "Settings::getCanConfig(CanDriver::Config&)", "runUnityMenu()", "OBD2::init()"):
        if symbol not in symbols:
            raise ValueError(f"missing production/legacy symbol: {symbol}")
    for forbidden in ("Settings::init()", "Settings::factoryReset()", "nvs_flash_erase", "esp_wifi_init", "esp_wifi_start", "SUPERVISOR::init("):
        if forbidden in symbols:
            raise ValueError(f"unexpected operation: {forbidden}")
    config = (build / "sdkconfig").read_text()
    for required in ('CONFIG_IDF_TARGET="esp32s3"', 'CONFIG_SNIFFER_HW_REV_V1_1=y', 'CONFIG_ESPTOOLPY_FLASHSIZE="16MB"'):
        if required not in config:
            raise ValueError(f"wrong configuration: {required}")
    firmware = pathlib.Path(__file__).resolve().parents[3]
    partition = firmware / "partitions_v1.csv"
    if f'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="{partition}"' not in config:
        raise ValueError("production partition CSV not selected")
    description = json.loads((build / "project_description.json").read_text())
    if pathlib.Path(description["project_path"]).resolve() != pathlib.Path(__file__).resolve().parents[1]:
        raise ValueError("not the canonical project")
    # Independently regenerate the production CSV and compare the emitted binary.
    generated = subprocess.check_output([sys.executable, str(pathlib.Path(description["idf_path"]) / "components/partition_table/gen_esp32part.py"), str(partition)])
    if generated != (build / "partition_table/partition-table.bin").read_bytes():
        raise ValueError("partition binary differs from production CSV")
    print("PASS: production simulator -> single combined wrapper -> real SDK; OBD injection retained")
    print("PASS: real stop/loader and legacy Unity/OBD linked; erase-recovery/Wi-Fi start/supervisor absent")
    print("PASS: isolated V1_1/16MB; partition binary exactly matches production CSV")
    print("LIMIT: offline wiring/config evidence only, no target execution")


if __name__ == "__main__":
    main()
