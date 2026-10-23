# Canonical OBD/CAN lifecycle harness

`tests/on_target` is the only lifecycle test application. It preserves the five
existing Unity CAN/OBD regressions and adds P2.8's safer simulator/NVS commands.
It compiles production CAN, simulator, OBD and settings sources unchanged.
The default boot waits for a command: no automatic tests, NVS initialization,
Wi-Fi, supervisor, web server, SD mount, erase-all or deliberate restart.

## Offline checks (authorized)

From the repository root, using the installed ESP-IDF 6.1 environment on PATH:

```sh
mkdir -p /tmp/opencode/p28/consolidation
g++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -Isw/firmware/tests/on_target/main \
  sw/firmware/tests/on_target/offline/test_support.cpp \
  -o /tmp/opencode/p28/consolidation/test-support
/tmp/opencode/p28/consolidation/test-support
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s sw/firmware/tests/on_target/offline -p 'test_*.py'
IDF_COMPONENT_MANAGER=0 idf.py -C sw/firmware/tests/on_target \
  -B /tmp/opencode/p28/consolidation/build build
PYTHONDONTWRITEBYTECODE=1 python3 sw/firmware/tests/on_target/offline/check_link.py \
  /tmp/opencode/p28/consolidation/build
git diff --check
```

Use the existing toolchain without `eim select` if environment settings writes
are prohibited. No install or component resolution is needed. The existing
managed cJSON component must be present; the manager must be disabled. CMake
requires an external build directory and seeds **only its isolated sdkconfig**
from the normal `sw/firmware/sdkconfig`. Only that copy's partition CSV path is
made absolute. Production Kconfig is imported rather than mirrored. V1_1/16MB
is required; no `set-target`, defconfig or changes to the normal config.
Discard an old single-app harness build directory before using this revision;
use a fresh external path, never the normal firmware build directory.

Host support tests cover one-shot selection, typed recovery parsing and
ordinary-error restoration orchestration. Source tests preserve all five legacy
test bodies and reject the old erase-capable bootstrap/restart/duplicate cases.
ELF checks establish simulator -> single wrapper -> SDK task creation, retention
of the original OBD injection branch and Unity entry, and absence of linked
settings erase-recovery/Wi-Fi startup/supervisor. The emitted partition binary
is compared against independently generated production `partitions_v1.csv`.
These are offline wiring/policy checks, not FreeRTOS, TWAI or NVS execution.

## Future operator preconditions (separate authorization required)

Offline implementation approval does NOT authorize serial attachment, reset,
flashing, target commands, NVS operations or normal-image boot experiments.
Serial attachment has reset this board before. Require exclusive ownership,
stable bench power and physical vehicle/CAN disconnection. V1_1 CAN pins are
RX4, TX5, LBK6, RS7. Legacy Unity tests can transmit; never use a vehicle.

Before any diagnostic installation:

1. Record device identity, current normal app/ELF SHA256, active OTA slot,
   boot selection and original config/partition/lockfile hashes. Keep the exact
   normal app and matching ELF ready for restoration.
2. Obtain and durably save an access-controlled external raw NVS backup with
   device identity, offset, size and checksum before any diagnostic flashing.
   It contains credentials: never paste it in logs. Raw partition recovery is
   an independently approved emergency operation, not a harness feature.
3. Independently verify the image fits the selected app slot and that the
   installed partition table is the production layout. The active slot is NOT
   assumed to be ota_0. Determine and approve the exact slot/address before
   installing **only the diagnostic app binary** there. Do not write bootloader,
   partition table, otadata, www, NVS or coredump. Do not use a full-project
   flash operation or its generated flash-arguments file. Account for existing
   OTA rollback/boot-selection state before deciding an app-only install is safe.
4. Plan bounded log capture and separately authorized resets. After any panic
   or watchdog, stop and preserve the matching ELF/core dump before image changes.

No device command is included in the offline recipe. Installation/restoration
must be reviewed as an exact operator-specific app-only procedure, not guessed
from a generic command. Verify the written app hash and subsequent boot identity.

## Explicit target commands

Commands are printable ASCII followed by newline; there is no network control.

### `unity`: retained legacy regressions

Enters the original ESP-IDF Unity menu after checking no pending NVS recovery
and cleaning CAN. Press Enter for its list; `*` runs the five retained cases:
CAN callback unregister/self-unregister, OBD shutdown/admission fencing, three
OBD worker-creation failures/retries, discovery reset and far-future poll wait.
The original initiating-task/ordinal fault branch remains in the single linker
wrapper. No P2.8 NVS test is registered in Unity, so `*` cannot mutate NVS.
Unity's menu may remain active indefinitely; returning to top-level commands
can require a separately approved reset. Finish NVS recovery before entering it.

The fixture still joins helpers and fences/cleans OBD after assertion longjmp.
Unsafe cleanup now halts without freeing live state or automatically rebooting.
The far-future test observes scheduler state and available locks, not the kernel
wait-object identity. Token/event resets, receive-owned multiframe cleanup,
diagnostic-wait cancellation and self-worker shutdown remain unclaimed.

### `lifecycle`: P2.8 cases 1–3

Warm up once, then ten real simulator init/deinit cycles plus double-deinit.
One friend definition supplies read-only snapshots of RX/TX queues, TX ring/size,
node, connected semaphore, task handles and running flags. Record stacks and
per-cycle internal/total 8-bit heap/task counts after 500 ms idle reclamation;
exact settled equality is the threshold, not an arbitrary permitted leak.
Lifetime mutexes and the static simulator queue intentionally persist.

Creation failure is one-shot at the actual DataSimTask creation call, selected
by name, real entry and driver instance. Require NO_MEM, empty partial resources
and successful retry. It does not exhaust global heap.

Stop timeout holds that task before its real entry, with production simRunning
already true. The real stop function waits two seconds and returns false.
Require TIMEOUT, identical retained storage/live sim handle, rejected reinit,
then release, cleanup and successful reinit. A ten-second gate expiry releases
it but invalidates the assertion. This avoids suspending inside arbitrary code;
it does not prove shutdown while actively processing simulator requests.

### `backup`, then `nvs <eight-hex-token>`: P2.8 case 4

`backup` directly calls non-erasing nvs_flash_init. Initialization/read errors
refuse the operation; there is no erase-all/retry or Settings::init call.
Only storage/c_rx_q and c_tx_q are backed up: exact absence or u32/value.
Other types or unreadable originals refuse mutation rather than treating them
as absent. Save the entire P28_BACKUP record (MAC, namespace, presence/type/value,
token) durably OFF DEVICE with image/device identity and verify the saved file.

`nvs <token>` attests that external durability and explicit mutation approval.
The random token is an accident/staleness guard, not authentication. The app
checks current originals still match, requires a complete readable CAN section
with V1_1 pins, and never creates missing defaults. Stored debug is not changed;
initialization uses loopback only in RAM.

Inputs (rx,tx) are (0,0), (UINT32_MAX,UINT32_MAX), (0,7), (31,UINT32_MAX).
The actual production loader must yield 20/20, 20/20, 20/7 and 31/20; each config
is initialized/cleaned. This proves per-key fallback, not blanket replacement.
Ordinary exits after the first attempted write always restore originals,
commit and read back exact type/presence/value. No Unity longjmp crosses it.
Restoration failure remains FAIL, retains known backup/recovery state and blocks
new tests/backups. `recover` retries that retained backup in the same boot.

### `restore <rx> <tx>`: interruption recovery

Reset/power loss destroys the in-RAM record: externally saved backup is mandatory.
After separately approved access to this same known image/device, use its exact
saved values, e.g. `restore absent u32:20`. Forms are `absent` and u32:0 through
u32:4294967295. Check the MAC externally; the app cannot infer originals after
interruption. Never substitute defaults. Only the two keys are written, committed
and verified. Failed restoration retains known recovery state; do not continue
tests or claim success. Unsupported types/partition failures require the approved
raw-NVS fallback, never erase-all. Retain backups until normal postflight passes.

## Evidence and restoration

Retain full CASE/RESTORE results, reset reason, timings, heap and stacks. A finite
flat run is not proof of zero leaks. Same-boot loader/init success does NOT prove
normal supervisor boot, reset survival or absence of a reboot loop. A separately
authorized normal-image invalid-key boot experiment with external restoration
remains necessary; this app never deliberately leaves invalid keys for it.

Before finishing, verify exact key restoration; restore the approved normal app
only to its intended slot, verify SHA/ELF identity and unchanged config/partition
hashes, capture a bounded normal boot/no panic/watchdog and approved read-only
settings postflight. No P2.8 hardware boxes may be checked from offline evidence.
T3/T4, B53 saturation/TX ordering/OpGate drain and urgent FC timing stay excluded.
Specialist owns focused offline/later approved hardware evidence; primary owns
full CI and a fresh independent reviewer owns the consolidation/safety review.
