#include <atomic>
#include <cstdint>

#include "can_driver.hpp"
#include "obd2.hpp"
#include "obd2_simulator.hpp"
#include "settings.hpp"
#include "unity.h"
#include "unity_test_runner.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "nvs.h"

namespace {
constexpr TickType_t kWatchdog = pdMS_TO_TICKS(5000);
CanDriver& can = CanDriver::getInstance();
OBD2& obd = OBD2::getInstance();
Settings& settings = Settings::getInstance();
constexpr EventBits_t CALLBACK_ENTERED = 1u << 0, CALLBACK_RELEASE = 1u << 1,
                      UNREGISTER_ENTERED = 1u << 2, UNREGISTER_DONE = 1u << 3,
                      DISPATCH_DONE = 1u << 4, OBD_DEINIT_DONE = 1u << 5,
                      SUBSCRIBER_ENTERED = 1u << 6, SUBSCRIBER_RELEASE = 1u << 7;

struct SuiteFixture {
    EventGroupHandle_t gates = nullptr;
    TaskHandle_t dispatchTask = nullptr;
    TaskHandle_t unregisterTask = nullptr;
    TaskHandle_t obdDeinitTask = nullptr;
    SemaphoreHandle_t subscriberEntered = nullptr;
    SemaphoreHandle_t subscriberRelease = nullptr;
    std::atomic<unsigned> callbackCalls{0};
    std::atomic<esp_err_t> selfUnregisterResult{ESP_FAIL};
    std::atomic<esp_err_t> unregisterResult{ESP_FAIL};
    std::atomic<esp_err_t> obdDeinitResult{ESP_FAIL};
    std::atomic<unsigned> createOrdinal{0};
    std::atomic<unsigned> failOrdinal{0};
    std::atomic<bool> injectFailure{false};
    std::atomic<TaskHandle_t> injectorTask{nullptr};
    bool canSetupOk = false;
    bool healthWasSuspended = false;
    CanDriver::STATE savedCanState = CanDriver::STATE::NOT_INITIALIZED;
    // P2.8 case 4: the two persisted CAN queue keys are backed up before injection and restored in tearDown, even when
    // an assertion longjmps out of the test body.
    bool canQueueKeysSaved = false;
    bool savedRxPresent = false;
    bool savedTxPresent = false;
    uint32_t savedRxValue = 0;
    uint32_t savedTxValue = 0;
    // P2.8 case 3: tearDown must resume a simulator task it suspended before it can retry teardown.
    bool simTaskSuspended = false;
};
SuiteFixture fixture;
void cleanupFixture();

void restartOnUnsafeCleanup(const char* reason) {
    ESP_LOGE("lifecycle_test", "Cannot prove test cleanup quiescent: %s; restarting", reason);
    esp_restart();
    for (;;) taskYIELD();
}

void blockingCallback(void*, bool) {
    ++fixture.callbackCalls;
    xEventGroupSetBits(fixture.gates, CALLBACK_ENTERED);
    (void)xEventGroupWaitBits(fixture.gates, CALLBACK_RELEASE, pdFALSE, pdTRUE, portMAX_DELAY);
}
void selfUnregisterCallback(void*, bool) {
    fixture.selfUnregisterResult.store(can.setConnectionChangeCallback(nullptr, nullptr, 0));
}
void dispatchTaskEntry(void*);
void unregisterTaskEntry(void*) {
    xEventGroupSetBits(fixture.gates, UNREGISTER_ENTERED);
    fixture.unregisterResult.store(can.setConnectionChangeCallback(nullptr, nullptr, kWatchdog));
    xEventGroupSetBits(fixture.gates, UNREGISTER_DONE);
    vTaskSuspend(nullptr);
}
void obdDeinitTaskEntry(void*) {
    fixture.obdDeinitResult.store(obd.deinit());
    xEventGroupSetBits(fixture.gates, OBD_DEINIT_DONE);
    vTaskSuspend(nullptr);
}

void releaseAllGates() {
    if (fixture.gates) xEventGroupSetBits(fixture.gates, CALLBACK_RELEASE | SUBSCRIBER_RELEASE);
    if (fixture.subscriberRelease) xSemaphoreGive(fixture.subscriberRelease);
}

bool waitTaskSuspended(TaskHandle_t task, TickType_t timeout) {
    if (!task) return true;
    TickType_t deadline = xTaskGetTickCount() + timeout;
    do {
        if (eTaskGetState(task) == eSuspended) return true;
        taskYIELD();
    } while (ticksUntilDeadline(deadline, xTaskGetTickCount()) != 0);
    return eTaskGetState(task) == eSuspended;
}

void joinSuspendedHelper(TaskHandle_t& task) {
    if (!task) return;
    if (!waitTaskSuspended(task, kWatchdog)) restartOnUnsafeCleanup("helper did not reach join suspension");
    vTaskDelete(task);
    task = nullptr;
}

CanDriver::Config canConfig() {
    CanDriver::Config c{};
    c.bitrate = CanDriver::Bitrate::BITRATE_500K;
    // Project board defaults (utilities.h): RX=4, TX=5, LBK=6, RS=7.
    c.rx_pin = GPIO_NUM_4; c.tx_pin = GPIO_NUM_5; c.lbk_pin = GPIO_NUM_6; c.rs_pin = GPIO_NUM_7;
    c.rx_queue_size = 20; c.tx_queue_depth = 20;
    return c;
}

// P2.8 case 1-3: the production simulator hooks, so a cycle covers the simulator task as well as the driver.
CanDriver::Config simCanConfig() {
    CanDriver::Config c = canConfig();
    c.debug = true;
    return c;
}

size_t internalFreeHeap() {
    return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

// P2.8 case 4 mirrors the keys Settings::getCanConfig() reads. Keep the names in sync with settings.cpp.
struct RawU32 { bool present = false; uint32_t value = 0; };

RawU32 readRawU32(const char* key) {
    RawU32 out;
    nvs_handle_t h;
    if (nvs_open("storage", NVS_READONLY, &h) != ESP_OK) return out;
    uint32_t value = 0;
    if (nvs_get_u32(h, key, &value) == ESP_OK) { out.present = true; out.value = value; }
    nvs_close(h);
    return out;
}

bool writeRawU32(const char* key, uint32_t value) {
    nvs_handle_t h;
    if (nvs_open("storage", NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t err = nvs_set_u32(h, key, value);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err == ESP_OK;
}

bool eraseRawKey(const char* key) {
    nvs_handle_t h;
    if (nvs_open("storage", NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t err = nvs_erase_key(h, key);
    if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err == ESP_OK;
}

void saveCanQueueKeys() {
    const RawU32 rx = readRawU32("c_rx_q");
    const RawU32 tx = readRawU32("c_tx_q");
    fixture.savedRxPresent = rx.present;
    fixture.savedTxPresent = tx.present;
    fixture.savedRxValue = rx.value;
    fixture.savedTxValue = tx.value;
    fixture.canQueueKeysSaved = true;
}

void restoreCanQueueKeys() {
    if (!fixture.canQueueKeysSaved) return;
    const bool rxOk = fixture.savedRxPresent ? writeRawU32("c_rx_q", fixture.savedRxValue) : eraseRawKey("c_rx_q");
    const bool txOk = fixture.savedTxPresent ? writeRawU32("c_tx_q", fixture.savedTxValue) : eraseRawKey("c_tx_q");
    fixture.canQueueKeysSaved = false;
    if (!rxOk || !txOk) ESP_LOGE("lifecycle_test", "Failed to restore the persisted CAN queue keys");
}

// Tests leave the driver in the same shape setUp() expects: initialized, no simulator hooks, default queues.
void leaveCanInitialized() {
    can.setSimHooks(nullptr, nullptr, nullptr);
    if (can.deinit() != ESP_OK) restartOnUnsafeCleanup("CAN did not deinitialize after the lifecycle test");
    if (can.init(canConfig()) != ESP_OK) restartOnUnsafeCleanup("CAN did not restart after the lifecycle test");
}
}

// Tests-only friends exist exclusively in this standalone target app.
struct CanDriverLifecycleTestAccess {
    static void dispatch(CanDriver& d, bool connected) { d.connectionChangeCb(connected); }
    static void holdConnectedForPollWait(CanDriver& d, SuiteFixture& f) {
        f.savedCanState = d.canState.load();
        f.healthWasSuspended = false;
        if (d.healthCheckTaskHandle && eTaskGetState(d.healthCheckTaskHandle) != eSuspended) {
            vTaskSuspend(d.healthCheckTaskHandle);
            f.healthWasSuspended = true;
            if (!waitTaskSuspended(d.healthCheckTaskHandle, kWatchdog))
                restartOnUnsafeCleanup("CAN health task did not suspend for poll fixture");
        }
        d.canState.store(CanDriver::STATE::CONNECTED);
    }
    static void resumeHealthForPollWait(CanDriver& d) {
        if (d.healthCheckTaskHandle && eTaskGetState(d.healthCheckTaskHandle) == eSuspended)
            vTaskResume(d.healthCheckTaskHandle);
    }
    static void restoreState(CanDriver& d, CanDriver::STATE state) { d.canState.store(state); }
    // P2.8: resource introspection for the init/deinit and fault-path cases.
    static bool hasResources(CanDriver& d) { return d.hasResources(); }
    static bool queuesPresent(CanDriver& d) { return d.rxQueue != nullptr && d.txQueue != nullptr && d.txRing_ != nullptr; }
    static bool tasksStopped(CanDriver& d) {
        return !d.txRunning_.load() && !d.healthRunning_.load() && d.txTaskHandle == nullptr &&
               d.healthCheckTaskHandle == nullptr;
    }
    static bool nodePresent(CanDriver& d) { return d.nodeHdl != nullptr; }
};

namespace {
void dispatchTaskEntry(void*) {
    CanDriverLifecycleTestAccess::dispatch(can, true);
    xEventGroupSetBits(fixture.gates, DISPATCH_DONE);
    vTaskSuspend(nullptr);
}
}

struct OBD2LifecycleTestAccess {
    static void injectConnection(OBD2& o, bool connected) { o.runOBDIIConnectedCallbacks(connected); }
    static bool isStopping(OBD2& o) {
        xSemaphoreTake(o.admissionMtx_, portMAX_DELAY);
        bool value = o.lifecycleState_ == OBD2::LifecycleState::Stopping;
        xSemaphoreGive(o.admissionMtx_);
        return value;
    }
    static bool isStopped(OBD2& o) {
        xSemaphoreTake(o.admissionMtx_, portMAX_DELAY);
        bool value = o.lifecycleState_ == OBD2::LifecycleState::Stopped;
        xSemaphoreGive(o.admissionMtx_);
        return value;
    }
    static bool fullyUnwound(OBD2& o) {
        xSemaphoreTake(o.admissionMtx_, portMAX_DELAY);
        const bool lifecycle = o.lifecycleState_ == OBD2::LifecycleState::Stopped &&
                               o.ReceiveTaskHandle == nullptr && o.PollTaskHandle == nullptr &&
                               o.callbackWorkerTaskHandle == nullptr && o.createdWorkers_ == 0 &&
                               o.activeOperations_ == 0 && !o.canCallbackInstalled_;
        xSemaphoreGive(o.admissionMtx_);
        return lifecycle && o.event_queue == nullptr && o.derivedPidQueue_ == nullptr &&
               o.xPidConnectedSemaphore == nullptr && o.xBusArbitrationMutex == nullptr &&
               o.xBusConnectionSemaphore == nullptr && o.xRequestNextPIDSemaphore == nullptr &&
               o.healthCheckSemaphore == nullptr && o.pollQueue.getConsumerTask() == nullptr;
    }
    static bool ownsRuntime(OBD2& o) {
        return o.event_queue && o.derivedPidQueue_ && o.xBusConnectionSemaphore && o.callbackWorkerTaskHandle;
    }
    static bool detached(OBD2& o) { return o.pollQueue.getConsumerTask() == nullptr; }
    static eTaskState pollTaskState(OBD2& o) {
        xSemaphoreTake(o.admissionMtx_, portMAX_DELAY);
        TaskHandle_t handle = o.PollTaskHandle;
        eTaskState state = handle ? eTaskGetState(handle) : eDeleted;
        xSemaphoreGive(o.admissionMtx_);
        return state;
    }
    static bool pollLocksAvailable(OBD2& o) {
        if (xSemaphoreTake(o.xBusArbitrationMutex, 0) != pdTRUE) return false;
        xSemaphoreGive(o.xBusArbitrationMutex);
        if (xSemaphoreTake(o.configurationMtx_, 0) != pdTRUE) return false;
        xSemaphoreGive(o.configurationMtx_);
        return true;
    }
    static TickType_t pollWaitTicks(OBD2& o) { return o.pollQueue.getWait(); }
    static bool queueFarFutureRequest(OBD2& o, TickType_t ticks) {
        PollRequest request{};
        request.nextWake = xTaskGetTickCount() + ticks;
        request.isRaw = true;
        return o.pollQueue.push(request);
    }
    static void seedDiscovery(OBD2& o) {
        // Discovery uses discovery -> configuration order in the production
        // response path. Taking both prevents a racing response from tearing
        // this fixture assignment.
        xSemaphoreTake(o.discoveryMtx_, portMAX_DELAY);
        xSemaphoreTake(o.configurationMtx_, portMAX_DELAY);
        o.discoveryActive_ = true; o.discoveryFailed_ = true;
        o.discoveryExpectedGroup_ = 3; o.discoverySeenGroups_ = 0x55;
        xSemaphoreGive(o.configurationMtx_);
        xSemaphoreGive(o.discoveryMtx_);
    }
    static bool discoveryReset(OBD2& o) {
        xSemaphoreTake(o.discoveryMtx_, portMAX_DELAY);
        xSemaphoreTake(o.configurationMtx_, portMAX_DELAY);
        const bool reset = !o.discoveryActive_ && !o.discoveryFailed_ &&
                           o.discoveryExpectedGroup_ == 0xFF && o.discoverySeenGroups_ == 0;
        xSemaphoreGive(o.configurationMtx_);
        xSemaphoreGive(o.discoveryMtx_);
        return reset;
    }
};

namespace {
void cleanupFixture() {
    // This runs from Unity tearDown, including after an assertion longjmp.
    fixture.injectFailure.store(false);
    fixture.injectorTask.store(nullptr);
    releaseAllGates();

    // Rejoin every helper before inspecting or destroying state it could touch.
    joinSuspendedHelper(fixture.dispatchTask);
    joinSuspendedHelper(fixture.unregisterTask);
    joinSuspendedHelper(fixture.obdDeinitTask);
    restoreCanQueueKeys();

    // A P2.8 fault-path assertion may have left the simulator suspended or the driver partially torn down.
    if (fixture.simTaskSuspended) {
        if (xDataSimTaskHandle != nullptr) vTaskResume(xDataSimTaskHandle);
        fixture.simTaskSuspended = false;
    }

    if (can.isInitialized()) {
        if (can.setConnectionChangeCallback(nullptr, nullptr, kWatchdog) != ESP_OK)
            restartOnUnsafeCleanup("CAN callback fence did not quiesce");
    }

    // A callback subscriber may have captured fixture-owned semaphores; stop
    // OBD before deleting those semaphores or the event group.
    if (obd.deinit() != ESP_OK || !OBD2LifecycleTestAccess::fullyUnwound(obd))
        restartOnUnsafeCleanup("OBD workers/resources did not fully unwind");

    CanDriverLifecycleTestAccess::restoreState(can, fixture.savedCanState);
    if (fixture.healthWasSuspended) {
        CanDriverLifecycleTestAccess::resumeHealthForPollWait(can);
        fixture.healthWasSuspended = false;
    }

    // The OBD tests keep CAN up across cases; the P2.8 cases explicitly return it to the non-debug baseline. If one of
    // them failed before doing so, restore that baseline here so the next setUp() does not skip a needed init.
    if (!can.isInitialized() || can.getConfig().debug) {
        if (can.deinit() != ESP_OK) restartOnUnsafeCleanup("CAN did not deinitialize after a failed lifecycle case");
        if (can.init(canConfig()) != ESP_OK) restartOnUnsafeCleanup("CAN did not restart after a failed lifecycle case");
    }
    can.setSimHooks(nullptr, nullptr, nullptr);

    if (fixture.subscriberEntered) { vSemaphoreDelete(fixture.subscriberEntered); fixture.subscriberEntered = nullptr; }
    if (fixture.subscriberRelease) { vSemaphoreDelete(fixture.subscriberRelease); fixture.subscriberRelease = nullptr; }
}
}

extern "C" BaseType_t __real_xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, UBaseType_t,
                                                       TaskHandle_t*, BaseType_t);
extern "C" BaseType_t __wrap_xTaskCreatePinnedToCore(TaskFunction_t entry, const char* name, uint32_t stack,
                                                       void* arg, UBaseType_t priority, TaskHandle_t* handle,
                                                       BaseType_t core) {
    if (fixture.injectFailure.load() && xTaskGetCurrentTaskHandle() == fixture.injectorTask.load()) {
        const unsigned ordinal = ++fixture.createOrdinal;
        if (ordinal == fixture.failOrdinal.load()) return pdFAIL;
    }
    return __real_xTaskCreatePinnedToCore(entry, name, stack, arg, priority, handle, core);
}

extern "C" void setUp(void) {
    TEST_ASSERT_NOT_NULL(fixture.gates);
    const esp_err_t canInit = can.isInitialized() ? ESP_OK : can.init(canConfig());
    if (canInit != ESP_OK) restartOnUnsafeCleanup("CAN setup failed; partial controller init is not safely reusable");
    TEST_ASSERT_EQUAL(ESP_OK, canInit);
    TEST_ASSERT_TRUE(OBD2LifecycleTestAccess::fullyUnwound(obd));
    fixture.dispatchTask = nullptr;
    fixture.unregisterTask = nullptr;
    fixture.obdDeinitTask = nullptr;
    fixture.callbackCalls = 0;
    fixture.selfUnregisterResult = ESP_FAIL;
    fixture.unregisterResult = ESP_FAIL;
    fixture.obdDeinitResult = ESP_FAIL;
    fixture.injectFailure = false;
    fixture.injectorTask = nullptr;
    fixture.healthWasSuspended = false;
    fixture.savedCanState = can.getState();
    if (fixture.subscriberEntered) { vSemaphoreDelete(fixture.subscriberEntered); fixture.subscriberEntered = nullptr; }
    if (fixture.subscriberRelease) { vSemaphoreDelete(fixture.subscriberRelease); fixture.subscriberRelease = nullptr; }
    xEventGroupClearBits(fixture.gates, 0xFF);
}

extern "C" void tearDown(void) { cleanupFixture(); }

TEST_CASE("CAN callback unregister fences in-flight dispatch and rejects self-unregister", "[lifecycle]") {
    TEST_ASSERT_EQUAL(ESP_OK, can.setConnectionChangeCallback(blockingCallback, nullptr));
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(dispatchTaskEntry, "dispatch", 3072, nullptr, 5, &fixture.dispatchTask));
    TEST_ASSERT_NOT_EQUAL(0, xEventGroupWaitBits(fixture.gates, CALLBACK_ENTERED, pdFALSE, pdTRUE, kWatchdog) & CALLBACK_ENTERED);
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(unregisterTaskEntry, "unregister", 3072, nullptr, 5, &fixture.unregisterTask));
    TEST_ASSERT_NOT_EQUAL(0, xEventGroupWaitBits(fixture.gates, UNREGISTER_ENTERED, pdFALSE, pdTRUE, kWatchdog) & UNREGISTER_ENTERED);
    TEST_ASSERT_EQUAL(0, xEventGroupWaitBits(fixture.gates, UNREGISTER_DONE, pdFALSE, pdTRUE, pdMS_TO_TICKS(50)) & UNREGISTER_DONE);
    xEventGroupSetBits(fixture.gates, CALLBACK_RELEASE);
    TEST_ASSERT_NOT_EQUAL(0, xEventGroupWaitBits(fixture.gates, UNREGISTER_DONE, pdFALSE, pdTRUE, kWatchdog) & UNREGISTER_DONE);
    TEST_ASSERT_EQUAL(ESP_OK, fixture.unregisterResult.load());
    TEST_ASSERT_NOT_EQUAL(0, xEventGroupWaitBits(fixture.gates, DISPATCH_DONE, pdFALSE, pdTRUE, kWatchdog) & DISPATCH_DONE);
    TEST_ASSERT_TRUE(waitTaskSuspended(fixture.dispatchTask, kWatchdog));
    TEST_ASSERT_TRUE(waitTaskSuspended(fixture.unregisterTask, kWatchdog));
    TEST_ASSERT_EQUAL(1, fixture.callbackCalls.load());

    TEST_ASSERT_EQUAL(ESP_OK, can.setConnectionChangeCallback(selfUnregisterCallback, nullptr));
    CanDriverLifecycleTestAccess::dispatch(can, true);
    // Assert only after production dispatch has returned and released its fence.
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, fixture.selfUnregisterResult.load());
    TEST_ASSERT_EQUAL(ESP_OK, can.setConnectionChangeCallback(nullptr, nullptr));
}

TEST_CASE("OBD shutdown fences callback, rejects new admission, and reinit resets queues", "[lifecycle]") {
    TEST_ASSERT_EQUAL(ESP_OK, obd.init());
    fixture.subscriberEntered = xSemaphoreCreateBinary();
    fixture.subscriberRelease = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(fixture.subscriberEntered); TEST_ASSERT_NOT_NULL(fixture.subscriberRelease);
    obd.connected_subscribe([](bool) {
        xSemaphoreGive(fixture.subscriberEntered);
        xSemaphoreTake(fixture.subscriberRelease, portMAX_DELAY);
    });
    OBD2LifecycleTestAccess::injectConnection(obd, true);
    TEST_ASSERT_EQUAL(pdTRUE, xSemaphoreTake(fixture.subscriberEntered, kWatchdog));
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(obdDeinitTaskEntry, "obd_stop", 4096, nullptr, 5, &fixture.obdDeinitTask));
    const TickType_t stoppingDeadline = xTaskGetTickCount() + kWatchdog;
    while (!OBD2LifecycleTestAccess::isStopping(obd) && ticksUntilDeadline(stoppingDeadline, xTaskGetTickCount()) != 0)
        taskYIELD();
    TEST_ASSERT_TRUE(OBD2LifecycleTestAccess::isStopping(obd));
    TEST_ASSERT_TRUE(OBD2LifecycleTestAccess::ownsRuntime(obd));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, obd.requestVIN());
    xSemaphoreGive(fixture.subscriberRelease);
    TEST_ASSERT_NOT_EQUAL(0, xEventGroupWaitBits(fixture.gates, OBD_DEINIT_DONE, pdFALSE, pdTRUE, kWatchdog) & OBD_DEINIT_DONE);
    TEST_ASSERT_TRUE(waitTaskSuspended(fixture.obdDeinitTask, kWatchdog));
    TEST_ASSERT_EQUAL(ESP_OK, fixture.obdDeinitResult.load());
    TEST_ASSERT_TRUE(OBD2LifecycleTestAccess::fullyUnwound(obd));
    TEST_ASSERT_EQUAL(ESP_OK, obd.init());
    TEST_ASSERT_EQUAL(ESP_OK, obd.deinit());
}

TEST_CASE("OBD init worker creation failures unwind and permit retry", "[lifecycle]") {
    for (unsigned position = 1; position <= 3; ++position) {
        fixture.createOrdinal = 0;
        fixture.failOrdinal = position;
        fixture.injectorTask = xTaskGetCurrentTaskHandle();
        fixture.injectFailure = true;
        const esp_err_t initResult = obd.init();
        const unsigned attempts = fixture.createOrdinal.load();
        fixture.injectFailure = false;
        fixture.injectorTask = nullptr;
        TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, initResult);
        TEST_ASSERT_EQUAL(position, attempts);
        TEST_ASSERT_TRUE(OBD2LifecycleTestAccess::fullyUnwound(obd));
        TEST_ASSERT_EQUAL(ESP_OK, obd.init());
        TEST_ASSERT_EQUAL(ESP_OK, obd.deinit());
        TEST_ASSERT_TRUE(OBD2LifecycleTestAccess::fullyUnwound(obd));
    }
}

TEST_CASE("OBD teardown/reinit resets discovery bookkeeping", "[lifecycle]") {
    TEST_ASSERT_EQUAL(ESP_OK, obd.init());
    OBD2LifecycleTestAccess::seedDiscovery(obd);
    TEST_ASSERT_EQUAL(ESP_OK, obd.deinit());
    TEST_ASSERT_TRUE(OBD2LifecycleTestAccess::fullyUnwound(obd));
    TEST_ASSERT_EQUAL(ESP_OK, obd.init());
    TEST_ASSERT_TRUE(OBD2LifecycleTestAccess::discoveryReset(obd));
    TEST_ASSERT_EQUAL(ESP_OK, obd.deinit());
}

TEST_CASE("OBD poll worker wakes from a far-future queue wait during shutdown", "[lifecycle]") {
    TEST_ASSERT_EQUAL(ESP_OK, obd.init());
    CanDriverLifecycleTestAccess::holdConnectedForPollWait(can, fixture);
    const TickType_t farFuture = pdMS_TO_TICKS(10000);
    TEST_ASSERT_TRUE(OBD2LifecycleTestAccess::queueFarFutureRequest(obd, farFuture));

    const TickType_t deadline = xTaskGetTickCount() + kWatchdog;
    bool parked = false;
    while (ticksUntilDeadline(deadline, xTaskGetTickCount()) != 0) {
        if (OBD2LifecycleTestAccess::pollTaskState(obd) == eBlocked &&
            OBD2LifecycleTestAccess::pollWaitTicks(obd) > pdMS_TO_TICKS(5000) &&
            OBD2LifecycleTestAccess::pollLocksAvailable(obd)) {
            parked = true;
            break;
        }
        taskYIELD();
    }
    TEST_ASSERT_TRUE_MESSAGE(parked, "poll worker did not reach far-future notification wait");

    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(obdDeinitTaskEntry, "obd_poll_stop", 4096, nullptr, 5, &fixture.obdDeinitTask));
    TEST_ASSERT_NOT_EQUAL(0, xEventGroupWaitBits(fixture.gates, OBD_DEINIT_DONE, pdFALSE, pdTRUE, kWatchdog) & OBD_DEINIT_DONE);
    TEST_ASSERT_TRUE(waitTaskSuspended(fixture.obdDeinitTask, kWatchdog));
    TEST_ASSERT_EQUAL(ESP_OK, fixture.obdDeinitResult.load());
    TEST_ASSERT_TRUE(OBD2LifecycleTestAccess::fullyUnwound(obd));
}

TEST_CASE("CAN simulator init/deinit cycles release tasks, queues and heap", "[lifecycle][p2.8]") {
    TEST_ASSERT_EQUAL(ESP_OK, can.deinit());
    can.setSimHooks(start_sim_task, stop_sim_task, sim_notify);
    constexpr unsigned kCycles = 20;
    size_t baseline = 0;
    for (unsigned cycle = 0; cycle < kCycles; ++cycle) {
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, can.init(simCanConfig()), "cycle init failed");
        TEST_ASSERT_TRUE(can.isInitialized());
        TEST_ASSERT_NOT_NULL(xDataSimTaskHandle);
        TEST_ASSERT_EQUAL(ESP_OK, can.deinit());
        TEST_ASSERT_EQUAL(CanDriver::STATE::NOT_INITIALIZED, can.getState());
        TEST_ASSERT_FALSE(CanDriverLifecycleTestAccess::hasResources(can));
        TEST_ASSERT_FALSE(CanDriverLifecycleTestAccess::queuesPresent(can));
        TEST_ASSERT_TRUE(CanDriverLifecycleTestAccess::tasksStopped(can));
        TEST_ASSERT_FALSE(CanDriverLifecycleTestAccess::nodePresent(can));
        TEST_ASSERT_NULL(xDataSimTaskHandle);
        // Self-deleted task stacks are reclaimed by the idle tasks.
        vTaskDelay(pdMS_TO_TICKS(20));
        if (cycle == 0) baseline = internalFreeHeap();
    }
    const size_t finalHeap = internalFreeHeap();
    ESP_LOGI("lifecycle_test", "CAN cycles: %u, heap after cycle 1 %u B, after cycle %u %u B, delta %d B",
             kCycles, static_cast<unsigned>(baseline), kCycles, static_cast<unsigned>(finalHeap),
             static_cast<int>(finalHeap) - static_cast<int>(baseline));
    TEST_ASSERT_TRUE_MESSAGE(finalHeap + 512 >= baseline, "heap lost across CAN init/deinit cycles");
    leaveCanInitialized();
}

TEST_CASE("CAN init rolls back when the simulator task cannot start", "[lifecycle][p2.8]") {
    TEST_ASSERT_EQUAL(ESP_OK, can.deinit());
    can.setSimHooks(start_sim_task, stop_sim_task, sim_notify);
    const size_t before = internalFreeHeap();

    fixture.createOrdinal = 0;
    fixture.failOrdinal = 1;  // start_sim_task is the first task bringUp() creates
    fixture.injectorTask = xTaskGetCurrentTaskHandle();
    fixture.injectFailure = true;
    const esp_err_t initResult = can.init(simCanConfig());
    const unsigned attempts = fixture.createOrdinal.load();
    fixture.injectFailure = false;
    fixture.injectorTask = nullptr;

    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, initResult);
    TEST_ASSERT_EQUAL(1u, attempts);
    TEST_ASSERT_EQUAL(CanDriver::STATE::NOT_INITIALIZED, can.getState());
    TEST_ASSERT_FALSE(CanDriverLifecycleTestAccess::hasResources(can));
    TEST_ASSERT_FALSE(CanDriverLifecycleTestAccess::queuesPresent(can));
    TEST_ASSERT_TRUE(CanDriverLifecycleTestAccess::tasksStopped(can));
    TEST_ASSERT_NULL(xDataSimTaskHandle);
    const size_t after = internalFreeHeap();
    TEST_ASSERT_TRUE_MESSAGE(after + 512 >= before, "failed bringUp leaked heap");

    // The failure is recoverable: a retry without injection comes up and tears down cleanly.
    TEST_ASSERT_EQUAL(ESP_OK, can.init(simCanConfig()));
    TEST_ASSERT_EQUAL(ESP_OK, can.deinit());
    TEST_ASSERT_FALSE(CanDriverLifecycleTestAccess::hasResources(can));
    leaveCanInitialized();
}

TEST_CASE("CAN teardown keeps resources when the simulator will not stop", "[lifecycle][p2.8]") {
    TEST_ASSERT_EQUAL(ESP_OK, can.deinit());
    can.setSimHooks(start_sim_task, stop_sim_task, sim_notify);
    TEST_ASSERT_EQUAL(ESP_OK, can.init(simCanConfig()));
    TEST_ASSERT_NOT_NULL(xDataSimTaskHandle);

    // Hold the simulator task so stop_sim_task() times out deterministically.
    vTaskSuspend(xDataSimTaskHandle);
    fixture.simTaskSuspended = true;
    TEST_ASSERT_TRUE(waitTaskSuspended(xDataSimTaskHandle, kWatchdog));

    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, can.deinit());
    // The callback-driven state is not asserted here: TWAI callbacks can still store NOT_CONNECTED/BUS_OFF while the
    // node stays enabled. Resource ownership is what the timeout contract keeps.
    TEST_ASSERT_TRUE(CanDriverLifecycleTestAccess::hasResources(can));
    TEST_ASSERT_TRUE(CanDriverLifecycleTestAccess::queuesPresent(can));
    TEST_ASSERT_TRUE(CanDriverLifecycleTestAccess::nodePresent(can));
    TEST_ASSERT_NOT_NULL(xDataSimTaskHandle);
    // The retained resources block a new init instead of being overwritten.
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, can.init(simCanConfig()));

    // Let the simulator stop; the next deinit finishes and a new cycle works.
    vTaskResume(xDataSimTaskHandle);
    fixture.simTaskSuspended = false;
    TEST_ASSERT_EQUAL(ESP_OK, can.deinit());
    TEST_ASSERT_EQUAL(CanDriver::STATE::NOT_INITIALIZED, can.getState());
    TEST_ASSERT_FALSE(CanDriverLifecycleTestAccess::hasResources(can));
    TEST_ASSERT_FALSE(CanDriverLifecycleTestAccess::queuesPresent(can));
    TEST_ASSERT_TRUE(CanDriverLifecycleTestAccess::tasksStopped(can));
    TEST_ASSERT_NULL(xDataSimTaskHandle);
    TEST_ASSERT_EQUAL(ESP_OK, can.init(simCanConfig()));
    TEST_ASSERT_EQUAL(ESP_OK, can.deinit());
    leaveCanInitialized();
}

TEST_CASE("CAN queue sizes seeded in NVS fall back to defaults and boot", "[lifecycle][p2.8]") {
    saveCanQueueKeys();
    struct BadQueues { uint32_t rx; uint32_t tx; };
    const BadQueues bad[] = {{0, 20}, {20, 0}, {0, 0}, {257, 20}, {20, 65}, {4096, 4096}};
    for (const BadQueues& queue : bad) {
        TEST_ASSERT_TRUE(writeRawU32("c_rx_q", queue.rx));
        TEST_ASSERT_TRUE(writeRawU32("c_tx_q", queue.tx));
        // Production starts from the compiled defaults, then overlays every usable stored key.
        CanDriver::Config loaded = Settings::defaultCanConfig();
        const esp_err_t err = settings.getCanConfig(loaded);
        TEST_ASSERT_TRUE(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(20, loaded.rx_queue_size, "stored invalid rx_queue_size was not defaulted");
        TEST_ASSERT_EQUAL_UINT32_MESSAGE(20, loaded.tx_queue_depth, "stored invalid tx_queue_depth was not defaulted");
    }

    // Bring the driver up from exactly what the boot loader would use; this is the path that used to abort on rx 0.
    TEST_ASSERT_EQUAL(ESP_OK, can.deinit());
    CanDriver::Config boot = Settings::defaultCanConfig();
    const esp_err_t err = settings.getCanConfig(boot);
    TEST_ASSERT_TRUE(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    if (boot.debug) {
        can.setSimHooks(start_sim_task, stop_sim_task, sim_notify);
    } else {
        can.setSimHooks(nullptr, nullptr, nullptr);
    }
    TEST_ASSERT_EQUAL(ESP_OK, can.init(boot));
    TEST_ASSERT_EQUAL_UINT32(20, can.getConfig().rx_queue_size);
    TEST_ASSERT_EQUAL_UINT32(20, can.getConfig().tx_queue_depth);
    TEST_ASSERT_EQUAL(ESP_OK, can.deinit());

    restoreCanQueueKeys();
    leaveCanInitialized();
}

extern "C" void app_main(void) {
    const esp_err_t nvsInit = Settings::getInstance().init();
    if (nvsInit != ESP_OK) {
        ESP_LOGE("lifecycle_test", "Cannot initialize NVS for the persisted-queue-size case: %s", esp_err_to_name(nvsInit));
        return;
    }
    fixture.gates = xEventGroupCreate();
    if (!fixture.gates) restartOnUnsafeCleanup("cannot allocate suite event group");
    UNITY_BEGIN();
    unity_run_menu();
    cleanupFixture();
    if (can.isInitialized() && can.deinit() != ESP_OK) restartOnUnsafeCleanup("CAN controller did not deinitialize");
    vEventGroupDelete(fixture.gates);
    fixture.gates = nullptr;
}
