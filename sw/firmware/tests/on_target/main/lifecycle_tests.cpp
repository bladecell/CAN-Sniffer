#include <atomic>

#include "can_driver.hpp"
#include "obd2.hpp"
#include "unity.h"
#include "unity_test_runner.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

namespace {
constexpr TickType_t kWatchdog = pdMS_TO_TICKS(5000);
CanDriver& can = CanDriver::getInstance();
OBD2& obd = OBD2::getInstance();
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

extern "C" void app_main(void) {
    fixture.gates = xEventGroupCreate();
    if (!fixture.gates) restartOnUnsafeCleanup("cannot allocate suite event group");
    UNITY_BEGIN();
    unity_run_menu();
    cleanupFixture();
    if (can.isInitialized() && can.deinit() != ESP_OK) restartOnUnsafeCleanup("CAN controller did not deinitialize");
    vEventGroupDelete(fixture.gates);
    fixture.gates = nullptr;
}
