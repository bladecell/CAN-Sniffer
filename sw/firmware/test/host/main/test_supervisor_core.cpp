#include "../../../main/include/supervisor_core.hpp"
#include "../../../main/include/supervisor_sleep.hpp"
#include "../../../main/include/manual_sleep_http.hpp"
#include "../../../main/include/production_components.hpp"
#include "unity.h"
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace application_components
{
// Host stand-ins for the hardware wiring; they only log the order the supervisor calls them in.
std::vector<int> productionCalls;
static esp_err_t mark(int id) { productionCalls.push_back(id); return ESP_OK; }
esp_err_t bootstrapSettings(void*) { return mark(0); }
esp_err_t setupPowerManagement(void*) { return mark(1); }
esp_err_t setupFlashFilesystem(void*) { return mark(2); }
esp_err_t setupSdCard(void*) { return mark(3); }
esp_err_t setupBattery(void*) { return mark(4); }
esp_err_t setupCan(void*) { return mark(5); }
esp_err_t setupObd(void*) { return mark(6); }
esp_err_t setupWifi(void*) { return mark(7); }
esp_err_t setupWebserver(void*) { return mark(8); }
esp_err_t stopSdCard(void*) { return mark(103); }
esp_err_t stopBattery(void*) { return mark(104); }
esp_err_t stopCan(void*) { return mark(105); }
esp_err_t stopObd(void*) { return mark(106); }
esp_err_t stopWifi(void*) { return mark(107); }
esp_err_t stopWebserver(void*) { return mark(108); }
esp_err_t releasePowerLock() { return ESP_OK; }
esp_err_t acquirePowerLock() { return ESP_OK; }
}

namespace
{
using namespace supervisor;
struct Fake;
Fake* current;

struct Fake
{
  std::array<Descriptor, 9> descriptors{};
  std::array<esp_err_t, 9> initErrors{};
  std::array<esp_err_t, 9> stopErrors{};
  std::array<int, 256> calls{};
  size_t count = 0;
  bool connected = false;
  bool alive = false;
  float volts = 12.0f;
  static thread_local int depth;
  std::mutex mutex;
  uint32_t failures = 0;
  uint32_t sleepS = 0;
  uint64_t lightUs = 0;
  uint32_t elapsedMs = 0;
  int sleeps = 0;
  int taskCalls = 0;
  int bootstrapCalls = 0;
  int policyReadsAtTaskCreation = 0;
  Lifecycle settingsAtTaskCreation{};
  power_settings::Config policyAtTaskCreation{};
  esp_err_t taskError = ESP_OK;
  bool registerDuringTask = false;
  esp_err_t taskRegistrationError = ESP_OK;
  esp_err_t prepareError = ESP_OK;
  esp_err_t finishError = ESP_OK;
  SleepTransaction* sleepPlatform = nullptr;
  int rollbackFailuresReported = 0;
  std::array<bool, 9> uncleanReported{};
  esp_err_t ledStopError = ESP_OK;
  esp_err_t standbyError = ESP_OK;
  bool stopDuringWebInit = false;
  bool stopDuringWebDeinit = false;
  power_settings::Config storedPower{};
  int policyReads = 0;
  Core core;

  template <int I> static esp_err_t init(void*)
  {
    TEST_ASSERT_EQUAL_INT(0, current->depth);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::STARTING), static_cast<int>(current->core.status(static_cast<Id>(I)).phase));
    if (I == 0)
    {
      ++current->bootstrapCalls;
      TEST_ASSERT_EQUAL_INT(0, current->taskCalls);
    }
    if (I == 8 && current->stopDuringWebInit) (void)current->core.requestStop();
    current->record(I);
    return current->initErrors[I];
  }
  template <int I> static esp_err_t stop(void*)
  {
    TEST_ASSERT_EQUAL_INT(0, current->depth);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::STOPPING), static_cast<int>(current->core.status(static_cast<Id>(I)).phase));
    if (I == 8 && current->stopDuringWebDeinit) (void)current->core.requestStop();
    current->record(100 + I);
    return current->stopErrors[I];
  }
  void record(int call)
  {
    TEST_ASSERT_LESS_THAN_UINT32(calls.size(), count);
    calls[count++] = call;
  }
  explicit Fake(bool registerAll = true) : core(operations())
  {
    current = this;
    const std::array<esp_err_t (*)(void*), 9> inits{init<0>, init<1>, init<2>, init<3>, init<4>, init<5>, init<6>, init<7>, init<8>};
    const std::array<esp_err_t (*)(void*), 9> stops{nullptr, nullptr, nullptr, stop<3>, stop<4>, stop<5>, stop<6>, stop<7>, stop<8>};
    const uint16_t shutdownPriorities[] = {0, 0, 0, 4, 0, 3, 1, 2, 0};
    for (size_t i = 0; i < descriptors.size(); ++i)
    {
      auto& d = descriptors[i];
      d.id = static_cast<Id>(i);
      d.name = Registry::legacyName(i);
      d.init = inits[i];
      d.deinit = stops[i];
      d.critical = i < 3;
      d.wakeCritical = i == 6;
      d.role = static_cast<Role>(i);
      d.participation = i < 3 ? Participation::RETAINED : i == 4 ? Participation::BATTERY :
        i == 5 ? Participation::PROBE : Participation::NORMAL;
      d.bootPriority = static_cast<uint16_t>(i);
      d.shutdownPriority = shutdownPriorities[i];
    }
    descriptors[6].dependencies = {{{Id::CAN}, {Id::SD_CARD}}};
    descriptors[6].dependencyCount = 2;
    descriptors[8].dependencies = {{{Id::OBD}, {Id::WIFI}, {Id::FLASH_FILESYSTEM}, {Id::SD_CARD}}};
    descriptors[8].dependencyCount = 4;
    if (registerAll)
      for (size_t i = 1; i < descriptors.size(); ++i) TEST_ASSERT_EQUAL_INT(ESP_OK, core.registerComponent(descriptors[i]));
  }
  Operations operations()
  {
    return {this,
      [](void* p) { auto& f = *static_cast<Fake*>(p); f.mutex.lock(); ++depth; },
      [](void* p) { --depth; static_cast<Fake*>(p)->mutex.unlock(); },
      [](void* p) {
        auto& f = *static_cast<Fake*>(p);
        f.policyReadsAtTaskCreation = f.policyReads;
        f.settingsAtTaskCreation = f.core.status(Id::SETTINGS);
        f.policyAtTaskCreation = f.core.powerPolicy();
        ++f.taskCalls;
        if (f.registerDuringTask) f.taskRegistrationError = f.core.registerComponent(f.descriptors[1]);
        return f.taskError;
      },
      [](void* p) { return static_cast<Fake*>(p)->connected; },
      [](void* p) { static_cast<Fake*>(p)->record(200); return static_cast<Fake*>(p)->alive; },
      [](void* p) { return static_cast<Fake*>(p)->volts; },
      [](void* p, uint64_t us) { auto& f = *static_cast<Fake*>(p); f.lightUs = us; f.record(201); return f.sleepPlatform ? f.sleepPlatform->prepare(us) : f.prepareError; },
      [](void* p) { auto& f = *static_cast<Fake*>(p); f.record(202); return f.sleepPlatform ? f.sleepPlatform->finish() : f.finishError; },
      [](void* p, uint32_t ms) { static_cast<Fake*>(p)->elapsedMs += ms; },
      [](void* p, bool standby) { auto& f = *static_cast<Fake*>(p); f.record(standby ? 203 : 204); return f.standbyError; },
      [](void* p) { static_cast<Fake*>(p)->record(205); return ESP_OK; },
      [](void* p) { auto& f = *static_cast<Fake*>(p); f.record(206); return f.ledStopError; },
      [](void*) {},
      [](void* p) { return static_cast<Fake*>(p)->failures; },
      [](void* p, uint32_t n) { static_cast<Fake*>(p)->failures = n; },
      [](void* p, uint32_t s) { auto& f = *static_cast<Fake*>(p); ++f.sleeps; f.sleepS = s; },
      [](void* p, const char* name, esp_err_t error, bool initializing) {
        if (std::strcmp(name, "Sleep PM rollback") == 0 && error != ESP_OK)
          ++static_cast<Fake*>(p)->rollbackFailuresReported;
        for (size_t i = 0; i < 9; ++i)
          if (!initializing && error != ESP_OK && std::strcmp(name, Registry::legacyName(i)) == 0)
            static_cast<Fake*>(p)->uncleanReported[i] = true;
      },
      [](void* p) { auto& f = *static_cast<Fake*>(p); ++f.policyReads; return f.storedPower; }};
  }
  esp_err_t start() { return core.init({this, init<0>}); }
  void boot()
  {
    TEST_ASSERT_EQUAL_INT(ESP_OK, start());
    core.step();
  }
  void expect(std::initializer_list<int> expected)
  {
    TEST_ASSERT_EQUAL_UINT32(expected.size(), count);
    size_t i = 0;
    for (int value : expected) TEST_ASSERT_EQUAL_INT(value, calls[i++]);
    count = 0;
  }
  void idle(unsigned ticks = 60)
  {
    for (unsigned i = 0; i < ticks; ++i) core.step();
  }
  void sleepReady()
  {
    boot();
    idle();
    core.step();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(State::SLEEPING), static_cast<int>(core.state()));
    count = 0;
  }
};
thread_local int Fake::depth = 0;

struct Extra
{
  Fake& fake;
  int number;
  esp_err_t initError = ESP_OK;
  esp_err_t stopError = ESP_OK;
  int initialized = 0;
  int stopped = 0;
  Descriptor descriptor(const char* name = "RECORDER")
  {
    Descriptor d;
    d.id = static_cast<Id>(number);
    d.name = name;
    d.context = this;
    d.init = [](void* p) { auto& e = *static_cast<Extra*>(p); ++e.initialized; e.fake.record(e.number); return e.initError; };
    d.deinit = [](void* p) { auto& e = *static_cast<Extra*>(p); ++e.stopped; e.fake.record(100 + e.number); return e.stopError; };
    return d;
  }
};

void test_boot_order_and_optional_failures()
{
  Fake f;
  f.initErrors[3] = ESP_FAIL;
  f.boot();
  f.expect({0, 1, 2, 3, 4, 5, 6, 7, 8});
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(ESP_FAIL, f.core.legacySetupSteps()[3].result.load());
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::PARTIAL), static_cast<int>(f.core.status(Id::SD_CARD).phase));
}

void test_settings_id_is_bootstrap_infrastructure_not_an_ordinary_registration()
{
  Fake f(false);
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, f.core.registerComponent(f.descriptors[0]));
  auto shadow = f.descriptors[0];
  shadow.id = static_cast<Id>(9);
  shadow.role = Role::NONE;
  shadow.participation = Participation::NORMAL;
  shadow.deinit = Fake::stop<3>;
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, f.core.registerComponent(shadow));
  TEST_ASSERT_EQUAL_UINT32(9, f.core.legacySetupSteps().size());
  TEST_ASSERT_EQUAL_STRING("SETTINGS", f.core.legacySetupSteps()[0].name);
  TEST_ASSERT_EQUAL_INT(ESP_FAIL, f.core.legacySetupSteps()[0].result.load());
  TEST_ASSERT_EQUAL_INT(0, f.taskCalls);
}

void test_missing_settings_reader_is_rejected_before_bootstrap_or_task_side_effects()
{
  Fake f;
  auto ops = f.operations();
  ops.powerSettings = nullptr;
  Core core(ops);
  for (size_t i = 1; i < f.descriptors.size(); ++i)
    TEST_ASSERT_EQUAL_INT(ESP_OK, core.registerComponent(f.descriptors[i]));
  const SettingsBootstrap bootstrap{&f, [](void* p) -> esp_err_t {
    ++static_cast<Fake*>(p)->bootstrapCalls;
    return ESP_FAIL;
  }};
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, core.init(bootstrap));
  TEST_ASSERT_EQUAL_INT(0, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(0, f.taskCalls);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::UNINITIALIZED), static_cast<int>(core.state()));
}

void test_failed_bootstrap_publishes_exact_error_and_only_starts_bounded_error_owner()
{
  Fake f;
  f.initErrors[0] = ESP_ERR_INVALID_STATE;
  f.storedPower = {15000, 10000, false};
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.start());
  f.expect({0});
  TEST_ASSERT_EQUAL_INT(1, f.taskCalls);
  TEST_ASSERT_EQUAL_INT(1, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(0, f.policyReads);
  TEST_ASSERT_EQUAL_INT(0, f.policyReadsAtTaskCreation);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::PARTIAL), static_cast<int>(f.settingsAtTaskCreation.phase));
  TEST_ASSERT_TRUE(f.policyAtTaskCreation.sleep_enabled);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.core.legacySetupSteps()[0].result.load());
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::PARTIAL), static_cast<int>(f.core.status(Id::SETTINGS).phase));
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.core.status(Id::SETTINGS).initError);
  for (size_t i = 1; i < 9; ++i)
  {
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, f.core.legacySetupSteps()[i].result.load());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::NEVER_STARTED), static_cast<int>(f.core.status(static_cast<Id>(i)).phase));
  }
  TEST_ASSERT_TRUE(f.core.powerPolicy().sleep_enabled);
  TEST_ASSERT_EQUAL_UINT32(13200, f.core.powerPolicy().stay_awake_voltage_mv);
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.start());
  f.idle(30);
  TEST_ASSERT_EQUAL_INT(0, f.sleeps);
  f.core.step();
  f.expect({203, 206});
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  TEST_ASSERT_EQUAL_UINT32(60, f.sleepS);
  TEST_ASSERT_EQUAL_UINT32(1, f.failures);
  TEST_ASSERT_EQUAL_INT(1, f.bootstrapCalls);
}

void test_missing_bootstrap_is_repairable_without_freezing_or_starting_anything()
{
  Fake f;
  SettingsBootstrap missing;
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, f.core.init(missing));
  TEST_ASSERT_EQUAL_INT(0, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(0, f.taskCalls);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::NEVER_STARTED), static_cast<int>(f.core.status(Id::SETTINGS).phase));
  Extra recorder{f, 9};
  auto d = recorder.descriptor();
  d.dependencies[0] = {Id::SETTINGS, true};
  d.dependencyCount = 1;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
  f.boot();
  TEST_ASSERT_EQUAL_INT(1, recorder.initialized);
  TEST_ASSERT_EQUAL_INT(1, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.start());
  TEST_ASSERT_EQUAL_INT(1, f.bootstrapCalls);
  TEST_ASSERT_TRUE(f.core.requestStop());
  f.core.step();
  f.alive = true;
  f.core.step();
  TEST_ASSERT_EQUAL_INT(2, recorder.initialized);
  TEST_ASSERT_EQUAL_INT(1, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(1, f.policyReads);
}

void test_bootstrap_cannot_reenter_or_register_and_pending_stop_waits_for_ordinary_startup()
{
  Fake f;
  const SettingsBootstrap bootstrap{&f, [](void* p) -> esp_err_t {
    auto& f = *static_cast<Fake*>(p);
    TEST_ASSERT_EQUAL_INT(0, Fake::depth);
    TEST_ASSERT_EQUAL_INT(0, f.taskCalls);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::STARTING), static_cast<int>(f.core.status(Id::SETTINGS).phase));
    ++f.bootstrapCalls;
    f.record(0);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.core.registerComponent(f.descriptors[1]));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.start());
    TEST_ASSERT_TRUE(f.core.requestStop());
    return ESP_OK;
  }};
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.init(bootstrap));
  TEST_ASSERT_EQUAL_INT(1, f.taskCalls);
  TEST_ASSERT_EQUAL_UINT32(1, f.count);
  f.core.step();
  f.expect({0, 1, 2, 3, 4, 5, 6, 7, 8});
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
  f.core.step();
  f.expect({108, 106, 107, 105, 103, 203, 206});
  f.alive = true;
  f.core.step();
  f.expect({201, 104, 4, 202, 204, 205, 5, 200, 3, 6, 7, 8});
  TEST_ASSERT_EQUAL_INT(1, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(1, f.policyReads);
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.legacySetupSteps()[0].result.load());
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::ACTIVE), static_cast<int>(f.core.status(Id::SETTINGS).phase));
}

void test_successful_bootstrap_policy_is_copied_before_failed_task_creation()
{
  Fake f;
  f.taskError = ESP_ERR_NO_MEM;
  f.storedPower = {14500, 12000, false};
  TEST_ASSERT_EQUAL_INT(ESP_ERR_NO_MEM, f.start());
  TEST_ASSERT_EQUAL_INT(1, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(1, f.policyReads);
  TEST_ASSERT_EQUAL_INT(1, f.policyReadsAtTaskCreation);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::ACTIVE), static_cast<int>(f.settingsAtTaskCreation.phase));
  TEST_ASSERT_EQUAL_UINT32(14500, f.policyAtTaskCreation.stay_awake_voltage_mv);
  TEST_ASSERT_FALSE(f.policyAtTaskCreation.sleep_enabled);
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  f.storedPower = {};
  TEST_ASSERT_EQUAL_UINT32(14500, f.core.powerPolicy().stay_awake_voltage_mv);
  TEST_ASSERT_EQUAL_UINT32(12000, f.core.powerPolicy().low_battery_voltage_mv);
  TEST_ASSERT_FALSE(f.core.powerPolicy().sleep_enabled);
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.start());
  TEST_ASSERT_EQUAL_INT(1, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(1, f.policyReads);
}

void test_bootstrap_status_and_admission_are_safe_while_caller_is_loading_settings()
{
  Fake f;
  struct Context
  {
    Fake* fake;
    std::atomic<bool> entered{false};
    std::atomic<bool> release{false};
  } context{&f};
  esp_err_t result = ESP_FAIL;
  std::thread caller([&]() {
    result = f.core.init({&context, [](void* p) -> esp_err_t {
      auto& c = *static_cast<Context*>(p);
      ++c.fake->bootstrapCalls;
      c.fake->record(0);
      c.entered.store(true);
      while (!c.release.load()) std::this_thread::yield();
      return ESP_OK;
    }});
  });
  while (!context.entered.load()) std::this_thread::yield();
  const auto state = f.core.state();
  const auto settings = f.core.status(Id::SETTINGS);
  const auto resultBeforeCompletion = f.core.legacySetupSteps()[0].result.load();
  const auto reservation = f.core.reserveSleepRequest();
  const auto reentry = f.start();
  const auto registration = f.core.registerComponent(f.descriptors[1]);
  context.release.store(true);
  caller.join();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::STARTING), static_cast<int>(state));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::STARTING), static_cast<int>(settings.phase));
  TEST_ASSERT_EQUAL_INT(ESP_FAIL, resultBeforeCompletion);
  TEST_ASSERT_EQUAL_UINT32(0, reservation);
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, reentry);
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, registration);
  TEST_ASSERT_EQUAL_INT(ESP_OK, result);
  TEST_ASSERT_EQUAL_INT(1, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(1, f.policyReads);
  TEST_ASSERT_EQUAL_INT(1, f.taskCalls);
  f.core.step();
  f.expect({0, 1, 2, 3, 4, 5, 6, 7, 8});
}

void test_missing_init_is_rejected_before_task_creation()
{
  Fake f;
  Descriptor invalid;
  invalid.id = static_cast<Id>(9);
  invalid.name = "BROKEN";
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, f.core.registerComponent(invalid));
  TEST_ASSERT_EQUAL_INT(0, f.taskCalls);
  TEST_ASSERT_EQUAL_UINT32(0, f.count);
}

void test_recorder_lifecycle_and_legacy_http_view_are_independent_of_registration_order()
{
  Fake f(false);
  Extra recorder{f, 1000};
  auto d = recorder.descriptor();
  d.dependencies = {{{Id::OBD, true}, {Id::SD_CARD, true}}};
  d.dependencyCount = 2;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
  for (size_t i = 9; i > 1; --i) TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(f.descriptors[i - 1]));
  f.boot();
  f.expect({0, 1, 2, 3, 4, 5, 6, 7, 8, 1000});
  const auto& legacy = f.core.legacySetupSteps();
  const char* names[] = {"SETTINGS", "POWER_MANAGEMENT", "FLASH_FILESYSTEM", "SD_CARD", "BATTERY", "CAN", "OBD", "WIFI", "WEBSERVER"};
  TEST_ASSERT_EQUAL_UINT32(9, legacy.size());
  for (size_t i = 0; i < 9; ++i)
  {
    TEST_ASSERT_EQUAL_STRING(names[i], legacy[i].name);
    TEST_ASSERT_EQUAL_INT(ESP_OK, legacy[i].result.load());
  }
  TEST_ASSERT_TRUE(f.core.requestStop());
  f.core.step();
  f.expect({108, 1100, 106, 107, 105, 103, 203, 206});
  f.alive = false;
  f.core.step();
  f.expect({201, 104, 4, 202, 204, 205, 5, 200, 105, 203, 206});
  TEST_ASSERT_EQUAL_INT(1, recorder.initialized);
  f.alive = true;
  f.core.step();
  f.expect({201, 104, 4, 202, 204, 205, 5, 200, 3, 6, 7, 8, 1000});
  TEST_ASSERT_EQUAL_INT(2, recorder.initialized);
  TEST_ASSERT_EQUAL_INT(1, f.policyReads);
  recorder.initError = ESP_FAIL;
  TEST_ASSERT_TRUE(f.core.requestStop());
  f.core.step();
  f.count = 0;
  f.core.step();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::PARTIAL), static_cast<int>(f.core.status(d.id).phase));
  for (size_t i = 0; i < 9; ++i) TEST_ASSERT_EQUAL_INT(ESP_OK, legacy[i].result.load());
}

void test_extra_provider_barrier_retains_all_later_dependencies_until_retry()
{
  Fake f;
  Extra recorder{f, 9};
  auto d = recorder.descriptor();
  d.dependencies = {{{Id::OBD, true}, {Id::SD_CARD, true}}};
  d.dependencyCount = 2;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
  f.boot();
  f.count = 0;
  recorder.stopError = ESP_ERR_TIMEOUT;
  TEST_ASSERT_TRUE(f.core.requestStop());
  f.core.step();
  f.expect({108, 109});
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::ACTIVE), static_cast<int>(f.core.status(Id::OBD).phase));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::ACTIVE), static_cast<int>(f.core.status(Id::SD_CARD).phase));
  recorder.stopError = ESP_OK;
  f.core.step();
  f.expect({109, 106, 107, 105, 103, 203, 206});
}

void test_required_success_blocks_callback_and_cleanup_but_order_only_still_runs()
{
  Fake f;
  Extra recorder{f, 9};
  auto d = recorder.descriptor();
  d.dependencies[0] = {Id::SD_CARD, true};
  d.dependencyCount = 1;
  d.wakeCritical = true;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
  f.initErrors[3] = ESP_FAIL;
  f.boot();
  TEST_ASSERT_EQUAL_INT(0, recorder.initialized);
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.core.status(d.id).initError);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::STOPPED), static_cast<int>(f.core.status(d.id).phase));
  f.expect({0, 1, 2, 3, 4, 5, 6, 7, 8});
  TEST_ASSERT_TRUE(f.core.requestStop());
  f.core.step();
  TEST_ASSERT_EQUAL_INT(0, recorder.stopped);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::SLEEPING), static_cast<int>(f.core.state()));
  f.alive = true;
  f.core.step();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::STOPPING), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.core.status(d.id).initError);
  TEST_ASSERT_EQUAL_INT(0, recorder.initialized);
  TEST_ASSERT_EQUAL_INT(ESP_FAIL, f.core.legacySetupSteps()[3].result.load());
}

void test_critical_blocked_extension_uses_existing_error_cleanup_without_invoking_its_callbacks()
{
  Fake f;
  Extra recorder{f, 9};
  auto d = recorder.descriptor();
  d.critical = true;
  d.dependencies[0] = {Id::SD_CARD, true};
  d.dependencyCount = 1;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
  f.initErrors[3] = ESP_FAIL;
  f.boot();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(0, recorder.initialized);
  f.idle(31);
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  TEST_ASSERT_EQUAL_INT(0, recorder.stopped);
}

void test_registry_is_frozen_before_task_publication_and_status_is_available_without_waiting()
{
  Fake f;
  f.registerDuringTask = true;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.start());
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.taskRegistrationError);
  TEST_ASSERT_EQUAL_INT(1, f.taskCalls);
  TEST_ASSERT_EQUAL_UINT32(1, f.count);
  TEST_ASSERT_EQUAL_INT(0, f.calls[0]);
  TEST_ASSERT_EQUAL_INT(1, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::ACTIVE), static_cast<int>(f.core.status(Id::SETTINGS).phase));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::NEVER_STARTED), static_cast<int>(f.core.status(Id::CAN).phase));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::STARTING), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(0, f.core.reserveSleepRequest());
  f.core.step();
  f.expect({0, 1, 2, 3, 4, 5, 6, 7, 8});
}

void test_registration_bounds_duplicates_copied_metadata_and_freeze()
{
  Fake f;
  Extra extra{f, 9};
  auto d = extra.descriptor();
  char name[32] = "RECORDER";
  d.name = name;
  d.dependencies[0] = {Id::OBD, true};
  d.dependencyCount = 1;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
  name[0] = 'X';
  d.dependencies[0].id = Id::INVALID;
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.core.registerComponent(extra.descriptor()));
  d = extra.descriptor();
  d.id = static_cast<Id>(10);
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.core.registerComponent(d));
  const char* names[] = {"N10", "N11", "N12", "N13", "N14", "N15", "1234567890123456789012345678901"};
  for (size_t i = 0; i < 7; ++i)
  {
    d.id = static_cast<Id>(10 + i);
    d.name = names[i];
    TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
  }
  d.id = static_cast<Id>(17);
  d.name = "FULL";
  TEST_ASSERT_EQUAL_INT(ESP_ERR_NO_MEM, f.core.registerComponent(d));
  f.boot();
  TEST_ASSERT_EQUAL_INT(1, f.taskCalls);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::ACTIVE), static_cast<int>(f.core.status(static_cast<Id>(9)).phase));
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.core.registerComponent(d));
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.start());
  TEST_ASSERT_EQUAL_INT(1, f.taskCalls);
}

void test_missing_roles_and_missing_provider_reject_without_side_effects_and_allow_completion()
{
  Fake f(false);
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.start());
  for (size_t i = 1; i < f.descriptors.size(); ++i) TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(f.descriptors[i]));
  Extra recorder{f, 9};
  auto d = recorder.descriptor();
  d.dependencies[0] = {static_cast<Id>(10), true};
  d.dependencyCount = 1;
  d.shutdownPriority = 500;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.start());
  TEST_ASSERT_EQUAL_INT(0, f.taskCalls);
  TEST_ASSERT_EQUAL_INT(0, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(0, f.policyReads);
  TEST_ASSERT_EQUAL_UINT32(0, f.count);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::UNINITIALIZED), static_cast<int>(f.core.state()));
  Extra provider{f, 10};
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(provider.descriptor("MODEM")));
  f.boot();
  f.expect({0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 9});
  TEST_ASSERT_TRUE(f.core.requestStop());
  f.core.step();
  f.expect({108, 106, 107, 105, 103, 109, 110, 203, 206});
}

void test_cycles_and_incompatible_sleep_dependencies_reject_before_task_creation()
{
  for (unsigned scenario = 0; scenario < 5; ++scenario)
  {
    Fake f(false);
    Extra recorder{f, 9};
    Extra modem{f, 10};
    auto d = recorder.descriptor();
    d.dependencies[0] = {scenario == 0 ? static_cast<Id>(10) : Id::OBD, false};
    d.dependencyCount = 1;
    if (scenario == 1)
    {
      d.participation = Participation::RETAINED;
      d.deinit = nullptr;
    }
    if (scenario >= 2)
    {
      auto& special = f.descriptors[scenario == 2 || scenario == 4 ? 4 : 5];
      special.dependencies[0] = {scenario == 4 ? Id::CAN : Id::OBD, false};
      special.dependencyCount = 1;
    }
    for (size_t i = 1; i < f.descriptors.size(); ++i) TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(f.descriptors[i]));
    TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
    if (scenario == 0)
    {
      auto m = modem.descriptor("MODEM");
      m.dependencies[0] = {static_cast<Id>(9), false};
      m.dependencyCount = 1;
      TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(m));
    }
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.start());
    TEST_ASSERT_EQUAL_INT(0, f.taskCalls);
    TEST_ASSERT_EQUAL_INT(0, f.bootstrapCalls);
    TEST_ASSERT_EQUAL_INT(0, f.policyReads);
    TEST_ASSERT_EQUAL_UINT32(0, f.count);
  }
}

void test_invalid_descriptor_fields_and_legacy_role_overrides_are_rejected()
{
  Fake f(false);
  Extra extra{f, 9};
  for (unsigned scenario = 0; scenario < 15; ++scenario)
  {
    auto d = extra.descriptor();
    switch (scenario)
    {
      case 0: d.name = ""; break;
      case 1: d.name = "12345678901234567890123456789012"; break;
      case 2: d.deinit = nullptr; break;
      case 3: d.participation = Participation::RETAINED; break;
      case 4: d.id = Id::SETTINGS; break;
      case 5: d.participation = Participation::PROBE; break;
      case 6: d.role = static_cast<Role>(255); break;
      case 7: d.dependencyCount = 5; break;
      case 8: d.dependencies[0] = {d.id, false}; d.dependencyCount = 1; break;
      case 9: d.dependencies = {{{Id::OBD}, {Id::OBD}}}; d.dependencyCount = 2; break;
      case 10: d.name = nullptr; break;
      case 11: d.init = nullptr; break;
      case 12: d.id = Id::INVALID; break;
      case 13: d.participation = static_cast<Participation>(255); break;
      case 14: d.dependencyCount = 1; break;
    }
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, f.core.registerComponent(d));
  }
  auto settings = f.descriptors[0];
  settings.critical = false;
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, f.core.registerComponent(settings));
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.start());
  TEST_ASSERT_EQUAL_INT(0, f.taskCalls);
  TEST_ASSERT_EQUAL_UINT32(0, f.count);
}

void test_retained_extension_runs_once_and_equal_priorities_use_stable_ids()
{
  Fake f;
  Extra modem{f, 11};
  Extra recorder{f, 10};
  Extra infrastructure{f, 9};
  auto retained = infrastructure.descriptor("INFRA");
  retained.participation = Participation::RETAINED;
  retained.deinit = nullptr;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(modem.descriptor("MODEM")));
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(recorder.descriptor()));
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(retained));
  f.boot();
  f.expect({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});
  TEST_ASSERT_TRUE(f.core.requestStop());
  f.core.step();
  f.expect({108, 110, 111, 106, 107, 105, 103, 203, 206});
  f.alive = true;
  f.core.step();
  f.expect({201, 104, 4, 202, 204, 205, 5, 200, 3, 6, 7, 8, 10, 11});
  TEST_ASSERT_EQUAL_INT(1, infrastructure.initialized);
  TEST_ASSERT_EQUAL_INT(0, infrastructure.stopped);
}

void test_concurrent_status_and_sleep_requests_do_not_mutate_frozen_registration()
{
  Fake f;
  f.boot();
  std::atomic<bool> done{false};
  std::atomic<bool> valid{true};
  std::atomic<bool> ready{false};
  std::thread reader([&]() {
    while (!done.load())
    {
      const auto s = f.core.status(Id::OBD);
      if (s.phase == Phase::NEVER_STARTED || s.initError != ESP_OK) valid.store(false);
      const uint32_t token = f.core.reserveSleepRequest();
      if (token) (void)f.core.finishSleepRequest(token, false);
      ready.store(true);
    }
  });
  while (!ready.load()) std::this_thread::yield();
  for (unsigned i = 0; i < 4; ++i)
  {
    (void)f.core.requestStop();
    f.core.step();
    f.alive = true;
    f.core.step();
  }
  done.store(true);
  reader.join();
  TEST_ASSERT_TRUE(valid.load());
  TEST_ASSERT_EQUAL_UINT32(9, f.core.legacySetupSteps().size());
}

void test_critical_failure_stops_startup_and_preserves_unattempted_results()
{
  Fake f;
  f.initErrors[2] = ESP_ERR_NO_MEM;
  f.boot();
  f.expect({0, 1, 2});
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(ESP_FAIL, f.core.legacySetupSteps()[3].result.load());
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::NEVER_STARTED), static_cast<int>(f.core.status(Id::CAN).phase));
  f.core.disconnected();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
}

void test_idle_threshold_reset_and_running_bus_guard()
{
  Fake f;
  f.boot();
  f.idle(59);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
  f.volts = 13.3f;
  f.core.step();
  f.volts = 13.2f;
  f.idle(59);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
  f.connected = true;
  f.core.step();
  f.idle(100);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::RUNNING), static_cast<int>(f.core.state()));
  f.connected = false;
  f.core.step();
  f.volts = NAN;
  f.idle();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::STOPPING), static_cast<int>(f.core.state()));
}

void test_shutdown_order_and_each_provider_barrier()
{
  const std::array<int, 5> order{8, 6, 7, 5, 3};
  for (size_t fail = 0; fail < order.size(); ++fail)
  {
    Fake f;
    f.boot();
    f.idle();
    f.count = 0;
    f.stopErrors[order[fail]] = ESP_ERR_TIMEOUT;
    f.core.step();
    TEST_ASSERT_EQUAL_UINT32(fail + 1, f.count);
    for (size_t i = 0; i <= fail; ++i) TEST_ASSERT_EQUAL_INT(100 + order[i], f.calls[i]);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(State::STOPPING), static_cast<int>(f.core.state()));
    const auto retained = f.core.status(static_cast<Id>(order[fail]));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::PARTIAL), static_cast<int>(retained.phase));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, retained.deinitError);
    f.count = 0;
    f.stopErrors[order[fail]] = ESP_OK;
    f.core.step();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(State::SLEEPING), static_cast<int>(f.core.state()));
    TEST_ASSERT_EQUAL_INT(100 + order[fail], f.calls[0]);
    TEST_ASSERT_EQUAL_INT(206, f.calls[f.count - 1]);
  }
}

void test_dead_bus_sleep_order_and_voltage_limits()
{
  for (float volts : {11.3f, 11.4f, 12.0f, NAN})
  {
    Fake f;
    f.sleepReady();
    f.volts = volts;
    f.core.disconnected();
    f.core.step();
    f.expect({201, 104, 4, 202, 204, 205, 5, 200, 105, 203, 206});
    TEST_ASSERT_EQUAL_UINT64(volts == 11.3f ? 60000000ULL : 10000000ULL, f.lightUs);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(State::SLEEPING), static_cast<int>(f.core.state()));
  }
}

void test_alive_wake_order_optional_sd_and_frozen_boot_status()
{
  Fake f;
  f.sleepReady();
  f.alive = true;
  f.initErrors[3] = ESP_ERR_NOT_FOUND;
  f.core.step();
  f.expect({201, 104, 4, 202, 204, 205, 5, 200, 3, 6, 7, 8});
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.legacySetupSteps()[3].result.load());
  TEST_ASSERT_EQUAL_INT(ESP_ERR_NOT_FOUND, f.core.status(Id::SD_CARD).initError);
}

void test_optional_wake_failure_and_required_success_dependents()
{
  for (bool strictDependent : {false, true})
  {
    Fake f(false);
    for (size_t i = 1; i < f.descriptors.size(); ++i)
      TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(f.descriptors[i]));
    Extra provider{f, 40};
    Extra dependent{f, 41};
    Extra unrelated{f, 42};
    auto p = provider.descriptor();
    auto d = dependent.descriptor("DEPENDENT");
    auto u = unrelated.descriptor("UNRELATED");
    d.dependencies = {{{p.id, true}}};
    d.dependencyCount = 1;
    d.wakeCritical = strictDependent;
    TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(p));
    TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
    TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(u));
    f.sleepReady();
    provider.initError = ESP_FAIL;
    f.alive = true;
    f.core.step();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(strictDependent ? State::STOPPING : State::NOT_CONNECTED),
                          static_cast<int>(f.core.state()));
    TEST_ASSERT_EQUAL_INT(2, provider.initialized);
    TEST_ASSERT_EQUAL_INT(1, dependent.initialized);
    TEST_ASSERT_EQUAL_INT(strictDependent ? 1 : 2, unrelated.initialized);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::STOPPED), static_cast<int>(f.core.status(d.id).phase));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.core.status(d.id).initError);
    TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.legacySetupSteps()[6].result.load());
    f.core.requestStop();
    f.core.step();
    TEST_ASSERT_EQUAL_INT(1, dependent.stopped);
    TEST_ASSERT_EQUAL_INT(2, provider.stopped);
  }
}

void test_wake_failures_do_not_start_later_components()
{
  for (int fail : {5, 6})
  {
    Fake f;
    f.sleepReady();
    f.alive = true;
    f.initErrors[fail] = ESP_FAIL;
    f.core.step();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(fail == 5 ? State::ERROR : State::STOPPING), static_cast<int>(f.core.state()));
    TEST_ASSERT_EQUAL_INT(fail, f.calls[f.count - 1]);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::PARTIAL), static_cast<int>(f.core.status(static_cast<Id>(fail)).phase));
  }
  // Wi-Fi and web are optional: a failure leaves them PARTIAL and wake carries on.
  for (int fail : {7, 8})
  {
    Fake f;
    f.sleepReady();
    f.alive = true;
    f.initErrors[fail] = ESP_FAIL;
    f.core.step();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::PARTIAL), static_cast<int>(f.core.status(static_cast<Id>(fail)).phase));
  }
}

void test_stop_is_a_coalesced_request_acknowledged_only_after_cleanup()
{
  Fake f;
  f.boot();
  f.count = 0;
  TEST_ASSERT_TRUE(f.core.requestStop());
  TEST_ASSERT_FALSE(f.core.requestStop());
  TEST_ASSERT_EQUAL_UINT32(0, f.count);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
  f.stopErrors[8] = ESP_ERR_TIMEOUT;
  f.core.step();
  f.expect({108});
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::PENDING), static_cast<int>(f.core.stopAcknowledgment()));
  f.core.disconnected();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::STOPPING), static_cast<int>(f.core.state()));
  f.stopErrors[8] = ESP_OK;
  f.core.step();
  f.expect({108, 106, 107, 105, 103, 203, 206});
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::SLEEP_READY), static_cast<int>(f.core.stopAcknowledgment()));
}

void test_missing_supervisor_task_recovers_on_start_caller_without_retrying_allocation()
{
  Fake f;
  f.taskError = ESP_ERR_NO_MEM;
  TEST_ASSERT_EQUAL_INT(ESP_ERR_NO_MEM, f.start());
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  TEST_ASSERT_EQUAL_UINT32(30000, f.elapsedMs);
  TEST_ASSERT_EQUAL_UINT32(60, f.sleepS);
  TEST_ASSERT_EQUAL_UINT32(1, f.failures);
  TEST_ASSERT_EQUAL_INT(1, f.taskCalls);
  f.expect({0, 203, 206});
  TEST_ASSERT_EQUAL_INT(1, f.bootstrapCalls);
  TEST_ASSERT_EQUAL_INT(1, f.policyReads);
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.legacySetupSteps()[0].result.load());
  TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, f.start());
  TEST_ASSERT_FALSE(f.core.requestStop());
  f.core.disconnected();
  f.idle(50);
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  TEST_ASSERT_EQUAL_INT(1, f.taskCalls);
}

void test_failure_history_survives_boot_until_successful_orderly_teardown()
{
  Fake f;
  f.failures = 7;
  f.boot();
  TEST_ASSERT_EQUAL_UINT32(0, f.failures);  // completed startup clears history
  f.failures = 7;
  f.core.requestStop();
  f.stopErrors[5] = ESP_ERR_TIMEOUT;
  f.core.step();
  TEST_ASSERT_EQUAL_UINT32(7, f.failures);
  f.stopErrors[5] = ESP_OK;
  f.ledStopError = ESP_ERR_TIMEOUT;
  f.core.step();
  TEST_ASSERT_EQUAL_UINT32(7, f.failures);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::STOPPING), static_cast<int>(f.core.state()));
  f.ledStopError = ESP_OK;
  f.core.step();
  TEST_ASSERT_EQUAL_UINT32(0, f.failures);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::SLEEPING), static_cast<int>(f.core.state()));
}

void test_failed_startup_keeps_history_and_error_cleanup_does_not_clear_it()
{
  Fake f;
  f.failures = 7;
  f.initErrors[1] = ESP_FAIL;
  f.boot();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_UINT32(7, f.failures);
  f.idle(31);
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  TEST_ASSERT_EQUAL_UINT32(8, f.failures);
}

void test_standby_error_blocks_wake_but_never_retries_error_sleep()
{
  Fake f;
  f.sleepReady();
  f.standbyError = ESP_FAIL;
  f.core.step();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
  f.expect({201, 104, 4, 202, 204});
  f.idle(31);
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  f.idle(40);
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
}

void test_error_standby_keeps_every_failed_provider_barrier()
{
  for (int fail : {8, 6, 7, 5, 3})
  {
    Fake f;
    f.boot();
    f.stopErrors[fail] = ESP_ERR_TIMEOUT;
    f.core.requestStop();
    f.idle(11);
    f.count = 0;
    f.idle(31);
    const int order[] = {8, 6, 7, 5, 3};
    size_t n = 0;
    while (order[n] != fail) ++n;
    // Completed consumers are skipped; the failed barrier alone retries.
    TEST_ASSERT_EQUAL_UINT32(2, f.count);
    TEST_ASSERT_EQUAL_INT(100 + order[n], f.calls[0]);
    TEST_ASSERT_EQUAL_INT(203, f.calls[1]);
    for (size_t i = n; i < 5; ++i) TEST_ASSERT_TRUE(f.uncleanReported[order[i]]);
    TEST_ASSERT_EQUAL_INT(1, f.sleeps);
    f.idle(3);
    TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  }
}

void test_bootstrap_error_standby_needs_no_component_init_and_led_failure_is_bounded()
{
  Fake f;
  f.initErrors[0] = ESP_FAIL;
  f.ledStopError = ESP_FAIL;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.start());
  f.expect({0});
  f.idle(31);
  f.expect({203, 206});
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  f.idle(40);
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
}

void test_shutdown_exhaustion_uses_backoff_once_and_keeps_provider_barriers()
{
  uint32_t history = 0;
  for (uint32_t seconds : {60U, 120U, 240U})
  {
    Fake f;
    f.boot();
    TEST_ASSERT_EQUAL_UINT32(0, f.failures);
    f.failures = history;  // episodes recorded after startup completed
    f.count = 0;
    f.stopErrors[8] = ESP_ERR_TIMEOUT;
    f.core.requestStop();
    f.idle(10);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(State::STOPPING), static_cast<int>(f.core.state()));
    f.core.step();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::RECOVERY), static_cast<int>(f.core.stopAcknowledgment()));
    f.idle(30);
    TEST_ASSERT_EQUAL_INT(0, f.sleeps);
    f.core.step();
    TEST_ASSERT_EQUAL_UINT32(seconds, f.sleepS);
    TEST_ASSERT_EQUAL_UINT32(history + 1, f.failures);
    TEST_ASSERT_EQUAL_UINT32(12, f.count);
    for (size_t i = 0; i + 1 < f.count; ++i) TEST_ASSERT_EQUAL_INT(108, f.calls[i]);
    TEST_ASSERT_EQUAL_INT(203, f.calls[f.count - 1]);
    f.idle(50);
    TEST_ASSERT_EQUAL_INT(1, f.sleeps);
    TEST_ASSERT_EQUAL_UINT32(history + 1, f.failures);
    history = f.failures;
  }
}

void test_recovery_counter_saturates_and_successful_error_cleanup_does_not_reset_it()
{
  for (uint32_t history : {15U, 16U, UINT32_MAX})
  {
    Fake f;
    Extra late{f, 9};
    late.initError = ESP_FAIL;
    auto d         = late.descriptor();
    d.critical     = true;
    TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
    f.failures = history;
    f.boot();
    f.count = 0;
    f.idle(31);
    f.expect({108, 109, 106, 107, 105, 103, 203, 206});
    TEST_ASSERT_EQUAL_UINT32(16, f.failures);
    TEST_ASSERT_EQUAL_UINT32(3600, f.sleepS);
  }
}

void test_stop_from_component_callbacks_never_waits_or_interrupts_lifecycle_work()
{
  Fake f;
  TEST_ASSERT_FALSE(f.core.requestStop());
  f.stopDuringWebInit = true;
  f.stopDuringWebDeinit = true;
  f.boot();
  f.expect({0, 1, 2, 3, 4, 5, 6, 7, 8});
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::PENDING), static_cast<int>(f.core.stopAcknowledgment()));
  f.core.step();
  f.expect({108, 106, 107, 105, 103, 203, 206});
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::SLEEP_READY), static_cast<int>(f.core.stopAcknowledgment()));
  TEST_ASSERT_FALSE(f.core.requestStop());
}

void test_sleep_platform_failures_take_error_without_starting_providers()
{
  for (bool prepare : {false, true})
  {
    Fake f;
    f.sleepReady();
    if (prepare) f.prepareError = ESP_FAIL;
    else f.finishError = ESP_FAIL;
    f.core.step();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
    if (prepare) f.expect({201});
    else f.expect({201, 104, 4, 202});
  }
}

void test_sleep_cycle_failure_reinitializes_led_once_before_error_blink()
{
  for (int stage = 0; stage < 3; ++stage)
  {
    Fake f;
    f.sleepReady();
    if (stage == 0) f.prepareError = ESP_FAIL;
    else if (stage == 1) f.finishError = ESP_FAIL;
    else f.stopErrors[4] = ESP_FAIL;
    f.core.step();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
    const size_t before = f.count;
    f.core.step();
    TEST_ASSERT_EQUAL_UINT32(before + 1, f.count);
    TEST_ASSERT_EQUAL_INT(205, f.calls[f.count - 1]);
    for (unsigned i = 0; i < 29; ++i) f.core.step();
    TEST_ASSERT_EQUAL_UINT32(before + 1, f.count);
    TEST_ASSERT_EQUAL_INT(0, f.sleeps);
    f.core.step();
    TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  }
}

void test_production_descriptor_table_is_admitted_with_expected_lifecycle()
{
  using application_components::productionCalls;
  productionCalls.clear();
  const auto table = production_components::descriptors();
  for (const auto& d : table)
  {
    TEST_ASSERT_EQUAL_STRING(Registry::legacyName(static_cast<size_t>(d.role)), d.name);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(d.id), static_cast<int>(d.role));
    TEST_ASSERT_NOT_NULL(d.init);
  }
  Fake f(false);
  for (const auto& d : table) TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.init({nullptr, application_components::bootstrapSettings}));
  f.core.step();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
  const std::vector<int> boot{0, 1, 2, 3, 4, 5, 6, 7, 8};
  TEST_ASSERT_TRUE(boot == productionCalls);
  productionCalls.clear();
  TEST_ASSERT_TRUE(f.core.requestStop());
  f.core.step();
  const std::vector<int> stop{108, 106, 107, 105, 103};
  TEST_ASSERT_TRUE(stop == productionCalls);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::SLEEPING), static_cast<int>(f.core.state()));
}

void test_manual_stop_does_not_carry_old_inactivity_into_the_next_wake()
{
  Fake f;
  f.boot();
  f.idle(59);
  f.core.requestStop();
  f.core.step();
  f.alive = true;
  f.core.step();
  for (unsigned i = 0; i < 59; ++i)
  {
    f.core.step();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
  }
  TEST_ASSERT_TRUE(f.core.requestStop());
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::PENDING), static_cast<int>(f.core.stopAcknowledgment()));
  f.core.step();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::SLEEP_READY), static_cast<int>(f.core.stopAcknowledgment()));
}

void test_sleep_transaction_owns_exactly_one_release_and_restore()
{
  struct Platform
  {
    int locks = 1;
    int timers = 0;
    int releases = 0;
    int acquires = 0;
    esp_err_t timerError = ESP_OK;
    esp_err_t releaseError = ESP_OK;
    esp_err_t acquireError = ESP_OK;
    SleepOperations operations()
    {
      return {this,
        [](void* p, uint64_t) { auto& f = *static_cast<Platform*>(p); ++f.timers; return f.timerError; },
        [](void* p) { auto& f = *static_cast<Platform*>(p); ++f.releases; if (f.releaseError == ESP_OK) --f.locks; return f.releaseError; },
        [](void* p) { auto& f = *static_cast<Platform*>(p); ++f.acquires; if (f.acquireError == ESP_OK) ++f.locks; return f.acquireError; }};
    }
  };
  Platform f;
  SleepTransaction transaction(f.operations());
  f.timerError = ESP_FAIL;
  TEST_ASSERT_EQUAL_INT(ESP_FAIL, transaction.prepare(10000));
  TEST_ASSERT_EQUAL_INT(0, f.releases);
  TEST_ASSERT_EQUAL_INT(ESP_OK, transaction.finish());
  TEST_ASSERT_EQUAL_INT(0, f.acquires);
  TEST_ASSERT_EQUAL_INT(1, f.locks);
  f.timerError = ESP_OK;
  f.releaseError = ESP_FAIL;
  TEST_ASSERT_EQUAL_INT(ESP_FAIL, transaction.prepare(10000));
  TEST_ASSERT_EQUAL_INT(ESP_OK, transaction.finish());
  TEST_ASSERT_EQUAL_INT(0, f.acquires);
  TEST_ASSERT_EQUAL_INT(1, f.locks);
  f.releaseError = ESP_OK;
  for (int i = 0; i < 3; ++i)
  {
    TEST_ASSERT_EQUAL_INT(ESP_OK, transaction.prepare(10000));
    TEST_ASSERT_EQUAL_INT(0, f.locks);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, transaction.prepare(10000));
    TEST_ASSERT_EQUAL_INT(ESP_OK, transaction.finish());
    TEST_ASSERT_EQUAL_INT(ESP_OK, transaction.finish());
    TEST_ASSERT_EQUAL_INT(i + 1, f.acquires);
    TEST_ASSERT_EQUAL_INT(1, f.locks);
  }
  TEST_ASSERT_EQUAL_INT(ESP_OK, transaction.prepare(10000));
  f.acquireError = ESP_FAIL;
  TEST_ASSERT_EQUAL_INT(ESP_FAIL, transaction.finish());
  TEST_ASSERT_EQUAL_INT(ESP_OK, transaction.finish());
  TEST_ASSERT_EQUAL_INT(4, f.acquires);
  TEST_ASSERT_EQUAL_INT(0, f.locks);
  // Exercise the production transaction through the actual Core failure exits.
  for (int failure = 0; failure < 6; ++failure)
  {
    Platform platform;
    SleepTransaction sleep(platform.operations());
    Fake coreFake;
    coreFake.sleepReady();
    coreFake.sleepPlatform = &sleep;
    if (failure == 0) platform.timerError = ESP_FAIL;
    if (failure == 1) platform.releaseError = ESP_FAIL;
    if (failure == 2 || failure == 3) coreFake.stopErrors[4] = ESP_ERR_TIMEOUT;
    if (failure == 3 || failure == 4) platform.acquireError = ESP_FAIL;
    coreFake.core.step();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(failure == 5 ? State::SLEEPING : State::ERROR),
                          static_cast<int>(coreFake.core.state()));
    TEST_ASSERT_EQUAL_INT(failure < 2 ? 0 : 1, platform.acquires);
    TEST_ASSERT_EQUAL_INT(failure == 0 ? 0 : 1, platform.releases);
    TEST_ASSERT_EQUAL_INT(failure == 3 || failure == 4 ? 0 : 1, platform.locks);
    TEST_ASSERT_EQUAL_INT(failure == 3 ? 1 : 0, coreFake.rollbackFailuresReported);
    if (failure < 5)
    {
      coreFake.idle(31);
      TEST_ASSERT_EQUAL_INT(1, coreFake.sleeps);
      TEST_ASSERT_EQUAL_INT(failure < 2 ? 0 : 1, platform.acquires);
    }
    else
    {
      coreFake.core.step();
      coreFake.core.step();
      TEST_ASSERT_EQUAL_INT(3, platform.acquires);
      TEST_ASSERT_EQUAL_INT(3, platform.releases);
      TEST_ASSERT_EQUAL_INT(1, platform.locks);
    }
  }
}

void test_battery_cleanup_failure_does_not_enter_light_sleep()
{
  Fake f;
  f.sleepReady();
  f.stopErrors[4] = ESP_ERR_TIMEOUT;
  f.core.step();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
  f.expect({201, 104, 202});
}

void test_configured_awake_boundary_and_boot_only_policy_snapshot()
{
  Fake f;
  f.storedPower = {14500, 12000, true};
  f.boot();
  TEST_ASSERT_EQUAL_UINT32(14500, f.core.powerPolicy().stay_awake_voltage_mv);
  f.storedPower = {13200, 11400, false};
  f.volts = 14.5f;
  f.idle(60);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::STOPPING), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(1, f.policyReads);
}

void test_disabled_automatic_sleep_includes_low_and_unknown_voltage_but_not_explicit_stop()
{
  Fake f;
  f.storedPower.sleep_enabled = false;
  f.boot();
  for (float volts : {0.0f, 11.0f, NAN})
  {
    f.volts = volts;
    for (unsigned i = 0; i < 120; ++i)
    {
      f.core.step();
      TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
    }
  }
  f.core.requestStop();
  f.core.step();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::SLEEPING), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::SLEEP_READY), static_cast<int>(f.core.stopAcknowledgment()));
  TEST_ASSERT_EQUAL_INT(1, f.policyReads);
}

void test_disabled_sleep_never_blocks_error_recovery()
{
  Fake f;
  Extra late{f, 9};
  late.initError = ESP_FAIL;
  auto d         = late.descriptor();
  d.critical     = true;
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.core.registerComponent(d));
  f.storedPower.sleep_enabled = false;
  f.boot();
  TEST_ASSERT_FALSE(f.core.powerPolicy().sleep_enabled);
  f.idle(31);
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  TEST_ASSERT_EQUAL_UINT32(60, f.sleepS);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
}

void test_configured_thresholds_use_exact_voltage_boundaries_and_nan_defaults()
{
  for (float volts : {14.499f, 14.5f, 14.501f, NAN})
  {
    Fake f;
    f.storedPower = {14500, 12000, true};
    f.boot();
    f.volts = volts;
    f.idle(60);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(volts > 14.5f ? State::NOT_CONNECTED : State::STOPPING), static_cast<int>(f.core.state()));
  }
  for (float volts : {11.999f, 12.0f, 12.001f, NAN})
  {
    Fake f;
    f.storedPower = {14500, 12000, true};
    f.sleepReady();
    f.volts = volts;
    f.core.step();
    TEST_ASSERT_EQUAL_UINT64(volts == 11.999f ? 60000000ULL : 10000000ULL, f.lightUs);
    TEST_ASSERT_EQUAL_INT(1, f.policyReads);
  }
}

void test_policy_defaults_survive_failed_bootstrap_and_invalid_boot_config()
{
  Fake f;
  f.storedPower = {15000, 10000, false};
  f.initErrors[0] = ESP_FAIL;
  f.taskError = ESP_ERR_NO_MEM;
  TEST_ASSERT_EQUAL_INT(ESP_ERR_NO_MEM, f.start());
  TEST_ASSERT_EQUAL_INT(0, f.policyReads);
  TEST_ASSERT_TRUE(f.core.powerPolicy().sleep_enabled);
  TEST_ASSERT_EQUAL_UINT32(13200, f.core.powerPolicy().stay_awake_voltage_mv);
  TEST_ASSERT_EQUAL_UINT32(11400, f.core.powerPolicy().low_battery_voltage_mv);
  Fake invalid;
  invalid.storedPower = {10000, 12000, false};
  invalid.boot();
  TEST_ASSERT_TRUE(invalid.core.powerPolicy().sleep_enabled);
  TEST_ASSERT_EQUAL_UINT32(13200, invalid.core.powerPolicy().stay_awake_voltage_mv);
}

void test_http_sleep_reservation_defers_stop_and_cancel_releases_without_stale_publication()
{
  Fake f;
  f.storedPower.sleep_enabled = false;
  f.boot();
  f.count = 0;
  const uint32_t first = f.core.reserveSleepRequest();
  TEST_ASSERT_NOT_EQUAL_UINT32(0, first);
  TEST_ASSERT_EQUAL_UINT32(0, f.core.reserveSleepRequest());
  f.core.step();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::NONE), static_cast<int>(f.core.stopAcknowledgment()));
  TEST_ASSERT_FALSE(f.core.finishSleepRequest(first, false));
  TEST_ASSERT_EQUAL_UINT32(0, f.count);
  const uint32_t second = f.core.reserveSleepRequest();
  TEST_ASSERT_TRUE(second > first);
  TEST_ASSERT_FALSE(f.core.finishSleepRequest(first, true));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::NONE), static_cast<int>(f.core.stopAcknowledgment()));
  TEST_ASSERT_TRUE(f.core.finishSleepRequest(second, true));
  TEST_ASSERT_FALSE(f.core.finishSleepRequest(second, true));
  TEST_ASSERT_EQUAL_UINT32(0, f.core.reserveSleepRequest());
  f.core.step();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::SLEEPING), static_cast<int>(f.core.state()));
}

void test_http_sleep_old_completion_cannot_trigger_another_wake_cycle()
{
  Fake f;
  f.storedPower.sleep_enabled = false;
  f.boot();
  const uint32_t old = f.core.reserveSleepRequest();
  TEST_ASSERT_NOT_EQUAL_UINT32(0, old);
  f.core.requestStop();
  f.core.step();
  f.alive = true;
  f.core.step();
  TEST_ASSERT_FALSE(f.core.finishSleepRequest(old, true));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::SLEEP_READY), static_cast<int>(f.core.stopAcknowledgment()));
  const uint32_t next = f.core.reserveSleepRequest();
  TEST_ASSERT_TRUE(next > old);
  TEST_ASSERT_FALSE(f.core.finishSleepRequest(old, true));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::SLEEP_READY), static_cast<int>(f.core.stopAcknowledgment()));
  TEST_ASSERT_TRUE(f.core.finishSleepRequest(next, true));
  f.core.step();
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::SLEEPING), static_cast<int>(f.core.state()));
  TEST_ASSERT_FALSE(f.core.finishSleepRequest(next, true));
}

void test_http_sleep_reservation_never_overrides_error_or_blocks_shutdown()
{
  Fake f;
  TEST_ASSERT_EQUAL_UINT32(0, f.core.reserveSleepRequest());
  TEST_ASSERT_EQUAL_INT(ESP_OK, f.start());
  TEST_ASSERT_EQUAL_UINT32(0, f.core.reserveSleepRequest());
  f.core.step();
  f.connected = true;
  f.core.step();
  const uint32_t token = f.core.reserveSleepRequest();
  TEST_ASSERT_NOT_EQUAL_UINT32(0, token);
  f.stopErrors[8] = ESP_ERR_TIMEOUT;
  f.core.requestStop();
  f.idle(11);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_UINT32(0, f.core.reserveSleepRequest());
  TEST_ASSERT_FALSE(f.core.finishSleepRequest(token, true));
  f.idle(31);
  TEST_ASSERT_EQUAL_INT(1, f.sleeps);
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::ERROR), static_cast<int>(f.core.state()));
}

void test_http_dispatch_with_real_core_sends_before_stop_and_keeps_provider_barriers()
{
  Fake f;
  f.storedPower.sleep_enabled = false;
  f.boot();
  f.count = 0;
  for (esp_err_t sendError : {ESP_FAIL, ESP_OK})
  {
    TEST_ASSERT_EQUAL_INT(sendError, manual_sleep_http::dispatch("/api/v1/system/sleep", 0, false,
      [&]() { return f.core.reserveSleepRequest(); },
      [&](int status)
      {
        TEST_ASSERT_EQUAL_INT(202, status);
        TEST_ASSERT_EQUAL_INT(static_cast<int>(StopAck::NONE), static_cast<int>(f.core.stopAcknowledgment()));
        f.core.step();
        TEST_ASSERT_EQUAL_INT(static_cast<int>(State::NOT_CONNECTED), static_cast<int>(f.core.state()));
        TEST_ASSERT_EQUAL_UINT32(0, f.count);
        return sendError;
      },
      [&](uint32_t token, bool sent) { (void)f.core.finishSleepRequest(token, sent); }));
  }
  f.stopErrors[8] = ESP_ERR_TIMEOUT;
  f.core.step();
  f.expect({108});
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::STOPPING), static_cast<int>(f.core.state()));
  TEST_ASSERT_EQUAL_INT(static_cast<int>(Phase::ACTIVE), static_cast<int>(f.core.status(Id::CAN).phase));
  f.stopErrors[8] = ESP_OK;
  f.core.step();
  f.expect({108, 106, 107, 105, 103, 203, 206});
  TEST_ASSERT_EQUAL_INT(static_cast<int>(State::SLEEPING), static_cast<int>(f.core.state()));
}

void test_state_numbers_remain_compatible()
{
  TEST_ASSERT_EQUAL_INT(0, static_cast<int>(supervisor::State::UNINITIALIZED));
  TEST_ASSERT_EQUAL_INT(3, static_cast<int>(supervisor::State::RUNNING));
  TEST_ASSERT_EQUAL_INT(6, static_cast<int>(supervisor::State::ERROR));
}
}

extern "C" void run_supervisor_core_tests()
{
  RUN_TEST(test_state_numbers_remain_compatible);
  RUN_TEST(test_boot_order_and_optional_failures);
  RUN_TEST(test_settings_id_is_bootstrap_infrastructure_not_an_ordinary_registration);
  RUN_TEST(test_missing_settings_reader_is_rejected_before_bootstrap_or_task_side_effects);
  RUN_TEST(test_failed_bootstrap_publishes_exact_error_and_only_starts_bounded_error_owner);
  RUN_TEST(test_missing_bootstrap_is_repairable_without_freezing_or_starting_anything);
  RUN_TEST(test_bootstrap_cannot_reenter_or_register_and_pending_stop_waits_for_ordinary_startup);
  RUN_TEST(test_successful_bootstrap_policy_is_copied_before_failed_task_creation);
  RUN_TEST(test_bootstrap_status_and_admission_are_safe_while_caller_is_loading_settings);
  RUN_TEST(test_missing_init_is_rejected_before_task_creation);
  RUN_TEST(test_recorder_lifecycle_and_legacy_http_view_are_independent_of_registration_order);
  RUN_TEST(test_extra_provider_barrier_retains_all_later_dependencies_until_retry);
  RUN_TEST(test_required_success_blocks_callback_and_cleanup_but_order_only_still_runs);
  RUN_TEST(test_critical_blocked_extension_uses_existing_error_cleanup_without_invoking_its_callbacks);
  RUN_TEST(test_registry_is_frozen_before_task_publication_and_status_is_available_without_waiting);
  RUN_TEST(test_registration_bounds_duplicates_copied_metadata_and_freeze);
  RUN_TEST(test_missing_roles_and_missing_provider_reject_without_side_effects_and_allow_completion);
  RUN_TEST(test_cycles_and_incompatible_sleep_dependencies_reject_before_task_creation);
  RUN_TEST(test_invalid_descriptor_fields_and_legacy_role_overrides_are_rejected);
  RUN_TEST(test_retained_extension_runs_once_and_equal_priorities_use_stable_ids);
  RUN_TEST(test_concurrent_status_and_sleep_requests_do_not_mutate_frozen_registration);
  RUN_TEST(test_critical_failure_stops_startup_and_preserves_unattempted_results);
  RUN_TEST(test_idle_threshold_reset_and_running_bus_guard);
  RUN_TEST(test_shutdown_order_and_each_provider_barrier);
  RUN_TEST(test_dead_bus_sleep_order_and_voltage_limits);
  RUN_TEST(test_alive_wake_order_optional_sd_and_frozen_boot_status);
  RUN_TEST(test_optional_wake_failure_and_required_success_dependents);
  RUN_TEST(test_wake_failures_do_not_start_later_components);
  RUN_TEST(test_stop_is_a_coalesced_request_acknowledged_only_after_cleanup);
  RUN_TEST(test_missing_supervisor_task_recovers_on_start_caller_without_retrying_allocation);
  RUN_TEST(test_failure_history_survives_boot_until_successful_orderly_teardown);
  RUN_TEST(test_failed_startup_keeps_history_and_error_cleanup_does_not_clear_it);
  RUN_TEST(test_standby_error_blocks_wake_but_never_retries_error_sleep);
  RUN_TEST(test_error_standby_keeps_every_failed_provider_barrier);
  RUN_TEST(test_bootstrap_error_standby_needs_no_component_init_and_led_failure_is_bounded);
  RUN_TEST(test_shutdown_exhaustion_uses_backoff_once_and_keeps_provider_barriers);
  RUN_TEST(test_recovery_counter_saturates_and_successful_error_cleanup_does_not_reset_it);
  RUN_TEST(test_stop_from_component_callbacks_never_waits_or_interrupts_lifecycle_work);
  RUN_TEST(test_sleep_platform_failures_take_error_without_starting_providers);
  RUN_TEST(test_sleep_cycle_failure_reinitializes_led_once_before_error_blink);
  RUN_TEST(test_production_descriptor_table_is_admitted_with_expected_lifecycle);
  RUN_TEST(test_manual_stop_does_not_carry_old_inactivity_into_the_next_wake);
  RUN_TEST(test_sleep_transaction_owns_exactly_one_release_and_restore);
  RUN_TEST(test_battery_cleanup_failure_does_not_enter_light_sleep);
  RUN_TEST(test_configured_awake_boundary_and_boot_only_policy_snapshot);
  RUN_TEST(test_disabled_automatic_sleep_includes_low_and_unknown_voltage_but_not_explicit_stop);
  RUN_TEST(test_disabled_sleep_never_blocks_error_recovery);
  RUN_TEST(test_configured_thresholds_use_exact_voltage_boundaries_and_nan_defaults);
  RUN_TEST(test_policy_defaults_survive_failed_bootstrap_and_invalid_boot_config);
  RUN_TEST(test_http_sleep_reservation_defers_stop_and_cancel_releases_without_stale_publication);
  RUN_TEST(test_http_sleep_old_completion_cannot_trigger_another_wake_cycle);
  RUN_TEST(test_http_sleep_reservation_never_overrides_error_or_blocks_shutdown);
  RUN_TEST(test_http_dispatch_with_real_core_sends_before_stop_and_keeps_provider_barriers);
}
