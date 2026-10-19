#include "supervisor_core.hpp"

#include "error_policy.hpp"

namespace supervisor
{
esp_err_t Core::registerComponent(const Descriptor& descriptor)
{
    ops_.lock(ops_.context);
    const esp_err_t error = admitting_ ? ESP_ERR_INVALID_STATE : registry_.add(descriptor);
    ops_.unlock(ops_.context);
    return error;
}

esp_err_t Core::init(const SettingsBootstrap& bootstrap)
{
    ops_.lock(ops_.context);
    if (admitting_ || state() != State::UNINITIALIZED)
    {
        ops_.unlock(ops_.context);
        return ESP_ERR_INVALID_STATE;
    }
    if (!bootstrap.initialize || !ops_.powerSettings)
    {
        ops_.unlock(ops_.context);
        return ESP_ERR_INVALID_ARG;
    }
    admitting_ = true;
    ops_.unlock(ops_.context);
    // Registration is closed while planning. Status readers only inspect the
    // unchanged records; bounded graph traversal never masks ESP interrupts.
    const esp_err_t admitted = registry_.freeze();
    ops_.lock(ops_.context);
    if (admitted == ESP_OK)
    {
        state_.store(State::STARTING);
        settings_.phase = Phase::STARTING;
    }
    else
    {
        admitting_ = false;
    }
    ops_.unlock(ops_.context);
    if (admitted != ESP_OK)
    {
        return admitted;
    }

    // Infrastructure precedes task publication, never registry validation. NVS
    // remains application-owned; failed bootstrap starts only the ERROR owner.
    const esp_err_t settingsError = bootstrap.initialize(bootstrap.context);
    power_settings::Config policy{};
    if (settingsError == ESP_OK)
    {
        const auto loaded = ops_.powerSettings(ops_.context);
        if (power_settings::check(loaded) == nullptr)
        {
            policy = loaded;
        }
    }
    ops_.lock(ops_.context);
    settings_ = {settingsError == ESP_OK ? Phase::ACTIVE : Phase::PARTIAL, settingsError, ESP_OK};
    legacy_[0].result.store(settingsError);
    policy_ = policy;
    ops_.unlock(ops_.context);
    ops_.report(ops_.context, Registry::legacyName(0), settingsError, true);
    if (settingsError != ESP_OK)
    {
        enterError();
    }
    const esp_err_t error = ops_.createTask(ops_.context);
    if (error != ESP_OK)
    {
        enterError();
        // No task exists to run ERROR. Use its caller's stack and the same recovery path;
        // never allocate a replacement task or leave ERROR without an owner.
        while (!recoveryIssued_)
        {
            recover();
            if (!recoveryIssued_)
            {
                ops_.delayMs(ops_.context, kLoopMs);
            }
        }
    }
    return error;
}

Lifecycle Core::status(Id id) const
{
    ops_.lock(ops_.context);
    const Component* c = registry_.find(id);
    const Lifecycle result = id == Id::SETTINGS ? settings_ : c ? c->lifecycle : Lifecycle{};
    ops_.unlock(ops_.context);
    return result;
}

power_settings::Config Core::powerPolicy() const
{
    ops_.lock(ops_.context);
    const auto result = policy_;
    ops_.unlock(ops_.context);
    return result;
}

uint32_t Core::reserveSleepRequest()
{
    ops_.lock(ops_.context);
    const State s     = state();
    uint32_t    token = 0;
    if ((s == State::RUNNING || s == State::NOT_CONNECTED) && stopAck_.load() != StopAck::PENDING &&
        sleepReservation_ == 0 && nextSleepReservation_ < UINT32_MAX)
    {
        // Do not wrap/reuse a token in this boot: late completions must never alias a new request.
        token             = ++nextSleepReservation_;
        sleepReservation_ = token;
    }
    ops_.unlock(ops_.context);
    return token;
}

bool Core::finishSleepRequest(uint32_t token, bool responseSent)
{
    ops_.lock(ops_.context);
    bool published = false;
    if (token != 0 && token == sleepReservation_)
    {
        sleepReservation_ = 0;
        const State s     = state();
        if (responseSent && (s == State::RUNNING || s == State::NOT_CONNECTED))
        {
            stopAck_.store(StopAck::PENDING);
            published = true;
        }
    }
    ops_.unlock(ops_.context);
    return published;
}

void Core::invalidateSleepReservation()
{
    ops_.lock(ops_.context);
    sleepReservation_ = 0;
    ops_.unlock(ops_.context);
}

esp_err_t Core::initialize(Id id, bool boot)
{
    Component& c = *registry_.find(id);
    const auto& d = c.descriptor;
    bool blocked = false;
    for (size_t i = 0; i < d.dependencyCount; ++i)
    {
        if (d.dependencies[i].requireSuccess && status(d.dependencies[i].id).phase != Phase::ACTIVE)
        {
            blocked = true;
        }
    }
    ops_.lock(ops_.context);
    c.lifecycle.phase = Phase::STARTING;
    ops_.unlock(ops_.context);
    const esp_err_t error = blocked ? ESP_ERR_INVALID_STATE : d.init(d.context);
    ops_.lock(ops_.context);
    c.lifecycle = {blocked ? Phase::STOPPED : error == ESP_OK ? Phase::ACTIVE : Phase::PARTIAL, error, ESP_OK};
    ops_.unlock(ops_.context);
    if (boot && d.role != Role::NONE)
    {
        legacy_[static_cast<size_t>(d.role)].result.store(error);
    }
    ops_.report(ops_.context, c.name.data(), error, true);
    return error;
}

esp_err_t Core::deinit(Id id)
{
    Component&  c     = *registry_.find(id);
    const Phase phase = status(id).phase;
    if (!c.descriptor.deinit || phase == Phase::NEVER_STARTED || phase == Phase::STOPPED)
    {
        return ESP_OK;
    }
    ops_.lock(ops_.context);
    c.lifecycle.phase = Phase::STOPPING;
    ops_.unlock(ops_.context);
    const esp_err_t error = c.descriptor.deinit(c.descriptor.context);
    ops_.lock(ops_.context);
    c.lifecycle.phase       = error == ESP_OK ? Phase::STOPPED : Phase::PARTIAL;
    c.lifecycle.deinitError = error;
    ops_.unlock(ops_.context);
    ops_.report(ops_.context, c.name.data(), error, false);
    return error;
}

bool Core::shutdown()
{
    for (size_t i = 0; i < registry_.shutdownCount(); ++i)
    {
        const Id id = registry_.at(registry_.shutdownSlot(i)).descriptor.id;
        if (deinit(id) != ESP_OK)
        {
            return false;
        }
    }
    ops_.standby(ops_.context, true);
    return ops_.ledDeinit(ops_.context) == ESP_OK;
}

void Core::disconnected()
{
    State expected = State::RUNNING;
    state_.compare_exchange_strong(expected, State::NOT_CONNECTED);
}

bool Core::requestStop()
{
    const State s = state();
    if (s == State::UNINITIALIZED || s == State::ERROR || s == State::SLEEPING)
    {
        return false;
    }
    StopAck expected = stopAck_.load();
    while (expected != StopAck::PENDING)
    {
        if (stopAck_.compare_exchange_weak(expected, StopAck::PENDING))
        {
            return true;
        }
    }
    return false;
}

void Core::enterError()
{
    state_.store(State::ERROR);
    invalidateSleepReservation();
    errorTicks_ = 0;
    if (stopAck_.load() == StopAck::PENDING)
    {
        stopAck_.store(StopAck::RECOVERY);
    }
}

void Core::recover()
{
    if (recoveryIssued_)
    {
        return;
    }
    if (errorTicks_ < error_policy::kBlinkS)
    {
        ++errorTicks_;
        ops_.blink(ops_.context);
        return;
    }
    const uint32_t attempt = ops_.readFailures(ops_.context);
    ops_.writeFailures(ops_.context, error_policy::nextCount(attempt));
    (void)shutdown();
    recoveryIssued_ = true;
    ops_.deepSleep(ops_.context, error_policy::backoffS(attempt));
}

void Core::sleepCycle()
{
    const float    voltage    = ops_.voltage(ops_.context);
    const float    lowVoltage = static_cast<float>(policy_.low_battery_voltage_mv) / 1000.0f;
    const uint64_t us         = voltage < lowVoltage ? 60000000ULL : 10000000ULL;
    if (ops_.prepareSleep(ops_.context, us) != ESP_OK)
    {
        enterError();
        return;
    }
    if (deinit(registry_.role(Role::BATTERY)) != ESP_OK)
    {
        enterError();
        return;
    }
    ops_.delayMs(ops_.context, static_cast<uint32_t>(us / 1000));
    (void)initialize(registry_.role(Role::BATTERY));
    if (ops_.finishSleep(ops_.context) != ESP_OK)
    {
        enterError();
        return;
    }
    ops_.standby(ops_.context, false);
    (void)ops_.ledInit(ops_.context);
    if (initialize(registry_.role(Role::CAN)) != ESP_OK)
    {
        enterError();
        return;
    }
    if (ops_.probeBus(ops_.context))
    {
        for (size_t i = 0; i < registry_.size(); ++i)
        {
            const auto& d = registry_.at(registry_.bootSlot(i)).descriptor;
            if (d.participation != Participation::NORMAL)
            {
                continue;
            }
            const esp_err_t error = initialize(d.id);
            // The legacy SD role still tolerates a missing/unusable card on wake.
            if (error != ESP_OK && d.role != Role::SD_CARD)
            {
                state_.store(State::STOPPING);
                return;
            }
        }
        state_.store(State::NOT_CONNECTED);
    }
    else
    {
        if (deinit(registry_.role(Role::WEBSERVER)) != ESP_OK || deinit(registry_.role(Role::CAN)) != ESP_OK)
        {
            state_.store(State::STOPPING);
            return;
        }
        ops_.standby(ops_.context, true);
        if (ops_.ledDeinit(ops_.context) != ESP_OK)
        {
            state_.store(State::STOPPING);
        }
    }
}

void Core::step()
{
    if (stopAck_.load() == StopAck::PENDING)
    {
        const State s = state();
        if (s == State::RUNNING || s == State::NOT_CONNECTED)
        {
            inactiveTicks_ = 0;
            state_.store(State::STOPPING);
        }
        else if (s == State::SLEEPING)
        {
            stopAck_.store(StopAck::SLEEP_READY);
        }
        else if (s == State::ERROR)
        {
            stopAck_.store(StopAck::RECOVERY);
        }
    }
    switch (state())
    {
        case State::UNINITIALIZED:
            break;
        case State::STARTING:
            for (size_t i = 0; i < registry_.size(); ++i)
            {
                const auto& d = registry_.at(registry_.bootSlot(i)).descriptor;
                const esp_err_t error = initialize(d.id, true);
                if (error != ESP_OK && d.critical)
                {
                    enterError();
                    return;
                }
            }
            state_.store(State::NOT_CONNECTED);
            break;
        case State::NOT_CONNECTED:
            if (ops_.busConnected(ops_.context))
            {
                inactiveTicks_ = 0;
                state_.store(State::RUNNING);
            }
            else if (!policy_.sleep_enabled ||
                     ops_.voltage(ops_.context) > static_cast<float>(policy_.stay_awake_voltage_mv) / 1000.0f)
            {
                inactiveTicks_ = 0;
            }
            else if (++inactiveTicks_ >= kInactiveTicks)
            {
                inactiveTicks_ = 0;
                state_.store(State::STOPPING);
            }
            break;
        case State::RUNNING:
            if (!ops_.busConnected(ops_.context))
            {
                disconnected();
            }
            break;
        case State::STOPPING:
            if (++stopAttempts_ > 10)
            {
                enterError();
                return;
            }
            if (shutdown())
            {
                invalidateSleepReservation();
                ops_.writeFailures(ops_.context, 0);
                stopAttempts_ = 0;
                state_.store(State::SLEEPING);
                if (stopAck_.load() == StopAck::PENDING)
                {
                    stopAck_.store(StopAck::SLEEP_READY);
                }
            }
            break;
        case State::SLEEPING:
            sleepCycle();
            break;
        case State::ERROR:
            recover();
            break;
    }
}
}  // namespace supervisor
