#include <array>
#include <cmath>
#include <cstdint>

#include <M5Unified.h>
#include <driver/gpio.h>
#include <esp_sleep.h>

#include "osal/osal.h"
#include "powerManagementTask.h"
#include "uiTask.h"

#define LOG_TAG "PowerTask"
#include "logger.h"

namespace
{
constexpr uint32_t kPowerTaskLoopMs = 100;
constexpr uint32_t kPowerInfoUpdatePeriodMs = 2000;
constexpr uint32_t kPowerKeyHoldMs = 3000;
constexpr uint32_t kPowerIdleSleepTimeoutMs = 60000;
constexpr uint32_t kPowerLightSleepPollWakeMs = 500;
constexpr uint32_t kPowerSleepRetryBackoffMs = 2000;
constexpr float kImuActivityDeltaG = 0.25f;
constexpr size_t kPowerEventQueueCapacity = 8;

constexpr uint8_t kAw9523Addr = 0x58;
constexpr uint8_t kAw9523Port0Reg = 0x02;
constexpr uint8_t kCoreS3BusEnableBit = 0x02;
constexpr uint8_t kCoreS3UsbEnableBit = 0x20;
constexpr uint8_t kCoreS3BoostEnableBit = 0x80;
constexpr uint32_t kPowerI2cFreq = 100000;
constexpr uint8_t kAxp2101ChargeControlReg = 0x18;
constexpr uint8_t kBackfeedDischargeThreshold = 2;

// On CoreS3 this is the shared interrupt line from AW9523 ("I2C_INT").
// If a board revision routes IMU/PWR IRQ into this line, light sleep can wake directly.
constexpr gpio_num_t kCoreS3ImuIrqGpio = GPIO_NUM_21;
constexpr gpio_num_t kCoreS3ButtonIrqGpio = GPIO_NUM_21;

constexpr uint64_t kAxp2101WakeIrqMask =
    static_cast<uint64_t>(m5::AXP2101_IRQ_VBUS_INSERT)
    | static_cast<uint64_t>(m5::AXP2101_IRQ_VBUS_REMOVE)
    | static_cast<uint64_t>(m5::AXP2101_IRQ_PKEY_SHORT_PRESS)
    | static_cast<uint64_t>(m5::AXP2101_IRQ_PKEY_LONG_PRESS)
    | static_cast<uint64_t>(m5::AXP2101_IRQ_PKEY_POSITIVE_EDGE)
    | static_cast<uint64_t>(m5::AXP2101_IRQ_PKEY_NEGATIVE_EDGE);

const char* axp2101EffectiveState(bool vbusPresent, bool chargeEnabled, int chargeState)
{
    if (!vbusPresent) {
        return "battery_discharging";
    }
    if (chargeState == 1) {
        return "usb_charging";
    }
    if (chargeEnabled) {
        return "usb_charge_idle_or_full";
    }
    return "usb_charge_disabled";
}

const char* stablePowerState(bool vbusPresent, bool chargeEnabled)
{
    if (!vbusPresent) {
        return "battery_powered";
    }
    if (chargeEnabled) {
        return "usb_powered";
    }
    return "usb_powered_charge_disabled";
}

int16_t quantizeMv(int16_t mv)
{
    if (mv < 0) {
        return mv;
    }
    return static_cast<int16_t>((mv / 10) * 10);
}

struct ImuMotionTracker
{
    bool initialized = false;
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
};

struct IdleSleepResult
{
    esp_sleep_wakeup_cause_t wakeCause = ESP_SLEEP_WAKEUP_UNDEFINED;
    esp_err_t sleepErr = ESP_OK;
    bool timerWakeEnabled = false;
};

struct AxpIrqStatus
{
    bool vbusInsert = false;
    bool vbusRemove = false;
    bool pkeyShort = false;
    bool pkeyLong = false;
    bool pkeyPosEdge = false;
    bool pkeyNegEdge = false;

    [[nodiscard]] bool hasPowerKeyActivity() const
    {
        return pkeyShort || pkeyLong || pkeyPosEdge || pkeyNegEdge;
    }
};

struct PowerReadings
{
    bool vbusPresent = false;
    bool chargeEnabled = false;
    bool batteryPresent = false;
    int chargeState = 0;

    int16_t vbusVoltageMv = 0;
    int16_t batteryVoltageMv = 0;
    int32_t batteryLevel = 0;
    int32_t batteryCurrentMa = 0;
    int32_t chargingCurrentMa = 0;

    const char* powerState = "unknown";
    const char* effectiveState = "unknown";
};

struct PowerTaskState
{
    bool usbCableConnected = false;
    uint8_t dischargeWhileVbusCount = 0;
    bool powerKeyHoldLatched = false;
    uint32_t powerInfoElapsedMs = 0;
    uint32_t lastActivityMs = 0;
    uint32_t nextSleepAttemptMs = 0;
    ImuMotionTracker imuMotionTracker = {};
    bool idleSleepLogged = false;
    bool displaySleeping = false;
};

class PowerManager
{
public:
    [[nodiscard]] bool start()
    {
        if (!ensureEventQueueReady()) {
            return false;
        }

        auto cfg = M5.config();
        cfg.clear_display = false;
        cfg.fallback_board = m5::board_t::board_M5StackCoreS3;
        cfg.output_power = false;
        M5.begin(cfg);

        forceDisableCoreS3BackPower();
        M5.Power.setExtOutput(false);
        M5.Power.setUsbOutput(false);
        M5.Power.setBatteryCharge(true);
        M5.Power.setChargeCurrent(500);
        M5.Power.setChargeVoltage(4200);
        M5.BtnPWR.setHoldThresh(kPowerKeyHoldMs);
        configureWakeIrqs();

        const osalThreadAttr_t powerTaskAttr = {
            .stack_size_bytes = 4096,
            .priority = 6,
            .detached = true,
            .name = "PowerTask",
        };

        if (osalThreadCreate(&taskThread_, PowerManager::taskEntry, this, &powerTaskAttr) != OSAL_STATUS_OK) {
            LOGE("Failed to create PowerTask");
            return false;
        }

        LOGI("PowerTask started and charging enabled");
        return true;
    }

    [[nodiscard]] bool postEvent(const PowerEvent& event)
    {
        if (!eventQueueReady_) {
            return false;
        }

        return osalQueueSend(&eventQueue_, &event, 0) == OSAL_STATUS_OK;
    }

private:
    static void taskEntry(void* context)
    {
        auto* self = static_cast<PowerManager*>(context);
        if (self != nullptr) {
            self->taskLoop();
        }
    }

    [[nodiscard]] bool ensureEventQueueReady()
    {
        if (eventQueueReady_) {
            return true;
        }

        if (osalQueueInit(&eventQueue_, sizeof(PowerEvent), kPowerEventQueueCapacity) != OSAL_STATUS_OK) {
            LOGE("Failed to create power event queue");
            return false;
        }

        eventQueueReady_ = true;
        return true;
    }

    void initializeRuntimeState()
    {
        state_ = {};
        state_.usbCableConnected = M5.Power.Axp2101.isVBUS();
        state_.powerInfoElapsedMs = kPowerInfoUpdatePeriodMs;
        state_.lastActivityMs = M5.millis();
    }

    void taskLoop()
    {
        initializeRuntimeState();

        while (true) {
            forceDisableCoreS3BackPower();
            M5.update();
            processPowerEvents();

            const uint32_t nowMs = M5.millis();
            bool activityDetected = detectUiActivity() || detectImuActivity();

            updatePowerKeyHold();

            const AxpIrqStatus irqStatus = readAxpIrqStatus();
            if (irqStatus.hasPowerKeyActivity()) {
                activityDetected = true;
            }

            const PowerReadings readings = collectPowerReadings(irqStatus);

            if (activityDetected) {
                markActivity(nowMs);
            }

            if (handleIdleSleep(nowMs)) {
                continue;
            }

            if (state_.powerInfoElapsedMs >= kPowerInfoUpdatePeriodMs) {
                publishPowerInfo(readings);

                LOGI("PMIC=AXP2101 power=%s phase=%s raw_vbus=%dmV bat=%ld%%(%dmV)",
                     readings.powerState,
                     readings.effectiveState,
                     static_cast<int>(readings.vbusVoltageMv),
                     static_cast<long>(readings.batteryLevel),
                     static_cast<int>(readings.batteryVoltageMv));

                state_.powerInfoElapsedMs = 0;
            }

            osalThreadSleepMs(kPowerTaskLoopMs);
            state_.powerInfoElapsedMs += kPowerTaskLoopMs;
        }
    }

    void forceDisableCoreS3BackPower()
    {
        std::array<uint8_t, 2> regs = {};
        if (M5.In_I2C.readRegister(kAw9523Addr, kAw9523Port0Reg, regs.data(), regs.size(), kPowerI2cFreq)) {
            regs[0] &= static_cast<uint8_t>(~(kCoreS3BusEnableBit | kCoreS3UsbEnableBit));
            regs[1] &= static_cast<uint8_t>(~kCoreS3BoostEnableBit);
            M5.In_I2C.writeRegister(kAw9523Addr, kAw9523Port0Reg, regs.data(), regs.size(), kPowerI2cFreq);
        }
    }

    static void sendPowerKeyLongPressToUi()
    {
        UiEvent event = {};
        event.type = UiEventType::KEY_EVENT;

        UiKeyEventData keyData = {};
        keyData.key_id = UiKeyId::POWER;
        keyData.action = UiKeyAction::LONG_PRESS;

        if (!uiSetEventData(event, keyData) || !uiPostEvent(event)) {
            LOGW("Failed to post POWER key event to UI");
        }
    }

    static bool detectUiActivity()
    {
        return (M5.Touch.isEnabled() && (M5.Touch.getCount() > 0))
               || M5.BtnA.wasPressed() || M5.BtnA.wasReleased() || M5.BtnA.wasClicked() || M5.BtnA.wasHold()
               || M5.BtnB.wasPressed() || M5.BtnB.wasReleased() || M5.BtnB.wasClicked() || M5.BtnB.wasHold()
               || M5.BtnC.wasPressed() || M5.BtnC.wasReleased() || M5.BtnC.wasClicked() || M5.BtnC.wasHold()
               || M5.BtnPWR.wasPressed() || M5.BtnPWR.wasReleased() || M5.BtnPWR.wasClicked() || M5.BtnPWR.wasHold();
    }

    bool detectImuActivity()
    {
        if (!M5.Imu.isEnabled()) {
            return false;
        }

        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;
        if (!M5.Imu.getAccel(&ax, &ay, &az)) {
            return false;
        }

        if (!state_.imuMotionTracker.initialized) {
            state_.imuMotionTracker.initialized = true;
            state_.imuMotionTracker.ax = ax;
            state_.imuMotionTracker.ay = ay;
            state_.imuMotionTracker.az = az;
            return false;
        }

        const float delta = std::fabs(ax - state_.imuMotionTracker.ax)
                          + std::fabs(ay - state_.imuMotionTracker.ay)
                          + std::fabs(az - state_.imuMotionTracker.az);
        state_.imuMotionTracker.ax = ax;
        state_.imuMotionTracker.ay = ay;
        state_.imuMotionTracker.az = az;
        return delta >= kImuActivityDeltaG;
    }

    static void armWakeGpio(gpio_num_t pin, bool& armed)
    {
        if (pin >= GPIO_NUM_MAX) {
            return;
        }

        m5gfx::pinMode(pin, m5gfx::pin_mode_t::input_pullup);
        if (gpio_get_level(pin) == 0) {
            LOGW("Wake GPIO %d is already low; skipping wake arm on this pin", static_cast<int>(pin));
            return;
        }

        if (gpio_wakeup_enable(pin, GPIO_INTR_LOW_LEVEL) == ESP_OK) {
            armed = true;
        } else {
            LOGW("Failed to arm wake GPIO %d", static_cast<int>(pin));
        }
    }

    static void disarmWakeGpio(gpio_num_t pin)
    {
        if (pin >= GPIO_NUM_MAX) {
            return;
        }

        gpio_wakeup_disable(pin);
    }

    void clearCoreS3InterruptLatch()
    {
        // AW9523 INT is level-latched; read input ports to release pending IRQ.
        (void)M5.In_I2C.readRegister8(kAw9523Addr, 0x00, kPowerI2cFreq);
        (void)M5.In_I2C.readRegister8(kAw9523Addr, 0x01, kPowerI2cFreq);
    }

    IdleSleepResult enterIdleLightSleep()
    {
        IdleSleepResult result = {};
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
        clearCoreS3InterruptLatch();

        bool gpioWakeArmed = false;
        armWakeGpio(kCoreS3ImuIrqGpio, gpioWakeArmed);
        if (kCoreS3ButtonIrqGpio != kCoreS3ImuIrqGpio) {
            armWakeGpio(kCoreS3ButtonIrqGpio, gpioWakeArmed);
        }

        if (gpioWakeArmed && esp_sleep_enable_gpio_wakeup() != ESP_OK) {
            LOGW("Failed to enable GPIO wakeup source");
            gpioWakeArmed = false;
        }

        result.timerWakeEnabled = !gpioWakeArmed;
        if (result.timerWakeEnabled) {
            esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(kPowerLightSleepPollWakeMs) * 1000ULL);
        }

        if (!state_.displaySleeping) {
            M5.Display.sleep();
            M5.Display.waitDisplay();
            state_.displaySleeping = true;
        }

        result.sleepErr = esp_light_sleep_start();

        disarmWakeGpio(kCoreS3ImuIrqGpio);
        if (kCoreS3ButtonIrqGpio != kCoreS3ImuIrqGpio) {
            disarmWakeGpio(kCoreS3ButtonIrqGpio);
        }

        result.wakeCause = esp_sleep_get_wakeup_cause();
        if (result.sleepErr != ESP_OK) {
            LOGW("esp_light_sleep_start failed: %d (wakeCause=%d)",
                 static_cast<int>(result.sleepErr),
                 static_cast<int>(result.wakeCause));
        }

        const bool timerFallbackWake = result.timerWakeEnabled && result.wakeCause == ESP_SLEEP_WAKEUP_TIMER;
        if (result.sleepErr != ESP_OK || !timerFallbackWake) {
            wakeDisplayIfNeeded();
        }

        if (!timerFallbackWake) {
            LOGI("Wakeup cause=%d", static_cast<int>(result.wakeCause));
        }

        return result;
    }

    void configureWakeIrqs()
    {
        if (!M5.Power.Axp2101.enableIRQ(kAxp2101WakeIrqMask)) {
            LOGW("Failed to enable AXP2101 wake IRQ mask");
        }
        M5.Power.Axp2101.clearIRQStatuses();

        // BMI270 interrupt routing is board-dependent; current CoreS3 wiring may not expose it.
        if (M5.Imu.isEnabled() && !M5.Imu.setINTPinActiveLogic(false)) {
            LOGW("IMU IRQ active-level setup unavailable; using motion polling fallback");
        }
    }

    void processPowerEvents()
    {
        if (!eventQueueReady_) {
            return;
        }

        PowerEvent event = {};
        while (osalQueueReceive(&eventQueue_, &event, 0) == OSAL_STATUS_OK) {
            if (event.type == PowerEventType::KEY_EVENT
                && event.key_event.action == PowerKeyAction::POWEROFF_CONFIRM_YES) {
                LOGI("Power off confirmed by UI");
                M5.Power.powerOff();
            }
        }
    }

    static void publishPowerInfo(const PowerReadings& readings)
    {
        UiPowerInfoEvent powerInfo = {};
        powerInfo.usb_powered = readings.vbusPresent;
        powerInfo.charge_enabled = readings.chargeEnabled;
        powerInfo.battery_present = readings.batteryPresent;
        powerInfo.charge_phase = static_cast<int8_t>(readings.chargeState);
        powerInfo.vbus_voltage_mv = readings.vbusVoltageMv;
        powerInfo.battery_voltage_mv = readings.batteryVoltageMv;
        powerInfo.battery_level_percent = readings.batteryLevel;
        powerInfo.battery_current_ma = readings.batteryCurrentMa;
        powerInfo.charging_current_ma = readings.chargingCurrentMa;

        UiEvent uiEvent = {};
        uiEvent.type = UiEventType::POWER_INFO;
        if (!uiSetEventData(uiEvent, powerInfo)) {
            LOGW("UI power payload too large; dropped power event");
        } else if (!uiPostEvent(uiEvent)) {
            LOGW("UI queue full or not ready; dropped power event");
        }
    }

    static UiWakeupSource mapWakeupSource(esp_sleep_wakeup_cause_t wakeCause)
    {
        switch (wakeCause) {
            case ESP_SLEEP_WAKEUP_GPIO:
            case ESP_SLEEP_WAKEUP_EXT0:
            case ESP_SLEEP_WAKEUP_EXT1:
                return UiWakeupSource::GPIO;

            case ESP_SLEEP_WAKEUP_TIMER:
                return UiWakeupSource::TIMER;

            case ESP_SLEEP_WAKEUP_UNDEFINED:
                return UiWakeupSource::UNKNOWN;

            default:
                return UiWakeupSource::OTHER;
        }
    }

    static bool isPollingTimerWakeup(const IdleSleepResult& sleepResult)
    {
        return sleepResult.sleepErr == ESP_OK
               && sleepResult.timerWakeEnabled
               && sleepResult.wakeCause == ESP_SLEEP_WAKEUP_TIMER;
    }

    static void publishWakeupEvent(const IdleSleepResult& sleepResult)
    {
        UiWakeupEventData wakeupEventData = {};
        wakeupEventData.source = mapWakeupSource(sleepResult.wakeCause);
        wakeupEventData.wakeup_cause = static_cast<int32_t>(sleepResult.wakeCause);
        wakeupEventData.sleep_error = static_cast<int32_t>(sleepResult.sleepErr);

        UiEvent event = {};
        event.type = UiEventType::WAKEUP_EVENT;

        if (!uiSetEventData(event, wakeupEventData)) {
            LOGW("UI wakeup payload too large; dropped wakeup event");
        } else if (!uiPostEvent(event)) {
            LOGW("UI queue full or not ready; dropped wakeup event");
        }
    }

    static AxpIrqStatus readAxpIrqStatus()
    {
        const uint64_t irqStatus = M5.Power.Axp2101.getIRQStatuses();

        AxpIrqStatus parsed = {};
        parsed.vbusInsert = (irqStatus & static_cast<uint64_t>(m5::AXP2101_IRQ_VBUS_INSERT)) != 0;
        parsed.vbusRemove = (irqStatus & static_cast<uint64_t>(m5::AXP2101_IRQ_VBUS_REMOVE)) != 0;
        parsed.pkeyShort = (irqStatus & static_cast<uint64_t>(m5::AXP2101_IRQ_PKEY_SHORT_PRESS)) != 0;
        parsed.pkeyLong = (irqStatus & static_cast<uint64_t>(m5::AXP2101_IRQ_PKEY_LONG_PRESS)) != 0;
        parsed.pkeyPosEdge = (irqStatus & static_cast<uint64_t>(m5::AXP2101_IRQ_PKEY_POSITIVE_EDGE)) != 0;
        parsed.pkeyNegEdge = (irqStatus & static_cast<uint64_t>(m5::AXP2101_IRQ_PKEY_NEGATIVE_EDGE)) != 0;

        M5.Power.Axp2101.clearIRQStatuses();
        return parsed;
    }

    void updatePowerKeyHold()
    {
        if (!state_.powerKeyHoldLatched
            && (M5.BtnPWR.wasHold() || (M5.BtnPWR.isHolding() && M5.BtnPWR.pressedFor(kPowerKeyHoldMs)))) {
            sendPowerKeyLongPressToUi();
            state_.powerKeyHoldLatched = true;
        }

        if (M5.BtnPWR.wasReleased() || M5.BtnPWR.isReleased()) {
            state_.powerKeyHoldLatched = false;
        }
    }

    void updateUsbCableStateFromIrq(const AxpIrqStatus& irqStatus)
    {
        if (irqStatus.vbusInsert && !irqStatus.vbusRemove) {
            state_.usbCableConnected = true;
        } else if (irqStatus.vbusRemove && !irqStatus.vbusInsert) {
            state_.usbCableConnected = false;
        }
    }

    void updateUsbCableStateFromBackfeedModel(bool vbusPresentRaw, int chargeState)
    {
        if (!vbusPresentRaw) {
            state_.usbCableConnected = false;
        }

        // Fallback for boards where VBUS level can stay high due backfeed.
        if (chargeState == -1 && vbusPresentRaw) {
            if (state_.dischargeWhileVbusCount < 0xFF) {
                state_.dischargeWhileVbusCount++;
            }
            if (state_.dischargeWhileVbusCount >= kBackfeedDischargeThreshold) {
                state_.usbCableConnected = false;
            }
        } else {
            state_.dischargeWhileVbusCount = 0;
            if (chargeState >= 0) {
                state_.usbCableConnected = true;
            }
        }
    }

    PowerReadings collectPowerReadings(const AxpIrqStatus& irqStatus)
    {
        updateUsbCableStateFromIrq(irqStatus);

        PowerReadings readings = {};
        readings.batteryLevel = M5.Power.getBatteryLevel();
        readings.batteryCurrentMa = M5.Power.getBatteryCurrent();
        readings.chargingCurrentMa = (readings.batteryCurrentMa > 0) ? readings.batteryCurrentMa : 0;
        readings.batteryVoltageMv = quantizeMv(M5.Power.getBatteryVoltage());
        readings.vbusVoltageMv = quantizeMv(M5.Power.getVBUSVoltage());

        const bool vbusPresentRaw = M5.Power.Axp2101.isVBUS();
        readings.batteryPresent = M5.Power.Axp2101.getBatState();
        readings.chargeState = M5.Power.Axp2101.getChargeStatus();

        const uint8_t reg18 = M5.Power.Axp2101.readRegister8(kAxp2101ChargeControlReg);
        readings.chargeEnabled = (reg18 & 0x02) != 0;

        updateUsbCableStateFromBackfeedModel(vbusPresentRaw, readings.chargeState);

        readings.vbusPresent = state_.usbCableConnected;
        readings.powerState = stablePowerState(readings.vbusPresent, readings.chargeEnabled);
        readings.effectiveState = axp2101EffectiveState(readings.vbusPresent,
                                                        readings.chargeEnabled,
                                                        readings.chargeState);

        return readings;
    }

    void wakeDisplayIfNeeded()
    {
        if (state_.displaySleeping) {
            M5.Display.wakeup();
            state_.displaySleeping = false;
        }
    }

    void markActivity(uint32_t nowMs)
    {
        state_.lastActivityMs = nowMs;
        state_.nextSleepAttemptMs = 0;
        state_.idleSleepLogged = false;
        wakeDisplayIfNeeded();
    }

    bool handleIdleSleep(uint32_t nowMs)
    {
        if ((nowMs - state_.lastActivityMs) < kPowerIdleSleepTimeoutMs || nowMs < state_.nextSleepAttemptMs) {
            return false;
        }

        if (!state_.idleSleepLogged) {
            LOGI("Idle for %u ms; entering light sleep", static_cast<unsigned>(kPowerIdleSleepTimeoutMs));
            state_.idleSleepLogged = true;
        }

        const IdleSleepResult sleepResult = enterIdleLightSleep();
        const uint32_t afterSleepMs = M5.millis();
        if (!isPollingTimerWakeup(sleepResult)) {
            publishWakeupEvent(sleepResult);
        }

        if (sleepResult.sleepErr != ESP_OK) {
            state_.nextSleepAttemptMs = afterSleepMs + kPowerSleepRetryBackoffMs;
        } else if (sleepResult.timerWakeEnabled && sleepResult.wakeCause == ESP_SLEEP_WAKEUP_TIMER) {
            state_.nextSleepAttemptMs = afterSleepMs + kPowerLightSleepPollWakeMs;
        } else {
            markActivity(afterSleepMs);
        }

        state_.imuMotionTracker.initialized = false;
        return true;
    }

    osalThread_t taskThread_ = {};
    osalQueue_t eventQueue_ = {};
    bool eventQueueReady_ = false;
    PowerTaskState state_ = {};
};

PowerManager gPowerManager = {};
} // namespace

bool startPowerManagementTask()
{
    return gPowerManager.start();
}

bool postPowerEvent(const PowerEvent& event)
{
    return gPowerManager.postEvent(event);
}
