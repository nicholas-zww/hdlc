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
constexpr uint32_t kPowerSleepRetryBackoffMs = 2000;
constexpr uint32_t kDisplaySleepRequestTimeoutMs = 800;
constexpr float kImuActivityDeltaAbsG = 0.12f;
constexpr float kImuActivityDeltaVecG = 0.08f;
constexpr float kImuActivityAccumulatedThresholdG = 0.20f;
constexpr float kImuActivityAccumulatorDecay = 0.85f;
constexpr size_t kPowerEventQueueCapacity = 8;

constexpr uint8_t kAw9523Addr = 0x58;
constexpr uint8_t kAw9523Port0Reg = 0x02;
constexpr uint8_t kCoreS3BusEnableBit = 0x02;
constexpr uint8_t kCoreS3UsbEnableBit = 0x20;
constexpr uint8_t kCoreS3BoostEnableBit = 0x80;
constexpr uint32_t kPowerI2cFreq = 100000;
constexpr uint8_t kAxp2101ChargeControlReg = 0x18;
constexpr uint8_t kBackfeedDischargeThreshold = 2;
constexpr uint32_t kPowerIdleBlockedRetryMs = 1000;

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
    float accumulatedDelta = 0.0f;
};

struct IdleSleepResult
{
    esp_sleep_wakeup_cause_t wakeCause = ESP_SLEEP_WAKEUP_UNDEFINED;
    esp_err_t sleepErr = ESP_OK;
};

struct DeepSleepWakeSelection
{
    gpio_num_t pin = GPIO_NUM_NC;
    bool ready = false;
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
    bool allowLowPowerFromChargeState = false;
    int8_t lastChargeState = 2;
};

class PowerManager
{
public:
    [[nodiscard]] bool start()
    {
        if (!ensureEventQueueReady()) {
            return false;
        }

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
            refreshRuntimeLowPowerGate(readings);

            if (activityDetected) {
                markActivity(nowMs);
            }

            if (handleIdleSleep(nowMs, readings)) {
                continue;
            }

            if (state_.powerInfoElapsedMs >= kPowerInfoUpdatePeriodMs) {
                publishPowerInfo(readings);

                // LOGI("PMIC=AXP2101 power=%s phase=%s raw_vbus=%dmV bat=%ld%%(%dmV)",
                //      readings.powerState,
                //      readings.effectiveState,
                //      static_cast<int>(readings.vbusVoltageMv),
                //      static_cast<long>(readings.batteryLevel),
                //      static_cast<int>(readings.batteryVoltageMv));

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

    static void requestUiDisplayPower(UiDisplayPowerAction action)
    {
        UiEvent event = {};
        event.type = UiEventType::DISPLAY_POWER;

        UiDisplayPowerEventData payload = {};
        payload.action = action;

        if (!uiSetEventData(event, payload) || !uiPostEvent(event)) {
            LOGW("Failed to post display power event to UI");
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
            state_.imuMotionTracker.accumulatedDelta = 0.0f;
            return false;
        }

        const float dx = std::fabs(ax - state_.imuMotionTracker.ax);
        const float dy = std::fabs(ay - state_.imuMotionTracker.ay);
        const float dz = std::fabs(az - state_.imuMotionTracker.az);
        const float deltaAbs = dx + dy + dz;
        const float deltaVec = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));

        state_.imuMotionTracker.accumulatedDelta =
            (state_.imuMotionTracker.accumulatedDelta * kImuActivityAccumulatorDecay) + deltaAbs;

        state_.imuMotionTracker.ax = ax;
        state_.imuMotionTracker.ay = ay;
        state_.imuMotionTracker.az = az;

        const bool motionDetected =
            (deltaAbs >= kImuActivityDeltaAbsG)
            || (deltaVec >= kImuActivityDeltaVecG)
            || (state_.imuMotionTracker.accumulatedDelta >= kImuActivityAccumulatedThresholdG);

        if (motionDetected) {
            state_.imuMotionTracker.accumulatedDelta = 0.0f;
        }

        return motionDetected;
    }

    static bool isWakePinHigh(gpio_num_t pin)
    {
        if (pin >= GPIO_NUM_MAX) {
            return false;
        }
        m5gfx::pinMode(pin, m5gfx::pin_mode_t::input_pullup);
        return gpio_get_level(pin) != 0;
    }

    void clearCoreS3InterruptLatch()
    {
        // AW9523 INT is level-latched; read input ports to release pending IRQ.
        (void)M5.In_I2C.readRegister8(kAw9523Addr, 0x00, kPowerI2cFreq);
        (void)M5.In_I2C.readRegister8(kAw9523Addr, 0x01, kPowerI2cFreq);
    }

    static DeepSleepWakeSelection selectDeepSleepWakePin()
    {
        DeepSleepWakeSelection selection = {};
        const bool imuWakePinHigh = isWakePinHigh(kCoreS3ImuIrqGpio);
        const bool buttonWakePinHigh = (kCoreS3ButtonIrqGpio == kCoreS3ImuIrqGpio)
                                       ? imuWakePinHigh
                                       : isWakePinHigh(kCoreS3ButtonIrqGpio);

        if (imuWakePinHigh) {
            selection.pin = kCoreS3ImuIrqGpio;
            selection.ready = true;
            return selection;
        }
        if (buttonWakePinHigh) {
            selection.pin = kCoreS3ButtonIrqGpio;
            selection.ready = true;
            return selection;
        }

        // If both lines are low, ext0 wake would trigger immediately.
        selection.pin = kCoreS3ImuIrqGpio;
        selection.ready = false;
        return selection;
    }

    static void forceDisplayOffFallback()
    {
        // Keep UI ownership, but hard-force panel/backlight off before deep sleep.
        M5.Display.startWrite();
        M5.Display.fillScreen(0x0000);
        M5.Display.endWrite();
        M5.Display.waitDisplay();
        M5.Display.setBrightness(0);
        M5.Display.sleep();
        M5.Display.waitDisplay();
    }

    IdleSleepResult enterIdleDeepSleep()
    {
        IdleSleepResult result = {};
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
        clearCoreS3InterruptLatch();

        const DeepSleepWakeSelection wakeSelection = selectDeepSleepWakePin();
        const gpio_num_t deepWakePin = wakeSelection.pin;
        if (!wakeSelection.ready) {
            result.sleepErr = ESP_ERR_INVALID_STATE;
            LOGW("Wake GPIO is low; postpone deep sleep to avoid immediate wake");
            return result;
        }
        m5gfx::pinMode(deepWakePin, m5gfx::pin_mode_t::input_pullup);

        result.sleepErr = esp_sleep_enable_ext0_wakeup(deepWakePin, 0);
        if (result.sleepErr != ESP_OK) {
            LOGW("Failed to enable ext0 deep sleep wakeup on GPIO%d: %d",
                 static_cast<int>(deepWakePin),
                 static_cast<int>(result.sleepErr));
            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
            return result;
        }

        if (!uiRequestDisplayPowerAndWait(UiDisplayPowerAction::SLEEP, kDisplaySleepRequestTimeoutMs)) {
            LOGW("UI display sleep ack timeout; entering deep sleep anyway");
        }
        forceDisplayOffFallback();
        forceDisableCoreS3BackPower();

        LOGI("Idle for %u ms; entering deep sleep via GPIO%d",
             static_cast<unsigned>(kPowerIdleSleepTimeoutMs),
             static_cast<int>(deepWakePin));
        esp_deep_sleep_start();

        result.sleepErr = ESP_FAIL;
        result.wakeCause = esp_sleep_get_wakeup_cause();
        LOGW("esp_deep_sleep_start returned unexpectedly (wakeCause=%d)",
             static_cast<int>(result.wakeCause));
        requestUiDisplayPower(UiDisplayPowerAction::WAKE);

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
            if (event.type == PowerEventType::KEY_EVENT) {
                if (event.key_event.action == PowerKeyAction::POWEROFF_CONFIRM_YES) {
                    LOGI("Power off confirmed by UI");
                    M5.Power.powerOff();
                }
                continue;
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

    void markActivity(uint32_t nowMs)
    {
        state_.lastActivityMs = nowMs;
        state_.nextSleepAttemptMs = 0;
        state_.idleSleepLogged = false;
        requestUiDisplayPower(UiDisplayPowerAction::WAKE);
    }

    static bool isDischargingNow(const PowerReadings& readings)
    {
        return readings.chargeState == -1;
    }

    static const char* chargeStateToString(int chargeState)
    {
        switch (chargeState) {
            case 1:
                return "charging";
            case -1:
                return "discharging";
            case 0:
                return "standby_or_full";
            default:
                return "unknown";
        }
    }

    static bool allowLowPowerForChargeState(const PowerReadings& readings)
    {
        // Runtime gate: low-power entry is only allowed while discharging on battery.
        return isDischargingNow(readings);
    }

    void refreshRuntimeLowPowerGate(const PowerReadings& readings)
    {
        const bool allowFromChargeState = allowLowPowerForChargeState(readings);
        const bool stateChanged = (state_.lastChargeState != readings.chargeState)
                                  || (state_.allowLowPowerFromChargeState != allowFromChargeState);

        state_.allowLowPowerFromChargeState = allowFromChargeState;
        state_.lastChargeState = static_cast<int8_t>(readings.chargeState);

        if (stateChanged) {
            LOGI("Low-power runtime gate: %s (charge_state=%s)",
                 state_.allowLowPowerFromChargeState ? "ENABLED" : "DISABLED",
                 chargeStateToString(readings.chargeState));
        }
    }

    bool handleIdleSleep(uint32_t nowMs, const PowerReadings& readings)
    {
        if ((nowMs - state_.lastActivityMs) < kPowerIdleSleepTimeoutMs || nowMs < state_.nextSleepAttemptMs) {
            return false;
        }

        if (!state_.allowLowPowerFromChargeState) {
            state_.idleSleepLogged = false;
            state_.nextSleepAttemptMs = nowMs + kPowerIdleBlockedRetryMs;
            return false;
        }

        if (!state_.idleSleepLogged) {
            LOGI("Idle for %u ms; entering low power",
                 static_cast<unsigned>(kPowerIdleSleepTimeoutMs));
            state_.idleSleepLogged = true;
        }

        const IdleSleepResult sleepResult = enterIdleDeepSleep();
        const uint32_t afterSleepMs = M5.millis();
        publishWakeupEvent(sleepResult);

        if (sleepResult.sleepErr != ESP_OK) {
            state_.nextSleepAttemptMs = afterSleepMs + kPowerSleepRetryBackoffMs;
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
