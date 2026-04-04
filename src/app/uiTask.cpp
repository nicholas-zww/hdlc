#include "uiTask.h"
#include "powerManagementTask.h"
#include "osal/osal.h"

#include <M5Unified.h>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <limits>

#define LOG_TAG "UiTask"
#include "logger.h"

namespace
{
constexpr size_t kEventQueueCapacity = 16;
constexpr uint32_t kLoopMs = 50;
constexpr uint32_t kColorBackground = 0x0000;
constexpr uint32_t kColorForeground = 0xFFFF;
constexpr uint8_t kDisplayRotation = 3;  // 180 degree rotation
constexpr uint8_t kDisplayBrightnessAwake = 128;
constexpr uint8_t kDisplayBrightnessSleep = 0;
constexpr int kOriginX = 8;
constexpr int kOriginY = 8;
constexpr int kLineHeight = 18;
constexpr uint32_t kI2cFreq = 100000;
constexpr uint8_t kLtr553Addr = 0x23;
constexpr uint8_t kLtrRegAlsContr = 0x80;
constexpr uint8_t kLtrRegPsContr = 0x81;
constexpr uint8_t kLtrRegPsLed = 0x82;
constexpr uint8_t kLtrRegPsNPulses = 0x83;
constexpr uint8_t kLtrRegPsMeasRate = 0x84;
constexpr uint8_t kLtrRegAlsMeasRate = 0x85;
constexpr uint8_t kLtrRegAlsDataCh1 = 0x88;
constexpr uint8_t kLtrRegPsData = 0x8D;
constexpr uint8_t kLtrPsModeMask = 0x02;
constexpr uint8_t kLtrPsModeShift = 1;
constexpr uint8_t kLtrAlsModeMask = 0x01;
constexpr uint8_t kLtrAlsModeShift = 0;
constexpr uint8_t kLtrAlsGainMask = 0x1C;
constexpr uint8_t kLtrAlsGainShift = 2;
constexpr uint8_t kLtrAlsIntegrationMask = 0x38;
constexpr uint8_t kLtrAlsMeasureMask = 0x07;
constexpr uint8_t kLtrPsDataHighMask = 0x07;
constexpr uint8_t kLtrDefaultPsLedReg = 0x7C;     // 60kHz, 100% duty, 100mA peak
constexpr uint8_t kLtrDefaultPsPulseCount = 0x01; // 1 pulse
constexpr uint8_t kLtrDefaultPsMeasRate = 0x02;   // 100ms
constexpr uint8_t kLtrDefaultAlsMeasRate = 0x03;  // 100ms integration, 500ms rate
constexpr uint16_t kProximityClosedThreshold = 180;
constexpr uint16_t kProximityOpenThreshold = 120;
constexpr uint32_t kProximitySamplePeriodMs = 100;

struct ProximityStatus
{
    bool available = false;
    bool closed = false;
    int16_t proximityRaw = -1;
    int16_t ambientRaw = -1;
};

struct Rect
{
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;

    [[nodiscard]] bool contains(int px, int py) const
    {
        return (px >= x && px < (x + w) && py >= y && py < (y + h));
    }
};

constexpr Rect kYesButton = {40, 170, 100, 44};
constexpr Rect kNoButton = {180, 170, 100, 44};

std::atomic<uint8_t> gDisplaySleepState = {0}; // 0=awake, 1=sleeping

class UiController
{
public:
    [[nodiscard]] bool start()
    {
        if (!ensureQueueReady()) {
            return false;
        }

        initializeDisplay();
        gDisplaySleepState.store(0, std::memory_order_release);
        proximitySensorReady_ = initializeProximitySensor();
        if (proximitySensorReady_) {
            LOGI("LTR553 proximity sensor initialized in UI task");
        } else {
            LOGW("LTR553 proximity sensor unavailable in UI task");
        }

        const osalThreadAttr_t uiTaskAttr = {
            .stack_size_bytes = 4096,
            .priority = 7,
            .detached = true,
            .name = "UiTask",
        };

        if (osalThreadCreate(&taskThread_, UiController::taskEntry, this, &uiTaskAttr) != OSAL_STATUS_OK) {
            LOGE("Failed to create UiTask");
            return false;
        }

        LOGI("UiTask started");
        return true;
    }

    [[nodiscard]] bool postEvent(const UiEvent& event)
    {
        if (!queueReady_) {
            return false;
        }
        return osalQueueSend(&eventQueue_, &event, 0) == OSAL_STATUS_OK;
    }

private:
    enum class Screen : uint8_t
    {
        PowerInfo,
        PowerOffConfirm,
    };

    struct State
    {
        UiPowerInfoEvent latestPowerInfo = {};
        bool hasPowerInfo = false;

        ProximityStatus latestProximity = {};
        bool hasProximity = false;

        UiWakeupEventData latestWakeup = {};
        bool hasWakeup = false;

        UiWifiStatusEventData latestWifiStatus = {};
        bool hasWifiStatus = false;

        UiPowerInfoEvent renderedPowerInfo = {};
        bool renderedPowerInfoValid = false;
        ProximityStatus renderedProximity = {};
        bool renderedProximityValid = false;
        UiWakeupEventData renderedWakeup = {};
        bool renderedWakeupValid = false;
        UiWifiStatusEventData renderedWifiStatus = {};
        bool renderedWifiStatusValid = false;
        bool renderedPowerInfoScreen = false;

        Screen screen = Screen::PowerInfo;
        bool touchWasDown = false;
        bool displaySleeping = false;
        bool proximityValid = false;
        uint32_t nextProximitySampleMs = 0;
    };

    static void taskEntry(void* context)
    {
        auto* self = static_cast<UiController*>(context);
        if (self != nullptr) {
            self->taskLoop();
        }
    }

    void taskLoop()
    {
        state_ = {};
        state_.latestProximity.available = proximitySensorReady_;
        state_.hasProximity = proximitySensorReady_;
        state_.nextProximitySampleMs = M5.millis();
        UiEvent event = {};

        while (true) {
            if (osalQueueReceive(&eventQueue_, &event, kLoopMs) == OSAL_STATUS_OK) {
                handleEvent(event);
            }

            handlePowerOffConfirmTouch();
            pollProximity();
        }
    }

    [[nodiscard]] bool ensureQueueReady()
    {
        if (queueReady_) {
            return true;
        }

        if (osalQueueInit(&eventQueue_, sizeof(UiEvent), kEventQueueCapacity) != OSAL_STATUS_OK) {
            LOGE("Failed to create UI event queue");
            return false;
        }

        queueReady_ = true;
        return true;
    }

    static const char* chargePhaseToString(int8_t phase)
    {
        switch (phase) {
            case 1:
                return "charging";
            case 0:
                return "standby_or_full";
            case -1:
                return "discharging";
            default:
                return "unknown";
        }
    }

    static const char* wakeupSourceToString(UiWakeupSource source)
    {
        switch (source) {
            case UiWakeupSource::GPIO:
                return "gpio";
            case UiWakeupSource::TIMER:
                return "timer";
            case UiWakeupSource::OTHER:
                return "other";
            case UiWakeupSource::UNKNOWN:
            default:
                return "unknown";
        }
    }

    static const char* wifiStateToString(UiWifiConnectionState state)
    {
        switch (state) {
            case UiWifiConnectionState::IDLE:
                return "idle";
            case UiWifiConnectionState::CONNECTING:
                return "connecting";
            case UiWifiConnectionState::CONNECTED:
                return "connected";
            case UiWifiConnectionState::DISCONNECTED:
                return "disconnected";
            case UiWifiConnectionState::RECONNECTING:
                return "reconnecting";
            default:
                return "unknown";
        }
    }

    static void initializeDisplay()
    {
        auto cfg = M5.config();
        cfg.clear_display = false;
        cfg.fallback_board = m5::board_t::board_M5StackCoreS3;
        cfg.output_power = false;
        M5.begin(cfg);
        M5.Display.setRotation(kDisplayRotation);
        M5.Display.setBrightness(kDisplayBrightnessAwake);
    }

    static void sendPowerOffConfirmToPowerTask()
    {
        PowerEvent event = {};
        event.type = PowerEventType::KEY_EVENT;
        event.key_event.action = PowerKeyAction::POWEROFF_CONFIRM_YES;
        if (!postPowerEvent(event)) {
            LOGW("Failed to post power-off confirm event");
        }
    }

    static bool proximityEqual(const ProximityStatus& a, const ProximityStatus& b)
    {
        return a.available == b.available
               && a.closed == b.closed
               && a.proximityRaw == b.proximityRaw
               && a.ambientRaw == b.ambientRaw;
    }

    [[nodiscard]] bool ltrReadRegister(uint8_t reg, uint8_t& value) const
    {
        return M5.In_I2C.readRegister(kLtr553Addr, reg, &value, 1, kI2cFreq);
    }

    [[nodiscard]] bool ltrReadRegisters(uint8_t reg, uint8_t* values, size_t count) const
    {
        return M5.In_I2C.readRegister(kLtr553Addr, reg, values, count, kI2cFreq);
    }

    [[nodiscard]] bool ltrWriteRegister(uint8_t reg, uint8_t value) const
    {
        return M5.In_I2C.writeRegister(kLtr553Addr, reg, &value, 1, kI2cFreq);
    }

    [[nodiscard]] bool ltrUpdateRegisterBits(uint8_t reg, uint8_t mask, uint8_t value) const
    {
        uint8_t current = 0;
        if (!ltrReadRegister(reg, current)) {
            return false;
        }

        current = static_cast<uint8_t>((current & static_cast<uint8_t>(~mask)) | (value & mask));
        return ltrWriteRegister(reg, current);
    }

    [[nodiscard]] bool ltrSetPsMode(bool active) const
    {
        const uint8_t value = static_cast<uint8_t>((active ? 1U : 0U) << kLtrPsModeShift);
        return ltrUpdateRegisterBits(kLtrRegPsContr, kLtrPsModeMask, value);
    }

    [[nodiscard]] bool ltrSetAlsMode(bool active) const
    {
        const uint8_t value = static_cast<uint8_t>((active ? 1U : 0U) << kLtrAlsModeShift);
        return ltrUpdateRegisterBits(kLtrRegAlsContr, kLtrAlsModeMask, value);
    }

    [[nodiscard]] bool initializeProximitySensor() const
    {
        uint8_t probe = 0;
        if (!ltrReadRegister(kLtrRegAlsMeasRate, probe)) {
            return false;
        }

        if (!ltrSetPsMode(false)) {
            return false;
        }
        if (!ltrSetAlsMode(false)) {
            return false;
        }

        if (!ltrWriteRegister(kLtrRegPsLed, kLtrDefaultPsLedReg)) {
            return false;
        }
        if (!ltrWriteRegister(kLtrRegPsNPulses, kLtrDefaultPsPulseCount)) {
            return false;
        }
        if (!ltrWriteRegister(kLtrRegPsMeasRate, kLtrDefaultPsMeasRate)) {
            return false;
        }

        if (!ltrUpdateRegisterBits(kLtrRegAlsContr, kLtrAlsGainMask, static_cast<uint8_t>(0U << kLtrAlsGainShift))) {
            return false;
        }

        const uint8_t alsRateValue =
            static_cast<uint8_t>((0U & kLtrAlsIntegrationMask) | (kLtrDefaultAlsMeasRate & kLtrAlsMeasureMask));
        if (!ltrWriteRegister(kLtrRegAlsMeasRate, alsRateValue)) {
            return false;
        }

        if (!ltrSetPsMode(true)) {
            return false;
        }
        if (!ltrSetAlsMode(true)) {
            return false;
        }

        return true;
    }

    [[nodiscard]] bool readProximity(uint16_t& proximityRaw, uint16_t& ambientRaw) const
    {
        uint8_t psData[2] = {};
        if (!ltrReadRegisters(kLtrRegPsData, psData, sizeof(psData))) {
            return false;
        }
        proximityRaw = static_cast<uint16_t>(((psData[1] & kLtrPsDataHighMask) << 8) | psData[0]);

        uint8_t alsData[4] = {};
        if (!ltrReadRegisters(kLtrRegAlsDataCh1, alsData, sizeof(alsData))) {
            return false;
        }
        const uint16_t ch1 = static_cast<uint16_t>((alsData[1] << 8) | alsData[0]);
        const uint16_t ch0 = static_cast<uint16_t>((alsData[3] << 8) | alsData[2]);
        ambientRaw = static_cast<uint16_t>((static_cast<uint32_t>(ch1) + static_cast<uint32_t>(ch0)) / 2U);
        return true;
    }

    void updateProximityState(uint32_t nowMs)
    {
        if (!state_.latestProximity.available || nowMs < state_.nextProximitySampleMs) {
            return;
        }

        state_.nextProximitySampleMs = nowMs + kProximitySamplePeriodMs;

        uint16_t proximityRaw = 0;
        uint16_t ambientRaw = 0;
        if (!readProximity(proximityRaw, ambientRaw)) {
            LOGW("Failed reading LTR553 in UI; disabling proximity sensor");
            state_.latestProximity = {};
            state_.latestProximity.available = false;
            state_.latestProximity.proximityRaw = -1;
            state_.latestProximity.ambientRaw = -1;
            state_.proximityValid = false;
            state_.hasProximity = true;
            if (state_.screen == Screen::PowerInfo) {
                renderPowerInfoScreen();
            }
            return;
        }

        const bool prevValid = state_.proximityValid;
        const bool prevClosed = state_.latestProximity.closed;

        state_.latestProximity.proximityRaw = static_cast<int16_t>(proximityRaw);
        state_.latestProximity.ambientRaw =
            (ambientRaw > static_cast<uint16_t>(std::numeric_limits<int16_t>::max()))
                ? std::numeric_limits<int16_t>::max()
                : static_cast<int16_t>(ambientRaw);
        state_.latestProximity.available = true;

        if (!state_.proximityValid) {
            state_.latestProximity.closed = (proximityRaw >= kProximityClosedThreshold);
            state_.proximityValid = true;
        } else if (state_.latestProximity.closed) {
            if (proximityRaw <= kProximityOpenThreshold) {
                state_.latestProximity.closed = false;
            }
        } else if (proximityRaw >= kProximityClosedThreshold) {
            state_.latestProximity.closed = true;
        }

        state_.hasProximity = true;

        const bool changed = !prevValid || (prevClosed != state_.latestProximity.closed);

        if (changed) {
            if (state_.screen == Screen::PowerInfo) {
                renderPowerInfoScreen();
            }
        }
    }

    void applyProximityDisplayPolicy()
    {
        if (!state_.hasProximity || !state_.latestProximity.available) {
            return;
        }

        if (state_.latestProximity.closed) {
            sleepDisplayIfNeeded();
        } else {
            wakeDisplayIfNeeded();
        }
    }

    void pollProximity()
    {
        if (!state_.latestProximity.available) {
            return;
        }

        updateProximityState(M5.millis());
        applyProximityDisplayPolicy();
    }

    static void formatPowerLine(char* out, size_t outSize, const UiPowerInfoEvent& info)
    {
        std::snprintf(out, outSize, "Power: %s", info.usb_powered ? "USB" : "BATTERY");
    }

    static void formatPhaseLine(char* out, size_t outSize, const UiPowerInfoEvent& info)
    {
        std::snprintf(out, outSize, "Phase: %s", chargePhaseToString(info.charge_phase));
    }

    static void formatChargeEnableLine(char* out, size_t outSize, const UiPowerInfoEvent& info)
    {
        std::snprintf(out, outSize, "Charge Enable: %s", info.charge_enabled ? "YES" : "NO");
    }

    static void formatBatteryLine(char* out, size_t outSize, const UiPowerInfoEvent& info)
    {
        std::snprintf(out,
                      outSize,
                      "Battery: %ld%%  %dmV",
                      static_cast<long>(info.battery_level_percent),
                      static_cast<int>(info.battery_voltage_mv));
    }

    static void formatVbusLine(char* out, size_t outSize, const UiPowerInfoEvent& info)
    {
        std::snprintf(out, outSize, "VBUS: %dmV", static_cast<int>(info.vbus_voltage_mv));
    }

    static void formatProximityLine(char* out, size_t outSize, const ProximityStatus* proximity)
    {
        if (proximity == nullptr || !proximity->available) {
            std::snprintf(out, outSize, "Prox: n/a");
            return;
        }

        std::snprintf(out,
                      outSize,
                      "Prox: %s ps=%d als=%d",
                      proximity->closed ? "CLOSED" : "OPEN",
                      static_cast<int>(proximity->proximityRaw),
                      static_cast<int>(proximity->ambientRaw));
    }

    static void formatWakeLine(char* out, size_t outSize, const UiWakeupEventData* wakeup)
    {
        if (wakeup == nullptr) {
            std::snprintf(out, outSize, "Wake: n/a");
            return;
        }

        std::snprintf(out,
                      outSize,
                      "Wake: %s (cause=%ld)",
                      wakeupSourceToString(wakeup->source),
                      static_cast<long>(wakeup->wakeup_cause));
    }

    static void formatSleepErrorLine(char* out, size_t outSize, const UiWakeupEventData* wakeup)
    {
        if (wakeup == nullptr) {
            std::snprintf(out, outSize, "SleepErr: n/a");
            return;
        }

        std::snprintf(out, outSize, "SleepErr: %ld", static_cast<long>(wakeup->sleep_error));
    }

    static void formatWifiLine(char* out, size_t outSize, const UiWifiStatusEventData* wifiStatus)
    {
        if (wifiStatus == nullptr) {
            std::snprintf(out, outSize, "WiFi: n/a");
            return;
        }

        if (wifiStatus->state == UiWifiConnectionState::CONNECTED && wifiStatus->ip_address[0] != '\0') {
            std::snprintf(out, outSize, "WiFi IP: %s", wifiStatus->ip_address);
            return;
        }

        std::snprintf(out, outSize, "WiFi: %s", wifiStateToString(wifiStatus->state));
    }

    static void drawPowerInfoLine(int lineIndex, const char* text)
    {
        const int y = kOriginY + (lineIndex * kLineHeight);
        int lineWidth = M5.Display.width() - (kOriginX * 2);
        if (lineWidth <= 0) {
            lineWidth = M5.Display.width();
        }
        M5.Display.fillRect(kOriginX, y, lineWidth, kLineHeight, kColorBackground);
        M5.Display.setCursor(kOriginX, y);
        M5.Display.print(text);
    }

    static bool wakeupChanged(const UiWakeupEventData* newWakeup,
                              bool renderedWakeupValid,
                              const UiWakeupEventData& renderedWakeup)
    {
        if (newWakeup == nullptr) {
            return renderedWakeupValid;
        }
        if (!renderedWakeupValid) {
            return true;
        }
        return (newWakeup->source != renderedWakeup.source)
               || (newWakeup->wakeup_cause != renderedWakeup.wakeup_cause)
               || (newWakeup->sleep_error != renderedWakeup.sleep_error);
    }

    static bool wifiChanged(const UiWifiStatusEventData* newWifi,
                            bool renderedWifiValid,
                            const UiWifiStatusEventData& renderedWifi)
    {
        if (newWifi == nullptr) {
            return renderedWifiValid;
        }

        if (!renderedWifiValid) {
            return true;
        }

        return (newWifi->state != renderedWifi.state)
               || (std::strncmp(newWifi->ip_address,
                                renderedWifi.ip_address,
                                sizeof(newWifi->ip_address))
                   != 0);
    }

    static bool proximityChanged(const ProximityStatus* newProximity,
                                 bool renderedProximityValid,
                                 const ProximityStatus& renderedProximity)
    {
        if (newProximity == nullptr) {
            return renderedProximityValid;
        }

        if (!renderedProximityValid) {
            return true;
        }

        return !proximityEqual(*newProximity, renderedProximity);
    }

    void renderPowerInfo(const UiPowerInfoEvent& info,
                         const ProximityStatus* proximity,
                         const UiWakeupEventData* wakeup,
                         const UiWifiStatusEventData* wifiStatus,
                         bool forceFullRedraw)
    {
        const bool powerChanged = !state_.renderedPowerInfoValid
                                  || (info.usb_powered != state_.renderedPowerInfo.usb_powered)
                                  || (info.charge_phase != state_.renderedPowerInfo.charge_phase)
                                  || (info.charge_enabled != state_.renderedPowerInfo.charge_enabled)
                                  || (info.battery_level_percent != state_.renderedPowerInfo.battery_level_percent)
                                  || (info.battery_voltage_mv != state_.renderedPowerInfo.battery_voltage_mv)
                                  || (info.vbus_voltage_mv != state_.renderedPowerInfo.vbus_voltage_mv);

        const bool proximityStatusChanged = proximityChanged(proximity,
                                                             state_.renderedProximityValid,
                                                             state_.renderedProximity);
        const bool wakeChanged = wakeupChanged(wakeup, state_.renderedWakeupValid, state_.renderedWakeup);
        const bool wifiStatusChanged = wifiChanged(wifiStatus,
                                                   state_.renderedWifiStatusValid,
                                                   state_.renderedWifiStatus);

        if (!forceFullRedraw && !powerChanged && !proximityStatusChanged && !wakeChanged && !wifiStatusChanged) {
            return;
        }

        M5.Display.startWrite();
        M5.Display.setTextColor(kColorForeground, kColorBackground);
        M5.Display.setTextSize(1);

        if (forceFullRedraw) {
            M5.Display.fillScreen(kColorBackground);
        }

        char line[64] = {};
        if (forceFullRedraw || powerChanged) {
            formatPowerLine(line, sizeof(line), info);
            drawPowerInfoLine(0, line);

            formatPhaseLine(line, sizeof(line), info);
            drawPowerInfoLine(1, line);

            formatChargeEnableLine(line, sizeof(line), info);
            drawPowerInfoLine(2, line);

            formatBatteryLine(line, sizeof(line), info);
            drawPowerInfoLine(3, line);

            formatVbusLine(line, sizeof(line), info);
            drawPowerInfoLine(4, line);

        }

        if (forceFullRedraw || proximityStatusChanged) {
            formatProximityLine(line, sizeof(line), proximity);
            drawPowerInfoLine(5, line);
        }

        if (forceFullRedraw || wakeChanged) {
            formatWakeLine(line, sizeof(line), wakeup);
            drawPowerInfoLine(6, line);

            formatSleepErrorLine(line, sizeof(line), wakeup);
            drawPowerInfoLine(7, line);
        }

        if (forceFullRedraw || wifiStatusChanged) {
            formatWifiLine(line, sizeof(line), wifiStatus);
            drawPowerInfoLine(8, line);
        }

        M5.Display.endWrite();

        state_.renderedPowerInfo = info;
        state_.renderedPowerInfoValid = true;

        if (wakeup != nullptr) {
            state_.renderedWakeup = *wakeup;
            state_.renderedWakeupValid = true;
        } else {
            state_.renderedWakeup = {};
            state_.renderedWakeupValid = false;
        }

        if (wifiStatus != nullptr) {
            state_.renderedWifiStatus = *wifiStatus;
            state_.renderedWifiStatusValid = true;
        } else {
            state_.renderedWifiStatus = {};
            state_.renderedWifiStatusValid = false;
        }

        if (proximity != nullptr) {
            state_.renderedProximity = *proximity;
            state_.renderedProximityValid = true;
        } else {
            state_.renderedProximity = {};
            state_.renderedProximityValid = false;
        }
    }

    void resetPowerInfoRenderCache()
    {
        state_.renderedPowerInfo = {};
        state_.renderedPowerInfoValid = false;
        state_.renderedProximity = {};
        state_.renderedProximityValid = false;
        state_.renderedWakeup = {};
        state_.renderedWakeupValid = false;
        state_.renderedWifiStatus = {};
        state_.renderedWifiStatusValid = false;
        state_.renderedPowerInfoScreen = false;
    }

    void renderPowerOffConfirm()
    {
        constexpr uint32_t kColorBackground = 0x0000;
        constexpr uint32_t kColorForeground = 0xFFFF;

        M5.Display.startWrite();
        M5.Display.fillScreen(kColorBackground);
        M5.Display.setTextColor(kColorForeground, kColorBackground);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(16, 36);
        M5.Display.print("Power Off?");
        M5.Display.setTextSize(1);
        M5.Display.setCursor(16, 80);
        M5.Display.print("Hold power key detected.");
        M5.Display.setCursor(16, 100);
        M5.Display.print("Do you want to shut down?");

        M5.Display.drawRect(kYesButton.x, kYesButton.y, kYesButton.w, kYesButton.h, kColorForeground);
        M5.Display.drawRect(kNoButton.x, kNoButton.y, kNoButton.w, kNoButton.h, kColorForeground);
        M5.Display.setCursor(kYesButton.x + 34, kYesButton.y + 14);
        M5.Display.print("YES");
        M5.Display.setCursor(kNoButton.x + 38, kNoButton.y + 14);
        M5.Display.print("NO");
        M5.Display.endWrite();
    }

    void renderPowerInfoScreen()
    {
        if (!state_.hasPowerInfo) {
            return;
        }

        const ProximityStatus* proximity = state_.hasProximity ? &state_.latestProximity : nullptr;
        const UiWakeupEventData* wakeup = state_.hasWakeup ? &state_.latestWakeup : nullptr;
        const UiWifiStatusEventData* wifiStatus = state_.hasWifiStatus ? &state_.latestWifiStatus : nullptr;
        const bool forceFullRedraw = !state_.renderedPowerInfoScreen;
        renderPowerInfo(state_.latestPowerInfo, proximity, wakeup, wifiStatus, forceFullRedraw);
        state_.renderedPowerInfoScreen = true;
    }

    void showPowerInfoScreen()
    {
        state_.screen = Screen::PowerInfo;
        state_.touchWasDown = false;
        resetPowerInfoRenderCache();
        renderPowerInfoScreen();
    }

    void showPowerOffConfirmScreen()
    {
        state_.screen = Screen::PowerOffConfirm;
        state_.touchWasDown = false;
        state_.renderedPowerInfoScreen = false;
        renderPowerOffConfirm();
    }

    void handlePowerInfoEvent(const UiEvent& event)
    {
        UiPowerInfoEvent payload = {};
        if (!uiTryGetEventData(event, payload)) {
            LOGW("UI power event has invalid payload size");
            return;
        }

        state_.latestPowerInfo = payload;
        state_.hasPowerInfo = true;

        if (state_.screen == Screen::PowerInfo) {
            renderPowerInfoScreen();
        }
    }

    void handleWakeupEvent(const UiEvent& event)
    {
        UiWakeupEventData payload = {};
        if (!uiTryGetEventData(event, payload)) {
            LOGW("UI wakeup event has invalid payload size");
            return;
        }

        state_.latestWakeup = payload;
        state_.hasWakeup = true;

        LOGI("Wakeup event received: source=%u cause=%ld sleepErr=%ld",
             static_cast<unsigned>(payload.source),
             static_cast<long>(payload.wakeup_cause),
             static_cast<long>(payload.sleep_error));

        if (state_.screen == Screen::PowerInfo) {
            renderPowerInfoScreen();
        }
    }

    void handleWifiStatusEvent(const UiEvent& event)
    {
        UiWifiStatusEventData payload = {};
        if (!uiTryGetEventData(event, payload)) {
            LOGW("UI wifi status event has invalid payload size");
            return;
        }

        state_.latestWifiStatus = payload;
        state_.hasWifiStatus = true;

        if (state_.screen == Screen::PowerInfo) {
            renderPowerInfoScreen();
        }
    }

    void sleepDisplayIfNeeded()
    {
        if (state_.displaySleeping) {
            gDisplaySleepState.store(1, std::memory_order_release);
            return;
        }

        M5.Display.startWrite();
        M5.Display.fillScreen(kColorBackground);
        M5.Display.endWrite();
        M5.Display.waitDisplay();
        M5.Display.setBrightness(kDisplayBrightnessSleep);
        M5.Display.sleep();
        M5.Display.waitDisplay();
        M5.Display.setBrightness(kDisplayBrightnessSleep);
        state_.displaySleeping = true;
        gDisplaySleepState.store(1, std::memory_order_release);
    }

    void wakeDisplayIfNeeded()
    {
        if (!state_.displaySleeping) {
            gDisplaySleepState.store(0, std::memory_order_release);
            return;
        }

        M5.Display.wakeup();
        M5.Display.setBrightness(kDisplayBrightnessAwake);
        state_.displaySleeping = false;
        gDisplaySleepState.store(0, std::memory_order_release);

        if (state_.screen == Screen::PowerInfo) {
            resetPowerInfoRenderCache();
            renderPowerInfoScreen();
        } else {
            renderPowerOffConfirm();
        }
    }

    void handleDisplayPowerEvent(const UiEvent& event)
    {
        UiDisplayPowerEventData payload = {};
        if (!uiTryGetEventData(event, payload)) {
            LOGW("UI display power event has invalid payload size");
            return;
        }

        if (payload.action == UiDisplayPowerAction::SLEEP) {
            sleepDisplayIfNeeded();
        } else {
            wakeDisplayIfNeeded();
        }
    }

    void handleKeyEvent(const UiEvent& event)
    {
        UiKeyEventData payload = {};
        if (!uiTryGetEventData(event, payload)) {
            LOGW("UI key event has invalid payload size");
            return;
        }

        if (payload.key_id == UiKeyId::POWER && payload.action == UiKeyAction::LONG_PRESS) {
            showPowerOffConfirmScreen();
        }
    }

    void handleEvent(const UiEvent& event)
    {
        switch (event.type) {
            case UiEventType::POWER_INFO:
                handlePowerInfoEvent(event);
                break;

            case UiEventType::KEY_EVENT:
                handleKeyEvent(event);
                break;

            case UiEventType::WAKEUP_EVENT:
                handleWakeupEvent(event);
                break;

            case UiEventType::WIFI_STATUS:
                handleWifiStatusEvent(event);
                break;

            case UiEventType::DISPLAY_POWER:
                handleDisplayPowerEvent(event);
                break;

            default:
                LOGW("UI received unknown event type");
                break;
        }
    }

    void handlePowerOffConfirmTouch()
    {
        if (state_.displaySleeping) {
            state_.touchWasDown = false;
            return;
        }

        if (state_.screen != Screen::PowerOffConfirm) {
            state_.touchWasDown = false;
            return;
        }

        int32_t tx = 0;
        int32_t ty = 0;
        const bool touchDown = M5.Display.getTouch(&tx, &ty) > 0;

        if (touchDown && !state_.touchWasDown) {
            const int touchX = static_cast<int>(tx);
            const int touchY = static_cast<int>(ty);

            if (kYesButton.contains(touchX, touchY)) {
                sendPowerOffConfirmToPowerTask();
                showPowerInfoScreen();
            } else if (kNoButton.contains(touchX, touchY)) {
                showPowerInfoScreen();
            }
        }

        state_.touchWasDown = touchDown;
    }

    osalQueue_t eventQueue_ = {};
    osalThread_t taskThread_ = {};
    bool queueReady_ = false;
    bool proximitySensorReady_ = false;
    State state_ = {};
};

UiController gUiController = {};
} // namespace

bool startUiTask()
{
    return gUiController.start();
}

bool uiPostEvent(const UiEvent& event)
{
    return gUiController.postEvent(event);
}

bool uiRequestDisplayPowerAndWait(UiDisplayPowerAction action, uint32_t timeout_ms)
{
    UiEvent event = {};
    event.type = UiEventType::DISPLAY_POWER;

    UiDisplayPowerEventData payload = {};
    payload.action = action;

    if (!uiSetEventData(event, payload) || !uiPostEvent(event)) {
        return false;
    }

    const uint8_t expectedState = (action == UiDisplayPowerAction::SLEEP) ? 1 : 0;
    const uint32_t startMs = M5.millis();
    while ((M5.millis() - startMs) < timeout_ms) {
        if (gDisplaySleepState.load(std::memory_order_acquire) == expectedState) {
            return true;
        }
        osalThreadSleepMs(10);
    }

    return gDisplaySleepState.load(std::memory_order_acquire) == expectedState;
}
