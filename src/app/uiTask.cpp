#include "uiTask.h"
#include "powerManagementTask.h"
#include "osal/osal.h"

#include <M5Unified.h>
#include <cstdio>
#include <cstring>

#define LOG_TAG "UiTask"
#include "logger.h"

namespace
{
constexpr size_t kEventQueueCapacity = 16;
constexpr uint32_t kLoopMs = 50;
constexpr uint32_t kColorBackground = 0x0000;
constexpr uint32_t kColorForeground = 0xFFFF;
constexpr int kOriginX = 8;
constexpr int kOriginY = 8;
constexpr int kLineHeight = 18;

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

class UiController
{
public:
    [[nodiscard]] bool start()
    {
        if (!ensureQueueReady()) {
            return false;
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

        UiWakeupEventData latestWakeup = {};
        bool hasWakeup = false;

        UiWifiStatusEventData latestWifiStatus = {};
        bool hasWifiStatus = false;

        UiPowerInfoEvent renderedPowerInfo = {};
        bool renderedPowerInfoValid = false;
        UiWakeupEventData renderedWakeup = {};
        bool renderedWakeupValid = false;
        UiWifiStatusEventData renderedWifiStatus = {};
        bool renderedWifiStatusValid = false;
        bool renderedPowerInfoScreen = false;

        Screen screen = Screen::PowerInfo;
        bool touchWasDown = false;
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
        UiEvent event = {};

        while (true) {
            if (osalQueueReceive(&eventQueue_, &event, kLoopMs) == OSAL_STATUS_OK) {
                handleEvent(event);
            }

            handlePowerOffConfirmTouch();
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

    static void sendPowerOffConfirmToPowerTask()
    {
        PowerEvent event = {};
        event.type = PowerEventType::KEY_EVENT;
        event.key_event.action = PowerKeyAction::POWEROFF_CONFIRM_YES;
        if (!postPowerEvent(event)) {
            LOGW("Failed to post power-off confirm event");
        }
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

    void renderPowerInfo(const UiPowerInfoEvent& info,
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

        const bool wakeChanged = wakeupChanged(wakeup, state_.renderedWakeupValid, state_.renderedWakeup);
        const bool wifiStatusChanged = wifiChanged(wifiStatus,
                                                   state_.renderedWifiStatusValid,
                                                   state_.renderedWifiStatus);

        if (!forceFullRedraw && !powerChanged && !wakeChanged && !wifiStatusChanged) {
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

        if (forceFullRedraw || wakeChanged) {
            formatWakeLine(line, sizeof(line), wakeup);
            drawPowerInfoLine(5, line);

            formatSleepErrorLine(line, sizeof(line), wakeup);
            drawPowerInfoLine(6, line);
        }

        if (forceFullRedraw || wifiStatusChanged) {
            formatWifiLine(line, sizeof(line), wifiStatus);
            drawPowerInfoLine(7, line);
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
    }

    void resetPowerInfoRenderCache()
    {
        state_.renderedPowerInfo = {};
        state_.renderedPowerInfoValid = false;
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

        const UiWakeupEventData* wakeup = state_.hasWakeup ? &state_.latestWakeup : nullptr;
        const UiWifiStatusEventData* wifiStatus = state_.hasWifiStatus ? &state_.latestWifiStatus : nullptr;
        const bool forceFullRedraw = !state_.renderedPowerInfoScreen;
        renderPowerInfo(state_.latestPowerInfo, wakeup, wifiStatus, forceFullRedraw);
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

            default:
                LOGW("UI received unknown event type");
                break;
        }
    }

    void handlePowerOffConfirmTouch()
    {
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
