#include "uiTask.h"
#include "powerManagementTask.h"
#include "osal/osal.h"
#include <M5Unified.h>

#define LOG_TAG "UiTask"
#include "logger.h"

namespace
{
constexpr size_t UI_EVENT_QUEUE_CAPACITY = 16;
constexpr uint32_t UI_LOOP_MS = 50;
static osalQueue_t uiEventQueue = {};
static osalThread_t uiTaskThread = {};
static bool uiQueueReady = false;

struct Rect
{
    int x;
    int y;
    int w;
    int h;
};

constexpr Rect YES_BTN = { 40, 170, 100, 44 };
constexpr Rect NO_BTN  = { 180, 170, 100, 44 };

const char* chargePhaseToString(int8_t phase)
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

bool pointInRect(int x, int y, const Rect& r)
{
    return (x >= r.x && x < (r.x + r.w) && y >= r.y && y < (r.y + r.h));
}

void renderPowerInfoToLcd(const UiPowerInfoEvent& info)
{
    constexpr uint32_t COLOR_BG = 0x0000;
    constexpr uint32_t COLOR_FG = 0xFFFF;
    constexpr int ORIGIN_X = 8;
    constexpr int ORIGIN_Y = 8;
    constexpr int LINE_H = 18;

    M5.Display.startWrite();
    M5.Display.fillScreen(COLOR_BG);
    M5.Display.setTextColor(COLOR_FG, COLOR_BG);
    M5.Display.setTextSize(1);

    int y = ORIGIN_Y;
    M5.Display.setCursor(ORIGIN_X, y);
    M5.Display.printf("Power: %s", info.usb_powered ? "USB" : "BATTERY");
    y += LINE_H;

    M5.Display.setCursor(ORIGIN_X, y);
    M5.Display.printf("Phase: %s", chargePhaseToString(info.charge_phase));
    y += LINE_H;

    M5.Display.setCursor(ORIGIN_X, y);
    M5.Display.printf("Charge Enable: %s", info.charge_enabled ? "YES" : "NO");
    y += LINE_H;

    M5.Display.setCursor(ORIGIN_X, y);
    M5.Display.printf("Battery: %ld%%  %dmV",
                      static_cast<long>(info.battery_level_percent),
                      static_cast<int>(info.battery_voltage_mv));
    y += LINE_H;

    M5.Display.setCursor(ORIGIN_X, y);
    M5.Display.printf("VBUS: %dmV", static_cast<int>(info.vbus_voltage_mv));

    M5.Display.endWrite();
}

void renderPowerOffConfirm()
{
    constexpr uint32_t COLOR_BG = 0x0000;
    constexpr uint32_t COLOR_FG = 0xFFFF;

    M5.Display.startWrite();
    M5.Display.fillScreen(COLOR_BG);
    M5.Display.setTextColor(COLOR_FG, COLOR_BG);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(16, 36);
    M5.Display.print("Power Off?");
    M5.Display.setTextSize(1);
    M5.Display.setCursor(16, 80);
    M5.Display.print("Hold power key detected.");
    M5.Display.setCursor(16, 100);
    M5.Display.print("Do you want to shut down?");

    M5.Display.drawRect(YES_BTN.x, YES_BTN.y, YES_BTN.w, YES_BTN.h, COLOR_FG);
    M5.Display.drawRect(NO_BTN.x, NO_BTN.y, NO_BTN.w, NO_BTN.h, COLOR_FG);
    M5.Display.setCursor(YES_BTN.x + 34, YES_BTN.y + 14);
    M5.Display.print("YES");
    M5.Display.setCursor(NO_BTN.x + 38, NO_BTN.y + 14);
    M5.Display.print("NO");
    M5.Display.endWrite();
}

void sendPowerOffConfirmToPowerTask()
{
    PowerEvent event = {};
    event.type = PowerEventType::KEY_EVENT;
    event.key_event.action = PowerKeyAction::POWEROFF_CONFIRM_YES;
    if (!postPowerEvent(event)) {
        LOGW("Failed to post power-off confirm event");
    }
}

void uiTask(void* context)
{
    (void)context;

    UiEvent event = {};
    UiPowerInfoEvent latestPowerInfo = {};
    bool hasPowerInfo = false;
    bool showPowerOffConfirmScreen = false;
    bool touchWasDown = false;

    while (true) {
        if (osalQueueReceive(&uiEventQueue, &event, UI_LOOP_MS) == OSAL_STATUS_OK) {
            switch (event.type) {
                case UiEventType::POWER_INFO:
                {
                    const UiPowerInfoEvent* powerInfo = uiGetEventData<UiPowerInfoEvent>(event);
                    if (powerInfo == nullptr) {
                        LOGW("UI power event has invalid payload size");
                        break;
                    }

                    latestPowerInfo = *powerInfo;
                    hasPowerInfo = true;
                    if (!showPowerOffConfirmScreen) {
                        renderPowerInfoToLcd(latestPowerInfo);
                    }
                    break;
                }
                case UiEventType::KEY_EVENT:
                {
                    const UiKeyEventData* keyEvent = uiGetEventData<UiKeyEventData>(event);
                    if (keyEvent == nullptr) {
                        LOGW("UI key event has invalid payload size");
                        break;
                    }
                    if (keyEvent->key_id == UiKeyId::POWER
                        && keyEvent->action == UiKeyAction::LONG_PRESS) {
                        showPowerOffConfirmScreen = true;
                        renderPowerOffConfirm();
                    }
                    break;
                }
                default:
                    LOGW("UI received unknown event type");
                    break;
            }
        }

        if (!showPowerOffConfirmScreen) {
            touchWasDown = false;
            continue;
        }

        int32_t tx = 0;
        int32_t ty = 0;
        const bool touchDown = M5.Display.getTouch(&tx, &ty) > 0;
        if (touchDown && !touchWasDown) {
            if (pointInRect(static_cast<int>(tx), static_cast<int>(ty), YES_BTN)) {
                sendPowerOffConfirmToPowerTask();
                showPowerOffConfirmScreen = false;
                if (hasPowerInfo) {
                    renderPowerInfoToLcd(latestPowerInfo);
                }
            } else if (pointInRect(static_cast<int>(tx), static_cast<int>(ty), NO_BTN)) {
                showPowerOffConfirmScreen = false;
                if (hasPowerInfo) {
                    renderPowerInfoToLcd(latestPowerInfo);
                }
            }
        }
        touchWasDown = touchDown;
    }
}
} // namespace

bool startUiTask()
{
    if (!uiQueueReady) {
        if (osalQueueInit(&uiEventQueue, sizeof(UiEvent), UI_EVENT_QUEUE_CAPACITY) != OSAL_STATUS_OK) {
            LOGE("Failed to create UI event queue");
            return false;
        }
        uiQueueReady = true;
    }

    const osalThreadAttr_t uiTaskAttr = {
        .stack_size_bytes = 4096,
        .priority = 7,
        .detached = true,
        .name = "UiTask",
    };

    if (osalThreadCreate(&uiTaskThread, uiTask, nullptr, &uiTaskAttr) != OSAL_STATUS_OK) {
        LOGE("Failed to create UiTask");
        return false;
    }

    LOGI("UiTask started");
    return true;
}

bool uiPostEvent(const UiEvent& event)
{
    if (!uiQueueReady) {
        return false;
    }
    return osalQueueSend(&uiEventQueue, &event, 0) == OSAL_STATUS_OK;
}
