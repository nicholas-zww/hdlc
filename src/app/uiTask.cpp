#include "uiTask.h"
#include "osal/osal.h"
#include <M5Unified.h>

#define LOG_TAG "UiTask"
#include "logger.h"

namespace
{
constexpr size_t UI_EVENT_QUEUE_CAPACITY = 16;
static osalQueue_t uiEventQueue = {};
static osalThread_t uiTaskThread = {};
static bool uiQueueReady = false;

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

void uiTask(void* context)
{
    (void)context;
    UiEvent event = {};

    while (true) {
        if (osalQueueReceive(&uiEventQueue, &event, OSAL_WAIT_FOREVER) != OSAL_STATUS_OK) {
            continue;
        }

        switch (event.type) {
            case UiEventType::POWER_INFO:
            {
                const UiPowerInfoEvent* powerInfo = uiGetEventData<UiPowerInfoEvent>(event);
                if (powerInfo == nullptr) {
                    LOGW("UI power event has invalid payload size");
                    break;
                }

                renderPowerInfoToLcd(*powerInfo);
                break;
            }
            default:
                LOGW("UI received unknown event type");
                break;
        }
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
