#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

enum class UiEventType : uint8_t
{
    POWER_INFO = 0,
    KEY_EVENT = 1,
    WAKEUP_EVENT = 2,
    WIFI_STATUS = 3,
    DISPLAY_POWER = 4,
};

enum class UiKeyId : uint8_t
{
    POWER = 0,
};

enum class UiKeyAction : uint8_t
{
    LONG_PRESS = 0,
};

struct UiPowerInfoEvent
{
    bool usb_powered = false;
    bool charge_enabled = false;
    bool battery_present = false;
    int8_t charge_phase = 0;  // -1=discharging, 0=standby/full, 1=charging
    int16_t vbus_voltage_mv = 0;
    int16_t battery_voltage_mv = 0;
    int32_t battery_level_percent = 0;
    int32_t battery_current_ma = 0;
    int32_t charging_current_ma = 0;
};

struct UiKeyEventData
{
    UiKeyId key_id = UiKeyId::POWER;
    UiKeyAction action = UiKeyAction::LONG_PRESS;
};

enum class UiWakeupSource : uint8_t
{
    UNKNOWN = 0,
    GPIO = 1,
    TIMER = 2,
    OTHER = 3,
};

struct UiWakeupEventData
{
    UiWakeupSource source = UiWakeupSource::UNKNOWN;
    int32_t wakeup_cause = 0;
    int32_t sleep_error = 0;
};

enum class UiWifiConnectionState : uint8_t
{
    IDLE = 0,
    CONNECTING = 1,
    CONNECTED = 2,
    DISCONNECTED = 3,
    RECONNECTING = 4,
};

struct UiWifiStatusEventData
{
    UiWifiConnectionState state = UiWifiConnectionState::IDLE;
    char ip_address[16] = {};
};

enum class UiDisplayPowerAction : uint8_t
{
    SLEEP = 0,
    WAKE = 1,
};

struct UiDisplayPowerEventData
{
    UiDisplayPowerAction action = UiDisplayPowerAction::WAKE;
};

constexpr size_t UI_EVENT_DATA_MAX_SIZE = 64;

struct UiEventData
{
    uint16_t size = 0;
    alignas(std::max_align_t) uint8_t bytes[UI_EVENT_DATA_MAX_SIZE] = {};
};

struct UiEvent
{
    UiEventType type = UiEventType::POWER_INFO;
    UiEventData data = {};
};

template <typename T>
bool uiSetEventData(UiEvent& event, const T& value)
{
    static_assert(std::is_trivially_copyable_v<T>, "UiEvent data must be trivially copyable");
    if (sizeof(T) > UI_EVENT_DATA_MAX_SIZE) {
        return false;
    }
    event.data.size = static_cast<uint16_t>(sizeof(T));
    std::memcpy(event.data.bytes, &value, sizeof(T));
    return true;
}

template <typename T>
const T* uiGetEventData(const UiEvent& event)
{
    static_assert(std::is_trivially_copyable_v<T>, "UiEvent data must be trivially copyable");
    static thread_local T value = {};
    if (event.data.size != sizeof(T)) {
        return nullptr;
    }
    std::memcpy(&value, event.data.bytes, sizeof(T));
    return &value;
}

template <typename T>
bool uiTryGetEventData(const UiEvent& event, T& out)
{
    static_assert(std::is_trivially_copyable_v<T>, "UiEvent data must be trivially copyable");
    if (event.data.size != sizeof(T)) {
        return false;
    }
    std::memcpy(&out, event.data.bytes, sizeof(T));
    return true;
}

[[nodiscard]] bool startUiTask();
[[nodiscard]] bool uiPostEvent(const UiEvent& event);
[[nodiscard]] bool uiRequestDisplayPowerAndWait(UiDisplayPowerAction action, uint32_t timeout_ms);
