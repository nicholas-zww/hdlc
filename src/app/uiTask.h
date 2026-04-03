#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

enum class UiEventType : uint8_t
{
    POWER_INFO = 0,
};

struct UiPowerInfoEvent
{
    bool usb_powered;
    bool charge_enabled;
    bool battery_present;
    int8_t charge_phase;  // -1=discharging, 0=standby/full, 1=charging
    int16_t vbus_voltage_mv;
    int16_t battery_voltage_mv;
    int32_t battery_level_percent;
    int32_t battery_current_ma;
    int32_t charging_current_ma;
};

constexpr size_t UI_EVENT_DATA_MAX_SIZE = 64;

struct UiEventData
{
    uint16_t size;
    alignas(std::max_align_t) uint8_t bytes[UI_EVENT_DATA_MAX_SIZE];
};

struct UiEvent
{
    UiEventType type;
    UiEventData data;
};

template <typename T>
bool uiSetEventData(UiEvent& event, const T& value)
{
    static_assert(std::is_trivially_copyable<T>::value, "UiEvent data must be trivially copyable");
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
    static_assert(std::is_trivially_copyable<T>::value, "UiEvent data must be trivially copyable");
    if (event.data.size != sizeof(T)) {
        return nullptr;
    }
    return reinterpret_cast<const T*>(event.data.bytes);
}

bool startUiTask();
bool uiPostEvent(const UiEvent& event);
