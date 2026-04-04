#pragma once

#include <cstdint>

enum class PowerEventType : uint8_t
{
    KEY_EVENT = 0,
};

enum class PowerKeyAction : uint8_t
{
    POWEROFF_CONFIRM_YES = 0,
};

struct PowerKeyEventData
{
    PowerKeyAction action = PowerKeyAction::POWEROFF_CONFIRM_YES;
};

struct PowerEvent
{
    PowerEventType type = PowerEventType::KEY_EVENT;
    PowerKeyEventData key_event = {};
};

[[nodiscard]] bool startPowerManagementTask();
[[nodiscard]] bool postPowerEvent(const PowerEvent& event);
