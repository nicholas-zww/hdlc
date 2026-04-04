#pragma once

#include <cstdint>

enum class PowerEventType : uint8_t
{
    KEY_EVENT = 0,
    IDLE_POLICY_UPDATE = 1,
};

enum class PowerKeyAction : uint8_t
{
    POWEROFF_CONFIRM_YES = 0,
};

struct PowerKeyEventData
{
    PowerKeyAction action = PowerKeyAction::POWEROFF_CONFIRM_YES;
};

struct PowerIdlePolicyEventData
{
    bool allow_low_power = false;
};

struct PowerEvent
{
    PowerEventType type = PowerEventType::KEY_EVENT;
    PowerKeyEventData key_event = {};
    PowerIdlePolicyEventData idle_policy_event = {};
};

[[nodiscard]] bool startPowerManagementTask();
[[nodiscard]] bool postPowerEvent(const PowerEvent& event);
