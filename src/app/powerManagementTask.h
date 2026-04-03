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
    PowerKeyAction action;
};

struct PowerEvent
{
    PowerEventType type;
    PowerKeyEventData key_event;
};

bool startPowerManagementTask();
bool postPowerEvent(const PowerEvent& event);
