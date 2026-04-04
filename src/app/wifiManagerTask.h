#pragma once

#include <cstddef>

[[nodiscard]] bool startWifiManagerTask();
[[nodiscard]] bool wifiSetCredentials(const char* ssid, const char* password);
