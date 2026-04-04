#include "wifiManagerTask.h"

#include <cstring>
#include <cstdio>

#include <esp_err.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <nvs.h>
#include <nvs_flash.h>

#include "uiTask.h"
#include "osal/osal.h"

#define LOG_TAG "WifiTask"
#include "logger.h"

namespace
{
#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID ""
#endif

#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD ""
#endif

constexpr const char* kWifiNvsNamespace = "wifi_cfg";
constexpr const char* kWifiNvsKeySsid = "ssid";
constexpr const char* kWifiNvsKeyPassword = "pass";
constexpr size_t kWifiSsidMaxLen = 32;
constexpr size_t kWifiPasswordMaxLen = 64;

enum class WifiState : uint8_t
{
    Idle = 0,
    Connecting = 1,
    Connected = 2,
    Disconnected = 3,
    Reconnecting = 4,
};

class WifiManager
{
public:
    [[nodiscard]] bool start()
    {
        if (!initializeNvs()) {
            return false;
        }
        if (!initializeNetif()) {
            return false;
        }
        if (!initializeWifiDriver()) {
            return false;
        }

        LOGI("Wifi manager started");
        return true;
    }

    [[nodiscard]] bool setCredentials(const char* ssid, const char* password)
    {
        if (ssid == nullptr || password == nullptr) {
            return false;
        }

        const size_t ssidLen = std::strlen(ssid);
        const size_t passwordLen = std::strlen(password);
        if (ssidLen > kWifiSsidMaxLen || passwordLen > kWifiPasswordMaxLen) {
            LOGW("Rejected WIFI credentials: ssidLen=%u passwordLen=%u",
                 static_cast<unsigned>(ssidLen),
                 static_cast<unsigned>(passwordLen));
            return false;
        }

        if (!withCredentialLock([&]() {
                std::strncpy(staSsid_, ssid, sizeof(staSsid_) - 1);
                std::strncpy(staPassword_, password, sizeof(staPassword_) - 1);
                credentialsConfigured_ = staSsid_[0] != '\0';
            })) {
            LOGW("Failed to lock credential mutex for update");
            return false;
        }

        LOGI("WIFI credentials updated (ssidLen=%u); will save to NVS after successful connection",
             static_cast<unsigned>(ssidLen));

        if (wifiInitialized_) {
            if (!updatePendingCredentialsForSave(credentialsConfigured_)) {
                LOGW("Failed to mark pending WIFI credentials for NVS save");
                return false;
            }

            reconnectAttempt_ = 0;
            clearIpAddress();
            pendingCredentialApply_ = false;

            const esp_err_t stopErr = esp_wifi_stop();
            if (stopErr != ESP_OK) {
                LOGW("esp_wifi_stop returned: %d", static_cast<int>(stopErr));
            }

            const esp_err_t applyErr = applyCurrentCredentialsToDriver();
            if (applyErr != ESP_OK) {
                LOGW("Failed to apply new WIFI credentials: %d", static_cast<int>(applyErr));
                setState(WifiState::Disconnected, "apply credentials failed");
                return false;
            }

            if (!credentialsConfigured_) {
                setState(WifiState::Disconnected, "credentials cleared");
                return true;
            }

            const esp_err_t startErr = esp_wifi_start();
            if (startErr != ESP_OK) {
                LOGE("esp_wifi_start failed after credential update: %d", static_cast<int>(startErr));
                setState(WifiState::Disconnected, "wifi start failed");
                return false;
            }

            setState(WifiState::Reconnecting, "applying new credentials");
        } else if (!updatePendingCredentialsForSave(credentialsConfigured_)) {
            LOGW("Failed to mark pending WIFI credentials for NVS save");
            return false;
        }

        return true;
    }

private:
    static const char* stateToString(WifiState state)
    {
        switch (state) {
            case WifiState::Idle:
                return "idle";
            case WifiState::Connecting:
                return "connecting";
            case WifiState::Connected:
                return "connected";
            case WifiState::Disconnected:
                return "disconnected";
            case WifiState::Reconnecting:
                return "reconnecting";
            default:
                return "unknown";
        }
    }

    static UiWifiConnectionState toUiWifiState(WifiState state)
    {
        switch (state) {
            case WifiState::Idle:
                return UiWifiConnectionState::IDLE;
            case WifiState::Connecting:
                return UiWifiConnectionState::CONNECTING;
            case WifiState::Connected:
                return UiWifiConnectionState::CONNECTED;
            case WifiState::Disconnected:
                return UiWifiConnectionState::DISCONNECTED;
            case WifiState::Reconnecting:
                return UiWifiConnectionState::RECONNECTING;
            default:
                return UiWifiConnectionState::IDLE;
        }
    }

    static void eventHandlerThunk(void* arg, esp_event_base_t eventBase, int32_t eventId, void* eventData)
    {
        auto* self = static_cast<WifiManager*>(arg);
        if (self != nullptr) {
            self->handleEvent(eventBase, eventId, eventData);
        }
    }

    static bool espOkOrAlreadyInitialized(esp_err_t err)
    {
        return (err == ESP_OK) || (err == ESP_ERR_INVALID_STATE);
    }

    [[nodiscard]] bool initializeNvs()
    {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            LOGW("NVS requires erase; erasing and reinitializing");
            if (nvs_flash_erase() != ESP_OK) {
                LOGE("Failed to erase NVS");
                return false;
            }
            err = nvs_flash_init();
        }
        if (err != ESP_OK) {
            LOGE("Failed to init NVS: %d", static_cast<int>(err));
            return false;
        }
        return true;
    }

    [[nodiscard]] bool initializeNetif()
    {
        const esp_err_t netifErr = esp_netif_init();
        if (!espOkOrAlreadyInitialized(netifErr)) {
            LOGE("esp_netif_init failed: %d", static_cast<int>(netifErr));
            return false;
        }

        const esp_err_t loopErr = esp_event_loop_create_default();
        if (!espOkOrAlreadyInitialized(loopErr)) {
            LOGE("esp_event_loop_create_default failed: %d", static_cast<int>(loopErr));
            return false;
        }

        if (wifiStaNetif_ == nullptr) {
            wifiStaNetif_ = esp_netif_create_default_wifi_sta();
            if (wifiStaNetif_ == nullptr) {
                LOGE("Failed to create default wifi STA netif");
                return false;
            }
        }

        return true;
    }

    [[nodiscard]] bool initializeWifiDriver()
    {
        if (wifiInitialized_) {
            return true;
        }

        wifi_init_config_t initCfg = WIFI_INIT_CONFIG_DEFAULT();
        const esp_err_t initErr = esp_wifi_init(&initCfg);
        if (initErr != ESP_OK) {
            LOGE("esp_wifi_init failed: %d", static_cast<int>(initErr));
            return false;
        }

        const esp_err_t storageErr = esp_wifi_set_storage(WIFI_STORAGE_RAM);
        if (storageErr != ESP_OK) {
            LOGE("esp_wifi_set_storage failed: %d", static_cast<int>(storageErr));
            return false;
        }

        const esp_err_t modeErr = esp_wifi_set_mode(WIFI_MODE_STA);
        if (modeErr != ESP_OK) {
            LOGE("esp_wifi_set_mode failed: %d", static_cast<int>(modeErr));
            return false;
        }

        const bool loadedFromNvs = loadCredentialsFromNvs();
        if (loadedFromNvs) {
            LOGI("Using stored WIFI credentials from NVS at startup");
        } else {
            LOGI("No valid WIFI credentials found in NVS; using current defaults");
        }
        if (applyCurrentCredentialsToDriver() != ESP_OK) {
            return false;
        }

        const esp_err_t wifiEvtRegErr = esp_event_handler_instance_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &WifiManager::eventHandlerThunk,
            this,
            &wifiEventHandler_);
        if (wifiEvtRegErr != ESP_OK) {
            LOGE("Failed to register WIFI_EVENT handler: %d", static_cast<int>(wifiEvtRegErr));
            return false;
        }

        const esp_err_t ipEvtRegErr = esp_event_handler_instance_register(
            IP_EVENT,
            ESP_EVENT_ANY_ID,
            &WifiManager::eventHandlerThunk,
            this,
            &ipEventHandler_);
        if (ipEvtRegErr != ESP_OK) {
            LOGE("Failed to register IP_EVENT handler: %d", static_cast<int>(ipEvtRegErr));
            return false;
        }

        const esp_err_t startErr = esp_wifi_start();
        if (startErr != ESP_OK) {
            LOGE("esp_wifi_start failed: %d", static_cast<int>(startErr));
            return false;
        }

        wifiInitialized_ = true;
        setState(WifiState::Idle, "wifi started");
        return true;
    }

    template <typename Fn>
    bool withCredentialLock(Fn&& fn)
    {
        if (!credentialMutexReady_) {
            if (osalMutexInit(&credentialMutex_) != OSAL_STATUS_OK) {
                return false;
            }
            credentialMutexReady_ = true;
        }

        if (osalMutexLock(&credentialMutex_, OSAL_WAIT_FOREVER) != OSAL_STATUS_OK) {
            return false;
        }

        fn();
        (void)osalMutexUnlock(&credentialMutex_);
        return true;
    }

    [[nodiscard]] bool loadCredentialsFromNvs()
    {
        nvs_handle_t nvs = 0;
        const esp_err_t openErr = nvs_open(kWifiNvsNamespace, NVS_READONLY, &nvs);
        if (openErr != ESP_OK) {
            return false;
        }

        char ssid[kWifiSsidMaxLen + 1] = {};
        size_t ssidLen = sizeof(ssid);
        const esp_err_t ssidErr = nvs_get_str(nvs, kWifiNvsKeySsid, ssid, &ssidLen);

        char password[kWifiPasswordMaxLen + 1] = {};
        size_t passwordLen = sizeof(password);
        const esp_err_t passErr = nvs_get_str(nvs, kWifiNvsKeyPassword, password, &passwordLen);
        nvs_close(nvs);

        if (ssidErr != ESP_OK || passErr != ESP_OK) {
            return false;
        }

        if (!withCredentialLock([&]() {
                std::strncpy(staSsid_, ssid, sizeof(staSsid_) - 1);
                std::strncpy(staPassword_, password, sizeof(staPassword_) - 1);
                credentialsConfigured_ = staSsid_[0] != '\0';
            })) {
            LOGW("Failed to lock credential mutex while loading NVS credentials");
            return false;
        }

        LOGI("Loaded WIFI credentials from NVS");
        return true;
    }

    [[nodiscard]] bool saveCredentialsToNvs(const char* ssid, const char* password)
    {
        nvs_handle_t nvs = 0;
        const esp_err_t openErr = nvs_open(kWifiNvsNamespace, NVS_READWRITE, &nvs);
        if (openErr != ESP_OK) {
            LOGE("nvs_open failed for wifi cfg: %d", static_cast<int>(openErr));
            return false;
        }

        esp_err_t err = nvs_set_str(nvs, kWifiNvsKeySsid, ssid);
        if (err == ESP_OK) {
            err = nvs_set_str(nvs, kWifiNvsKeyPassword, password);
        }
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
        }

        nvs_close(nvs);
        if (err != ESP_OK) {
            LOGE("Failed to save WIFI credentials to NVS: %d", static_cast<int>(err));
            return false;
        }
        return true;
    }

    [[nodiscard]] bool updatePendingCredentialsForSave(bool enabled)
    {
        return withCredentialLock([&]() {
            pendingCredentialSave_ = enabled;
            if (enabled) {
                std::strncpy(pendingSsid_, staSsid_, sizeof(pendingSsid_) - 1);
                std::strncpy(pendingPassword_, staPassword_, sizeof(pendingPassword_) - 1);
            } else {
                pendingSsid_[0] = '\0';
                pendingPassword_[0] = '\0';
            }
        });
    }

    void savePendingCredentialsIfAny()
    {
        char pendingSsid[kWifiSsidMaxLen + 1] = {};
        char pendingPassword[kWifiPasswordMaxLen + 1] = {};
        bool hasPending = false;

        if (!withCredentialLock([&]() {
                hasPending = pendingCredentialSave_;
                if (hasPending) {
                    std::strncpy(pendingSsid, pendingSsid_, sizeof(pendingSsid) - 1);
                    std::strncpy(pendingPassword, pendingPassword_, sizeof(pendingPassword) - 1);
                }
            })) {
            LOGW("Failed to lock credential mutex while reading pending WIFI credentials");
            return;
        }

        if (!hasPending) {
            return;
        }

        if (!saveCredentialsToNvs(pendingSsid, pendingPassword)) {
            LOGW("Pending WIFI credentials not saved to NVS yet");
            return;
        }

        (void)withCredentialLock([&]() {
            pendingCredentialSave_ = false;
            pendingSsid_[0] = '\0';
            pendingPassword_[0] = '\0';
        });
        LOGI("Saved new WIFI credentials to NVS after successful connection");
    }

    [[nodiscard]] esp_err_t applyCurrentCredentialsToDriver()
    {
        wifi_config_t wifiCfg = {};
        if (!withCredentialLock([&]() {
                std::strncpy(reinterpret_cast<char*>(wifiCfg.sta.ssid), staSsid_, sizeof(wifiCfg.sta.ssid) - 1);
                std::strncpy(reinterpret_cast<char*>(wifiCfg.sta.password), staPassword_, sizeof(wifiCfg.sta.password) - 1);
            })) {
            LOGE("Failed to lock credential mutex while applying config");
            return ESP_FAIL;
        }

        wifiCfg.sta.failure_retry_cnt = 1;
        wifiCfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        wifiCfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        wifiCfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
        wifiCfg.sta.pmf_cfg.capable = true;
        wifiCfg.sta.pmf_cfg.required = false;

        credentialsConfigured_ = wifiCfg.sta.ssid[0] != '\0';
        if (!credentialsConfigured_) {
            LOGW("WIFI STA credentials are empty. Use runtime API or build flags.");
            return ESP_OK;
        }

        const esp_err_t cfgErr = esp_wifi_set_config(WIFI_IF_STA, &wifiCfg);
        if (cfgErr != ESP_OK) {
            LOGE("esp_wifi_set_config failed: %d", static_cast<int>(cfgErr));
            return cfgErr;
        }

        LOGI("WIFI STA configured for SSID: %s", reinterpret_cast<char*>(wifiCfg.sta.ssid));
        return ESP_OK;
    }

    void setState(WifiState newState, const char* reason)
    {
        if (state_ == newState) {
            return;
        }
        LOGI("Wifi state: %s -> %s (%s)", stateToString(state_), stateToString(newState), reason);
        state_ = newState;
        publishUiWifiStatus();
    }

    void clearIpAddress()
    {
        ipAddress_[0] = '\0';
    }

    void setIpAddress(const esp_ip4_addr_t& ip)
    {
        std::snprintf(ipAddress_, sizeof(ipAddress_), IPSTR, IP2STR(&ip));
    }

    void publishUiWifiStatus()
    {
        UiWifiStatusEventData data = {};
        data.state = toUiWifiState(state_);
        std::strncpy(data.ip_address, ipAddress_, sizeof(data.ip_address) - 1);

        UiEvent event = {};
        event.type = UiEventType::WIFI_STATUS;
        if (!uiSetEventData(event, data) || !uiPostEvent(event)) {
            LOGW("Failed to publish WIFI_STATUS event to UI");
        }
    }

    void requestConnect(bool reconnect)
    {
        if (!credentialsConfigured_) {
            setState(WifiState::Disconnected, "missing credentials");
            return;
        }

        const esp_err_t connectErr = esp_wifi_connect();
        if (connectErr == ESP_OK) {
            setState(reconnect ? WifiState::Reconnecting : WifiState::Connecting,
                     reconnect ? "retry connect" : "start connect");
            return;
        }

        // ESP_ERR_WIFI_CONN means station is already connecting/connected.
        if (connectErr == ESP_ERR_WIFI_CONN) {
            LOGW("esp_wifi_connect: station already connecting/connected");
            setState(WifiState::Connecting, "connect already in progress");
            return;
        }

        LOGW("esp_wifi_connect failed: %d", static_cast<int>(connectErr));
        setState(WifiState::Disconnected, "connect request failed");
    }

    void handleWifiDisconnected(void* eventData)
    {
        wifi_event_sta_disconnected_t* disconnected =
            static_cast<wifi_event_sta_disconnected_t*>(eventData);
        if (disconnected != nullptr) {
            LOGW("STA disconnected, reason=%d", static_cast<int>(disconnected->reason));
        }

        clearIpAddress();

        if (pendingCredentialApply_) {
            const esp_err_t applyErr = applyCurrentCredentialsToDriver();
            if (applyErr == ESP_OK) {
                pendingCredentialApply_ = false;
                requestConnect(false);
                return;
            }

            if (applyErr == ESP_ERR_WIFI_STATE) {
                setState(WifiState::Reconnecting, "waiting to apply credentials");
                return;
            }

            pendingCredentialApply_ = false;
            setState(WifiState::Disconnected, "apply credentials failed");
            return;
        }

        setState(WifiState::Disconnected, "link lost");

        ++reconnectAttempt_;
        LOGI("Reconnect attempt=%lu", static_cast<unsigned long>(reconnectAttempt_));
        requestConnect(true);
    }

    void handleEvent(esp_event_base_t eventBase, int32_t eventId, void* eventData)
    {
        if (eventBase == WIFI_EVENT) {
            if (eventId == WIFI_EVENT_STA_START) {
                reconnectAttempt_ = 0;
                requestConnect(false);
            } else if (eventId == WIFI_EVENT_STA_CONNECTED) {
                setState(WifiState::Connecting, "sta connected, waiting ip");
            } else if (eventId == WIFI_EVENT_STA_DISCONNECTED) {
                handleWifiDisconnected(eventData);
            } else if (eventId == WIFI_EVENT_STA_STOP) {
                setState(WifiState::Idle, "wifi stop");
            }
            return;
        }

        if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP) {
            reconnectAttempt_ = 0;
            ip_event_got_ip_t* gotIp = static_cast<ip_event_got_ip_t*>(eventData);
            if (gotIp != nullptr) {
                setIpAddress(gotIp->ip_info.ip);
                LOGI("STA got ip: " IPSTR, IP2STR(&gotIp->ip_info.ip));
            }
            setState(WifiState::Connected, "got ip");
            savePendingCredentialsIfAny();
            return;
        }

        if (eventBase == IP_EVENT && eventId == IP_EVENT_STA_LOST_IP) {
            LOGW("STA lost IP");
            clearIpAddress();
            setState(WifiState::Disconnected, "lost ip");
            requestConnect(true);
        }
    }

    esp_netif_t* wifiStaNetif_ = nullptr;
    esp_event_handler_instance_t wifiEventHandler_ = nullptr;
    esp_event_handler_instance_t ipEventHandler_ = nullptr;
    WifiState state_ = WifiState::Idle;
    uint32_t reconnectAttempt_ = 0;
    bool wifiInitialized_ = false;
    bool credentialsConfigured_ = false;
    char ipAddress_[16] = {};
    osalMutex_t credentialMutex_ = {};
    bool credentialMutexReady_ = false;
    char staSsid_[kWifiSsidMaxLen + 1] = WIFI_STA_SSID;
    char staPassword_[kWifiPasswordMaxLen + 1] = WIFI_STA_PASSWORD;
    bool pendingCredentialSave_ = false;
    char pendingSsid_[kWifiSsidMaxLen + 1] = {};
    char pendingPassword_[kWifiPasswordMaxLen + 1] = {};
    bool pendingCredentialApply_ = false;
};

WifiManager gWifiManager = {};
} // namespace

bool startWifiManagerTask()
{
    return gWifiManager.start();
}

bool wifiSetCredentials(const char* ssid, const char* password)
{
    return gWifiManager.setCredentials(ssid, password);
}
