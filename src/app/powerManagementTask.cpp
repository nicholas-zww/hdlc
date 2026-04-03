#include <cstdint>
#include "osal/osal.h"
#include <M5Unified.h>
#include "uiTask.h"

#define LOG_TAG "PowerTask"
#include "logger.h"

namespace
{
constexpr uint32_t POWER_TASK_PERIOD_MS = 2000;
static osalThread_t powerTaskThread = {};
constexpr uint8_t AW9523_ADDR = 0x58;
constexpr uint8_t AW9523_PORT0_REG = 0x02;
constexpr uint8_t CORE_S3_BUS_EN_BIT = 0x02;
constexpr uint8_t CORE_S3_USB_EN_BIT = 0x20;
constexpr uint8_t CORE_S3_BOOST_EN_BIT = 0x80;
constexpr uint8_t AXP2101_IRQEN1_REG = 0x41;
constexpr uint8_t AXP2101_VBUS_IRQ_MASK = 0xC0; // bit7:insert, bit6:remove
constexpr uint32_t POWER_I2C_FREQ = 100000;

const char* axp2101EffectiveState(bool vbusPresent, bool chargeEnabled, int chargeState)
{
    if (!vbusPresent) {
        return "battery_discharging";
    }
    if (chargeState == 1) {
        return "usb_charging";
    }
    if (chargeEnabled) {
        return "usb_charge_idle_or_full";
    }
    return "usb_charge_disabled";
}

const char* stablePowerState(bool vbusPresent, bool chargeEnabled)
{
    if (!vbusPresent) {
        return "battery_powered";
    }
    if (chargeEnabled) {
        return "usb_powered";
    }
    return "usb_powered_charge_disabled";
}

int16_t quantizeMv(int16_t mv)
{
    if (mv < 0) {
        return mv;
    }
    return static_cast<int16_t>((mv / 10) * 10);
}

void forceDisableCoreS3BackPower()
{
    uint8_t regs[2] = {};
    if (M5.In_I2C.readRegister(AW9523_ADDR, AW9523_PORT0_REG, regs, sizeof(regs), POWER_I2C_FREQ)) {
        regs[0] &= static_cast<uint8_t>(~(CORE_S3_BUS_EN_BIT | CORE_S3_USB_EN_BIT));
        regs[1] &= static_cast<uint8_t>(~CORE_S3_BOOST_EN_BIT);
        M5.In_I2C.writeRegister(AW9523_ADDR, AW9523_PORT0_REG, regs, sizeof(regs), POWER_I2C_FREQ);
    }
}

void powerTask(void* context)
{
    (void)context;

    bool usbCableConnected = M5.Power.Axp2101.isVBUS();
    uint8_t dischargeWhileVbusCount = 0;

    while (true) {
        // Keep CoreS3 boost/output paths hard-disabled so VBUS sensing reflects external USB only.
        forceDisableCoreS3BackPower();

        const uint64_t irqStatus = M5.Power.Axp2101.getIRQStatuses();
        const bool vbusInsertIrq = (irqStatus & static_cast<uint64_t>(m5::AXP2101_IRQ_VBUS_INSERT)) != 0;
        const bool vbusRemoveIrq = (irqStatus & static_cast<uint64_t>(m5::AXP2101_IRQ_VBUS_REMOVE)) != 0;
        M5.Power.Axp2101.clearIRQStatuses();

        if (vbusInsertIrq && !vbusRemoveIrq) {
            usbCableConnected = true;
        } else if (vbusRemoveIrq && !vbusInsertIrq) {
            usbCableConnected = false;
        }

        const int32_t batteryLevel = M5.Power.getBatteryLevel();
        const int32_t batteryCurrentMa = M5.Power.getBatteryCurrent();
        const int32_t chargingCurrentMa = (batteryCurrentMa > 0) ? batteryCurrentMa : 0;
        const int16_t batteryVoltageMv = quantizeMv(M5.Power.getBatteryVoltage());
        const int16_t vbusVoltageMv = quantizeMv(M5.Power.getVBUSVoltage());

        const bool vbusPresentRaw = M5.Power.Axp2101.isVBUS();
        const bool batteryPresent = M5.Power.Axp2101.getBatState();
        const int chargeState = M5.Power.Axp2101.getChargeStatus();
        const uint8_t reg18 = M5.Power.Axp2101.readRegister8(0x18);
        const bool chargeEnabled = (reg18 & 0x02) != 0;

        if (vbusInsertIrq && !vbusRemoveIrq) {
            usbCableConnected = true;
        } else if (vbusRemoveIrq && !vbusInsertIrq) {
            usbCableConnected = false;
        } else if (!vbusPresentRaw) {
            usbCableConnected = false;
        }

        // Fallback for boards where VBUS level can stay high due backfeed:
        // sustained discharge state implies USB input is not actually supplying power.
        if (chargeState == -1 && vbusPresentRaw) {
            if (dischargeWhileVbusCount < 255) {
                dischargeWhileVbusCount++;
            }
            if (dischargeWhileVbusCount >= 2) { // 2 consecutive cycles (~4s)
                usbCableConnected = false;
            }
        } else {
            dischargeWhileVbusCount = 0;
            if (chargeState >= 0) {
                usbCableConnected = true;
            }
        }

        const bool vbusPresent = usbCableConnected;
        const char* powerState = stablePowerState(vbusPresent, chargeEnabled);
        const char* effectiveState = axp2101EffectiveState(vbusPresent, chargeEnabled, chargeState);

        UiPowerInfoEvent powerInfo = {};
        powerInfo.usb_powered = vbusPresent;
        powerInfo.charge_enabled = chargeEnabled;
        powerInfo.battery_present = batteryPresent;
        powerInfo.charge_phase = static_cast<int8_t>(chargeState);
        powerInfo.vbus_voltage_mv = vbusVoltageMv;
        powerInfo.battery_voltage_mv = batteryVoltageMv;
        powerInfo.battery_level_percent = batteryLevel;
        powerInfo.battery_current_ma = batteryCurrentMa;
        powerInfo.charging_current_ma = chargingCurrentMa;

        UiEvent uiEvent = {};
        uiEvent.type = UiEventType::POWER_INFO;
        if (!uiSetEventData(uiEvent, powerInfo)) {
            LOGW("UI power payload too large; dropped power event");
        } else if (!uiPostEvent(uiEvent)) {
            LOGW("UI queue full or not ready; dropped power event");
        }

        LOGI("PMIC=AXP2101 power=%s phase=%s, raw_vbus=%dmV, bat_present=%d bat=%ld%%(%dmV)",
                powerState,
                effectiveState,
                static_cast<int>(vbusVoltageMv),
                static_cast<int>(batteryPresent),
                static_cast<long>(batteryLevel),
                static_cast<int>(batteryVoltageMv));

        osalThreadSleepMs(POWER_TASK_PERIOD_MS);
    }
}
} // namespace

bool startPowerManagementTask()
{
    auto cfg = M5.config();
    cfg.clear_display = false;
    cfg.fallback_board = m5::board_t::board_M5StackCoreS3;
    cfg.output_power = false;
    M5.begin(cfg);

    forceDisableCoreS3BackPower();
    M5.Power.setExtOutput(false);
    M5.Power.setUsbOutput(false);
    M5.Power.setBatteryCharge(true);
    M5.Power.setChargeCurrent(500);
    M5.Power.setChargeVoltage(4200);
    // M5Unified AXP2101 enableIRQ() has unreliable return value in this version.
    // Enable VBUS insert/remove IRQs directly via IRQEN1 register.
    const uint8_t irqen1 = M5.Power.Axp2101.readRegister8(AXP2101_IRQEN1_REG);
    M5.Power.Axp2101.writeRegister8(AXP2101_IRQEN1_REG, static_cast<uint8_t>(irqen1 | AXP2101_VBUS_IRQ_MASK));
    M5.Power.Axp2101.clearIRQStatuses();

    const osalThreadAttr_t powerTaskAttr = {
        .stack_size_bytes = 4096,
        .priority = 6,
        .detached = true,
        .name = "PowerTask",
    };

    if (osalThreadCreate(&powerTaskThread, powerTask, nullptr, &powerTaskAttr) != OSAL_STATUS_OK) {
        LOGE("Failed to create PowerTask");
        return false;
    }

    LOGI("PowerTask started and charging enabled");
    return true;
}
