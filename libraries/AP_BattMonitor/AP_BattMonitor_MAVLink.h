#pragma once

#include "AP_BattMonitor_config.h"

#if AP_BATTERY_MAVLINK_ENABLED

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

// Battery monitor backend that accepts inbound MAVLink BATTERY_STATUS telemetry.
class AP_BattMonitor_MAVLink : public AP_BattMonitor_Backend
{
public:
    // Construct a MAVLink-backed battery monitor backend for a frontend instance.
    AP_BattMonitor_MAVLink(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    static const struct AP_Param::GroupInfo var_info[];

    // Copy the latest decoded MAVLink telemetry into the frontend state.
    void read() override;
    // Return remaining capacity from MAVLink SoC or the backend fallback estimate.
    bool capacity_remaining_pct(uint8_t &percentage) const override;
    // Reset the remaining estimate when MAVLink SoC is not in use.
    bool reset_remaining(float percentage) override;

    bool has_current() const override { return _state.healthy && _has_current; }
    bool has_consumed_energy() const override { return _state.healthy && _has_consumed_energy; }
    bool has_temperature() const override { return _state.healthy && _has_temperature; }
    bool has_time_remaining() const override { return _state.healthy && _has_time_remaining; }
    bool has_cell_voltages() const override { return _state.healthy && _has_cell_voltages; }

    // Return the MAVLink battery fault bitmask from the last accepted message.
    uint32_t get_mavlink_fault_bitmask() const override;

    // Accept an inbound BATTERY_STATUS message when it matches this instance.
    bool handle_battery_status(const mavlink_message_t &msg);

private:
    enum class MAVLinkOptions : uint16_t {
        IgnoreMAVLinkSoc         = (1U << 0),
        IgnoreMAVLinkConsumedMah = (1U << 1),
        IgnoreMAVLinkConsumedWh  = (1U << 2),
    };

    bool use_mavlink_soc() const;
    bool option_is_set(MAVLinkOptions option) const;
    bool matches_message(const mavlink_message_t &msg, const mavlink_battery_status_t &packet) const;

    struct InterimData {
        AP_BattMonitor::BattMonitor_State state {};
        uint8_t soc;
        uint32_t fault_bitmask;
        bool has_current;
        bool has_consumed_energy;
        bool has_cell_voltages;
        bool has_time_remaining;
        bool has_external_soc;
    } _interim {};

    HAL_Semaphore _sem_battmon;

    AP_Int16 _mav_compid;
    AP_Int16 _mav_id;
    AP_Int16 _mav_options;

    uint8_t _soc = 0;
    uint32_t _fault_bitmask = 0;
    bool _has_current = false;
    bool _has_consumed_energy = false;
    bool _has_temperature = false;
    bool _has_cell_voltages = false;
    bool _has_time_remaining = false;
    bool _has_external_soc = false;
};

#endif  // AP_BATTERY_MAVLINK_ENABLED
