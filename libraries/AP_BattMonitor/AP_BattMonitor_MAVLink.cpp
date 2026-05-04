#include "AP_BattMonitor_config.h"

#if AP_BATTERY_MAVLINK_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "AP_BattMonitor_MAVLink.h"

/*
 * MAVLink battery monitor backend.
 *
 * This backend accepts BATTERY_STATUS messages from a battery component in the
 * same MAVLink system as the autopilot. The component id and BATTERY_STATUS.id
 * are configurable per battery monitor instance, and incoming telemetry is
 * exposed through the normal AP_BattMonitor frontend.
 */
const AP_Param::GroupInfo AP_BattMonitor_MAVLink::var_info[] = {

    // @Param: MAV_COMPID
    // @DisplayName: MAVLink battery source component id
    // @Description: MAVLink component id accepted for inbound BATTERY_STATUS from the autopilot's MAVLink system. Use 180 for MAV_COMP_ID_BATTERY, 181 for MAV_COMP_ID_BATTERY2, or another component id if needed.
    // @Range: 0 255
    // @User: Advanced
    AP_GROUPINFO("MAV_COMPID", 30, AP_BattMonitor_MAVLink, _mav_compid, MAV_COMP_ID_BATTERY),

    // @Param: MAV_ID
    // @DisplayName: MAVLink battery id
    // @Description: BATTERY_STATUS.id value accepted for this monitor instance.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("MAV_ID", 31, AP_BattMonitor_MAVLink, _mav_id, 0),

    // @Param: MAV_OPTIONS
    // @DisplayName: MAVLink battery options
    // @Description: Options that control how MAVLink BATTERY_STATUS values are used by this battery monitor.
    // @Bitmask: 0:Ignore MAVLink SoC, 1:Ignore MAVLink consumed mAh, 2:Ignore MAVLink consumed Wh
    // @User: Advanced
    AP_GROUPINFO("MAV_OPTIONS", 32, AP_BattMonitor_MAVLink, _mav_options, 0),

    AP_GROUPEND
};

AP_BattMonitor_MAVLink::AP_BattMonitor_MAVLink(AP_BattMonitor &mon,
                                               AP_BattMonitor::BattMonitor_State &mon_state,
                                               AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
    _state.healthy = false;
}

bool AP_BattMonitor_MAVLink::option_is_set(MAVLinkOptions option) const
{
    return (uint16_t(_mav_options.get()) & static_cast<uint16_t>(option)) != 0;
}

bool AP_BattMonitor_MAVLink::matches_message(const mavlink_message_t &msg, const mavlink_battery_status_t &packet) const
{
    if (msg.sysid != mavlink_system.sysid) {
        return false;
    }

    if (packet.id != uint8_t(constrain_int16(_mav_id.get(), 0, UINT8_MAX))) {
        return false;
    }
    if (msg.compid != uint8_t(constrain_int16(_mav_compid.get(), 0, UINT8_MAX))) {
        return false;
    }
    return true;
}

bool AP_BattMonitor_MAVLink::handle_battery_status(const mavlink_message_t &msg)
{
    mavlink_battery_status_t packet {};
    mavlink_msg_battery_status_decode(&msg, &packet);

    if (!matches_message(msg, packet)) {
        return false;
    }

    float voltage = 0.0f;
    uint8_t valid_cell_count = 0;
    AP_BattMonitor::cells cell_voltages {};

    // ArduPilot also uses a single populated BATTERY_STATUS.voltages[] entry to
    // represent total pack voltage when per-cell data is unavailable, so accept
    // the same convention on input by summing all valid entries.
    for (uint8_t i = 0; i < MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN; i++) {
        const uint16_t cell_mv = packet.voltages[i];
        if (cell_mv != UINT16_MAX && cell_mv > 0) {
            cell_voltages.cells[i] = cell_mv;
            voltage += cell_mv * 0.001f;
            valid_cell_count++;
        }
    }
    for (uint8_t i = 0; i < MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN; i++) {
        const uint16_t cell_mv = packet.voltages_ext[i];
        if (cell_mv > 0) {
            const uint8_t cell_index = MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN + i;
            if (cell_index < ARRAY_SIZE(cell_voltages.cells)) {
                cell_voltages.cells[cell_index] = cell_mv;
                voltage += cell_mv * 0.001f;
                valid_cell_count++;
            }
        }
    }

    if (voltage <= 0.0f) {
        return false;
    }

    const uint32_t now_us = AP_HAL::micros();
    const bool current_valid = packet.current_battery != -1;
    const bool consumed_mah_valid = packet.current_consumed >= 0;
    const bool consumed_wh_valid = packet.energy_consumed >= 0;
    const bool soc_valid = packet.battery_remaining >= 0 && packet.battery_remaining <= 100;
    const bool use_consumed_mah = current_valid && consumed_mah_valid && !option_is_set(MAVLinkOptions::IgnoreMAVLinkConsumedMah);
    const bool use_consumed_wh = consumed_wh_valid && !option_is_set(MAVLinkOptions::IgnoreMAVLinkConsumedWh);
    const uint32_t now_ms = AP_HAL::millis();

    WITH_SEMAPHORE(_sem_battmon);

    const uint32_t previous_update_us = _interim.state.last_time_micros;
    const uint32_t dt_us = now_us - previous_update_us;
    const bool can_integrate = previous_update_us != 0 && dt_us < 2000000U;

    _interim.state.voltage = voltage;
    _interim.state.cell_voltages = cell_voltages;
    _interim.state.healthy = true;
    _interim.state.last_time_micros = now_us;
    // A single valid voltage entry is treated as pack voltage; expose cell
    // voltages only when multiple entries are present.
    _interim.has_cell_voltages = valid_cell_count > 1;

    _interim.has_current = current_valid;
    if (_interim.has_current) {
        float current_amps = packet.current_battery * 0.01f;
        if (current_amps < 0.0f) {
            current_amps = 0.0f;
        }
        _interim.state.current_amps = current_amps;
    } else {
        _interim.state.current_amps = 0.0f;
    }

    if (use_consumed_mah) {
        _interim.state.consumed_mah = packet.current_consumed;
    } else if (_interim.has_current && can_integrate) {
        _interim.state.consumed_mah += calculate_mah(_interim.state.current_amps, dt_us);
    }

    if (use_consumed_wh) {
        // MAVLink BATTERY_STATUS.energy_consumed is in hJ (100 J), so convert to Wh.
        _interim.state.consumed_wh = packet.energy_consumed / 36.0f;
    } else if (_interim.has_current && can_integrate) {
        _interim.state.consumed_wh += 0.001f * calculate_mah(_interim.state.current_amps, dt_us) * _interim.state.voltage;
    }
    _interim.has_consumed_energy = use_consumed_wh || _interim.has_current;

    _interim.has_external_soc = soc_valid;
    if (_interim.has_external_soc) {
        _interim.soc = packet.battery_remaining;
    }

    if (packet.temperature != INT16_MAX) {
        _interim.state.temperature = packet.temperature * 0.01f;
        _interim.state.temperature_time = now_ms;
    }

    _interim.state.has_time_remaining = packet.time_remaining > 0;
    if (_interim.state.has_time_remaining) {
        _interim.state.time_remaining = packet.time_remaining;
    }
    _interim.has_time_remaining = _interim.state.has_time_remaining;

    _interim.fault_bitmask = packet.fault_bitmask;

    return true;
}

void AP_BattMonitor_MAVLink::read()
{
    const uint32_t now_us = AP_HAL::micros();
    const uint32_t now_ms = AP_HAL::millis();

    WITH_SEMAPHORE(_sem_battmon);

    const bool timed_out = _interim.state.last_time_micros == 0 ||
        (now_us - _interim.state.last_time_micros) > (AP_BATT_MONITOR_TIMEOUT * 1000U);

    _state.voltage = _interim.state.voltage;
    _state.current_amps = _interim.state.current_amps;
    _state.consumed_mah = _interim.state.consumed_mah;
    _state.consumed_wh = _interim.state.consumed_wh;
    _state.temperature = _interim.state.temperature;
    _state.temperature_time = _interim.state.temperature_time;
    _state.last_time_micros = _interim.state.last_time_micros;
    _state.healthy = _interim.state.healthy && !timed_out;
    _state.time_remaining = _interim.state.time_remaining;
    _state.has_time_remaining = _interim.state.has_time_remaining;
    memcpy(_state.cell_voltages.cells, _interim.state.cell_voltages.cells, sizeof(_state.cell_voltages.cells));

    _soc = _interim.soc;
    _fault_bitmask = _interim.fault_bitmask;
    _has_current = _interim.has_current;
    _has_consumed_energy = _interim.has_consumed_energy;
    _has_cell_voltages = _interim.has_cell_voltages;
    _has_time_remaining = _interim.has_time_remaining;
    _has_external_soc = _interim.has_external_soc;
    _has_temperature = _state.temperature_time != 0 &&
        (now_ms - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;
}

bool AP_BattMonitor_MAVLink::use_mavlink_soc() const
{
    return !option_is_set(MAVLinkOptions::IgnoreMAVLinkSoc) && _has_external_soc && _soc <= 100;
}

bool AP_BattMonitor_MAVLink::capacity_remaining_pct(uint8_t &percentage) const
{
    if (!use_mavlink_soc()) {
        return AP_BattMonitor_Backend::capacity_remaining_pct(percentage);
    }
    if (!_state.healthy) {
        return false;
    }
    percentage = _soc;
    return true;
}

bool AP_BattMonitor_MAVLink::reset_remaining(float percentage)
{
    if (use_mavlink_soc()) {
        return false;
    }

    WITH_SEMAPHORE(_sem_battmon);

    if (!AP_BattMonitor_Backend::reset_remaining(percentage)) {
        return false;
    }

    _interim.state.consumed_mah = _state.consumed_mah;
    _interim.state.consumed_wh = _state.consumed_wh;
    return true;
}

uint32_t AP_BattMonitor_MAVLink::get_mavlink_fault_bitmask() const
{
    return _fault_bitmask;
}

#endif  // AP_BATTERY_MAVLINK_ENABLED
