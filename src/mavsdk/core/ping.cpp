#include "log.h"
#include "mavlink_address.h"
#include "ping.h"
#include "mavsdk_impl.h"

namespace mavsdk {

Ping::Ping(MavsdkImpl& mavsdk_impl) : _mavsdk_impl(mavsdk_impl)
{
    _mavsdk_impl.mavlink_message_handler.register_one(
        MAVLINK_MSG_ID_PING,
        [this](const mavlink_message_t& message) { Ping::process_ping(message); },
        this);
}

Ping::~Ping()
{
    _mavsdk_impl.mavlink_message_handler.unregister_all(this);
}

void Ping::run_once()
{
    mavlink_message_t message;
    mavlink_msg_ping_pack_chan(
        _mavsdk_impl.get_own_system_id(),
        _mavsdk_impl.get_own_component_id(),
        _mavsdk_impl.channel(),
        &message,
        _mavsdk_impl.time.elapsed_us(),
        _ping_sequence,
        0,
        0); // to all
    _mavsdk_impl.send_message(message);
}

void Ping::process_ping(const mavlink_message_t& message)
{
    mavlink_ping_t ping;
    mavlink_msg_ping_decode(&message, &ping);

    if (ping.target_system == 0 && ping.target_component == 0) {
        // Response to ping request.
        mavlink_message_t response_message;
        mavlink_msg_ping_pack_chan(
            _mavsdk_impl.get_own_system_id(),
            _mavsdk_impl.get_own_component_id(),
            _mavsdk_impl.channel(),
            &response_message,
            ping.time_usec,
            ping.seq,
            message.sysid,
            message.compid);
        _mavsdk_impl.send_message(response_message);

    } else {
        if (message.compid != MAV_COMP_ID_AUTOPILOT1) {
            // We're currently only interested in the ping of the autopilot.
            return;
        }

        // Answer from ping request.
        if (ping.seq != _ping_sequence) {
            // Ignoring unknown ping sequence.
            return;
        }

        _last_ping_time_us = _mavsdk_impl.time.elapsed_us() - ping.time_usec;
    }
}

} // namespace mavsdk
