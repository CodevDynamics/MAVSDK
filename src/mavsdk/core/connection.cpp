#include "connection.h"

#include <memory>
#include <utility>
#include "mavsdk_impl.h"

namespace mavsdk {

std::atomic<unsigned> Connection::_forwarding_connections_count = 0;

Connection::Connection(ReceiverCallback receiver_callback, ForwardingOption forwarding_option) :
    _receiver_callback(std::move(receiver_callback)),
    _mavlink_receiver(),
    _forwarding_option(forwarding_option)
{
    // Insert system ID 0 in all connections for broadcast.
    _system_ids.insert(0);
    _component_ids.insert(0);

    if (forwarding_option == ForwardingOption::ForwardingOn) {
        _forwarding_connections_count++;
    }
}

Connection::~Connection()
{
    // Just in case a specific connection didn't call it already.
    stop_mavlink_receiver();
    _receiver_callback = {};
}

bool Connection::start_mavlink_receiver()
{
    _mavlink_receiver = std::make_unique<MavlinkReceiver>();
    return true;
}

void Connection::stop_mavlink_receiver()
{
    if (_mavlink_receiver) {
        _mavlink_receiver.reset();
    }
}

void Connection::receive_message(mavlink_message_t& message, Connection* connection)
{
    if(message.msgid == MAVLINK_MSG_ID_PING && message.compid == MAV_COMP_ID_UDP_BRIDGE) {
        mavlink_ping_t ping;
        mavlink_msg_ping_decode(&message, &ping);
        if(ping.target_component == MAV_COMP_ID_MISSIONPLANNER && ping.target_system == 0) {
            mavlink_message_t msg;
            mavlink_msg_ping_pack(ping.target_system,
                                  MAV_COMP_ID_MISSIONPLANNER,
                                  &msg,
                                  ping.time_usec,
                                  ping.seq,
                                  message.sysid,
                                  message.compid);
            send_message(msg);
        }
    }
    // Register system ID when receiving a message from a new system.
    if (_system_ids.find(message.sysid) == _system_ids.end()) {
        _system_ids.insert(message.sysid);
    }
    if (_component_ids.find(message.compid) == _component_ids.end()) {
        _component_ids.insert(message.compid);
    }
    _receiver_callback(message, connection);
}

bool Connection::should_forward_messages() const
{
    return _forwarding_option == ForwardingOption::ForwardingOn;
}

unsigned Connection::forwarding_connections_count()
{
    return _forwarding_connections_count;
}

bool Connection::has_system_id(uint8_t system_id)
{
    return _system_ids.find(system_id) != _system_ids.end();
}

bool Connection::has_component_id(uint8_t component_id)
{
    return _component_ids.find(component_id) != _component_ids.end();
}

} // namespace mavsdk
