#include "connection.h"

#include <memory>
#include <utility>
#include "mavsdk_impl.h"
#include "log.h"

#ifdef WINDOWS
#include <winsock2.h>
#endif

namespace mavsdk {

std::atomic<unsigned> Connection::_forwarding_connections_count = 0;

Connection::Connection(
    ReceiverCallback receiver_callback,
    LibmavReceiverCallback libmav_receiver_callback,
    MavsdkImpl& mavsdk_impl,
    ForwardingOption forwarding_option) :
    _receiver_callback(std::move(receiver_callback)),
    _libmav_receiver_callback(std::move(libmav_receiver_callback)),
    _mavsdk_impl(mavsdk_impl),
    _mavlink_receiver(),
    _libmav_receiver(),
    _forwarding_option(forwarding_option)
{
    // Insert system ID 0 in all connections for broadcast.
    _system_ids.insert(0);
    _component_ids.insert(0);

    if (forwarding_option == ForwardingOption::ForwardingOn) {
        _forwarding_connections_count++;
    }

    if (const char* env_p = std::getenv("MAVSDK_MAVLINK_DIRECT_DEBUGGING")) {
        if (std::string(env_p) == "1") {
            _debugging = true;
        }
    }
}

Connection::~Connection()
{
    // Just in case a specific connection didn't call it already.
    stop_mavlink_receiver();
    stop_libmav_receiver();
    _receiver_callback = {};
    _libmav_receiver_callback = {};
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

bool Connection::start_libmav_receiver()
{
    _libmav_receiver = std::make_unique<LibmavReceiver>(_mavsdk_impl);
    return true;
}

void Connection::stop_libmav_receiver()
{
    if (_libmav_receiver) {
        _libmav_receiver.reset();
    }
}

void Connection::receive_libmav_message(
    const Mavsdk::MavlinkMessage& message, Connection* connection)
{
    // Register system ID when receiving a message from a new system.
    if (_system_ids.find(message.system_id) == _system_ids.end()) {
        _system_ids.insert(message.system_id);
    }

    if (_debugging) {
        LogDebug() << "Connection::receive_libmav_message: " << message.message_name
                   << " from system " << message.system_id;
    }

    if (_libmav_receiver_callback) {
        if (_debugging) {
            LogDebug() << "Calling libmav receiver callback for: " << message.message_name;
        }
        _libmav_receiver_callback(message, connection);
    } else {
        LogWarn() << "No libmav receiver callback set!";
    }
}

void Connection::receive_message(
    MavlinkReceiver::ParseResult result, mavlink_message_t& message, Connection* connection)
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
    // Register system ID for valid messages
    if (result == MavlinkReceiver::ParseResult::MessageParsed) {
        std::lock_guard<std::mutex> lock(_system_ids_mutex);
        if (_system_ids.find(message.sysid) == _system_ids.end()) {
            _system_ids.insert(message.sysid);
        }
        if (_component_ids.find(message.compid) == _component_ids.end()) {
            _component_ids.insert(message.compid);
        }
    }
    // Let MavsdkImpl handle the ParseResult (queue for processing or forward-only)
    _receiver_callback(result, message, connection);
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
    std::lock_guard<std::mutex> lock(_system_ids_mutex);
    return _system_ids.find(system_id) != _system_ids.end();
}

bool Connection::has_component_id(uint8_t component_id)
{
    std::lock_guard<std::mutex> lock(_system_ids_mutex);
    return _component_ids.find(component_id) != _component_ids.end();
}
#ifdef WINDOWS
std::string get_socket_error_string(int error_code)
{
    if (error_code == 0) {
        return "";
    }

    LPVOID lpMsgBuf = nullptr;
    DWORD bufLen = FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        error_code,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR)&lpMsgBuf,
        0,
        NULL);

    if (bufLen) {
        LPCSTR lpMsgStr = (LPCSTR)lpMsgBuf;
        std::string result(lpMsgStr, lpMsgStr + bufLen);
        LocalFree(lpMsgBuf);

        // Remove trailing newline if present
        if (!result.empty() && result[result.length() - 1] == '\n') {
            result.erase(result.length() - 1);
        }
        if (!result.empty() && result[result.length() - 1] == '\r') {
            result.erase(result.length() - 1);
        }

        return result;
    }

    return std::to_string(error_code);
}
#endif

} // namespace mavsdk
