#pragma once

#include "mavlink_include.h"
#include <atomic>

namespace mavsdk {

class MavsdkImpl;

class Ping {
public:
    explicit Ping(MavsdkImpl& mavsdk_impl);
    ~Ping();

    void run_once();
    [[nodiscard]] double last_ping_time_s() const
    {
        return static_cast<double>(_last_ping_time_us) * 1e-6;
    }

private:
    void process_ping(const mavlink_message_t& message);

    MavsdkImpl& _mavsdk_impl;
    uint32_t _ping_sequence{0};
    std::atomic<uint64_t> _last_ping_time_us{0};
};

} // namespace mavsdk
