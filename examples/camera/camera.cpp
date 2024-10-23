//
// Example to demonstrate how to switch to photo mode and take a picture.
//

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/camera/camera.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

void usage(std::string bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    std::cout << "Waiting to discover system...\n";
    int timeout_count = 0;
    std::shared_ptr<System> system;
    do {
        auto systems = mavsdk.systems();
        for (auto& sys : systems) {
            if (sys->has_camera()) {
                system = sys;
                break;
            }
        }
        if(!system) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (timeout_count++ < 300 && !system);

    if(system) {
        std::cout << "Discovered camera." << std::endl;
    } else {
        std::cerr << "No camera found, exiting" << std::endl;
        return 1;
    }

    // Instantiate plugins.
    auto camera = Camera{system};

    // First, make sure camera is in photo mode.
    const auto mode_result = camera.set_mode(Camera::Mode::Photo);
    if (mode_result != Camera::Result::Success) {
        std::cerr << "Could not switch to Photo mode: " << mode_result;
        return 1;
    }

    // We want to subscribe to information about pictures that are taken.
    camera.subscribe_capture_info([](Camera::CaptureInfo capture_info) {
        std::cout << "Image captured, stored at: " << capture_info.file_url << '\n';
    });

    const auto photo_result = camera.take_photo();
    if (photo_result != Camera::Result::Success) {
        std::cerr << "Taking Photo failed: " << photo_result;
    }

    // Wait a bit to make sure we see capture information.
    sleep_for(seconds(2));

    return 0;
}
