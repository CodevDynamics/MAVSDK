#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/camera/camera.h>
#include <mavsdk/plugins/param/param_impl.h>
#include <mavsdk/plugins/gimbal/gimbal.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
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
std::vector<std::string> detected_labels_database =
{
    "person", "car", "bus", "truck", "bike", "train", "boat", "aeroplane",
    "bicycle", "motorcycle", "airplane", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear",
    "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
    "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet",
    "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
};
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    uint16_t score;
    uint16_t type;
} DetectObject;

typedef struct {
    uint16_t index;
    uint16_t size;
    uint16_t total;
    DetectObject objects[10];
} DetectObjectsPacket;

std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);

    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
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
    auto gimbal = Gimbal{system};
    auto param = ParamImpl{system};
    param.select_component(100, Param::ProtocolVersion::Ext);
    auto mavlink_passthrough = MavlinkPassthrough{system};
    // Stop tracking
    param.set_param_custom("TRACK_ALGORITHM", "None");

    // Gimbal
    auto attitudeHandle = gimbal.subscribe_attitude([](Gimbal::Attitude attitude) {
        std::cout << "Gimbal angle pitch: " << attitude.euler_angle_forward.pitch_deg
                  << " deg, yaw: " << attitude.euler_angle_forward.yaw_deg
                  << " yaw (relative to forward)\n";
        std::cout << "Gimbal angle pitch: " << attitude.euler_angle_north.pitch_deg
                  << " deg, yaw: " << attitude.euler_angle_north.yaw_deg
                  << " yaw (relative to North)\n";
    });
    std::cout << "Start controlling gimbal...\n";
    Gimbal::Result gimbal_result = gimbal.take_control(Gimbal::ControlMode::Primary);
    if (gimbal_result != Gimbal::Result::Success) {
        std::cerr << "Could not take gimbal control: " << gimbal_result << '\n';
        return 1;
    }

    std::cout << "Set yaw mode to follow...\n";
    gimbal_result = gimbal.set_mode(Gimbal::GimbalMode::YawFollow);
    if (gimbal_result != Gimbal::Result::Success) {
        std::cerr << "Could not set to follow mode: " << gimbal_result << '\n';
        return 1;
    }
    std::cout << "And center first...\n";
    gimbal.set_pitch_and_yaw(0.0f, 0.0f);
    sleep_for(seconds(2));

    std::cout << "Tilt gimbal down...\n";
    gimbal.set_pitch_and_yaw(-90.0f, 0.0f);
    sleep_for(seconds(2));

    std::cout << "Slowly tilt up ...\n";
    gimbal.set_pitch_rate_and_yaw_rate(10.0f, 0.0f);
    sleep_for(seconds(4));

    std::cout << "Pan to the right...\n";
    gimbal.set_pitch_and_yaw(0.0f, 90.0f);
    sleep_for(seconds(2));

    std::cout << "Pan slowly to the left...\n";
    gimbal.set_pitch_rate_and_yaw_rate(0.0f, -10.0f);
    sleep_for(seconds(4));

    std::cout << "Back to the center...\n";
    gimbal.set_pitch_and_yaw(0.0f, 0.0f);
    sleep_for(seconds(2));

        std::cout << "Set yaw mode to lock to a specific direction...\n";
    gimbal_result = gimbal.set_mode(Gimbal::GimbalMode::YawLock);
    if (gimbal_result != Gimbal::Result::Success) {
        std::cerr << "Could not set to lock mode: " << gimbal_result << '\n';
        return 1;
    }
    std::cout << "Look North...\n";
    gimbal.set_pitch_and_yaw(0.0f, 0.0f);
    sleep_for(seconds(2));

    std::cout << "Look East...\n";
    gimbal.set_pitch_and_yaw(0.0f, 90.0f);
    sleep_for(seconds(2));

    std::cout << "Look South...\n";
    gimbal.set_pitch_and_yaw(0.0f, 180.0f);
    sleep_for(seconds(2));

    std::cout << "Look West...\n";
    gimbal.set_pitch_and_yaw(0.0f, -90.0f);
    sleep_for(seconds(2));

    std::cout << "Pan very slowly to the right...\n";
    gimbal.set_pitch_rate_and_yaw_rate(0.0f, 2.5f);
    gimbal.unsubscribe_attitude(attitudeHandle); 

    // Camera
    // Get detect plugins
    bool have_detect_plugin = false;
    std::vector<std::string> detect_plugins;
    auto param_res = param.get_param_custom("DETECT_PLUGINS");
    if (param_res.first == mavsdk::Param::Result::Success && !param_res.second.empty()) {
        detect_plugins = split(param_res.second, ',');
    }
    if(detect_plugins.size()) {
        std::cout << "Found detect plugins: " << param_res.second << std::endl;
        // Open smart select function.
        auto res = param.set_param_custom("SMART_SELECT", detect_plugins.front());
        if(res == mavsdk::Param::Result::Success) {
            have_detect_plugin = true;
            std::cout << "Set 'SMART_SELECT' to '" <<  detect_plugins.front() << "'." << std::endl;
        } else {
            std::cerr << "Set 'SMART_SELECT' to '" <<  detect_plugins.front() << "' failed." << std::endl;
        }
    } else {
        std::cerr << "Not found detect plugins." << std::endl;
    }

    // Get track plugins
    std::vector<std::string> track_plugins;
    param_res = param.get_param_custom("TRACK_PLUGINS");
    if (param_res.first == mavsdk::Param::Result::Success && !param_res.second.empty()) {
        track_plugins = split(param_res.second, ',');
        // Start tracking(&detecting) proccess
        auto res = param.set_param_custom("TRACK_ALGORITHM", track_plugins.front());
        if(res == mavsdk::Param::Result::Success) {
            std::cout << "Set 'TRACK_ALGORITHM' to '" <<  track_plugins.front() << "'." << std::endl;
        } else {
            std::cerr << "Set 'TRACK_ALGORITHM' to '" <<  track_plugins.front() << "' failed." << std::endl;
        }
    }

    {
        // Create a promise for asynchronously waiting for AI process to start
        auto prom = std::promise<void>{};
        auto fut = prom.get_future();
        bool satisfied = false;
        int waiting_count = 0;
        // Subscribe to camera tracking status message
        auto tracking_handle = mavlink_passthrough.subscribe_message(
            MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS, [&prom, &satisfied](const mavlink_message_t& message) {
                mavlink_camera_tracking_image_status_t tracking_image_status;
                memcpy(&tracking_image_status, &message, sizeof(tracking_image_status));
                // When tracking status is less than 3 and condition not satisfied, indicates AI process has started
                if(tracking_image_status.tracking_status < 3 && !satisfied) {
                    satisfied = true;
                    prom.set_value();
                }
            });
        // Check if AI process has started every second
        do {
            std::cout << "Waiting for the AI process to start..." << std::endl;
        } while(fut.wait_for(std::chrono::seconds(1)) == std::future_status::timeout);
        // Unsubscribe from camera tracking status message
        mavlink_passthrough.unsubscribe_message(MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS, tracking_handle);
    }

    float optimal_yaw_value = 0.0f;
    if(have_detect_plugin) {
        auto prom = std::promise<DetectObjectsPacket>{};
        auto fut = prom.get_future();
        int detect_count = 0;
        auto last_time = std::chrono::steady_clock::now();
        std::map<int,int> per_second_counts;
        int max_per_second_count = 0;
        DetectObjectsPacket packet;
        // Subscribe detect objects
        param.subscribe_param_custom_changed("DETECT_OBJECTS", [&optimal_yaw_value, &gimbal, &prom, &detect_count, &last_time, &per_second_counts, &max_per_second_count, &packet](std::string value) {
            if(!value.empty()) {
                memset(&packet, 0, sizeof(packet));
                memcpy(&packet, value.data(), value.size());
                
                // Accumulate the number of targets detected in the current second
                for(int i = 0; i < packet.size; i++) {
                    per_second_counts[packet.objects[i].type]++;
                }
                
                // Check if a second has passed
                auto current_time = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();
                
                if(duration >= 1000) {
                    int total = 0;
                    // Print statistics for this second
                    std::cout << "Detection objects per second: ";
                    for (const auto& [type, count] : per_second_counts) {
                        std::cout << "[" << detected_labels_database[type] << ": " << count << "]";
                        total += count;
                    }
                    std::cout << std::endl;
                    
                    // Reset counters and time
                    per_second_counts.clear();
                    last_time = current_time;
                    if(total > max_per_second_count && detect_count < 250) {
                        max_per_second_count = total;
                        optimal_yaw_value = gimbal.attitude().euler_angle_north.yaw_deg;
                    }
                }
                
                if(packet.size > 0 && (++detect_count == 250)) {
                    prom.set_value(packet);
                }
            }
        });
        fut.get();
        std::cout << "Stop controlling gimbal and control yaw to " << optimal_yaw_value << "deg.\n";
        gimbal.set_pitch_and_yaw(0.0f, optimal_yaw_value);
        sleep_for(seconds(10));
        auto p = packet;
        param.unsubscribe_param_all_changed();
        float point_x = (p.objects[0].x + p.objects[0].width / 2) / 10000.0f;
        float point_y = (p.objects[0].y + p.objects[0].height / 2) / 10000.0f;
        if(camera.track_point(point_x, point_y, 1.0f) == Camera::Result::Success) {
            std::cout << "Use smart selection to specify tracking targets: " << detected_labels_database[p.objects[0].type] << std::endl;
        } else {
            std::cerr << "Track point failed!" << std::endl;
        }
    } else {
        float x1 = 0.4f;
        float y1 = 0.4f;
        float x2 = 0.5f;
        float y2 = 0.5f;
        if(camera.track_rectangle(x1, y1, x2, y2) == Camera::Result::Success) {
            std::cout << "Use manual box selection to specify tracking targets." << std::endl;
        } else {
            std::cerr << "Track rectangle failed!" << std::endl;
        }
    }

    {
        // Subscribe to camera tracking status message
        int frame_count = 0;
        auto last_time = std::chrono::steady_clock::now();
        
        auto track_handle = mavlink_passthrough.subscribe_message(
            MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS, [&frame_count, &last_time](const mavlink_message_t& message) {
                mavlink_camera_tracking_image_status_t tracking_image_status;
                memcpy(&tracking_image_status, &message, sizeof(tracking_image_status));
                frame_count++;
                
                auto current_time = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count();
                
                // Calculate and print FPS once per second
                if (duration >= 1000) {
                    float fps = frame_count * 1000.0f / duration;
                    std::cout << "Tracking FPS: " << fps << std::endl;
                    
                    // Reset counters and time
                    frame_count = 0;
                    last_time = current_time;
                }
            });
        
        sleep_for(seconds(10));
        // Unsubscribe from camera tracking status message
        mavlink_passthrough.unsubscribe_message(MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS, track_handle);
    }

    // Subscribe take photo feedback.
    camera.subscribe_capture_info([](Camera::CaptureInfo capture_info) {
        std::cout << "Image captured, stored at: " << capture_info.file_url << '\n';
    });

    // Take photo with Zoom 1x
    camera.zoom_range(1.0f);
    auto photo_result = camera.take_photo();
    if (photo_result != Camera::Result::Success) {
        std::cerr << "Taking Photo(1x) failed: " << photo_result << '\n';
    }

    // Take photo with Zoom 10x
    camera.zoom_range(10.0f);
    photo_result = camera.take_photo();
    if (photo_result != Camera::Result::Success) {
        std::cerr << "Taking Photo(10x) failed: " << photo_result << '\n';
    }

    // Stop tracking
    param.set_param_custom("TRACK_ALGORITHM", "None");

    // Wait a bit to make sure we see capture information.
    sleep_for(seconds(2));

    // gimbal_result = gimbal.release_control();
    // if (gimbal_result != Gimbal::Result::Success) {
    //     std::cerr << "Could not take gimbal control: " << gimbal_result << '\n';
    //     return 1;
    // }

    return 0;
}
