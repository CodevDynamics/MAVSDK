#pragma once

#include "plugins/camera_server/camera_server.h"
#include "server_plugin_impl_base.h"
#include "callback_list.h"

namespace mavsdk {

class CameraServerImpl : public ServerPluginImplBase {
public:
    using VideoStatusCallback = std::function<void(uint8_t&,uint32_t&,float&)>;
    using CameraSettingsCallback = std::function<void(uint8_t&,float&,float&)>;
    using VideoHandle = Handle<int32_t,bool&>;
    using VideoCallback = std::function<void(int32_t,bool&)>;
    using ZoomFocusHandle = Handle<uint8_t,float>;
    using ZoomFocusCallback = std::function<void(uint8_t,float)>;
    using ModeHandle = Handle<uint8_t>;
    using ModeCallback = std::function<void(uint8_t)>;
    using ResetHandle = Handle<bool>;
    using ResetCallback = std::function<void(bool)>;
    using FormatHandle = Handle<uint8_t,bool&,bool>;
    using FormatCallback = std::function<void(uint8_t,bool&,bool)>;
    struct StorageInformation {
        uint8_t storage_count;
        uint8_t status;
        uint8_t type;
        uint8_t storage_usage;
        float total_capacity;
        float used_capacity;
        float available_capacity;
        float read_speed;
        float write_speed;
        std::string name;
    };

    CameraServer::Result set_storage_information(CameraServerImpl::StorageInformation information);
    void register_video_status_callback(const CameraServerImpl::VideoStatusCallback& callback);
    void register_camera_settings_callback(const CameraServerImpl::CameraSettingsCallback& callback);
    CameraServerImpl::VideoHandle subscribe_video(const CameraServerImpl::VideoCallback& callback);
    void unsubscribe_video(CameraServerImpl::VideoHandle handle);
    CameraServerImpl::ZoomFocusHandle subscribe_zoom(const CameraServerImpl::ZoomFocusCallback& callback);
    void unsubscribe_zoom(CameraServerImpl::ZoomFocusHandle handle);
    CameraServerImpl::ZoomFocusHandle subscribe_focus(const CameraServerImpl::ZoomFocusCallback& callback);
    void unsubscribe_focus(CameraServerImpl::ZoomFocusHandle handle);
    CameraServerImpl::ModeHandle subscribe_mode(const CameraServerImpl::ModeCallback& callback);
    void unsubscribe_mode(CameraServerImpl::ModeHandle handle);
    CameraServerImpl::ResetHandle subscribe_reset(const CameraServerImpl::ResetCallback& callback);
    void unsubscribe_reset(CameraServerImpl::ResetHandle handle);
    CameraServerImpl::FormatHandle subscribe_format(const CameraServerImpl::FormatCallback& callback);
    void unsubscribe_format(CameraServerImpl::FormatHandle handle);

    void push_stream_info(const mavlink_video_stream_information_t& info);
    void clean_stream_info();

    void update_camera_capture_status_idle(float available_capacity, int32_t image_capture_count);

    explicit CameraServerImpl(std::shared_ptr<ServerComponent> server_component);
    ~CameraServerImpl() override;

    void init() override;
    void deinit() override;

    CameraServer::Result set_information(CameraServer::Information information);
    CameraServer::Result set_in_progress(bool in_progress);

    CameraServer::TakePhotoHandle
    subscribe_take_photo(const CameraServer::TakePhotoCallback& callback);
    void unsubscribe_take_photo(CameraServer::TakePhotoHandle handle);

    CameraServer::Result respond_take_photo(
        CameraServer::TakePhotoFeedback take_photo_feedback,
        CameraServer::CaptureInfo capture_info);

private:
    bool _is_storage_information_set{};
    CameraServerImpl::StorageInformation _storage_information{};
    CameraServerImpl::VideoStatusCallback _video_status_callback{nullptr};
    CameraServerImpl::CameraSettingsCallback _camera_settings_callback{nullptr};
    CallbackList<int32_t,bool&> _video_callbacks{};
    CallbackList<uint8_t,float> _focus_callbacks{};
    CallbackList<uint8_t,float> _zoom_callbacks{};
    CallbackList<uint8_t> _mode_callbacks{};
    CallbackList<bool> _reset_callbacks{};
    CallbackList<uint8_t,bool&,bool> _format_callbacks{};

    std::mutex _stream_info_mutex{};
    std::vector<mavlink_video_stream_information_t> _stream_info;

    enum StatusFlags {
        IN_PROGRESS = 1 << 0,
        INTERVAL_SET = 1 << 1,
    };

    enum class TriggerControl {
        IGNORE = -1,
        DISABLE = 0,
        ENABLE = 1,
    };

    bool _is_information_set{};
    CameraServer::Information _information{};

    // CAMERA_CAPTURE_STATUS fields
    // TODO: how do we keep this info in sync between plugin instances?
    bool _is_image_capture_in_progress{};
    bool _is_image_capture_interval_set{};
    float _image_capture_timer_interval_s{};
    void* _image_capture_timer_cookie{};
    int32_t _image_capture_count{};

    CallbackList<int32_t> _take_photo_callbacks{};

    MavlinkCommandReceiver::CommandLong _last_take_photo_command;

    bool parse_version_string(const std::string& version_str);
    bool parse_version_string(const std::string& version_str, uint32_t& version);
    void start_image_capture_interval(float interval, int32_t count, int32_t index);
    void stop_image_capture_interval();

    std::optional<mavlink_command_ack_t>
    process_camera_information_request(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_camera_settings_request(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_storage_information_request(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_storage_format(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_camera_capture_status_request(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_reset_camera_settings(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_set_camera_mode(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_set_camera_zoom(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_set_camera_focus(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_set_storage_usage(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_image_start_capture(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_image_stop_capture(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_camera_image_capture_request(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_video_start_capture(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_video_stop_capture(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_video_start_streaming(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_video_stop_streaming(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_video_stream_information_request(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_video_stream_status_request(const MavlinkCommandReceiver::CommandLong& command);
};

} // namespace mavsdk
