#include "camera_server_impl.h"
#include "callback_list.tpp"

namespace mavsdk {

template class CallbackList<int32_t>;

CameraServerImpl::CameraServerImpl(std::shared_ptr<ServerComponent> server_component) :
    ServerPluginImplBase(server_component)
{
    _server_component_impl->register_plugin(this);
}

CameraServerImpl::~CameraServerImpl()
{
    _server_component_impl->unregister_plugin(this);
}

void CameraServerImpl::init()
{
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_REQUEST_CAMERA_INFORMATION,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_camera_information_request(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_REQUEST_CAMERA_SETTINGS,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_camera_settings_request(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_REQUEST_STORAGE_INFORMATION,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_storage_information_request(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_STORAGE_FORMAT,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_storage_format(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_camera_capture_status_request(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_RESET_CAMERA_SETTINGS,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_reset_camera_settings(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_SET_CAMERA_MODE,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_set_camera_mode(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_SET_CAMERA_ZOOM,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_set_camera_zoom(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_SET_CAMERA_FOCUS,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_set_camera_focus(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_SET_STORAGE_USAGE,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_set_storage_usage(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_IMAGE_START_CAPTURE,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_image_start_capture(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_IMAGE_STOP_CAPTURE,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_image_stop_capture(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_camera_image_capture_request(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_VIDEO_START_CAPTURE,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_video_start_capture(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_VIDEO_STOP_CAPTURE,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_video_stop_capture(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_VIDEO_START_STREAMING,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_video_start_streaming(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_VIDEO_STOP_STREAMING,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_video_stop_streaming(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_video_stream_information_request(command);
        },
        this);
    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_REQUEST_VIDEO_STREAM_STATUS,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_video_stream_status_request(command);
        },
        this);
}

void CameraServerImpl::deinit()
{
    stop_image_capture_interval();
    _server_component_impl->unregister_all_mavlink_command_handlers(this);
}

bool CameraServerImpl::parse_version_string(const std::string& version_str)
{
    uint32_t unused;

    return parse_version_string(version_str, unused);
}

bool CameraServerImpl::parse_version_string(const std::string& version_str, uint32_t& version)
{
    // empty string means no version
    if (version_str.empty()) {
        version = 0;

        return true;
    }

    uint8_t major{}, minor{}, patch{}, dev{};

    auto ret = sscanf(version_str.c_str(), "%hhu.%hhu.%hhu.%hhu", &major, &minor, &patch, &dev);

    if (ret == EOF) {
        return false;
    }

    // pack version according to MAVLINK spec
    version = dev << 24 | patch << 16 | minor << 8 | major;

    return true;
}

CameraServer::Result CameraServerImpl::set_information(CameraServer::Information information)
{
    if (!parse_version_string(information.firmware_version)) {
        LogDebug() << "incorrectly formatted firmware version string: "
                   << information.firmware_version;
        return CameraServer::Result::WrongArgument;
    }

    // TODO: validate information.definition_file_uri

    _is_information_set = true;
    _information = information;

    return CameraServer::Result::Success;
}

CameraServer::Result CameraServerImpl::set_in_progress(bool in_progress)
{
    _is_image_capture_in_progress = in_progress;
    return CameraServer::Result::Success;
}

CameraServer::Result CameraServerImpl::set_storage_information(CameraServerImpl::StorageInformation information)
{
    _storage_information = information;
    if(_is_storage_information_set) {
        _server_component_impl->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
            mavlink_message_t message{};
            mavlink_msg_storage_information_pack_chan(
                mavlink_address.system_id,
                mavlink_address.component_id,
                channel,
                &message,
                static_cast<uint32_t>(_server_component_impl->get_time().elapsed_s() * 1e3),
                0,
                _storage_information.storage_count,
                _storage_information.status,
                _storage_information.total_capacity,
                _storage_information.used_capacity,
                _storage_information.available_capacity,
                _storage_information.read_speed,
                _storage_information.write_speed,
                _storage_information.type,
                _storage_information.name.data(),
                _storage_information.storage_usage);

            return message;
        });
    } else {
        _is_storage_information_set = true;
    }
    return CameraServer::Result::Success;
}

void CameraServerImpl::register_video_status_callback(const CameraServerImpl::VideoStatusCallback& callback)
{
    _video_status_callback = callback;
}

void CameraServerImpl::register_camera_settings_callback(const CameraServerImpl::CameraSettingsCallback& callback)
{
    _camera_settings_callback = callback;
}

CameraServerImpl::VideoHandle
CameraServerImpl::subscribe_video(const CameraServerImpl::VideoCallback& callback)
{
    return _video_callbacks.subscribe(callback);
}

void CameraServerImpl::unsubscribe_video(CameraServerImpl::VideoHandle handle)
{
    _video_callbacks.unsubscribe(handle);
}

CameraServerImpl::ZoomFocusHandle CameraServerImpl::subscribe_zoom(const CameraServerImpl::ZoomFocusCallback& callback)
{
    return _zoom_callbacks.subscribe(callback);
}

void CameraServerImpl::unsubscribe_zoom(CameraServerImpl::ZoomFocusHandle handle)
{
    _zoom_callbacks.unsubscribe(handle);
}

CameraServerImpl::ZoomFocusHandle CameraServerImpl::subscribe_focus(const CameraServerImpl::ZoomFocusCallback& callback)
{
    return _focus_callbacks.subscribe(callback);
}

void CameraServerImpl::unsubscribe_focus(CameraServerImpl::ZoomFocusHandle handle)
{
    _focus_callbacks.unsubscribe(handle);
}

CameraServerImpl::ModeHandle CameraServerImpl::subscribe_mode(const CameraServerImpl::ModeCallback& callback)
{
    return _mode_callbacks.subscribe(callback);
}

void CameraServerImpl::unsubscribe_mode(CameraServerImpl::ModeHandle handle)
{
    _mode_callbacks.unsubscribe(handle);
}

CameraServerImpl::ResetHandle CameraServerImpl::subscribe_reset(const CameraServerImpl::ResetCallback& callback)
{
    return _reset_callbacks.subscribe(callback);
}

void CameraServerImpl::unsubscribe_reset(CameraServerImpl::ResetHandle handle)
{
    _reset_callbacks.unsubscribe(handle);
}

CameraServerImpl::FormatHandle CameraServerImpl::subscribe_format(const CameraServerImpl::FormatCallback& callback)
{
    return _format_callbacks.subscribe(callback);
}

void CameraServerImpl::unsubscribe_format(CameraServerImpl::FormatHandle handle)
{
    _format_callbacks.unsubscribe(handle);
}

CameraServer::TakePhotoHandle
CameraServerImpl::subscribe_take_photo(const CameraServer::TakePhotoCallback& callback)
{
    return _take_photo_callbacks.subscribe(callback);
}

void CameraServerImpl::unsubscribe_take_photo(CameraServer::TakePhotoHandle handle)
{
    _take_photo_callbacks.unsubscribe(handle);
}

CameraServer::Result CameraServerImpl::respond_take_photo(
    CameraServer::TakePhotoFeedback take_photo_feedback, CameraServer::CaptureInfo capture_info)
{
    // If capture_info.index == INT32_MIN, it means this was an interval
    // capture rather than a single image capture.
    if (capture_info.index != INT32_MIN) {
        // We expect each capture to be the next sequential number.
        // If _image_capture_count == 0, we ignore since it means that this is
        // the first photo since the plugin was initialized.
        if (_image_capture_count != 0 && capture_info.index != _image_capture_count + 1) {
            LogErr() << "unexpected image index, expecting " << +(_image_capture_count + 1)
                     << " but was " << +capture_info.index;
        }

        _image_capture_count = capture_info.index;
    }

    switch (take_photo_feedback) {
        default:
            // Fallthrough
        case CameraServer::TakePhotoFeedback::Unknown:
            return CameraServer::Result::Error;
        case CameraServer::TakePhotoFeedback::Ok: {
            // Check for error above
            auto command_ack = _server_component_impl->make_command_ack_message(
                _last_take_photo_command, MAV_RESULT_ACCEPTED);
            _server_component_impl->send_command_ack(command_ack);
            // Only break and send the captured below.
            break;
        }
        case CameraServer::TakePhotoFeedback::Busy: {
            auto command_ack = _server_component_impl->make_command_ack_message(
                _last_take_photo_command, MAV_RESULT_TEMPORARILY_REJECTED);
            _server_component_impl->send_command_ack(command_ack);
            return CameraServer::Result::Success;
        }
        case CameraServer::TakePhotoFeedback::Failed: {
            auto command_ack = _server_component_impl->make_command_ack_message(
                _last_take_photo_command, MAV_RESULT_TEMPORARILY_REJECTED);
            _server_component_impl->send_command_ack(command_ack);
            return CameraServer::Result::Success;
        }
    }

    // REVISIT: Should we cache all CaptureInfo in memory for single image
    // captures so that we can respond to requests for lost CAMERA_IMAGE_CAPTURED
    // messages without calling back to user code?

    static const uint8_t camera_id = 0; // deprecated unused field

    const float attitude_quaternion[] = {
        capture_info.attitude_quaternion.w,
        capture_info.attitude_quaternion.x,
        capture_info.attitude_quaternion.y,
        capture_info.attitude_quaternion.z,
    };

    // There needs to be enough data to be copied mavlink internal.
    capture_info.file_url.resize(205);

    // TODO: this should be a broadcast message
    _server_component_impl->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
        mavlink_message_t message{};
        mavlink_msg_camera_image_captured_pack_chan(
            mavlink_address.system_id,
            mavlink_address.component_id,
            channel,
            &message,
            static_cast<uint32_t>(_server_component_impl->get_time().elapsed_s() * 1e3),
            capture_info.time_utc_us,
            camera_id,
            static_cast<int32_t>(capture_info.position.latitude_deg * 1e7),
            static_cast<int32_t>(capture_info.position.longitude_deg * 1e7),
            static_cast<int32_t>(capture_info.position.absolute_altitude_m * 1e3f),
            static_cast<int32_t>(capture_info.position.relative_altitude_m * 1e3f),
            attitude_quaternion,
            capture_info.index,
            capture_info.is_success,
            capture_info.file_url.c_str());
        return message;
    });
    LogDebug() << "sent camera image captured msg - index: " << +capture_info.index;

    return CameraServer::Result::Success;
}

/**
 * Starts capturing images with the given interval.
 * @param [in]  interval_s      The interval between captures in seconds.
 * @param [in]  count           The number of images to capture or 0 for "forever".
 * @param [in]  index           The index/sequence number pass to the user callback (always
 *                              @c INT32_MIN).
 */
void CameraServerImpl::start_image_capture_interval(float interval_s, int32_t count, int32_t index)
{
    // If count == 0, it means capture "forever" until a stop command is received.
    auto remaining = std::make_shared<int32_t>(count == 0 ? INT32_MAX : count);

    _server_component_impl->add_call_every(
        [this, remaining, index]() {
            LogDebug() << "capture image timer triggered";

            if (!_take_photo_callbacks.empty()) {
                _take_photo_callbacks(index);
                (*remaining)--;
            }

            if (*remaining == 0) {
                stop_image_capture_interval();
            }
        },
        interval_s,
        &_image_capture_timer_cookie);

    _is_image_capture_interval_set = true;
    _image_capture_timer_interval_s = interval_s;
}

/**
 * Stops any pending image capture interval timer.
 */
void CameraServerImpl::stop_image_capture_interval()
{
    if (_image_capture_timer_cookie) {
        _server_component_impl->remove_call_every(_image_capture_timer_cookie);
    }

    _image_capture_timer_cookie = nullptr;
    _is_image_capture_interval_set = false;
    _image_capture_timer_interval_s = 0;
}

std::optional<mavlink_command_ack_t> CameraServerImpl::process_camera_information_request(
    const MavlinkCommandReceiver::CommandLong& command)
{
    auto capabilities = static_cast<bool>(command.params.param1);

    if (!capabilities) {
        LogDebug() << "early info return";
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_ACCEPTED);
    }

    if (!_is_information_set) {
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_TEMPORARILY_REJECTED);
    }

    // It is safe to ignore the return value of parse_version_string() here
    // since the string was already validated in set_information().
    uint32_t firmware_version;
    parse_version_string(_information.firmware_version, firmware_version);

    // capability flags are determined by subscriptions
    uint32_t capability_flags{};

    if (!_take_photo_callbacks.empty()) {
        capability_flags |= CAMERA_CAP_FLAGS::CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
    }

    _server_component_impl->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
        mavlink_message_t message{};
        mavlink_msg_camera_information_pack_chan(
            mavlink_address.system_id,
            mavlink_address.component_id,
            channel,
            &message,
            static_cast<uint32_t>(_server_component_impl->get_time().elapsed_s() * 1e3),
            reinterpret_cast<const uint8_t*>(_information.vendor_name.c_str()),
            reinterpret_cast<const uint8_t*>(_information.model_name.c_str()),
            firmware_version,
            _information.focal_length_mm,
            _information.horizontal_sensor_size_mm,
            _information.vertical_sensor_size_mm,
            _information.horizontal_resolution_px,
            _information.vertical_resolution_px,
            _information.lens_id,
            capability_flags,
            _information.definition_file_version,
            _information.definition_file_uri.c_str());
        return message;
    });
    LogDebug() << "sent info msg";

    // ack was already sent
    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t> CameraServerImpl::process_camera_settings_request(
    const MavlinkCommandReceiver::CommandLong& command)
{
    auto settings = static_cast<bool>(command.params.param1);

    if (!settings) {
        LogDebug() << "early settings return";
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_ACCEPTED);
    }

    // unsupported
    uint8_t mode_id = static_cast<uint8_t>(CAMERA_MODE::CAMERA_MODE_IMAGE);
    float zoom_level = 0;
    float focus_level = 0;
    if(_camera_settings_callback) {
        _camera_settings_callback(mode_id, zoom_level, focus_level);
    }

    _server_component_impl->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
        mavlink_message_t message{};
        mavlink_msg_camera_settings_pack_chan(
            mavlink_address.system_id,
            mavlink_address.component_id,
            channel,
            &message,
            static_cast<uint32_t>(_server_component_impl->get_time().elapsed_s() * 1e3),
            mode_id,
            zoom_level,
            focus_level);
        return message;
    });
    LogDebug() << "sent settings msg";

    // ack was already sent
    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t> CameraServerImpl::process_storage_information_request(
    const MavlinkCommandReceiver::CommandLong& command)
{
    auto storage_id = static_cast<uint8_t>(command.params.param1);
    auto information = static_cast<bool>(command.params.param2);

    if (!information) {
        LogDebug() << "early storage return";
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_ACCEPTED);
    }

    if (!_is_storage_information_set) {
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_TEMPORARILY_REJECTED);
    }

    uint8_t video_status = 0;
    uint32_t recording_time_ms = 0;
    if(_video_status_callback) {
        _video_status_callback(video_status, recording_time_ms, _storage_information.available_capacity);
    }

    _server_component_impl->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
        mavlink_message_t message{};
        mavlink_msg_storage_information_pack_chan(
            mavlink_address.system_id,
            mavlink_address.component_id,
            channel,
            &message,
            static_cast<uint32_t>(_server_component_impl->get_time().elapsed_s() * 1e3),
            storage_id,
            _storage_information.storage_count,
            _storage_information.status,
            _storage_information.total_capacity,
            _storage_information.used_capacity,
            _storage_information.available_capacity,
            _storage_information.read_speed,
            _storage_information.write_speed,
            _storage_information.type,
            _storage_information.name.data(),
            _storage_information.storage_usage);

        return message;
    });

    LogDebug() << "sent storage msg";

    // ack was already sent
    return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_storage_format(const MavlinkCommandReceiver::CommandLong& command)
{
    auto storage_id = static_cast<uint8_t>(command.params.param1);
    auto format = static_cast<bool>(command.params.param2);
    auto reset_image_log = static_cast<bool>(command.params.param3);

    if (_format_callbacks.empty()) {
        LogDebug() << "format requested with no format subscriber";
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
    }

    _format_callbacks(storage_id, format, reset_image_log);

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t> CameraServerImpl::process_camera_capture_status_request(
    const MavlinkCommandReceiver::CommandLong& command)
{
    auto capture_status = static_cast<bool>(command.params.param1);

    if (!capture_status) {
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_ACCEPTED);
    }

    uint8_t image_status{};

    if (_is_image_capture_in_progress) {
        image_status |= StatusFlags::IN_PROGRESS;
    }

    if (_is_image_capture_interval_set) {
        image_status |= StatusFlags::INTERVAL_SET;
    }

    uint8_t video_status = 0;
    uint32_t recording_time_ms = 0;
    float available_capacity = 0;
    if(_video_status_callback) {
        _video_status_callback(video_status, recording_time_ms, available_capacity);
    }

    _server_component_impl->queue_message([&](MavlinkAddress mavlink_address, uint8_t channel) {
        mavlink_message_t message{};
        mavlink_msg_camera_capture_status_pack_chan(
            mavlink_address.system_id,
            mavlink_address.component_id,
            channel,
            &message,
            static_cast<uint32_t>(_server_component_impl->get_time().elapsed_s() * 1e3),
            image_status,
            video_status,
            _image_capture_timer_interval_s,
            recording_time_ms,
            available_capacity,
            _image_capture_count);
        return message;
    });

    // ack was already sent
    return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_reset_camera_settings(const MavlinkCommandReceiver::CommandLong& command)
{
    auto reset = static_cast<bool>(command.params.param1);

    if (_reset_callbacks.empty()) {
        LogDebug() << "reset requested with no reset subscriber";
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
    }

    _reset_callbacks(reset);

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_set_camera_mode(const MavlinkCommandReceiver::CommandLong& command)
{
    auto camera_mode = static_cast<CAMERA_MODE>(command.params.param2);

    if (_mode_callbacks.empty()) {
        LogDebug() << "mode requested with no mode subscriber";
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
    }

    _mode_callbacks(camera_mode);

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_set_camera_zoom(const MavlinkCommandReceiver::CommandLong& command)
{
    auto zoom_type = static_cast<CAMERA_ZOOM_TYPE>(command.params.param1);
    auto zoom_value = command.params.param2;

    if (_zoom_callbacks.empty()) {
        LogDebug() << "zoom requested with no zoom subscriber";
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
    }

    _zoom_callbacks(zoom_type, zoom_value);

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_set_camera_focus(const MavlinkCommandReceiver::CommandLong& command)
{
    auto focus_type = static_cast<SET_FOCUS_TYPE>(command.params.param1);
    auto focus_value = command.params.param2;

    if (_focus_callbacks.empty()) {
        LogDebug() << "focus requested with no focus subscriber";
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
    }

    _focus_callbacks(focus_type, focus_value);

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_set_storage_usage(const MavlinkCommandReceiver::CommandLong& command)
{
    auto storage_id = static_cast<uint8_t>(command.params.param1);
    auto usage = static_cast<STORAGE_USAGE_FLAG>(command.params.param2);

    UNUSED(storage_id);
    UNUSED(usage);

    LogDebug() << "unsupported set storage usage request";

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_image_start_capture(const MavlinkCommandReceiver::CommandLong& command)
{
    auto interval_s = command.params.param2;
    auto total_images = static_cast<int32_t>(command.params.param3);
    auto seq_number = static_cast<int32_t>(command.params.param4);

    LogDebug() << "received image start capture request - interval: " << +interval_s
               << " total: " << +total_images << " index: " << +seq_number;

    // TODO: validate parameters and return MAV_RESULT_DENIED not valid

    stop_image_capture_interval();

    if (_take_photo_callbacks.empty()) {
        LogDebug() << "image capture requested with no take photo subscriber";
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
    }

    // single image capture
    if (total_images == 1) {
        if (seq_number <= _image_capture_count) {
            LogDebug() << "received duplicate single image capture request";
            // We know we already captured this request, so we can just ack it.
            return _server_component_impl->make_command_ack_message(
                command, MAV_RESULT::MAV_RESULT_ACCEPTED);
        }

        // MAV_RESULT_ACCEPTED must be sent before CAMERA_IMAGE_CAPTURED
        auto command_ack = _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_IN_PROGRESS);
        _server_component_impl->send_command_ack(command_ack);

        _last_take_photo_command = command;

        _take_photo_callbacks(seq_number);

        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_ACCEPTED);
    }

    start_image_capture_interval(interval_s, total_images, seq_number);

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_image_stop_capture(const MavlinkCommandReceiver::CommandLong& command)
{
    LogDebug() << "received image stop capture request";

    // REVISIT: should we return something other that MAV_RESULT_ACCEPTED if
    // there is not currently a capture interval active?
    stop_image_capture_interval();

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t> CameraServerImpl::process_camera_image_capture_request(
    const MavlinkCommandReceiver::CommandLong& command)
{
    auto seq_number = static_cast<uint32_t>(command.params.param1);

    UNUSED(seq_number);

    LogDebug() << "unsupported image capture request";

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_video_start_capture(const MavlinkCommandReceiver::CommandLong& command)
{
    auto stream_id = static_cast<uint8_t>(command.params.param1);
    auto status_frequency = command.params.param2;

    UNUSED(status_frequency);

    if (_video_callbacks.empty()) {
        LogDebug() << "video start capture requested with no video subscriber";
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
    }

    bool ok = true;
    _video_callbacks(stream_id, ok);

    return _server_component_impl->make_command_ack_message(
        command, ok ? MAV_RESULT::MAV_RESULT_ACCEPTED : MAV_RESULT::MAV_RESULT_TEMPORARILY_REJECTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_video_stop_capture(const MavlinkCommandReceiver::CommandLong& command)
{
    auto stream_id = static_cast<uint8_t>(command.params.param1);

    if (_video_callbacks.empty()) {
        LogDebug() << "video stop capture requested with no video subscriber";
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
    }

    bool ok = false;
    _video_callbacks(stream_id, ok);

    return _server_component_impl->make_command_ack_message(
        command, ok ? MAV_RESULT::MAV_RESULT_ACCEPTED : MAV_RESULT::MAV_RESULT_TEMPORARILY_REJECTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_video_start_streaming(const MavlinkCommandReceiver::CommandLong& command)
{
    auto stream_id = static_cast<uint8_t>(command.params.param1);

    UNUSED(stream_id);

    LogDebug() << "unsupported video start streaming request";

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
}

std::optional<mavlink_command_ack_t>
CameraServerImpl::process_video_stop_streaming(const MavlinkCommandReceiver::CommandLong& command)
{
    auto stream_id = static_cast<uint8_t>(command.params.param1);

    UNUSED(stream_id);

    LogDebug() << "unsupported video stop streaming request";

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
}

std::optional<mavlink_command_ack_t> CameraServerImpl::process_video_stream_information_request(
    const MavlinkCommandReceiver::CommandLong& command)
{
    auto stream_id = static_cast<uint8_t>(command.params.param1);

    UNUSED(stream_id);

    LogDebug() << "unsupported video stream information request";

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
}

std::optional<mavlink_command_ack_t> CameraServerImpl::process_video_stream_status_request(
    const MavlinkCommandReceiver::CommandLong& command)
{
    auto stream_id = static_cast<uint8_t>(command.params.param1);

    UNUSED(stream_id);

    LogDebug() << "unsupported video stream status request";

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_UNSUPPORTED);
}

} // namespace mavsdk
