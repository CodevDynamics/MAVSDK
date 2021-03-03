// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/main/protos/camera/camera.proto)

#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "plugin_base.h"

namespace mavsdk {

class System;
class CameraImpl;

/**
 * @brief Can be used to manage cameras that implement the MAVLink
 * Camera Protocol: https://mavlink.io/en/protocol/camera.html.
 *
 * Currently only a single camera is supported.
 * When multiple cameras are supported the plugin will need to be
 * instantiated separately for every camera and the camera selected using
 * `select_camera`.
 */
class Camera : public PluginBase {
public:
    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto camera = Camera(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit Camera(System& system); // deprecated

    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto camera = Camera(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit Camera(std::shared_ptr<System> system); // new

    /**
     * @brief Destructor (internal use only).
     */
    ~Camera();

    /**
     * @brief Camera mode type.
     */
    enum class Mode {
        Unknown, /**< @brief Unknown. */
        Photo, /**< @brief Photo mode. */
        Video, /**< @brief Video mode. */
    };

    /**
     * @brief Stream operator to print information about a `Camera::Mode`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Camera::Mode const& mode);

    /**
     * @brief Photos range type.
     */
    enum class PhotosRange {
        All, /**< @brief All the photos present on the camera. */
        SinceConnection, /**< @brief Photos taken since MAVSDK got connected. */
    };

    /**
     * @brief Stream operator to print information about a `Camera::PhotosRange`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Camera::PhotosRange const& photos_range);

    /**
     * @brief Possible results returned for camera commands
     */
    enum class Result {
        Unknown, /**< @brief Unknown result. */
        Success, /**< @brief Command executed successfully. */
        InProgress, /**< @brief Command in progress. */
        Busy, /**< @brief Camera is busy and rejected command. */
        Denied, /**< @brief Camera denied the command. */
        Error, /**< @brief An error has occured while executing the command. */
        Timeout, /**< @brief Command timed out. */
        WrongArgument, /**< @brief Command has wrong argument(s). */
    };

    /**
     * @brief Stream operator to print information about a `Camera::Result`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Camera::Result const& result);

    /**
     * @brief Position type in global coordinates.
     */
    struct Position {
        double latitude_deg{}; /**< @brief Latitude in degrees (range: -90 to +90) */
        double longitude_deg{}; /**< @brief Longitude in degrees (range: -180 to +180) */
        float absolute_altitude_m{}; /**< @brief Altitude AMSL (above mean sea level) in metres */
        float relative_altitude_m{}; /**< @brief Altitude relative to takeoff altitude in metres */
    };

    /**
     * @brief Equal operator to compare two `Camera::Position` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Camera::Position& lhs, const Camera::Position& rhs);

    /**
     * @brief Stream operator to print information about a `Camera::Position`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Camera::Position const& position);

    /**
     * @brief Quaternion type.
     *
     * All rotations and axis systems follow the right-hand rule.
     * The Hamilton quaternion product definition is used.
     * A zero-rotation quaternion is represented by (1,0,0,0).
     * The quaternion could also be written as w + xi + yj + zk.
     *
     * For more info see: https://en.wikipedia.org/wiki/Quaternion
     */
    struct Quaternion {
        float w{}; /**< @brief Quaternion entry 0, also denoted as a */
        float x{}; /**< @brief Quaternion entry 1, also denoted as b */
        float y{}; /**< @brief Quaternion entry 2, also denoted as c */
        float z{}; /**< @brief Quaternion entry 3, also denoted as d */
    };

    /**
     * @brief Equal operator to compare two `Camera::Quaternion` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Camera::Quaternion& lhs, const Camera::Quaternion& rhs);

    /**
     * @brief Stream operator to print information about a `Camera::Quaternion`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Camera::Quaternion const& quaternion);

    /**
     * @brief Euler angle type.
     *
     * All rotations and axis systems follow the right-hand rule.
     * The Euler angles follow the convention of a 3-2-1 intrinsic Tait-Bryan rotation sequence.
     *
     * For more info see https://en.wikipedia.org/wiki/Euler_angles
     */
    struct EulerAngle {
        float roll_deg{}; /**< @brief Roll angle in degrees, positive is banking to the right */
        float pitch_deg{}; /**< @brief Pitch angle in degrees, positive is pitching nose up */
        float yaw_deg{}; /**< @brief Yaw angle in degrees, positive is clock-wise seen from above */
    };

    /**
     * @brief Equal operator to compare two `Camera::EulerAngle` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Camera::EulerAngle& lhs, const Camera::EulerAngle& rhs);

    /**
     * @brief Stream operator to print information about a `Camera::EulerAngle`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Camera::EulerAngle const& euler_angle);

    /**
     * @brief Information about a picture just captured.
     */
    struct CaptureInfo {
        Position position{}; /**< @brief Location where the picture was taken */
        Quaternion attitude_quaternion{}; /**< @brief Attitude of the camera when the picture was
                                             taken (quaternion) */
        EulerAngle attitude_euler_angle{}; /**< @brief Attitude of the camera when the picture was
                                              taken (euler angle) */
        uint64_t time_utc_us{}; /**< @brief Timestamp in UTC (since UNIX epoch) in microseconds */
        bool is_success{}; /**< @brief True if the capture was successful */
        int32_t index{}; /**< @brief Zero-based index of this image since vehicle was armed */
        std::string file_url{}; /**< @brief Download URL of this image */
    };

    /**
     * @brief Equal operator to compare two `Camera::CaptureInfo` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Camera::CaptureInfo& lhs, const Camera::CaptureInfo& rhs);

    /**
     * @brief Stream operator to print information about a `Camera::CaptureInfo`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Camera::CaptureInfo const& capture_info);

    /**
     * @brief Type for video stream settings.
     */
    struct VideoStreamSettings {
        float frame_rate_hz{}; /**< @brief Frames per second */
        uint32_t horizontal_resolution_pix{}; /**< @brief Horizontal resolution (in pixels) */
        uint32_t vertical_resolution_pix{}; /**< @brief Vertical resolution (in pixels) */
        uint32_t bit_rate_b_s{}; /**< @brief Bit rate (in bits per second) */
        uint32_t rotation_deg{}; /**< @brief Video image rotation (clockwise, 0-359 degrees) */
        std::string uri{}; /**< @brief Video stream URI */
        float horizontal_fov_deg{}; /**< @brief Horizontal fov in degrees */
    };

    /**
     * @brief Equal operator to compare two `Camera::VideoStreamSettings` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool
    operator==(const Camera::VideoStreamSettings& lhs, const Camera::VideoStreamSettings& rhs);

    /**
     * @brief Stream operator to print information about a `Camera::VideoStreamSettings`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, Camera::VideoStreamSettings const& video_stream_settings);

    /**
     * @brief Information about the video stream.
     */
    struct VideoStreamInfo {
        /**
         * @brief Video stream status type.
         */
        enum class VideoStreamStatus {
            NotRunning, /**< @brief Video stream is not running. */
            InProgress, /**< @brief Video stream is running. */
        };

        /**
         * @brief Stream operator to print information about a `Camera::VideoStreamStatus`.
         *
         * @return A reference to the stream.
         */
        friend std::ostream& operator<<(
            std::ostream& str,
            Camera::VideoStreamInfo::VideoStreamStatus const& video_stream_status);

        /**
         * @brief Video stream light spectrum type
         */
        enum class VideoStreamSpectrum {
            Unknown, /**< @brief Unknown. */
            VisibleLight, /**< @brief Visible light. */
            Infrared, /**< @brief Infrared. */
        };

        /**
         * @brief Stream operator to print information about a `Camera::VideoStreamSpectrum`.
         *
         * @return A reference to the stream.
         */
        friend std::ostream& operator<<(
            std::ostream& str,
            Camera::VideoStreamInfo::VideoStreamSpectrum const& video_stream_spectrum);

        VideoStreamSettings settings{}; /**< @brief Video stream settings */
        VideoStreamStatus status{}; /**< @brief Current status of video streaming */
        VideoStreamSpectrum spectrum{}; /**< @brief Light-spectrum of the video stream */
    };

    /**
     * @brief Equal operator to compare two `Camera::VideoStreamInfo` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Camera::VideoStreamInfo& lhs, const Camera::VideoStreamInfo& rhs);

    /**
     * @brief Stream operator to print information about a `Camera::VideoStreamInfo`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, Camera::VideoStreamInfo const& video_stream_info);

    /**
     * @brief Information about the camera status.
     */
    struct Status {
        /**
         * @brief Storage status type.
         */
        enum class StorageStatus {
            NotAvailable, /**< @brief Status not available. */
            Unformatted, /**< @brief Storage is not formatted (i.e. has no recognized file system).
                          */
            Formatted, /**< @brief Storage is formatted (i.e. has recognized a file system). */
        };

        /**
         * @brief Stream operator to print information about a `Camera::StorageStatus`.
         *
         * @return A reference to the stream.
         */
        friend std::ostream&
        operator<<(std::ostream& str, Camera::Status::StorageStatus const& storage_status);

        bool video_on{}; /**< @brief Whether video recording is currently in process */
        bool photo_interval_on{}; /**< @brief Whether a photo interval is currently in process */
        float used_storage_mib{}; /**< @brief Used storage (in MiB) */
        float available_storage_mib{}; /**< @brief Available storage (in MiB) */
        float total_storage_mib{}; /**< @brief Total storage (in MiB) */
        float recording_time_s{}; /**< @brief Elapsed time since starting the video recording (in
                                     seconds) */
        std::string media_folder_name{}; /**< @brief Current folder name where media are saved */
        StorageStatus storage_status{}; /**< @brief Storage status */
    };

    /**
     * @brief Equal operator to compare two `Camera::Status` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Camera::Status& lhs, const Camera::Status& rhs);

    /**
     * @brief Stream operator to print information about a `Camera::Status`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Camera::Status const& status);

    /**
     * @brief Type to represent a setting option.
     */
    struct Option {
        std::string option_id{}; /**< @brief Name of the option (machine readable) */
        std::string option_description{}; /**< @brief Description of the option (human readable) */
    };

    /**
     * @brief Equal operator to compare two `Camera::Option` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Camera::Option& lhs, const Camera::Option& rhs);

    /**
     * @brief Stream operator to print information about a `Camera::Option`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Camera::Option const& option);

    /**
     * @brief Type to represent a setting with a selected option.
     */
    struct Setting {
        std::string setting_id{}; /**< @brief Name of a setting (machine readable) */
        std::string setting_description{}; /**< @brief Description of the setting (human readable).
                                              This field is meant to be read from the drone, ignore
                                              it when setting. */
        Option option{}; /**< @brief Selected option */
        bool is_range{}; /**< @brief If option is given as a range. This field is meant to be read
                            from the drone, ignore it when setting. */
    };

    /**
     * @brief Equal operator to compare two `Camera::Setting` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Camera::Setting& lhs, const Camera::Setting& rhs);

    /**
     * @brief Stream operator to print information about a `Camera::Setting`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Camera::Setting const& setting);

    /**
     * @brief Type to represent a setting with a list of options to choose from.
     */
    struct SettingOptions {
        std::string setting_id{}; /**< @brief Name of the setting (machine readable) */
        std::string
            setting_description{}; /**< @brief Description of the setting (human readable) */
        std::vector<Option>
            options{}; /**< @brief List of options or if range [min, max] or [min, max, interval] */
        bool is_range{}; /**< @brief If option is given as a range */
    };

    /**
     * @brief Equal operator to compare two `Camera::SettingOptions` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Camera::SettingOptions& lhs, const Camera::SettingOptions& rhs);

    /**
     * @brief Stream operator to print information about a `Camera::SettingOptions`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, Camera::SettingOptions const& setting_options);

    /**
     * @brief Type to represent a camera information.
     */
    struct Information {
        std::string vendor_name{}; /**< @brief Name of the camera vendor */
        std::string model_name{}; /**< @brief Name of the camera model */
    };

    /**
     * @brief Equal operator to compare two `Camera::Information` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Camera::Information& lhs, const Camera::Information& rhs);

    /**
     * @brief Stream operator to print information about a `Camera::Information`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Camera::Information const& information);

    /**
     * @brief Callback type for asynchronous Camera calls.
     */
    using ResultCallback = std::function<void(Result)>;

    /**
     * @brief Take one photo.
     *
     * This function is non-blocking. See 'take_photo' for the blocking counterpart.
     */
    void take_photo_async(const ResultCallback callback);

    /**
     * @brief Take one photo.
     *
     * This function is blocking. See 'take_photo_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result take_photo() const;

    /**
     * @brief Start photo timelapse with a given interval.
     *
     * This function is non-blocking. See 'start_photo_interval' for the blocking counterpart.
     */
    void start_photo_interval_async(float interval_s, const ResultCallback callback);

    /**
     * @brief Start photo timelapse with a given interval.
     *
     * This function is blocking. See 'start_photo_interval_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result start_photo_interval(float interval_s) const;

    /**
     * @brief Stop a running photo timelapse.
     *
     * This function is non-blocking. See 'stop_photo_interval' for the blocking counterpart.
     */
    void stop_photo_interval_async(const ResultCallback callback);

    /**
     * @brief Stop a running photo timelapse.
     *
     * This function is blocking. See 'stop_photo_interval_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result stop_photo_interval() const;

    /**
     * @brief Start a video recording.
     *
     * This function is non-blocking. See 'start_video' for the blocking counterpart.
     */
    void start_video_async(const ResultCallback callback);

    /**
     * @brief Start a video recording.
     *
     * This function is blocking. See 'start_video_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result start_video() const;

    /**
     * @brief Stop a running video recording.
     *
     * This function is non-blocking. See 'stop_video' for the blocking counterpart.
     */
    void stop_video_async(const ResultCallback callback);

    /**
     * @brief Stop a running video recording.
     *
     * This function is blocking. See 'stop_video_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result stop_video() const;

    /**
     * @brief Start video streaming.
     *
     * This function is blocking.
     *
     * @return Result of request.
     */
    Result start_video_streaming() const;

    /**
     * @brief Stop current video streaming.
     *
     * This function is blocking.
     *
     * @return Result of request.
     */
    Result stop_video_streaming() const;

    /**
     * @brief Set camera mode.
     *
     * This function is non-blocking. See 'set_mode' for the blocking counterpart.
     */
    void set_mode_async(Mode mode, const ResultCallback callback);

    /**
     * @brief Set camera mode.
     *
     * This function is blocking. See 'set_mode_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result set_mode(Mode mode) const;

    /**
     * @brief Callback type for list_photos_async.
     */
    using ListPhotosCallback = std::function<void(Result, std::vector<CaptureInfo>)>;

    /**
     * @brief List photos available on the camera.
     *
     * This function is non-blocking. See 'list_photos' for the blocking counterpart.
     */
    void list_photos_async(PhotosRange photos_range, const ListPhotosCallback callback);

    /**
     * @brief List photos available on the camera.
     *
     * This function is blocking. See 'list_photos_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    std::pair<Result, std::vector<Camera::CaptureInfo>> list_photos(PhotosRange photos_range) const;

    /**
     * @brief Callback type for subscribe_mode.
     */

    using ModeCallback = std::function<void(Mode)>;

    /**
     * @brief Subscribe to camera mode updates.
     */
    void subscribe_mode(ModeCallback callback);

    /**
     * @brief Poll for 'Mode' (blocking).
     *
     * @return One Mode update.
     */
    Mode mode() const;

    /**
     * @brief Callback type for subscribe_information.
     */

    using InformationCallback = std::function<void(Information)>;

    /**
     * @brief Subscribe to camera information updates.
     */
    void subscribe_information(InformationCallback callback);

    /**
     * @brief Poll for 'Information' (blocking).
     *
     * @return One Information update.
     */
    Information information() const;

    /**
     * @brief Callback type for subscribe_video_stream_info.
     */

    using VideoStreamInfoCallback = std::function<void(VideoStreamInfo)>;

    /**
     * @brief Subscribe to video stream info updates.
     */
    void subscribe_video_stream_info(VideoStreamInfoCallback callback);

    /**
     * @brief Poll for 'VideoStreamInfo' (blocking).
     *
     * @return One VideoStreamInfo update.
     */
    VideoStreamInfo video_stream_info() const;

    /**
     * @brief Callback type for subscribe_capture_info.
     */

    using CaptureInfoCallback = std::function<void(CaptureInfo)>;

    /**
     * @brief Subscribe to capture info updates.
     */
    void subscribe_capture_info(CaptureInfoCallback callback);

    /**
     * @brief Callback type for subscribe_status.
     */

    using StatusCallback = std::function<void(Status)>;

    /**
     * @brief Subscribe to camera status updates.
     */
    void subscribe_status(StatusCallback callback);

    /**
     * @brief Poll for 'Status' (blocking).
     *
     * @return One Status update.
     */
    Status status() const;

    /**
     * @brief Callback type for subscribe_current_settings.
     */

    using CurrentSettingsCallback = std::function<void(std::vector<Setting>)>;

    /**
     * @brief Get the list of current camera settings.
     */
    void subscribe_current_settings(CurrentSettingsCallback callback);

    /**
     * @brief Callback type for subscribe_possible_setting_options.
     */

    using PossibleSettingOptionsCallback = std::function<void(std::vector<SettingOptions>)>;

    /**
     * @brief Get the list of settings that can be changed.
     */
    void subscribe_possible_setting_options(PossibleSettingOptionsCallback callback);

    /**
     * @brief Poll for 'std::vector<SettingOptions>' (blocking).
     *
     * @return One std::vector<SettingOptions> update.
     */
    std::vector<SettingOptions> possible_setting_options() const;

    /**
     * @brief Set a setting to some value.
     *
     * Only setting_id of setting and option_id of option needs to be set.
     *
     * This function is non-blocking. See 'set_setting' for the blocking counterpart.
     */
    void set_setting_async(Setting setting, const ResultCallback callback);

    /**
     * @brief Set a setting to some value.
     *
     * Only setting_id of setting and option_id of option needs to be set.
     *
     * This function is blocking. See 'set_setting_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result set_setting(Setting setting) const;

    /**
     * @brief Callback type for get_setting_async.
     */
    using GetSettingCallback = std::function<void(Result, Setting)>;

    /**
     * @brief Get a setting.
     *
     * Only setting_id of setting needs to be set.
     *
     * This function is non-blocking. See 'get_setting' for the blocking counterpart.
     */
    void get_setting_async(Setting setting, const GetSettingCallback callback);

    /**
     * @brief Get a setting.
     *
     * Only setting_id of setting needs to be set.
     *
     * This function is blocking. See 'get_setting_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    std::pair<Result, Camera::Setting> get_setting(Setting setting) const;

    /**
     * @brief Format storage (e.g. SD card) in camera.
     *
     * This will delete all content of the camera storage!
     *
     * This function is non-blocking. See 'format_storage' for the blocking counterpart.
     */
    void format_storage_async(const ResultCallback callback);

    /**
     * @brief Format storage (e.g. SD card) in camera.
     *
     * This will delete all content of the camera storage!
     *
     * This function is blocking. See 'format_storage_async' for the non-blocking counterpart.
     *
     * @return Result of request.
     */
    Result format_storage() const;

    /**
     * @brief Copy constructor.
     */
    Camera(const Camera& other);

    /**
     * @brief Equality operator (object is not copyable).
     */
    const Camera& operator=(const Camera&) = delete;

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<CameraImpl> _impl;
};

} // namespace mavsdk