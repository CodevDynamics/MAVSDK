#pragma once

#include "mavlink_include.h"
#include "log.h"
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <cstring>
#include <cassert>

namespace mavsdk {

/**
 * This is a c++ helper for a mavlink extended or non-extended param value.
 */
class ParamValue {
public:
    bool set_from_mavlink_param_value_bytewise(const mavlink_param_value_t& mavlink_value);
    bool set_from_mavlink_param_value_cast(const mavlink_param_value_t& mavlink_value);
    bool set_from_mavlink_param_set_bytewise(const mavlink_param_set_t& mavlink_set);
    bool set_from_mavlink_param_ext_set(const mavlink_param_ext_set_t& mavlink_ext_set);
    bool set_from_mavlink_param_ext_value(const mavlink_param_ext_value_t& mavlink_ext_value);
    bool set_from_mavlink_param_ext_ack(const mavlink_param_ext_ack_t& mavlink_ext_ack);
    bool set_from_xml(const std::string& type_str, const std::string& value_str);
    bool set_to_min_from_xml_type(const std::string& type_str);
    bool set_to_max_from_xml_type(const std::string& type_str);
    bool set_empty_type_from_xml(const std::string& type_str);
    enum class Conversion { Cast, Bitwise };
    bool set_from_mavlink_param_value(
        const mavlink_param_value_t& mavlink_value, const Conversion& conversion);

    [[nodiscard]] MAV_PARAM_TYPE get_mav_param_type() const;
    [[nodiscard]] MAV_PARAM_EXT_TYPE get_mav_param_ext_type() const;

    bool set_as_same_type(const std::string& value_str);

    [[nodiscard]] float get_4_float_bytes_bytewise() const;
    [[nodiscard]] float get_4_float_bytes_cast() const;

    [[nodiscard]] std::optional<int> get_int() const;
    [[nodiscard]] std::optional<float> get_float() const;
    [[nodiscard]] std::optional<std::string> get_custom() const;

    [[nodiscard]] std::optional<int64_t> get_int64() const;
    [[nodiscard]] std::optional<double> get_double() const;

    bool set_int(int new_value);
    void set_float(float new_value);
    void set_custom(const std::string& new_value);

    [[nodiscard]] std::array<char, 128> get_128_bytes() const;

    [[nodiscard]] std::string get_string() const;

    // Note: the implementation here needs to stay in the header,unfortunately.
    template<typename T> [[nodiscard]] constexpr bool is() const
    {
        // is this better ? std::holds_alternative<T>(_value)
        return (std::get_if<T>(&_value) != nullptr);
    }

    template<typename T> T get() const { return std::get<T>(_value); }

    template<typename T> void set(T new_value) { _value = new_value; }

    [[nodiscard]] bool is_same_type(const ParamValue& rhs) const;

    // update the value of a parameter without mutating its type
    void update_value_typesafe(const ParamValue& new_value);

    bool operator==(const ParamValue& rhs) const
    {
        if (!is_same_type(rhs)) {
            LogWarn() << "Trying to compare different types.";
            return false;
        }

        return _value == rhs._value;
    }
    bool operator!=(const ParamValue& rhs) const
    {
        if (!is_same_type(rhs)) {
            LogWarn() << "Trying to compare different types.";
            return true;
        }
        return !(*this == rhs);
    }

    bool operator<(const ParamValue& rhs) const
    {
        if (!is_same_type(rhs)) {
            LogWarn() << "Trying to compare different types.";
            return false;
        }

        return _value < rhs._value;
    }

    bool operator>(const ParamValue& rhs) const
    {
        if (!is_same_type(rhs)) {
            LogWarn() << "Trying to compare different types.";
            return false;
        }

        return _value > rhs._value;
    }

    bool operator==(const std::string& value_str) const;

    [[nodiscard]] std::string typestr() const;

    [[nodiscard]] constexpr bool needs_extended() const
    {
        // Returns true if this parameter needs the extended parameters' protocol
        // which is the case when its value is represented by a string or bigger than 4 bytes.
        return !is<float>() && !is<int32_t>() && !is<uint32_t>();
    }

private:
    std::variant<
        uint8_t,
        int8_t,
        uint16_t,
        int16_t,
        uint32_t,
        int32_t,
        uint64_t,
        int64_t,
        float,
        double,
        std::string>
        _value{};
};

std::ostream& operator<<(std::ostream& str, const ParamValue& obj);

} // namespace mavsdk