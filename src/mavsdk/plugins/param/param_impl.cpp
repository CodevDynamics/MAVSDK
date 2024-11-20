#include <functional>
#include "param_impl.h"
#include "system.h"
#include "unused.h"

namespace mavsdk {

ParamImpl::ParamImpl(System& system) : PluginImplBase(system)
{
    _system_impl->register_plugin(this);
}

ParamImpl::ParamImpl(std::shared_ptr<System> system) : PluginImplBase(std::move(system))
{
    _system_impl->register_plugin(this);
}

ParamImpl::~ParamImpl()
{
    _system_impl->unregister_plugin(this);
}

void ParamImpl::init()
{
    _param_clients.push_back(_system_impl->param_sender(_component_id, _protocol_version == Param::ProtocolVersion::Ext));
}

void ParamImpl::deinit()
{
    for (auto* client : _param_clients) {
        client->unsubscribe_all_params_changed(this);
    }
}

void ParamImpl::enable() {}

void ParamImpl::disable() {}

std::pair<Param::Result, ParamValue> ParamImpl::get_param(const std::string& name)
{
    auto result = _system_impl->param_sender(_component_id, _protocol_version == Param::ProtocolVersion::Ext)->get_param(name);
    return std::make_pair<>(
        result_from_mavlink_parameter_client_result(result.first), result.second);
}

Param::Result ParamImpl::set_param(const std::string& name, ParamValue value)
{
    auto result = _system_impl->param_sender(_component_id, _protocol_version == Param::ProtocolVersion::Ext)->set_param(name, value);
    return result_from_mavlink_parameter_client_result(result);
}

std::pair<Param::Result, int32_t> ParamImpl::get_param_int(const std::string& name)
{
    std::pair<MavlinkParameterClient::Result, int32_t> result = _system_impl->get_param_int(
        name, _component_id, _protocol_version == Param::ProtocolVersion::Ext);
    return std::make_pair<>(
        result_from_mavlink_parameter_client_result(result.first), result.second);
}

Param::Result ParamImpl::set_param_int(const std::string& name, int32_t value)
{
    MavlinkParameterClient::Result result = _system_impl->set_param_int(
        name, value, _component_id, _protocol_version == Param::ProtocolVersion::Ext);
    return result_from_mavlink_parameter_client_result(result);
}

std::pair<Param::Result, float> ParamImpl::get_param_float(const std::string& name)
{
    std::pair<MavlinkParameterClient::Result, float> result = _system_impl->get_param_float(
        name, _component_id, _protocol_version == Param::ProtocolVersion::Ext);
    return std::make_pair<>(
        result_from_mavlink_parameter_client_result(result.first), result.second);
}

Param::Result ParamImpl::set_param_float(const std::string& name, float value)
{
    MavlinkParameterClient::Result result = _system_impl->set_param_float(
        name, value, _component_id, _protocol_version == Param::ProtocolVersion::Ext);
    return result_from_mavlink_parameter_client_result(result);
}

std::pair<Param::Result, std::string> ParamImpl::get_param_custom(const std::string& name)
{
    auto result = _system_impl->get_param_custom(name, _component_id);
    return std::make_pair<>(
        result_from_mavlink_parameter_client_result(result.first), result.second);
}

Param::Result ParamImpl::set_param_custom(const std::string& name, const std::string& value)
{
    auto result = _system_impl->set_param_custom(name, value, _component_id);
    return result_from_mavlink_parameter_client_result(result);
}

Param::AllParams ParamImpl::get_all_params()
{
    auto tmp = _system_impl->get_all_params(
        _component_id, _protocol_version == Param::ProtocolVersion::Ext);

    if (tmp.first != MavlinkParameterClient::Result::Success) {
        return {};
    }

    Param::AllParams res{};
    for (auto const& param_pair : tmp.second) {
        if (param_pair.second.is<float>()) {
            Param::FloatParam tmp_param;
            tmp_param.name = param_pair.first;
            tmp_param.value = param_pair.second.get<float>();
            res.float_params.push_back(tmp_param);
        } else if (param_pair.second.is<int32_t>()) {
            Param::IntParam tmp_param;
            tmp_param.name = param_pair.first;
            tmp_param.value = param_pair.second.get<int32_t>();
            res.int_params.push_back(tmp_param);
        } else if (param_pair.second.is<int16_t>()) {
            Param::IntParam tmp_param;
            tmp_param.name = param_pair.first;
            tmp_param.value = param_pair.second.get<int16_t>();
            res.int_params.push_back(tmp_param);
        } else if (param_pair.second.is<int8_t>()) {
            Param::IntParam tmp_param;
            tmp_param.name = param_pair.first;
            tmp_param.value = param_pair.second.get<int8_t>();
            res.int_params.push_back(tmp_param);
        } else if (param_pair.second.is<std::string>()) {
            Param::CustomParam tmp_param;
            tmp_param.name = param_pair.first;
            tmp_param.value = param_pair.second.get<std::string>();
            res.custom_params.push_back(tmp_param);
        }
    }

    return res;
}

Param::Result
ParamImpl::select_component(int32_t component_id, Param::ProtocolVersion protocol_version)
{
    _component_id = component_id;
    _protocol_version = protocol_version;

    auto param_client = _system_impl->param_sender(_component_id, _protocol_version == Param::ProtocolVersion::Ext);
    auto it = std::find(_param_clients.begin(), _param_clients.end(), param_client);
    if (it == _param_clients.end()) {
        _param_clients.push_back(param_client);
    }
    
    return Param::Result::Unknown;
}

void
ParamImpl::subscribe_param_float_changed(const std::string& name, const MavlinkParameterSubscription::ParamFloatChangedCallback& callback)
{
    _system_impl->param_sender(_component_id, _protocol_version == Param::ProtocolVersion::Ext)->subscribe_param_float_changed(name, callback, this);
}

void
ParamImpl::subscribe_param_int_changed(const std::string& name, const MavlinkParameterSubscription::ParamIntChangedCallback& callback)
{
    _system_impl->param_sender(_component_id, _protocol_version == Param::ProtocolVersion::Ext)->subscribe_param_int_changed(name, callback, this);
}

void
ParamImpl::subscribe_param_custom_changed(const std::string& name, const MavlinkParameterSubscription::ParamCustomChangedCallback& callback)
{
    _system_impl->param_sender(_component_id, _protocol_version == Param::ProtocolVersion::Ext)->subscribe_param_custom_changed(name, callback, this);
}

void
ParamImpl::unsubscribe_param_all_changed()
{
    for (auto* client : _param_clients) {
        client->unsubscribe_all_params_changed(this);
    }
}

Param::Result
ParamImpl::result_from_mavlink_parameter_client_result(MavlinkParameterClient::Result result)
{
    switch (result) {
        case MavlinkParameterClient::Result::Success:
            return Param::Result::Success;
        case MavlinkParameterClient::Result::Timeout:
            return Param::Result::Timeout;
        case MavlinkParameterClient::Result::ConnectionError:
            return Param::Result::ConnectionError;
        case MavlinkParameterClient::Result::WrongType:
            return Param::Result::WrongType;
        case MavlinkParameterClient::Result::ParamNameTooLong:
            return Param::Result::ParamNameTooLong;
        case MavlinkParameterClient::Result::NotFound:
            LogWarn() << "NotFound";
            return Param::Result::Unknown; // TODO fix
        case MavlinkParameterClient::Result::ValueUnsupported:
            LogWarn() << "ValueUnsupported";
            return Param::Result::Unknown; // TODO fix
        case MavlinkParameterClient::Result::Failed:
            LogWarn() << "Failed";
            return Param::Result::Unknown; // TODO fix
        case MavlinkParameterClient::Result::ParamValueTooLong:
            return Param::Result::ParamValueTooLong;
        case MavlinkParameterClient::Result::StringTypeUnsupported:
            LogWarn() << "StringTypeUnsupported";
            return Param::Result::Unknown; // TODO fix
        case MavlinkParameterClient::Result::InconsistentData:
            LogWarn() << "InconsistentData";
            return Param::Result::Unknown; // TODO fix
        case MavlinkParameterClient::Result::UnknownError:
            LogErr() << "Unknown 2 param error";
            return Param::Result::Unknown;
        default:
            LogErr() << "Unknown param error" << (int)result;
            return Param::Result::Unknown;
    }
}

} // namespace mavsdk
