#include "MultiverseConnector.h"

enum class EObjectType : unsigned char
{
    LINK = 0,
    JOINT = 1,
};

bool is_attribute_valid(const std::string &obj_name, const std::string &attr_name, const EObjectType &object_type, int &attr_size)
{
    switch (object_type)
    {
    case EObjectType::LINK:
    {
        if (strcmp(attr_name.c_str(), "position") == 0)
        {
            attr_size = 3;
            return true;
        }
        else if (strcmp(attr_name.c_str(), "quaternion") == 0)
        {
            attr_size = 4;
            return true;
        }
        else if (strcmp(attr_name.c_str(), "relative_velocity") == 0)
        {
            attr_size = 6;
            return true;
        }
        else if (strcmp(attr_name.c_str(), "odometric_velocity") == 0)
        {
            attr_size = 6;
            return true;
        }
        else if (strcmp(attr_name.c_str(), "force") == 0)
        {
            attr_size = 3;
            return true;
        }
        else if (strcmp(attr_name.c_str(), "torque") == 0)
        {
            attr_size = 3;
            return true;
        }
    }

    case EObjectType::JOINT:
    {
        const std::set<const char *> joint_attributes = {"joint_rvalue", "joint_angular_velocity", "joint_angular_acceleration", "joint_torque"};
        if (std::find(joint_attributes.begin(), joint_attributes.end(), attr_name) != joint_attributes.end())
        {
            attr_size = 1;
            return true;
        }
    }

    default:
        gzwarn << "Object type " << static_cast<int>(object_type) << " is not supported" << std::endl;
        return false;
    }
}

constexpr char host_str[] = "host";
constexpr char server_port_str[] = "server_port";
constexpr char client_port_str[] = "client_port";
constexpr char world_name_str[] = "world_name";
constexpr char simulation_name_str[] = "simulation_name";
constexpr char send_str[] = "send";
constexpr char receive_str[] = "receive";

void replace_all(std::string &str, const std::string &from, const std::string &to)
{
    if (from.empty())
    {
        return; // Avoid infinite loop
    }

    size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos)
    {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // Move past the last replaced segment
    }
}

Json::Value string_to_json(std::string &str)
{
    replace_all(str, "'", "\"");
    if (str.empty())
    {
        return Json::Value();
    }
    Json::Value json;
    Json::Reader reader;
    if (reader.parse(str, json) && !str.empty())
    {
        return json;
    }
    else
    {
        gzwarn << "Cannot parse " << str << " into a map" << std::endl;
        return Json::Value();
    }
}

void MultiverseConnector::Configure(const Entity &_entity,
                                    const std::shared_ptr<const sdf::Element> &_sdf,
                                    EntityComponentManager &_ecm,
                                    EventManager & /*_eventMgr*/)
{
    model = gz::sim::Model(_entity);

    if (_sdf && _sdf->HasElement(host_str))
    {
        config.host = _sdf->Get<std::string>(host_str);
    }
    if (_sdf && _sdf->HasElement(server_port_str))
    {
        config.server_port = _sdf->Get<std::string>(server_port_str);
    }
    if (_sdf && _sdf->HasElement(client_port_str))
    {
        config.client_port = _sdf->Get<std::string>(client_port_str);
    }
    if (_sdf && _sdf->HasElement(world_name_str))
    {
        config.world_name = _sdf->Get<std::string>(world_name_str);
    }
    if (_sdf && _sdf->HasElement(simulation_name_str))
    {
        config.simulation_name = _sdf->Get<std::string>(simulation_name_str);
    }
    if (_sdf && _sdf->HasElement(send_str))
    {
        std::string send_json_str = _sdf->Get<std::string>(send_str);
        replace_all(send_json_str, "'", "\"");
        Json::Value send_json = string_to_json(send_json_str);
        _ecm.Each<components::Model, components::Name>(
            [&](const Entity &,
                const components::Model *,
                const components::Name *name) -> bool
            {
                const std::string &model_name = name->Data();
                if (send_json.isMember(model_name) || send_json.isMember("body"))
                {
                    config.send_objects[model_name] = {};
                    for (const Json::Value &attribute_json : send_json[send_json.isMember(model_name) ? model_name : "body"])
                    {
                        const std::string attribute_name = attribute_json.asString();
                        int attr_size = 0;
                        if (is_attribute_valid(model_name, attribute_name, EObjectType::LINK, attr_size))
                        {
                            config.send_objects[model_name].insert(attribute_name);
                        }
                    }
                }
                return true;
            });
        gzmsg << "Send JSON: " << send_json_str << std::endl;
    }
    if (_sdf && _sdf->HasElement(receive_str))
    {
        std::string send_json_str = _sdf->Get<std::string>(receive_str);
        replace_all(send_json_str, "'", "\"");
        gzmsg << "Receive JSON: " << send_json_str << std::endl;
    }

    host = config.host;
    server_port = config.server_port;
    client_port = config.client_port;

    *world_time = 0.0;

    printf("Multiverse Server: %s:%s - Multiverse Client: %s:%s\n", host.c_str(), server_port.c_str(), host.c_str(), client_port.c_str());

    connect();

    communicate(true);
}

void MultiverseConnector::PreUpdate(
    const sim::UpdateInfo &_info,
    sim::EntityComponentManager &_ecm)
{
    *world_time = _info.simTime.count() / 1E9;

    int i = 0;
    for (const std::pair<const std::string, std::set<std::string>> &send_object : config.send_objects)
    {
        const std::string &object_name = send_object.first;
        for (const std::string &attribute_name : send_object.second)
        {
            if (strcmp(attribute_name.c_str(), "position") == 0 || strcmp(attribute_name.c_str(), "quaternion") == 0)
            {
                Entity entity = model.ModelByName(_ecm, object_name);
                auto world_pose = worldPose(entity, _ecm);
                if (strcmp(attribute_name.c_str(), "position") == 0)
                {
                    send_buffer.buffer_double.data[i++] = world_pose.Pos()[0];
                    send_buffer.buffer_double.data[i++] = world_pose.Pos()[1];
                    send_buffer.buffer_double.data[i++] = world_pose.Pos()[2];
                }
                else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
                {
                    send_buffer.buffer_double.data[i++] = world_pose.Rot().X();
                    send_buffer.buffer_double.data[i++] = world_pose.Rot().Y();
                    send_buffer.buffer_double.data[i++] = world_pose.Rot().Z();
                    send_buffer.buffer_double.data[i++] = world_pose.Rot().W();
                }
            }
        }
    }
}

void MultiverseConnector::Update(
    const sim::UpdateInfo &,
    sim::EntityComponentManager &_ecm)
{
    communicate();
}

void MultiverseConnector::PostUpdate(const UpdateInfo &_info,
                                     const EntityComponentManager &_ecm)
{
    
}

void MultiverseConnector::start_connect_to_server_thread()
{
    connect_to_server();
}

void MultiverseConnector::wait_for_connect_to_server_thread_finish()
{
}

void MultiverseConnector::start_meta_data_thread()
{
    send_and_receive_meta_data();
}

void MultiverseConnector::wait_for_meta_data_thread_finish()
{
}

bool MultiverseConnector::init_objects(bool from_request_meta_data)
{
    if (from_request_meta_data)
    {
        if (request_meta_data_json["receive"].empty())
        {
            config.receive_objects.clear();
        }
        if (request_meta_data_json["send"].empty())
        {
            config.send_objects.clear();
        }
        for (const std::string &object_name : request_meta_data_json["receive"].getMemberNames())
        {
            for (const Json::Value &attribute_json : request_meta_data_json["receive"][object_name])
            {
                const std::string attribute_name = attribute_json.asString();
                config.receive_objects[object_name].insert(attribute_name);
            }
        }
        for (const std::string &object_name : request_meta_data_json["send"].getMemberNames())
        {
            for (const Json::Value &attribute_json : request_meta_data_json["send"][object_name])
            {
                const std::string attribute_name = attribute_json.asString();
                config.send_objects[object_name].insert(attribute_name);
            }
        }
    }
    return true;
}

void MultiverseConnector::bind_request_meta_data()
{
    const Json::Value api_callbacks = request_meta_data_json["api_callbacks"];
    const Json::Value api_callbacks_response = request_meta_data_json["api_callbacks_response"];
    request_meta_data_json.clear();

    if (!api_callbacks.isNull())
    {
        request_meta_data_json["api_callbacks"] = api_callbacks;
    }
    if (!api_callbacks_response.isNull())
    {
        request_meta_data_json["api_callbacks_response"] = api_callbacks_response;
    }

    request_meta_data_json["meta_data"]["world_name"] = config.world_name;
    request_meta_data_json["meta_data"]["simulation_name"] = config.simulation_name;
    request_meta_data_json["meta_data"]["length_unit"] = "m";
    request_meta_data_json["meta_data"]["angle_unit"] = "rad";
    request_meta_data_json["meta_data"]["mass_unit"] = "kg";
    request_meta_data_json["meta_data"]["time_unit"] = "s";
    request_meta_data_json["meta_data"]["handedness"] = "rhs";

    for (const std::pair<const std::string, std::set<std::string>> &send_object : config.send_objects)
    {
        const std::string object_name = send_object.first;
        for (const std::string &attribute_name : send_object.second)
        {
            request_meta_data_json["send"][object_name].append(attribute_name);
        }
    }

    for (const std::pair<const std::string, std::set<std::string>> &receive_object : config.receive_objects)
    {
        const std::string object_name = receive_object.first;
        for (const std::string &attribute_name : receive_object.second)
        {
            request_meta_data_json["receive"][object_name].append(attribute_name);
        }
    }

    request_meta_data_str = request_meta_data_json.toStyledString();

    gzmsg << "Request JSON: " << request_meta_data_str << std::endl;
}

void MultiverseConnector::bind_api_callbacks()
{
}

void MultiverseConnector::bind_api_callbacks_response()
{
}

void MultiverseConnector::bind_response_meta_data()
{
}

void MultiverseConnector::init_send_and_receive_data()
{
}

void MultiverseConnector::bind_send_data()
{
}

void MultiverseConnector::bind_receive_data()
{
}

void MultiverseConnector::clean_up()
{
}

void MultiverseConnector::reset()
{
}