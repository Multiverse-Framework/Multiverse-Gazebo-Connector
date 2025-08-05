#include "MultiverseConnector.h"

bool is_attribute_valid(EntityComponentManager &ecm, const Entity &entity, const std::string &attr_name, int &attr_size)
{
    if (strcmp(attr_name.c_str(), "position") == 0)
    {
        if (!ecm.Component<components::Pose>(entity))
        {
            return false;
        }
        gzmsg << "Attribute " << attr_name << " of " << entity << " is valid" << std::endl;
        attr_size = 3;
        return true;
    }
    else if (strcmp(attr_name.c_str(), "quaternion") == 0)
    {
        if (!ecm.Component<components::Pose>(entity))
        {
            return false;
        }
        attr_size = 4;
        return true;
    }
    else if (strcmp(attr_name.c_str(), "linear_velocity") == 0)
    {
        if (!ecm.Component<components::LinearVelocity>(entity))
        {
            return false;
        }
        attr_size = 3;
        return true;
    }
    else if (strcmp(attr_name.c_str(), "angular_velocity") == 0)
    {
        if (!ecm.Component<components::AngularVelocity>(entity))
        {
            return false;
        }
        attr_size = 3;
        return true;
    }
    else if (strcmp(attr_name.c_str(), "linear_acceleration") == 0)
    {
        if (!ecm.Component<components::WorldLinearAcceleration>(entity))
        {
            return false;
        }
        attr_size = 3;
        return true;
    }
    else if (strcmp(attr_name.c_str(), "angular_acceleration") == 0)
    {
        if (!ecm.Component<components::WorldAngularAcceleration>(entity))
        {
            return false;
        }
        attr_size = 3;
        return true;
    }
    else if (strcmp(attr_name.c_str(), "odometric_velocity") == 0)
    {
        if (!ecm.Component<components::LinearVelocity>(entity) ||
            !ecm.Component<components::AngularVelocity>(entity))
        {
            return false;
        }
        attr_size = 6;
        return true;
    }
    else if (strcmp(attr_name.c_str(), "force") == 0)
    {
        if (!ecm.Component<components::ForceTorque>(entity))
        {
            gzwarn << "Entity [" << entity << "] does not have a ForceTorque component" << std::endl;
            return false;
        }
        attr_size = 3;
        return true;
    }
    else if (strcmp(attr_name.c_str(), "torque") == 0)
    {
        attr_size = 3;
        return true;
    }
    else
    {
        gzwarn << "Attribute " << attr_name << " is not valid" << std::endl;
        attr_size = 0;
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
        _ecm.Each<components::Link, components::Name>(
            [&](const Entity &entity,
                const components::Link *,
                const components::Name *name) -> bool
            {
                const std::string &link_name = name->Data();
                if (send_json.isMember(link_name) || send_json.isMember("body"))
                {
                    config.send_objects[link_name] = {};
                    for (const Json::Value &attribute_json : send_json[send_json.isMember(link_name) ? link_name : "body"])
                    {
                        const std::string attribute_name = attribute_json.asString();
                        if (strcmp(attribute_name.c_str(), "linear_velocity") == 0)
                        {
                            _ecm.CreateComponent(entity, components::LinearVelocity());
                        }
                        else if (strcmp(attribute_name.c_str(), "angular_velocity") == 0)
                        {
                            _ecm.CreateComponent(entity, components::AngularVelocity());
                        }
                        int attr_size = 0;
                        if (is_attribute_valid(_ecm, entity, attribute_name, attr_size))
                        {
                            if (object_entities.find(link_name) == object_entities.end())
                            {
                                object_entities[link_name] = entity;
                            }
                            config.send_objects[link_name].insert(attribute_name);
                        }
                    }
                }
                return true;
            });
        gzmsg << "Send JSON: " << send_json_str << std::endl;
    }
    if (_sdf && _sdf->HasElement(receive_str))
    {
        std::string receive_json_str = _sdf->Get<std::string>(receive_str);
        replace_all(receive_json_str, "'", "\"");
        Json::Value receive_json = string_to_json(receive_json_str);
        _ecm.Each<components::Link, components::Name>(
            [&](const Entity &entity,
                const components::Link *,
                const components::Name *name) -> bool
            {
                const std::string &link_name = name->Data();
                if (receive_json.isMember(link_name) || receive_json.isMember("body"))
                {
                    config.receive_objects[link_name] = {};
                    for (const Json::Value &attribute_json : receive_json[receive_json.isMember(link_name) ? link_name : "body"])
                    {
                        const std::string attribute_name = attribute_json.asString();
                        if (strcmp(attribute_name.c_str(), "force") == 0 ||
                            strcmp(attribute_name.c_str(), "torque") == 0)
                        {
                            if (object_entities.find(link_name) == object_entities.end())
                            {
                                object_entities[link_name] = entity;
                            }
                            config.receive_objects[link_name].insert(attribute_name);
                        }
                    }
                }
                return true;
            });
        gzmsg << "Receive JSON: " << receive_json_str << std::endl;
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
    int receive_id = 0;
    for (const std::pair<const std::string, std::set<std::string>> &receive_object : config.receive_objects)
    {
        const std::string &object_name = receive_object.first;
        if (receive_object.second.find("force") != receive_object.second.end() ||
            receive_object.second.find("torque") != receive_object.second.end())
        {
            gz::math::Vector3d tmp_body_force;
            gz::math::Vector3d tmp_body_torque;
            if (receive_object.second.find("force") != receive_object.second.end())
            {
                tmp_body_force[0] = receive_buffer.buffer_double.data[receive_id++];
                tmp_body_force[1] = receive_buffer.buffer_double.data[receive_id++];
                tmp_body_force[2] = receive_buffer.buffer_double.data[receive_id++];
            }
            if (receive_object.second.find("torque") != receive_object.second.end())
            {
                tmp_body_torque[0] = receive_buffer.buffer_double.data[receive_id++];
                tmp_body_torque[1] = receive_buffer.buffer_double.data[receive_id++];
                tmp_body_torque[2] = receive_buffer.buffer_double.data[receive_id++];
            }
            const Entity body_entity = object_entities[object_name];
            gz::sim::v9::Link body = Link(body_entity);
            gzmsg << "Received force: " << tmp_body_force
                 << " and torque: " << tmp_body_torque << std::endl;
            body.AddWorldWrench(_ecm, tmp_body_force, tmp_body_torque);
        }
    }

    *world_time = _info.simTime.count() / 1E9;

    int send_id = 0;
    gz::math::Pose3d tmp_body_pose;
    gz::math::Vector3d tmp_body_linear_velocity;
    gz::math::Vector3d tmp_body_angular_velocity;
    gz::math::Vector3d tmp_body_force;
    gz::math::Vector3d tmp_body_torque;
    for (const std::pair<const std::string, std::set<std::string>> &send_object : config.send_objects)
    {
        const std::string &object_name = send_object.first;
        if (send_object.second.find("position") != send_object.second.end() ||
            send_object.second.find("quaternion") != send_object.second.end() ||
            send_object.second.find("linear_velocity") != send_object.second.end() ||
            send_object.second.find("angular_velocity") != send_object.second.end())
        {
            const Entity body_entity = object_entities[object_name];
            tmp_body_pose = worldPose(body_entity, _ecm);
        }
        if (send_object.second.find("linear_velocity") != send_object.second.end() ||
            send_object.second.find("angular_velocity") != send_object.second.end())
        {
            const Entity body_entity = object_entities[object_name];
            if (send_object.second.find("linear_velocity") != send_object.second.end())
            {
                components::LinearVelocity *lin_vel = _ecm.Component<components::LinearVelocity>(body_entity);
                if (!lin_vel)
                {
                    gzwarn << "Link [" << object_name
                           << "] does not have a LinearVelocity component" << std::endl;
                    continue;
                };
                tmp_body_linear_velocity = tmp_body_pose.Rot().RotateVector(lin_vel->Data());
            }
            else if (send_object.second.find("angular_velocity") != send_object.second.end())
            {
                components::AngularVelocity *ang_vel = _ecm.Component<components::AngularVelocity>(body_entity);
                if (!ang_vel)
                {
                    gzwarn << "Link [" << object_name
                           << "] does not have a AngularVelocity component" << std::endl;
                    continue;
                }
                tmp_body_angular_velocity = tmp_body_pose.Rot().RotateVector(ang_vel->Data());
            }
        }
        if (send_object.second.find("force") != send_object.second.end() ||
            send_object.second.find("torque") != send_object.second.end())
        {
            const Entity body_entity = object_entities[object_name];
            components::ForceTorque *force_torque = _ecm.Component<components::ForceTorque>(body_entity);
            if (!force_torque)
            {
                gzwarn << "Link [" << object_name
                       << "] does not have a ForceTorque component" << std::endl;
                continue;
            }
            auto data = force_torque->Data(); // TODO: Get force and torque from ForceTorque component
        }
        for (const std::string &attribute_name : send_object.second)
        {
            if (strcmp(attribute_name.c_str(), "position") == 0)
            {
                send_buffer.buffer_double.data[send_id++] = tmp_body_pose.Pos().X();
                send_buffer.buffer_double.data[send_id++] = tmp_body_pose.Pos().Y();
                send_buffer.buffer_double.data[send_id++] = tmp_body_pose.Pos().Z();
            }
            else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
            {
                send_buffer.buffer_double.data[send_id++] = tmp_body_pose.Rot().X();
                send_buffer.buffer_double.data[send_id++] = tmp_body_pose.Rot().Y();
                send_buffer.buffer_double.data[send_id++] = tmp_body_pose.Rot().Z();
                send_buffer.buffer_double.data[send_id++] = tmp_body_pose.Rot().W();
            }
            else if (strcmp(attribute_name.c_str(), "linear_velocity") == 0)
            {
                send_buffer.buffer_double.data[send_id++] = tmp_body_linear_velocity.X();
                send_buffer.buffer_double.data[send_id++] = tmp_body_linear_velocity.Y();
                send_buffer.buffer_double.data[send_id++] = tmp_body_linear_velocity.Z();
            }
            else if (strcmp(attribute_name.c_str(), "angular_velocity") == 0)
            {
                send_buffer.buffer_double.data[send_id++] = tmp_body_angular_velocity.X();
                send_buffer.buffer_double.data[send_id++] = tmp_body_angular_velocity.Y();
                send_buffer.buffer_double.data[send_id++] = tmp_body_angular_velocity.Z();
            }
            else if (strcmp(attribute_name.c_str(), "force") == 0)
            {
                send_buffer.buffer_double.data[send_id++] = tmp_body_force.X();
                send_buffer.buffer_double.data[send_id++] = tmp_body_force.Y();
                send_buffer.buffer_double.data[send_id++] = tmp_body_force.Z();
            }
            else if (strcmp(attribute_name.c_str(), "torque") == 0)
            {
                send_buffer.buffer_double.data[send_id++] = tmp_body_torque.X();
                send_buffer.buffer_double.data[send_id++] = tmp_body_torque.Y();
                send_buffer.buffer_double.data[send_id++] = tmp_body_torque.Z();
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
    gzmsg << "Response JSON: " << response_meta_data_json.toStyledString() << std::endl;
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