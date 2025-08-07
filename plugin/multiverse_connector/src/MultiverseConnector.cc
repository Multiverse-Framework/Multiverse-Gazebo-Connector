#include "MultiverseConnector.h"

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
                    if (config.send_objects.find(link_name) == config.send_objects.end())
                    {
                        config.send_objects[link_name] = {};
                    }
                    const std::string &object_name = send_json.isMember(link_name) ? link_name : "body";
                    bool need_pose = false;
                    bool need_velocity = false;
                    for (const Json::Value &attribute_json : send_json[object_name])
                    {
                        const std::string &attribute_name = attribute_json.asString();
                        if (strcmp(attribute_name.c_str(), "position") == 0 ||
                            strcmp(attribute_name.c_str(), "quaternion") == 0)
                        {
                            need_pose = true;
                        }
                        else if (strcmp(attribute_name.c_str(), "linear_velocity") == 0 ||
                                 strcmp(attribute_name.c_str(), "angular_velocity") == 0)
                        {
                            need_velocity = true;
                        }
                    }
                    if (need_pose)
                    {
                        _ecm.CreateComponent(entity, components::WorldPose());
                        _ecm.CreateComponent(entity, components::Pose());
                    }
                    if (need_velocity)
                    {
                        const Link body = Link(entity);
                        body.EnableVelocityChecks(_ecm, true);
                    }
                    for (const Json::Value &attribute_json : send_json[object_name])
                    {
                        const std::string &attribute_name = attribute_json.asString();
                        if (link_entities.find(link_name) == link_entities.end())
                        {
                            link_entities[link_name] = entity;
                        }
                        config.send_objects[link_name].insert(attribute_name);
                    }
                }
                return true;
            });
        _ecm.Each<components::Joint, components::Name>(
            [&](const Entity &entity,
                const components::Joint *,
                const components::Name *name) -> bool
            {
                const std::string &joint_name = name->Data();
                if (send_json.isMember(joint_name) || send_json.isMember("joint"))
                {
                    if (config.send_objects.find(joint_name) == config.send_objects.end())
                    {
                        config.send_objects[joint_name] = {};
                    }
                    const std::string &object_name = send_json.isMember(joint_name) ? joint_name : "joint";
                    bool need_position = false;
                    bool need_velocity = false;
                    for (const Json::Value &attribute_json : send_json[object_name])
                    {
                        const std::string &attribute_name = attribute_json.asString();
                        if (strcmp(attribute_name.c_str(), "joint_angular_position") == 0 ||
                            strcmp(attribute_name.c_str(), "joint_linear_position") == 0)
                        {
                            need_position = true;
                        }
                        else if (strcmp(attribute_name.c_str(), "joint_linear_velocity") == 0 ||
                                 strcmp(attribute_name.c_str(), "joint_angular_velocity") == 0)
                        {
                            need_velocity = true;
                        }
                    }
                    if (need_position)
                    {
                        const Joint joint = Joint(entity);
                        joint.EnablePositionCheck(_ecm, true);
                    }
                    if (need_velocity)
                    {
                        const Joint joint = Joint(entity);
                        joint.EnableVelocityCheck(_ecm, true);
                    }
                    for (const Json::Value &attribute_json : send_json[object_name])
                    {
                        const std::string &attribute_name = attribute_json.asString();
                        if (joint_entities.find(joint_name) == joint_entities.end())
                        {
                            joint_entities[joint_name] = entity;
                        }
                        config.send_objects[joint_name].insert(attribute_name);
                    }
                }
                return true;
            });
    }
    if (_sdf && _sdf->HasElement(receive_str))
    {
        std::string receive_json_str = _sdf->Get<std::string>(receive_str);
        replace_all(receive_json_str, "'", "\"");
        Json::Value receive_json = string_to_json(receive_json_str);
        _ecm.Each<components::Model, components::Name>(
            [&](const Entity &entity,
                const components::Model *,
                const components::Name *name) -> bool
            {
                const std::string &model_name = name->Data();
                if (receive_json.isMember(model_name) || receive_json.isMember("body"))
                {
                    if (config.receive_objects.find(model_name) == config.receive_objects.end())
                    {
                        config.receive_objects[model_name] = {};
                    }
                    for (const Json::Value &attribute_json : receive_json[receive_json.isMember(model_name) ? model_name : "body"])
                    {
                        const std::string &attribute_name = attribute_json.asString();
                        if (strcmp(attribute_name.c_str(), "position") == 0 ||
                            strcmp(attribute_name.c_str(), "quaternion") == 0)
                        {
                            if (model_entities.find(model_name) == model_entities.end())
                            {
                                model_entities[model_name] = entity;
                            }
                            config.receive_objects[model_name].insert(attribute_name);
                        }
                    }
                }
                return true;
            });
        _ecm.Each<components::Link, components::Name>(
            [&](const Entity &entity,
                const components::Link *,
                const components::Name *name) -> bool
            {
                const std::string &link_name = name->Data();
                if (receive_json.isMember(link_name) || receive_json.isMember("body"))
                {
                    if (config.receive_objects.find(link_name) == config.receive_objects.end())
                    {
                        config.receive_objects[link_name] = {};
                    }
                    for (const Json::Value &attribute_json : receive_json[receive_json.isMember(link_name) ? link_name : "body"])
                    {
                        const std::string attribute_name = attribute_json.asString();
                        if (strcmp(attribute_name.c_str(), "force") == 0 ||
                            strcmp(attribute_name.c_str(), "torque") == 0)
                        {
                            if (link_entities.find(link_name) == link_entities.end())
                            {
                                link_entities[link_name] = entity;
                            }
                            config.receive_objects[link_name].insert(attribute_name);
                        }
                    }
                }
                return true;
            });
        _ecm.Each<components::Joint, components::Name>(
            [&](const Entity &entity,
                const components::Joint *,
                const components::Name *name) -> bool
            {
                const std::string &joint_name = name->Data();
                if (receive_json.isMember(joint_name) || receive_json.isMember("joint"))
                {
                    if (config.receive_objects.find(joint_name) == config.receive_objects.end())
                    {
                        config.receive_objects[joint_name] = {};
                    }
                    for (const Json::Value &attribute_json : receive_json[receive_json.isMember(joint_name) ? joint_name : "joint"])
                    {
                        const std::string attribute_name = attribute_json.asString();
                        if (strcmp(attribute_name.c_str(), "cmd_joint_torque") == 0 ||
                            strcmp(attribute_name.c_str(), "cmd_joint_force") == 0)
                        {
                            if (joint_entities.find(joint_name) == link_entities.end())
                            {
                                joint_entities[joint_name] = entity;
                            }
                            config.receive_objects[joint_name].insert(attribute_name);
                        }
                    }
                }
                return true;
            });
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
    communicate();
}

void MultiverseConnector::Update(
    const sim::UpdateInfo &_info,
    sim::EntityComponentManager &_ecm)
{
    int receive_id = 0;
    gzmsg << "------" << std::endl;
    for (const std::pair<const std::string, std::set<std::string>> &receive_object : config.receive_objects)
    {
        const std::string &object_name = receive_object.first;
        gz::math::Vector3d tmp_body_position;
        gz::math::Quaterniond tmp_body_quaternion;
        gz::math::Vector3d tmp_body_force;
        gz::math::Vector3d tmp_body_torque;
        double tmp_cmd_joint_effort = 0.0;

        for (const std::string &attribute_name : receive_object.second)
        {
            if (strcmp(attribute_name.c_str(), "position") == 0)
            {
                tmp_body_position.X() = receive_buffer.buffer_double.data[receive_id++];
                tmp_body_position.Y() = receive_buffer.buffer_double.data[receive_id++];
                tmp_body_position.Z() = receive_buffer.buffer_double.data[receive_id++];
            }
            else if (strcmp(attribute_name.c_str(), "quaternion") == 0)
            {
                tmp_body_quaternion.SetW(receive_buffer.buffer_double.data[receive_id++]);
                tmp_body_quaternion.SetX(receive_buffer.buffer_double.data[receive_id++]);
                tmp_body_quaternion.SetY(receive_buffer.buffer_double.data[receive_id++]);
                tmp_body_quaternion.SetZ(receive_buffer.buffer_double.data[receive_id++]);
            }
            else if (strcmp(attribute_name.c_str(), "force") == 0)
            {
                tmp_body_force.X() = receive_buffer.buffer_double.data[receive_id++];
                tmp_body_force.Y() = receive_buffer.buffer_double.data[receive_id++];
                tmp_body_force.Z() = receive_buffer.buffer_double.data[receive_id++];
            }
            else if (strcmp(attribute_name.c_str(), "torque") == 0)
            {
                tmp_body_torque.X() = receive_buffer.buffer_double.data[receive_id++];
                tmp_body_torque.Y() = receive_buffer.buffer_double.data[receive_id++];
                tmp_body_torque.Z() = receive_buffer.buffer_double.data[receive_id++];
            }
            else if (strcmp(attribute_name.c_str(), "cmd_joint_torque") == 0 ||
                     strcmp(attribute_name.c_str(), "cmd_joint_force") == 0)
            {
                tmp_cmd_joint_effort = receive_buffer.buffer_double.data[receive_id++];
            }
        }

        if (receive_object.second.find("position") != receive_object.second.end() ||
            receive_object.second.find("quaternion") != receive_object.second.end())
        {
            const Entity model_entity = model_entities[object_name];
            Model model = Model(model_entity);
            model.SetWorldPoseCmd(_ecm, gz::math::Pose3d(tmp_body_position, tmp_body_quaternion));
        }
        if (receive_object.second.find("force") != receive_object.second.end() ||
            receive_object.second.find("torque") != receive_object.second.end())
        {
            const Entity body_entity = link_entities[object_name];
            Link body = Link(body_entity);
            body.AddWorldWrench(_ecm, tmp_body_force, tmp_body_torque);
        }
        if (receive_object.second.find("cmd_joint_torque") != receive_object.second.end() ||
            receive_object.second.find("cmd_joint_force") != receive_object.second.end())
        {
            const Entity joint_entity = joint_entities[object_name];
            Joint joint = Joint(joint_entity);
            gzmsg << "Setting joint [" << object_name << "] - " << joint_entity
                  << " - cmd_joint_effort to " << tmp_cmd_joint_effort << std::endl;
            joint.SetForce(_ecm, {tmp_cmd_joint_effort});
        }
    }

    *world_time = _info.simTime.count() / 1E9;

    int send_id = 0;
    gz::math::Pose3d tmp_body_pose;
    gz::math::Vector3d tmp_body_linear_velocity;
    gz::math::Vector3d tmp_body_angular_velocity;
    gz::math::Vector3d tmp_body_force;
    gz::math::Vector3d tmp_body_torque;
    double tmp_joint_position = 0.0;
    double tmp_joint_velocity = 0.0;
    for (const std::pair<const std::string, std::set<std::string>> &send_object : config.send_objects)
    {
        const std::string &object_name = send_object.first;
        if (send_object.second.find("position") != send_object.second.end() ||
            send_object.second.find("quaternion") != send_object.second.end() ||
            send_object.second.find("linear_velocity") != send_object.second.end() ||
            send_object.second.find("angular_velocity") != send_object.second.end())
        {
            const Entity body_entity = link_entities[object_name];
            const Link body = Link(body_entity);
            if (send_object.second.find("position") != send_object.second.end() ||
                send_object.second.find("quaternion") != send_object.second.end())
            {
                auto pose_ptr = body.WorldPose(_ecm);
                if (!pose_ptr)
                {
                    gzwarn << "Link [" << object_name
                           << "] does not have a WorldPose component" << std::endl;
                    continue;
                }
                tmp_body_pose = *pose_ptr;
            }
            if (send_object.second.find("linear_velocity") != send_object.second.end())
            {
                auto lin_vel_ptr = body.WorldLinearVelocity(_ecm);
                if (!lin_vel_ptr)
                {
                    gzwarn << "Link [" << object_name
                           << "] does not have a WorldLinearVelocity component" << std::endl;
                    continue;
                };
                tmp_body_linear_velocity = *lin_vel_ptr;
            }
            if (send_object.second.find("angular_velocity") != send_object.second.end())
            {
                auto ang_vel_ptr = _ecm.ComponentData<components::AngularVelocity>(body_entity);
                if (!ang_vel_ptr)
                {
                    gzwarn << "Link [" << object_name
                           << "] does not have a AngularVelocity component" << std::endl;
                    continue;
                }
                tmp_body_angular_velocity = *ang_vel_ptr;
            }
            if (send_object.second.find("force") != send_object.second.end() ||
                send_object.second.find("torque") != send_object.second.end())
            {
                gzerr << "Not implemented yet: force and torque" << std::endl;
            }
        }
        if (send_object.second.find("joint_angular_position") != send_object.second.end() || send_object.second.find("joint_linear_position") != send_object.second.end() ||
            send_object.second.find("joint_angular_velocity") != send_object.second.end() || send_object.second.find("joint_linear_velocity") != send_object.second.end())
        {
            const Entity joint_entity = joint_entities[object_name];
            const Joint joint = Joint(joint_entity);
            if (send_object.second.find("joint_angular_position") != send_object.second.end() ||
                send_object.second.find("joint_linear_position") != send_object.second.end())
            {
                auto joint_pos_ptr = joint.Position(_ecm);
                if (!joint_pos_ptr)
                {
                    gzwarn << "Joint [" << object_name << "] - " << joint_entity
                           << " - does not have a JointPosition component" << std::endl;
                    continue;
                }
                if (joint_pos_ptr->size() == 0)
                {
                    gzwarn << "Joint [" << object_name << "] - " << joint_entity
                           << " - does not have a JointPosition component with data" << std::endl;
                    continue;
                }
                if (joint_pos_ptr->size() > 1)
                {
                    gzwarn << "Joint [" << object_name << "] - " << joint_entity
                           << " - has more than one position, only the first one will be sent" << std::endl;
                }
                tmp_joint_position = (*joint_pos_ptr)[0];
            }
            if (send_object.second.find("joint_angular_velocity") != send_object.second.end() ||
                send_object.second.find("joint_linear_velocity") != send_object.second.end())
            {
                auto joint_vel_ptr = joint.Velocity(_ecm);
                if (!joint_vel_ptr)
                {
                    gzwarn << "Joint [" << object_name << "] - " << joint_entity
                           << " - does not have a JointVelocity component" << std::endl;
                    continue;
                }
                if (joint_vel_ptr->size() == 0)
                {
                    gzwarn << "Joint [" << object_name << "] - " << joint_entity
                           << " - does not have a JointVelocity component with data" << std::endl;
                    continue;
                }
                if (joint_vel_ptr->size() > 1)
                {
                    gzwarn << "Joint [" << object_name << "] - " << joint_entity
                           << " - has more than one velocity, only the first one will be sent" << std::endl;
                }
                tmp_joint_velocity = (*joint_vel_ptr)[0];
            }
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
                send_buffer.buffer_double.data[send_id++] = tmp_body_pose.Rot().W();
                send_buffer.buffer_double.data[send_id++] = tmp_body_pose.Rot().X();
                send_buffer.buffer_double.data[send_id++] = tmp_body_pose.Rot().Y();
                send_buffer.buffer_double.data[send_id++] = tmp_body_pose.Rot().Z();
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
            else if (strcmp(attribute_name.c_str(), "joint_angular_position") == 0 ||
                     strcmp(attribute_name.c_str(), "joint_linear_position") == 0)
            {
                send_buffer.buffer_double.data[send_id++] = tmp_joint_position;
            }
            else if (strcmp(attribute_name.c_str(), "joint_angular_velocity") == 0 ||
                     strcmp(attribute_name.c_str(), "joint_linear_velocity") == 0)
            {
                send_buffer.buffer_double.data[send_id++] = tmp_joint_velocity;
            }
            else
            {
                gzerr << "Unknown attribute [" << attribute_name << "] for object ["
                      << object_name << "]" << std::endl;
            }
        }
    }
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