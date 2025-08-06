#pragma once

#include "multiverse_client_json.h"

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/plugin/Register.hh>

using namespace gz;
using namespace sim;
using namespace systems;

struct MultiverseConfig
{
    std::string host = "tcp://127.0.0.1";
    std::string server_port = "7000";
    std::string client_port = "6500";
    std::string world_name = "world";
    std::string simulation_name = "gazebo_simulation";
    std::map<std::string, std::set<std::string>> send_objects = {};
    std::map<std::string, std::set<std::string>> receive_objects = {};
};

class MultiverseConnector
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemUpdate,
      public ISystemPostUpdate,
      public MultiverseClientJson
{
    virtual void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager & /*_eventMgr*/) override;

    virtual void PreUpdate(const UpdateInfo &,
                           EntityComponentManager &_ecm) override;

    virtual void Update(const UpdateInfo &,
                        EntityComponentManager &_ecm) override;

    virtual void PostUpdate(const UpdateInfo & /*_info*/,
                            const EntityComponentManager &_ecm) override;

private:
    MultiverseConfig config;

    std::map<std::string, Entity> model_entities;

    std::map<std::string, Entity> link_entities;

    std::map<std::string, Entity> joint_entities;

    std::vector<double *> send_data_vec;

    std::vector<double *> receive_data_vec;

private:
    void start_connect_to_server_thread() override;

    void wait_for_connect_to_server_thread_finish() override;

    void start_meta_data_thread() override;

    void wait_for_meta_data_thread_finish() override;

    bool init_objects(bool from_request_meta_data = false) override;

    void bind_request_meta_data() override;

    void bind_api_callbacks() override;

    void bind_api_callbacks_response() override;

    void bind_response_meta_data() override;

    void init_send_and_receive_data() override;

    void bind_send_data() override;

    void bind_receive_data() override;

    void clean_up() override;

    void reset() override;
};

GZ_ADD_PLUGIN(MultiverseConnector,
              gz::sim::System,
              MultiverseConnector::ISystemConfigure,
              MultiverseConnector::ISystemPreUpdate,
              MultiverseConnector::ISystemUpdate,
              MultiverseConnector::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(MultiverseConnector, "gz::sim::systems::MultiverseConnector")
