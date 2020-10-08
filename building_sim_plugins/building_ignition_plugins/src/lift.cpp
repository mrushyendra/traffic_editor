#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <rclcpp/rclcpp.hpp>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/lift_common.hpp>

// TODO remove this
using namespace ignition;
using namespace gazebo;
using namespace systems;

using namespace building_sim_common;

namespace building_sim_ign {

//==============================================================================

class IGNITION_GAZEBO_VISIBLE LiftPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
private:
  rclcpp::Node::SharedPtr _ros_node;
  Entity _cabin_joint;
  Entity _lift_en;
  std::unordered_set<Entity> _payloads;
  bool moving = false;

  ignition::transport::Node _ign_node;
  ignition::transport::Node::Publisher _entered_lift_pub;
  ignition::transport::Node::Publisher _exited_lift_pub;

  std::unique_ptr<LiftCommon> _lift_common = nullptr;

  bool _initialized = false;

  void create_entity_components(Entity entity, EntityComponentManager& ecm)
  {
    if (!ecm.EntityHasComponentType(entity,
      components::JointPosition().TypeId()))
      ecm.CreateComponent(entity, components::JointPosition({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointPositionReset().TypeId()))
      ecm.CreateComponent(entity, components::JointPositionReset({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointVelocity().TypeId()))
      ecm.CreateComponent(entity, components::JointVelocity({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointVelocityCmd().TypeId()))
      ecm.CreateComponent(entity, components::JointVelocityCmd({0}));
  }

  std::unordered_set<Entity> get_payloads(
    Entity& lift,
    EntityComponentManager& ecm)
  {
    const auto lift_aabb = ecm.Component<components::AxisAlignedBox>(_lift_en);
    std::unordered_set<Entity> payloads;
    ecm.Each<components::Model, components::Name, components::Pose,
      components::Static>(
      [&](const Entity& entity,
      const components::Model*,
      const components::Name* nm,
      const components::Pose* pose,
      const components::Static* is_static
      ) -> bool
      {
        // Object should not be static
        // It should not be part of infrastructure (doors / lifts)
        const auto payload_position = pose->Data().Pos();
        if (is_static->Data() == false && entity != lift)
        {
          if(lift_aabb->Data().Contains(payload_position))
          {
            payloads.insert(entity);
            //std::cout << "Payload: " << nm->Data() << std::endl;
          }
        }
        return true;
      });
    return payloads;
  }

public:
  LiftPlugin()
  {
    // TODO init ros node
    // Do nothing
  }

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& /*_eventMgr*/) override
  {
    _lift_en = entity;
    //_ros_node = gazebo_ros::Node::Get(sdf);
    // TODO get properties from sdf instead of hardcoded (will fail for multiple instantiations)
    // TODO proper rclcpp init (only once and pass args)
    auto model = Model(entity);
    char const** argv = NULL;
    if (!rclcpp::is_initialized())
      rclcpp::init(0, argv);
    std::string plugin_name("plugin_" + model.Name(ecm));
    ignwarn << "Initializing plugin with name " << plugin_name << std::endl;
    _ros_node = std::make_shared<rclcpp::Node>(plugin_name);

    RCLCPP_INFO(_ros_node->get_logger(),
      "Loading LiftPlugin for [%s]",
      model.Name(ecm).c_str());

    _lift_common = LiftCommon::make(
      model.Name(ecm),
      _ros_node,
      sdf);

    if (!_lift_common)
      return;

    // Keep track of payloads on the lift and issue messages when payload enters/exits
    _entered_lift_pub = _ign_node.Advertise<ignition::msgs::UInt64_V>(
      "/entered_lift");
    if (!_entered_lift_pub)
    {
      std::cerr << "Error advertising topic [/entered_lift]" << std::endl;
    }
    _exited_lift_pub = _ign_node.Advertise<ignition::msgs::UInt64_V>(
      "/exited_lift");
    if (!_exited_lift_pub)
    {
      std::cerr << "Error advertising topic [/exited_lift]" << std::endl;
    }

    const auto joint = model.JointByName(ecm, _lift_common->get_joint_name());
    if (!joint)
    {
      RCLCPP_ERROR(_ros_node->get_logger(),
        " -- Model is missing the joint [%s]",
        _lift_common->get_joint_name().c_str());
      return;
    }

    create_entity_components(joint, ecm);
    _cabin_joint = joint;

    auto position_cmd = ecm.Component<components::JointPositionReset>(
      _cabin_joint);
    position_cmd->Data()[0] = _lift_common->get_elevation();

    // Initialize Bounding Box component
    if (!ecm.EntityHasComponentType(_lift_en,
      components::AxisAlignedBox().TypeId()))
    {
      ecm.CreateComponent(_lift_en, components::AxisAlignedBox());
    }

    _initialized = true;

    RCLCPP_INFO(_ros_node->get_logger(),
      "Finished loading [%s]",
      model.Name(ecm).c_str());
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    // TODO parallel thread executor?
    rclcpp::spin_some(_ros_node);
    if (!_initialized)
      return;

    // Send update request
    const double t =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
      count()) * 1e-9;
    const double position = ecm.Component<components::JointPosition>(
      _cabin_joint)->Data()[0];
    const double velocity = ecm.Component<components::JointVelocity>(
      _cabin_joint)->Data()[0];

    auto result = _lift_common->update(t, position, velocity);

    // Apply motion to the joint
    auto vel_cmd = ecm.Component<components::JointVelocityCmd>(
      _cabin_joint);
    vel_cmd->Data()[0] = result.velocity;

    std::unordered_set<Entity> curr_payloads = get_payloads(_lift_en, ecm);
    for (const Entity& payload : curr_payloads)
    {
      if (ecm.EntityHasComponentType(payload,
        components::LinearVelocityCmd().TypeId()))
      {
        ignition::math::Vector3d old_lin_vel_cmd = ecm.Component<components::LinearVelocityCmd>(payload)->Data();
        ecm.Component<components::LinearVelocityCmd>(payload)->Data() = {old_lin_vel_cmd[0], old_lin_vel_cmd[1], result.velocity};
      }
      //std::cout << "Linear angular velocity cmd: " << ecm.Component<components::LinearVelocityCmd>(payload)->Data() << std::endl;
    }

    //std::cout << "velocity: " << result.velocity << std::endl;
    /*bool state_changed = false;
    if(std::abs(result.velocity) >= 0.1 && !moving)
    {
      moving = true;
      state_changed = true;
    }
    else if(std::abs(result.velocity) < 0.1 && moving)
    {
      moving = false;
      state_changed = true;
    }

    if(!state_changed)
    {
      return;
    }

    std::unordered_set<Entity> curr_payloads = get_payloads(_lift_en, ecm);

    ignition::msgs::UInt64_V exited_lift_msg;
    for(const Entity& payload : _payloads)
    {
      if (!curr_payloads.count(payload))
      {
        std::cout << "Payload exited " << std::endl;
        exited_lift_msg.add_data(google::protobuf::uint64(payload));
      }
    }
    if (exited_lift_msg.data_size())
    {
      _exited_lift_pub.Publish(exited_lift_msg);
    }
    ignition::msgs::UInt64_V entered_lift_msg;
    for(const Entity& payload : curr_payloads)
    {
      if (!_payloads.count(payload))
      {
        std::cout << "Payload entered " << std::endl;
        entered_lift_msg.add_data(google::protobuf::uint64(payload));
      }
    }
    if (entered_lift_msg.data_size())
    {
      _entered_lift_pub.Publish(entered_lift_msg);
    }
    _payloads = std::move(curr_payloads);*/

    //std::cout << "lift aabb: " << lift_aabb->Data().Max().X() << " " << lift_aabb->Data().Max().Y() << " " << lift_aabb->Data().Max().Z() << std::endl;
    //std::cout << "lift aabb size: " << lift_aabb->Data().XLength() << " " << lift_aabb->Data().YLength() << " " << lift_aabb->Data().ZLength() << std::endl;
  }
};


IGNITION_ADD_PLUGIN(
  LiftPlugin,
  System,
  LiftPlugin::ISystemConfigure,
  LiftPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(LiftPlugin, "lift")

} // namespace building_sim_ign
