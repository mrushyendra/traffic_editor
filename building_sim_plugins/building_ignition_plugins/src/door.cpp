#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/PhysicsEnginePlugin.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Geometry>

#include <building_sim_common/utils.hpp>
#include <building_sim_common/door_common.hpp>

// TODO remove this
using namespace ignition;
using namespace gazebo;
using namespace systems;

using namespace building_sim_common;

namespace building_ignition_plugins {

//==============================================================================

class IGNITION_GAZEBO_VISIBLE DoorPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
private:
  rclcpp::Node::SharedPtr _ros_node;
  std::unordered_map<std::string, Entity> _joints;
  Entity _en;

  ignition::math::Pose3d orig_pos;
  bool pos_set = false;
  int axis = 0;
  Eigen::Matrix3d rot_mat;
  double vel_cmd = 0.0;

  std::shared_ptr<DoorCommon> _door_common = nullptr;

  bool _initialized = false;

  Eigen::Matrix3d compute_rot_mat(ignition::math::Pose3d pose){
    double roll = pose.Roll();
    double pitch = pose.Pitch();
    double yaw = pose.Yaw();
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
    return q.matrix();
  }

  void create_entity_components(Entity entity, EntityComponentManager& ecm)
  {
    if (!ecm.EntityHasComponentType(entity,
      components::JointPosition().TypeId()))
      ecm.CreateComponent(entity, components::JointPosition({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointVelocity().TypeId()))
      ecm.CreateComponent(entity, components::JointVelocity({0}));
    if (!ecm.EntityHasComponentType(entity,
      components::JointVelocityCmd().TypeId()))
      ecm.CreateComponent(entity, components::JointVelocityCmd({0}));
  }

public:
  DoorPlugin()
  {
    // TODO init ros node
    // Do nothing
  }

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& /*_eventMgr*/) override
  {
    //_ros_node = gazebo_ros::Node::Get(sdf);
    // TODO get properties from sdf instead of hardcoded (will fail for multiple instantiations)
    // TODO proper rclcpp init (only once and pass args)
    auto model = Model(entity);
    _en = entity;
    char const** argv = NULL;
    std::string name;
    auto door_ele = sdf->GetElementImpl("door");
    get_sdf_attribute_required<std::string>(door_ele, "name", name);
    if (!rclcpp::is_initialized())
      rclcpp::init(0, argv);
    std::string plugin_name("plugin_" + name);
    ignwarn << "Initializing plugin with name " << plugin_name << std::endl;
    _ros_node = std::make_shared<rclcpp::Node>(plugin_name);

    RCLCPP_INFO(_ros_node->get_logger(),
      "Loading DoorPlugin for [%s]",
      name.c_str());

    _door_common = DoorCommon::make(
      name,
      _ros_node,
      sdf);

    if (!_door_common)
      return;

    for (const auto& joint_name : _door_common->joint_names())
    {
      const auto joint = model.JointByName(ecm, joint_name);
      if (!joint)
      {
        RCLCPP_ERROR(_ros_node->get_logger(),
          " -- Model is missing the joint [%s]",
          joint_name.c_str());
        return;
      }
      create_entity_components(joint, ecm);
      _joints.insert({joint_name, joint});
    }

    //create components for door, to change to door section
    if (!ecm.EntityHasComponentType(_en,
      components::LinearVelocityCmd().TypeId()))
      ecm.CreateComponent(_en, components::LinearVelocityCmd());
    if (!ecm.EntityHasComponentType(_en,
        components::Pose().TypeId()))
      ecm.CreateComponent(_en, components::Pose());

    _initialized = true;

    RCLCPP_INFO(_ros_node->get_logger(),
      "Finished loading [%s]",
      name.c_str());
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    if(!pos_set){
      rot_mat = compute_rot_mat(ecm.Component<components::Pose>(_en)->Data());
      orig_pos = ecm.Component<components::Pose>(_en)->Data();
      pos_set = true;
    }

    // TODO parallel thread executor?
    rclcpp::spin_some(_ros_node);
    if (!_initialized)
      return;

    double t =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
      count()) * 1e-9;

    /*if (!ecm.EntityHasComponentType(_en,
        components::PhysicsEnginePlugin().TypeId())){
          ecm.CreateComponent(_en, components::PhysicsEnginePlugin());
        }
    std::string physics_plugin_name = ecm.Component<components::PhysicsEnginePlugin>(_en)->Data();
    std::cout << physics_plugin_name << std::endl;*/

    // Create DoorUpdateRequest
    std::vector<DoorCommon::DoorUpdateRequest> requests;
    for (const auto& joint : _joints)
    {
      DoorCommon::DoorUpdateRequest request;
      request.joint_name = joint.first;
      /*request.position = ecm.Component<components::JointPosition>(
        joint.second)->Data()[0];
      request.velocity = ecm.Component<components::JointVelocity>(
        joint.second)->Data()[0];*/
      ignition::math::Pose3d curr_pos = ecm.Component<components::Pose>(_en)->Data();
      ignition::math::Vector3d displacement = curr_pos.CoordPositionSub(orig_pos);
      std::cout << "diff: " << displacement.Length() << std::endl;
      request.position = displacement.Length();
      request.velocity = vel_cmd;
      requests.push_back(request);
    }

    auto results = _door_common->update(t, requests);

    // Apply motions to the joints
    for (const auto& result : results)
    {
      const auto it = _joints.find(result.joint_name);
      assert(it != _joints.end());

      //get door element section
      //if plugin is tpe:
      // set linear/angular velocity depending on joint type
      vel_cmd = result.velocity;
      Eigen::Vector3d lin_vel_cmd_init(0,vel_cmd,0);
      Eigen::Vector3d lin_vel_cmd = rot_mat * lin_vel_cmd_init;
      ecm.Component<components::LinearVelocityCmd>(_en)->Data() = ignition::math::Vector3d(lin_vel_cmd(0), lin_vel_cmd(1), lin_vel_cmd(2));
      //std::cout << "velocity: " << result.velocity << std::endl;
      //std::cout << "Pose: " << ecm.Component<components::Pose>(_en)->Data() << std::endl;

      auto vel_cmd = ecm.Component<components::JointVelocityCmd>(
        it->second);
      vel_cmd->Data()[0] = result.velocity;
    }
  }

};

IGNITION_ADD_PLUGIN(
  DoorPlugin,
  System,
  DoorPlugin::ISystemConfigure,
  DoorPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(DoorPlugin, "door")

} // namespace building_ignition_plugins
