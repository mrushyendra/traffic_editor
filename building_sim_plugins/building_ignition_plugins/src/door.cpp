#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/gazebo/components/PhysicsEnginePlugin.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/ChildLinkName.hh>
#include <ignition/gazebo/components/Name.hh>

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
  std::unordered_map<std::string, std::pair<Entity, DoorCommon::DoorElement>> _doors;
  Entity _en;
  Entity _lnk;

  std::string _physics_plugin_name;

  ignition::math::Pose3d orig_pos;
  double orig_rot = 0.0;
  bool pos_set = false;
  int axis = 0;
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

  bool is_tpe_plugin(const std::string& plugin_name)
  {
    static const std::string tpe_plugin = "ignition-physics-tpe-plugin";
    return plugin_name == tpe_plugin;
  }

public:
  DoorPlugin()
  {
  }

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>& sdf,
    EntityComponentManager& ecm, EventManager& /*_eventMgr*/) override
  {
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

    for (const auto& door : _door_common->_doors){
      const std::string& joint_name = door.first;
      const auto joint = model.JointByName(ecm, joint_name);
      if (!joint)
      {
        RCLCPP_ERROR(_ros_node->get_logger(),
          " -- Model is missing the joint [%s]",
          joint_name.c_str());
        return;
      }
      create_entity_components(joint, ecm);

      _doors.insert({joint_name, std::make_pair(joint, door.second)});
    }

    //create components for door, to change to door section
    if (!ecm.EntityHasComponentType(_en,
      components::LinearVelocityCmd().TypeId()))
      ecm.CreateComponent(_en, components::LinearVelocityCmd());
    if (!ecm.EntityHasComponentType(_en,
      components::AngularVelocityCmd().TypeId()))
      ecm.CreateComponent(_en, components::AngularVelocityCmd());
    if (!ecm.EntityHasComponentType(_en,
        components::Pose().TypeId()))
      ecm.CreateComponent(_en, components::Pose());

    /*_lnk = model.LinkByName(ecm, "left");
    if (!ecm.EntityHasComponentType(_lnk,
      components::LinearVelocityCmd().TypeId()))
      ecm.CreateComponent(_lnk, components::LinearVelocityCmd());
    if (!ecm.EntityHasComponentType(_lnk,
      components::AngularVelocityCmd().TypeId()))
      ecm.CreateComponent(_lnk, components::AngularVelocityCmd());
    if (!ecm.EntityHasComponentType(_lnk,
        components::Pose().TypeId()))
      ecm.CreateComponent(_lnk, components::Pose());
    if (!ecm.EntityHasComponentType(_lnk,
        components::WorldPoseCmd().TypeId())){
      ecm.CreateComponent(_lnk, components::WorldPoseCmd());
    }
    Entity left_joint = model.JointByName(ecm, "left_joint");
    ecm.RequestRemoveEntity(left_joint);*/

    _initialized = true;

    RCLCPP_INFO(_ros_node->get_logger(),
      "Finished loading [%s]",
      name.c_str());
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    if(!pos_set){
      orig_pos = ecm.Component<components::Pose>(_en)->Data();
      orig_rot = ecm.Component<components::Pose>(_en)->Data().Yaw();

      Entity parent = _en;
      while(ecm.ParentEntity(parent)){
        parent = ecm.ParentEntity(parent);
      }
      if (ecm.EntityHasComponentType(parent,
          components::PhysicsEnginePlugin().TypeId())){
            _physics_plugin_name = ecm.Component<components::PhysicsEnginePlugin>(parent)->Data();
      }
      pos_set = true;
    }

    //std::cout << "Link pose: " << _lnk.WorldInertialPose(ecm)->Pos() << std::endl;
    //std::cout << "Link pose: " << *(_lnk.WorldLinearVelocity(ecm)) << std::endl;
    //std::cout << ecm.Component<components::WorldPoseCmd>(_lnk)->Data().Pos() << std::endl;
    //ecm.Component<components::AngularVelocityCmd>(_en)->Data() = ignition::math::Vector3d(0,0,0.5);

    // TODO parallel thread executor?
    rclcpp::spin_some(_ros_node);
    if (!_initialized)
      return;

    double t =
      (std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).
      count()) * 1e-9;

    // Create DoorUpdateRequest
    std::vector<DoorCommon::DoorUpdateRequest> requests;
    for (const auto& door : _doors)
    {
      DoorCommon::DoorUpdateRequest request;
      request.joint_name = door.first;

      if(is_tpe_plugin(_physics_plugin_name)) // No joint features support, use velocity commands instead. May need axis too
      {
        const DoorCommon::DoorElement& door_elem = door.second.second;
        constexpr double eps = 0.01;
        if(door_elem.door_type == "SwingDoor")
        {
          // In the event that Yaw angle of the door moves past -Pi rads, it experiences a discontinuous jump from -Pi to Pi (likewise when the Yaw angle moves past Pi)
          // We need to take this jump into account when calculating the relative orientation of the door w.r.t to its original orientation
          if(door_elem.closed_position > door_elem.open_position)
          {
            // Yaw may go past -Pi rads when opening, and experience discontinuous jump to +Pi. For e.g. when original angle is -(7/8)Pi, opening the door by Pi/2 rads would
            // push it past -Pi rads.
            // If closed position (0 rads) is larger than open position, then the relative orientation should theoretically never exceed 0,
            //which is how we identify when it has experienced a discontinuous jump
            request.position = ecm.Component<components::Pose>(_en)->Data().Yaw() - orig_rot > eps ?
              -3.14 - fmod(ecm.Component<components::Pose>(_en)->Data().Yaw() - orig_rot,3.14) : ecm.Component<components::Pose>(_en)->Data().Yaw() - orig_rot;
          }
          else // Yaw may go past +180, and experience discontinuous jump to -180
          {
            request.position = ecm.Component<components::Pose>(_en)->Data().Yaw() - orig_rot < -eps ?
              3.14 + abs(-3.14 - (ecm.Component<components::Pose>(_en)->Data().Yaw() - orig_rot)) : ecm.Component<components::Pose>(_en)->Data().Yaw() - orig_rot;
          }
        }
        else if(door_elem.door_type == "SlidingDoor")
        {
            ignition::math::Pose3d curr_pos = ecm.Component<components::Pose>(_en)->Data();
            ignition::math::Vector3d displacement = curr_pos.CoordPositionSub(orig_pos);
            //std::cout << "diff: " << displacement.Length() << std::endl;
            request.position = displacement.Length();
        }
        else
        {
          continue;
        }
        //std::cout << "pos: " << request.position << std::endl;
        //std::cout << "result vel: " << vel_cmd << std::endl;
        request.velocity = vel_cmd;
      }
      else // Default Physics Engine with Joint Features support
      {
        Entity joint = door.second.first;
        request.position = ecm.Component<components::JointPosition>(
          joint)->Data()[0];
        request.velocity = ecm.Component<components::JointVelocity>(
          joint)->Data()[0];
      }

      requests.push_back(request);
    }

    // Get and apply motions to the joints
    auto results = _door_common->update(t, requests);
    for (const auto& result : results)
    {
      const auto it = _doors.find(result.joint_name);
      assert(it != _doors.end());

      //to get door element section
      if(is_tpe_plugin(_physics_plugin_name))
      {
        vel_cmd = result.velocity;
        const DoorCommon::DoorElement& door_elem = it->second.second;
        if(door_elem.door_type == "SwingDoor")
        {
          ecm.Component<components::LinearVelocityCmd>(_en)->Data() = ignition::math::Vector3d(0,-0.5*vel_cmd,0);
          ecm.Component<components::AngularVelocityCmd>(_en)->Data() = ignition::math::Vector3d(0,0,vel_cmd);
        }
        else if(door_elem.door_type == "SlidingDoor")
        {
          ecm.Component<components::LinearVelocityCmd>(_en)->Data() = ignition::math::Vector3d(vel_cmd,0,0);
        }
        else
        {
          continue;
        }
        //std::cout << "velocity: " << result.velocity << std::endl;
        //std::cout << "Pose: " << ecm.Component<components::Pose>(_en)->Data() << std::endl;
      }
      else // Default Physics Engine with Joint Features support
      {
        auto joint_vel_cmd = ecm.Component<components::JointVelocityCmd>(
          it->second.first);
        joint_vel_cmd->Data()[0] = result.velocity;
      }
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
