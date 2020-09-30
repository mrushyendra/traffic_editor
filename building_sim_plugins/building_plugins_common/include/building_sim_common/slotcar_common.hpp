/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef BUILDING_SIM_COMMON__SLOTCAR_COMMON_HPP
#define BUILDING_SIM_COMMON__SLOTCAR_COMMON_HPP

#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <building_map_msgs/msg/building_map.hpp>
#include <charge_msgs/msg/charge_state.hpp>
//#include <rmf_battery/agv/BatterySystem.hpp>
//#include <rmf_traffic/Motion.hpp>
//#include <rmf_traffic/Time.hpp>

namespace building_sim_common {

// TODO migrate ign-math-eigen conversions when upgrading to ign-math5

// Edit reference of parameter for template type deduction
template<typename IgnQuatT>
inline void convert(const Eigen::Quaterniond& _q, IgnQuatT& quat)
{
  quat.W() = _q.w();
  quat.X() = _q.x();
  quat.Y() = _q.y();
  quat.Z() = _q.z();
}

template<typename IgnVec3T>
inline void convert(const Eigen::Vector3d& _v, IgnVec3T& vec)
{
  vec.X() = _v[0];
  vec.Y() = _v[1];
  vec.Z() = _v[2];
}

template<typename IgnVec3T>
inline Eigen::Vector3d convert_vec(const IgnVec3T& _v)
{
  return Eigen::Vector3d(_v[0], _v[1], _v[2]);
}

template<typename IgnQuatT>
inline Eigen::Quaterniond convert_quat(const IgnQuatT& _q)
{
  Eigen::Quaterniond quat;
  quat.w() = _q.W();
  quat.x() = _q.X();
  quat.y() = _q.Y();
  quat.z() = _q.Z();

  return quat;
}

template<typename IgnPoseT>
inline auto convert(const Eigen::Isometry3d& _tf)
{
  IgnPoseT pose;
  convert(Eigen::Vector3d(_tf.translation()), pose.Pos());
  convert(Eigen::Quaterniond(_tf.linear()), pose.Rot());

  return pose;
}

template<typename IgnPoseT>
inline Eigen::Isometry3d convert_pose(const IgnPoseT& _pose)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = convert_vec(_pose.Pos());
  tf.linear() = Eigen::Matrix3d(convert_quat(_pose.Rot()));

  return tf;
}

typedef struct TrajectoryPoint
{
  Eigen::Vector3d pos;
  Eigen::Quaterniond quat;
  TrajectoryPoint(const Eigen::Vector3d& _pos, const Eigen::Quaterniond& _quat)
  : pos(_pos), quat(_quat) {}
} TrajectoryPoint;

class SlotcarCommon
{
public:
  rclcpp::Logger logger() const;

  template<typename SdfPtrT>
  void read_sdf(SdfPtrT& sdf);

  void set_model_name(const std::string& model_name);

  std::string model_name() const;

  void init_ros_node(const rclcpp::Node::SharedPtr node);

  std::pair<double, double> update(const Eigen::Isometry3d& pose,
    const std::vector<Eigen::Vector3d>& obstacle_positions,
    const double time);

  bool emergency_stop(const std::vector<Eigen::Vector3d>& obstacle_positions,
    const Eigen::Vector3d& current_heading);

  std::array<double, 2> calculate_control_signals(const std::array<double,
    2>& w_tire,
    const std::pair<double, double>& velocities,
    const double dt) const;

  void publish_robot_state(const double time);

private:
  struct SlotcarParams
  {
    double nominal_voltage = 12; // V
    double nominal_capacity = 24; // Ah
    double charging_current = 2; // A
    double mass = 20;
    double inertia = 10;
    double friction_coefficient = 0.3;
    double nominal_power = 10;
    double efficiency = 1;
  };

  struct ChargerWaypoint
  {
    double x;
    double y;
    ChargerWaypoint(double x, double y) : x(x), y(y)
    {
    }
  };

  // Constants for update rate of tf2 and robot_state topic
  static constexpr float TF2_RATE = 100.0;
  static constexpr float STATE_TOPIC_RATE = 2.0;

  // Initial distance threshold over which a fleet adapter error is reported
  static constexpr float INITIAL_DISTANCE_THRESHOLD = 1.0;

  rclcpp::Node::SharedPtr _ros_node;

  double _last_update_time = 0.0;
  double last_tf2_pub = 0.0;
  double last_topic_pub = 0.0;

  std::vector<Eigen::Isometry3d> trajectory;
  std::size_t _traj_wp_idx;

  std::vector<rclcpp::Time> _hold_times;

  std::string _model_name;
  bool _emergency_stop = false;
  bool _adapter_error = false;

  bool _initialized_pose = false; // True if at least 1 call to update() has been made
  Eigen::Isometry3d _old_pose; // Pose at previous time step
  Eigen::Vector3d _old_vel;
  Eigen::Isometry3d _pose; // Pose at current time step

  std::unordered_map<std::string, double> _level_to_elevation;
  bool _initialized_levels = false;

  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf2_broadcaster;
  rclcpp::Publisher<rmf_fleet_msgs::msg::RobotState>::SharedPtr _robot_state_pub;

  rclcpp::Subscription<rmf_fleet_msgs::msg::PathRequest>::SharedPtr _traj_sub;
  rclcpp::Subscription<rmf_fleet_msgs::msg::ModeRequest>::SharedPtr _mode_sub;
  rclcpp::Subscription<building_map_msgs::msg::BuildingMap>::SharedPtr
    _building_map_sub;
  rclcpp::Subscription<charge_msgs::msg::ChargeState>::SharedPtr _charge_state_sub;

  rmf_fleet_msgs::msg::RobotMode _current_mode;

  std::string _current_task_id;
  std::vector<rmf_fleet_msgs::msg::Location> _remaining_path;

  // Vehicle dynamic constants
  // TODO(MXG): Consider fetching these values from model data
  // Radius of a tire
  double _tire_radius = 0.1;
  // Distance of a tire from the origin
  double _base_width = 0.52;

  double _nominal_drive_speed = 0.5;         // nominal robot velocity (m/s)
  double _nominal_drive_acceleration = 0.05; // nominal robot forward acceleration (m/s^2)
  double _max_drive_acceleration = 0.1;      // maximum robot forward acceleration (m/s^2)

  double _nominal_turn_speed = M_PI / 8.0;         // nominal robot turning speed (half a rotation per 8 seconds)
  double _nominal_turn_acceleration = M_PI / 10.0; // nominal robot turning acceleration (rad/s^2)

  double _max_turn_acceleration = M_PI; // maximum robot turning acceleration (rad/s^2)

  double _stop_distance = 1.0;
  double _stop_radius = 1.0;

  SlotcarParams _params;
  bool _enable_charge = false;
  bool _enable_instant_charge = false;
  bool _enable_drain = false;
  double _soc = 100.0;
  std::unordered_map<std::string, std::vector<ChargerWaypoint>> charger_waypoints;
  static constexpr double _charger_dist_thres = 1; // Straight line distance to charging waypoint within which charging can occur

  std::string get_level_name(const double z) const;

  double compute_change_in_rotation(Eigen::Vector3d heading_vec,
    const Eigen::Vector3d& dpos,
    double* permissive = nullptr);

  void publish_tf2(const rclcpp::Time& t);

  void publish_state_topic(const rclcpp::Time& t);

  bool path_request_valid(
    const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);

  void path_request_cb(const rmf_fleet_msgs::msg::PathRequest::SharedPtr msg);

  void mode_request_cb(const rmf_fleet_msgs::msg::ModeRequest::SharedPtr msg);

  void map_cb(const building_map_msgs::msg::BuildingMap::SharedPtr msg);

  void charge_state_cb(const charge_msgs::msg::ChargeState::SharedPtr msg);

  bool near_charger(const Eigen::Isometry3d& pose) const;

  double compute_charge(const double run_time) const;

  double compute_discharge(
    const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration,
    const double run_time) const;
};

template<typename SdfPtrT>
void SlotcarCommon::read_sdf(SdfPtrT& sdf)
{
  if (sdf->HasElement("nominal_drive_speed"))
    _nominal_drive_speed = sdf->template Get<double>("nominal_drive_speed");
  RCLCPP_INFO(logger(),
    "Setting nominal drive speed to: " +
    std::to_string(_nominal_drive_speed));

  if (sdf->HasElement("nominal_drive_acceleration"))
    _nominal_drive_acceleration = sdf->template Get<double>(
      "nominal_drive_acceleration");
  RCLCPP_INFO(
    logger(),
    "Setting nominal drive acceleration to: " + std::to_string(
      _nominal_drive_acceleration));

  if (sdf->HasElement("max_drive_acceleration"))
    _max_drive_acceleration =
      sdf->template Get<double>("max_drive_acceleration");
  RCLCPP_INFO(logger(),
    "Setting max drive acceleration to: "
    + std::to_string(_max_drive_acceleration));

  if (sdf->HasElement("nominal_turn_speed"))
    _nominal_turn_speed = sdf->template Get<double>("nominal_turn_speed");
  RCLCPP_INFO(logger(),
    "Setting nominal turn speed to:"
    + std::to_string(_nominal_turn_speed));

  if (sdf->HasElement("nominal_turn_acceleration"))
    _nominal_turn_acceleration = sdf->template Get<double>(
      "nominal_turn_acceleration");
  RCLCPP_INFO(logger(), "Setting nominal turn acceleration to:" + std::to_string(
      _nominal_turn_acceleration));

  if (sdf->HasElement("max_turn_acceleration"))
    _max_turn_acceleration = sdf->template Get<double>("max_turn_acceleration");
  RCLCPP_INFO(logger(),
    "Setting max turn acceleration to:"
    + std::to_string(_max_turn_acceleration));

  if (sdf->HasElement("stop_distance"))
    _stop_distance = sdf->template Get<double>("stop_distance");
  RCLCPP_INFO(logger(),
    "Setting stop distance to:" + std::to_string(_stop_distance));

  if (sdf->HasElement("stop_radius"))
    _stop_radius = sdf->template Get<double>("stop_radius");
  RCLCPP_INFO(logger(),
    "Setting stop radius to:" + std::to_string(_stop_radius));

  if (sdf->HasElement("tire_radius"))
    _tire_radius = sdf->template Get<double>("tire_radius");
  RCLCPP_INFO(logger(),
    "Setting tire radius to:" + std::to_string(_tire_radius));

  if (sdf->HasElement("base_width"))
    _base_width = sdf->template Get<double>("base_width");
  RCLCPP_INFO(logger(), "Setting base width to:" + std::to_string(_base_width));

  if (sdf->GetParent() && sdf->GetParent()->GetParent())
  {
    auto parent = sdf->GetParent()->GetParent(); // Brittle, perhaps change
    //std::cout << "parent is ....................... " << parent->GetName() << std::endl;
    if(parent->HasElement("rmf:charger_waypoints"))
    {
      auto waypoints = parent->GetElement("rmf:charger_waypoints");
      if(waypoints->HasElement("rmf:vertex"))
      {
        auto waypoint = waypoints->GetElement("rmf:vertex");
        while(waypoint)
        {
          //std::cout << "waypoint: " << waypoint->GetName() << std::endl;
          if(waypoint->HasAttribute("x") && waypoint->HasAttribute("y") && waypoint->HasAttribute("level"))
          {
            std::string x_val, y_val, lvl_name;
            waypoint->GetAttribute("x")->Get(x_val);
            waypoint->GetAttribute("y")->Get(y_val);
            waypoint->GetAttribute("level")->Get(lvl_name);

            double x,y;
            std::stringstream ss;
            ss << x_val << y_val;
            ss >> x >> y; // to set precision
            charger_waypoints[lvl_name].push_back(ChargerWaypoint(x,y));
            //std::cout << "x value: " << x_val << "y: " << y_val << " level: " << lvl_name << std::endl;
          }
          waypoint = waypoint->GetNextElement("rmf:vertex");
        }
      }
    }
  }

  RCLCPP_INFO(logger(), "Setting name to: " + _model_name);
}
} // namespace building_sim_common

#endif // BUILDING_SIM_COMMON__SLOTCAR_COMMON_HPP