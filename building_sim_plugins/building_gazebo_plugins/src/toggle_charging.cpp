#include <functional>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <charge_msgs/msg/charge_state.hpp>

using ChargeState = charge_msgs::msg::ChargeState;

class ToggleCharging : public gazebo::GUIPlugin
{
  Q_OBJECT
  gazebo_ros::Node::SharedPtr _ros_node;

  bool _enable_charge = true;
  bool _enable_instant_charge = false;
  bool _enable_drain = true;
  rclcpp::Publisher<ChargeState>::SharedPtr _charge_state_pub;

public:
  virtual ~ToggleCharging()
  {
  }

  void Load(sdf::ElementPtr sdf)
  {
    printf("ToggleCharging::Load()\n");
    _ros_node = gazebo_ros::Node::Get(sdf);

    _charge_state_pub = _ros_node->create_publisher<ChargeState>(
      "/charge_state", rclcpp::SystemDefaultsQoS());

    QVBoxLayout* vbox = new QVBoxLayout;

    QPushButton* charge_button = create_button(
      "Allow Charging", _enable_charge);
    connect(
      charge_button,
      &QAbstractButton::clicked,
      [this]()
      {
        this->_enable_charge = !this->_enable_charge;
        this->button_clicked();
      });
    vbox->addWidget(charge_button);

    QPushButton* instant_charge_button = create_button(
      "Instant Charging", _enable_instant_charge);
    connect(
      instant_charge_button,
      &QAbstractButton::clicked,
      [this]()
      {
        this->_enable_instant_charge = !this->_enable_instant_charge;
        this->button_clicked();
      });
    vbox->addWidget(instant_charge_button);

    QPushButton* drain_button = create_button(
      "Allow Battery Drain", _enable_drain);
    connect(
      drain_button,
      &QAbstractButton::clicked,
      [this]()
      {
        this->_enable_drain = !this->_enable_drain;
        this->button_clicked();
      });
    vbox->addWidget(drain_button);

    setLayout(vbox);
    this->move(0, 50);
  }

  void button_clicked()
  {
    ChargeState _state;
    _state.enable_charge = _enable_charge;
    _state.enable_drain = _enable_drain;
    _state.enable_instant_charge = _enable_instant_charge;
    _charge_state_pub->publish(_state);
  }

private:
  QPushButton* create_button(const std::string& button_nm, bool init_val)
  {
    auto new_btn = new QPushButton(QString::fromStdString(button_nm));
    new_btn->setCheckable(true);
    new_btn->setChecked(init_val);
    return new_btn;
  }
};

#include "toggle_charging.moc"

GZ_REGISTER_GUI_PLUGIN(ToggleCharging)
