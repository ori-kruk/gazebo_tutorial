#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/float32.hpp>
#include <gazebo_ros/node.hpp>

namespace gazebo
{
    const std::string POSITION_COMMAND_TOPIC = "/from_ros";
    const std::string VERSION = "0.2";
    
    class RosNodeDemoPlugin : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            gzwarn << "Version:" << VERSION << "\n";
            

            ros_node_ = gazebo_ros::Node::Get(_sdf);
            RCLCPP_INFO(ros_node_->get_logger(), "Loading Gazebo Plugin");

            auto qos = rclcpp::SystemDefaultsQoS();
            this->joint_command_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Float32>(
                POSITION_COMMAND_TOPIC,
                qos,
                std::bind(&RosNodeDemoPlugin::message_handler, this, std::placeholders::_1));
        }

        void message_handler(std::shared_ptr<const std_msgs::msg::Float32> msg)
        {
            gzmsg << "------------------\n";
            gzmsg << "--" << msg->data << " --";
            gzmsg << "++++++-----------\n";
        }


    private:
        physics::ModelPtr model;
        physics::JointPtr joint;
        event::ConnectionPtr updateConnection;

        rclcpp::Node::SharedPtr ros_node_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint_command_sub_;
    };
    GZ_REGISTER_MODEL_PLUGIN(RosNodeDemoPlugin)
}


// apt install ros-humble-rmw-cyclonedds-cpp
// export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
//  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 topic pub  /from_ros std_msgs/msg/Float32 "{data: 1.57}"