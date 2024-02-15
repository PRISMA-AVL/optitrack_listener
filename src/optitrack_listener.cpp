#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include "utils.h"

#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <Eigen/Eigen>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std;

class OptitrackListener : public rclcpp::Node {
public:
	
    OptitrackListener();
    void timerCallback();
    rclcpp::TimerBase::SharedPtr timer_;


	
private:

	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_publisher;
	rclcpp::Subscription<mocap_optitrack_interfaces::msg::RigidBodyArray>::SharedPtr _optitrack_sub;

	std::atomic<uint64_t> _timestamp_sample;
	
	VehicleOdometry _odometry{};

    //New variables
    Eigen::Vector3d _p_opt;
    Eigen::Vector4d _q_opt;

    //Params
    string _input_topic;
    string _output_topic;
    int _rate;
    string _output_ref_frame;

};

OptitrackListener::OptitrackListener(): Node("optitrack_listener"){
    
    // declare_parameter("topic_name",  "/natnet_ros/Robot_1/pose");
    declare_parameter("topic_name",  "/avl_rigid_body");
    auto input = get_parameter("topic_name").as_string();
    _input_topic = input;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(4),
        std::bind(&OptitrackListener::timerCallback, this));

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
	_optitrack_sub = this->create_subscription<mocap_optitrack_interfaces::msg::RigidBodyArray>(_input_topic.c_str(),qos,
		[this](const mocap_optitrack_interfaces::msg::RigidBodyArray::UniquePtr pose_opt) {
			_p_opt << pose_opt->rigid_bodies[0].pose_stamped.pose.position.x, pose_opt->rigid_bodies[0].pose_stamped.pose.position.y, pose_opt->rigid_bodies[0].pose_stamped.pose.position.z;
            _q_opt << pose_opt->rigid_bodies[0].pose_stamped.pose.orientation.w, pose_opt->rigid_bodies[0].pose_stamped.pose.orientation.x, pose_opt->rigid_bodies[0].pose_stamped.pose.orientation.y, pose_opt->rigid_bodies[0].pose_stamped.pose.orientation.z; 
		}
	);

	// Publishers
	_odom_publisher = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/vehicle_odometry/in", 1);
  	
  	_timestamp_sample = 0;

}

void OptitrackListener::timerCallback() {
    
    Eigen::Matrix3d R_o;

    R_o << 0, 0, 1, 
            1, 0, 0,
            0, 1, 0;

    if(rclcpp::ok()){
            
        Matrix3d R_opt = utilities::QuatToMat( _q_opt );

        Eigen::Matrix3d R_o_ned;
        R_o_ned << 0,  0, 1,
                    -1,  0, 0,
                    0, -1, 0;
        //Rotate optitrack data
        Eigen::Vector3d p_opt2_ned = R_o_ned*_p_opt;
        Eigen::Matrix3d R_opt2_ned = R_o_ned*R_opt*R_o.transpose();
        Eigen::Vector4d q_opt2_ned = utilities::rot2quat(R_opt2_ned);

        _odometry.pose_frame = 0; //NED
        _odometry.position[0] = p_opt2_ned[0];
        _odometry.position[1] = p_opt2_ned[1];
        _odometry.position[2] = p_opt2_ned[2];

        _odometry.q = {float(q_opt2_ned[0]), float(q_opt2_ned[1]), float(q_opt2_ned[2]), float(q_opt2_ned[3])};
    
        _timestamp_sample++;
        
        _odometry.timestamp =  std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
        _odometry.timestamp_sample =  std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
                    
        //std::cout << "P: " << position_.t() << std::endl;
    
        _odom_publisher->publish(_odometry);
    }
}

int main(int argc, char **argv) {
    std::cout << "Starting optitrack_listener node..." << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OptitrackListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




