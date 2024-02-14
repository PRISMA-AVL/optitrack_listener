#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/timesync.hpp>
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
	
private:

	void Publisher();

	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr _odom_publisher;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped >::SharedPtr _optitrack_sub;

	std::atomic<uint64_t> _timesync;   //!< common synced timestamped

	std::atomic<uint64_t> _timestamp_sample;
	
	VehicleVisualOdometry _odometry{};

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
    
    declare_parameter("topic_name",  "/natnet_ros/Robot_1/pose");
    auto input = get_parameter("topic_name").as_string();
    _input_topic = input;

	_timesync_sub = this->create_subscription<px4_msgs::msg::Timesync>("/fmu/timesync/out",10,
		[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
			_timesync.store(msg->timestamp);
		}
	);

    _optitrack_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(_input_topic.c_str(),10,
		[this](const geometry_msgs::msg::PoseStamped::UniquePtr pose_opt) {
			_p_opt << pose_opt->pose.position.x, pose_opt->pose.position.y, pose_opt->pose.position.z;
            _q_opt << pose_opt->pose.orientation.w, pose_opt->pose.orientation.x, pose_opt->pose.orientation.y, pose_opt->pose.orientation.z; 
		}
	);

	// Publishers
	_odom_publisher = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("/fmu/vehicle_visual_odometry/in", 1);
  	
  	_timestamp_sample = 0;

	Publisher();
}

void OptitrackListener::Publisher(){
  	
    auto timer_callback = [this]() -> void {
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

            _odometry.local_frame = 0; //NED
            _odometry.x = p_opt2_ned[0];
            _odometry.y = p_opt2_ned[1];
            _odometry.z = p_opt2_ned[2];

            _odometry.q = {float(q_opt2_ned[0]), float(q_opt2_ned[1]), float(q_opt2_ned[2]), float(q_opt2_ned[3])};
        
            _timestamp_sample++;
            
            _odometry.timestamp = _timesync.load();
            _odometry.timestamp_sample = _timesync.load();
                        
            //std::cout << "P: " << position_.t() << std::endl;
        
            _odom_publisher->publish(_odometry);
        }

    };
    timer_ = this->create_wall_timer(10ms, timer_callback); // 100 Hz
	
}


int main(int argc, char* argv[]) {
	std::cout << "Starting optitrack_listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<OptitrackListener>());

	rclcpp::shutdown();
	return 0;
}





