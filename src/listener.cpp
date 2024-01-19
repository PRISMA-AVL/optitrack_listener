// ROS libraries
#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"

// Custom Library
#include "utils.h"

// C++ libraries
#include <Eigen/QR>
#include <eigen3/Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <fstream>
#include <math.h>

//Namespace Definition
using namespace std;
using namespace Eigen;

class LISTENER {
	public:
		LISTENER();
		void loop();

		// Publishers fun
		void repub();

		// Subscribers Callback fun
	    void opt_track_cb( const geometry_msgs::PoseStamped pose_fb_msg );

		// Various fun
		void run();

	private:
		// ROS variables
		ros::NodeHandle _nh;
		ros::Publisher  _vision_pose_repub;
		ros::Subscriber _optitrack_sub;

		geometry_msgs::PoseStamped _pose_fb;
		int _rate;
		// Interaction forces
		Eigen::Matrix4d _T_trans;
        Eigen::Vector3d _pos_enu;
        Eigen::Vector4d _quat_enu;
};

// Class constructor
LISTENER::LISTENER() {

	// ROS publishers
	_vision_pose_repub   = _nh.advertise< geometry_msgs::PoseStamped >("/mavros/vision_pose/pose", 0);
	// ROS subscribers
	_optitrack_sub  = _nh.subscribe("/natnet_ros/Robot_1/pose", 1, &LISTENER::opt_track_cb, this);
    	
    _rate = 120;

    // Transformation matrix between motive frame and ENU mavros frame
    _T_trans.setZero();
    _T_trans << 0, 1, 0, 0,
                0, 0, 1, 0,
                1, 0, 0, 0,
                0, 0, 0, 1;
    _pos_enu.setZero();
    _quat_enu.setZero();
    _quat_enu(0) = 1;
}

// Callback force sensor feedback
void LISTENER::opt_track_cb( const geometry_msgs::PoseStamped pose_fb_msg ){
	_pose_fb = pose_fb_msg;
}

void LISTENER::loop() {

	ros::Rate r(_rate);
	Eigen::Matrix4d T_pose, T_ENU;
    Eigen::Matrix3d R_pose;
	while ( ros::ok() ) {
        R_pose = utilities::QuatToMat(Eigen::Vector4d(_pose_fb.pose.orientation.w,_pose_fb.pose.orientation.x,_pose_fb.pose.orientation.y,_pose_fb.pose.orientation.z));
        T_pose <<   R_pose(0,0),R_pose(0,1),R_pose(0,2),_pose_fb.pose.position.x,
                    R_pose(1,0),R_pose(1,1),R_pose(1,2),_pose_fb.pose.position.y,
                    R_pose(2,0),R_pose(2,1),R_pose(2,2),_pose_fb.pose.position.z,
                    0,0,0,1;
        T_ENU = _T_trans*T_pose;
        _pos_enu << T_ENU(0,3),T_ENU(1,3),T_ENU(2,3);
        _quat_enu = utilities::RpyToQuat(utilities::MatToRpy(T_ENU.block<3,3>(0,0)));
        r.sleep();
	}
}

// Arm torques command
void LISTENER::repub() {
    geometry_msgs::PoseStamped msg;
	ros::Rate r(_rate);
	
    while( ros::ok() ) {
        msg.pose.position.x = _pos_enu(0);
        msg.pose.position.y = _pos_enu(1);
        msg.pose.position.z = _pos_enu(2);
        msg.pose.orientation.w = _quat_enu(0);
        msg.pose.orientation.x = _quat_enu(1);
        msg.pose.orientation.y = _quat_enu(2);
        msg.pose.orientation.z = _quat_enu(3);
        _vision_pose_repub.publish(msg);

    	r.sleep();
    }

}

// Mult-thread
void LISTENER::run() {
	boost::thread loop_t( &LISTENER::loop, this );
	boost::thread republish_t( &LISTENER::repub, this );

	ros::spin();
}

// Main
int main(int argc, char** argv ) {

	ros::init(argc, argv, "listener_optitrack_ctrl");

	LISTENER kc;
	kc.run();

	return 0;
}