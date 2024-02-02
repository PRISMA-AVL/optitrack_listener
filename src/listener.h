// ROS libraries
#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"

// C++ libraries
#include <Eigen/QR>
#include <eigen3/Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>

// Custom Library
#include "utils.h"

//Namespace Definition
using namespace std;

class LISTENER {
	public:
		LISTENER();
		void loop();

		// Subscribers Callback fun
	    void opt_track_cb( const geometry_msgs::PoseStamped pose_fb_msg );

		// Various fun
		void run();
		void get_params();

	private:
		// ROS variables
		ros::NodeHandle _nh;
		ros::Publisher  _vision_pose_repub;
		ros::Subscriber _optitrack_sub;

		//New variables
		Eigen::Vector3d _p_opt;
		Eigen::Vector4d _q_opt;

		//Params
		string _input_topic;
		string _output_topic;
		int _rate;
		string _output_ref_frame;
};