#include "listener.h"

void LISTENER::get_params() {

	if( !_nh.getParam( "optitrack_topic", _input_topic ) ) {
		_input_topic = "/natnet_ros/Robot_1/pose";
	}

	if( !_nh.getParam( "mavros_topic", _output_topic ) ) {
		_output_topic = "/mavros/vision_pose/pose";
	}

	if( !_nh.getParam( "publisher_rate", _rate ) ) {
		_rate = 120;
	}

	if( !_nh.getParam( "output_reference_frame", _output_ref_frame ) ) {
		_output_ref_frame = "enu";
	}
}
// Class constructor
LISTENER::LISTENER() {
	get_params();
	// ROS publishers
	_vision_pose_repub   = _nh.advertise< geometry_msgs::PoseStamped >( _output_topic.c_str(), 0 );
	// ROS subscribers
	_optitrack_sub  = _nh.subscribe( _input_topic.c_str(), 1, &LISTENER::opt_track_cb, this );
    	
    _rate = 120;

	//New variables initialization
	_p_opt << 0.0, 0.0, 0.0;
	_q_opt << 1.0, 0.0, 0.0, 0.0;
}

// Callback force sensor feedback
void LISTENER::opt_track_cb( const geometry_msgs::PoseStamped pose_fb_msg ){

	_p_opt << pose_fb_msg.pose.position.x, pose_fb_msg.pose.position.y, pose_fb_msg.pose.position.z;
	_q_opt << pose_fb_msg.pose.orientation.w, pose_fb_msg.pose.orientation.x, pose_fb_msg.pose.orientation.y, pose_fb_msg.pose.orientation.z; 
}

void LISTENER::loop() {
	
	ros::Rate r(_rate);
	geometry_msgs::PoseStamped pose;
	Eigen::Matrix3d R_o;

	R_o << 0, 0, 1, 
				1, 0, 0,
				0, 1, 0;

	while( ros::ok() ) {

		Matrix3d R_opt = utilities::QuatToMat( _q_opt );

		if( _output_ref_frame == "ned" ) {

			Eigen::Matrix3d R_o_ned;
			R_o_ned << 0,  0, 1,
						-1,  0, 0,
						0, -1, 0;
			//Rotate optitrack data
			Eigen::Vector3d p_opt2_ned = R_o_ned*_p_opt;
			Eigen::Matrix3d R_opt2_ned = R_o_ned*R_opt*R_o.transpose();
			Eigen::Vector4d q_opt2_ned = utilities::rot2quat(R_opt2_ned);

			pose.header.stamp = ros::Time::now();
			pose.pose.position.x = p_opt2_ned[0];
			pose.pose.position.y = p_opt2_ned[1];
			pose.pose.position.z = p_opt2_ned[2];


			pose.pose.orientation.w = q_opt2_ned[0];
			pose.pose.orientation.x = q_opt2_ned[1];
			pose.pose.orientation.y = q_opt2_ned[2];
			pose.pose.orientation.z = q_opt2_ned[3];
		}
		else if( _output_ref_frame == "enu" ) {

			Matrix3d R_o_enu;
			R_o_enu << -1,  0, 0,
									0,  0, 1,
									0,  1, 0;
			Vector3d p_opt2_enu = R_o_enu*_p_opt;
			Matrix3d R_opt2_enu = R_o_enu*R_opt*R_o.transpose();
			Vector4d q_opt2_enu = utilities::rot2quat(R_opt2_enu);

			pose.header.stamp = ros::Time::now();
			pose.pose.position.x = p_opt2_enu[0];
			pose.pose.position.y = p_opt2_enu[1];
			pose.pose.position.z = p_opt2_enu[2];


			pose.pose.orientation.w = q_opt2_enu[0];
			pose.pose.orientation.x = q_opt2_enu[1];
			pose.pose.orientation.y = q_opt2_enu[2];
			pose.pose.orientation.z = q_opt2_enu[3];	

		}
		_vision_pose_repub.publish(pose);
		r.sleep();
	}

}

// Mult-thread
void LISTENER::run() {
	boost::thread loop_t( &LISTENER::loop, this );

	ros::spin();
}

// Main
int main(int argc, char** argv ) {

	ros::init(argc, argv, "listener_optitrack_ctrl");

	LISTENER kc;
	kc.run();

	return 0;
}
