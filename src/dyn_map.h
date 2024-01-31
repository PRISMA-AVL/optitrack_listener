//ROS libraries
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>


// Custom Library
#include "utils.h"

#include <string>
#include "boost/thread.hpp"

using namespace std;

class DYN_MAP {
    public:
        DYN_MAP();
        void odom_cb( const nav_msgs::Odometry );
        void pose_cb( const geometry_msgs::PoseStamped );

        void get_params();
        void create_BB();
        void run();

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _pos_sub;
        ros::Subscriber _odom_sub;
        ros::Publisher  _BB_pub;

        Eigen::Vector3d _pos;
        Eigen::Vector3d _vel;

        //Params
        int _rate;
        bool _get_vel;
        string _pos_topic_name;
        string _odom_topic_name;
        double _x_bound;
        double _y_bound;
        double _z_bound;

};