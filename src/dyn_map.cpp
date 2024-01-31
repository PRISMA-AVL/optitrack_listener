#include "dyn_map.h"

void DYN_MAP::get_params() {

    if( !_nh.getParam( "get_vel", _get_vel ) ) {
        _get_vel = false;
    }

    if( !_nh.getParam( "pos_topic_name", _pos_topic_name ) ) {
        _pos_topic_name = "/obstacle_1/pose";
    }

    if( !_nh.getParam( "odom_topic_name", _odom_topic_name ) ) {
        _odom_topic_name = "/obstacle_1/odom";
    }

    if( !_nh.getParam( "x_bound", _x_bound ) ) {
        _x_bound = 0.2;
    }

    if( !_nh.getParam( "y_bound", _y_bound ) ) {
        _y_bound = 0.2;
    }

    if( !_nh.getParam( "z_bound", _z_bound ) ) {
        _z_bound = 0.2;
    }

    if( !_nh.getParam( "ros_rate", _rate ) ) {
        _rate = 50;
    }
}

DYN_MAP::DYN_MAP() {

    get_params();

    //---Input
    if( _get_vel ) 
        _odom_sub = _nh.subscribe( _odom_topic_name.c_str(), 1, &DYN_MAP::odom_cb, this );

    else
        _pos_sub = _nh.subscribe( _pos_topic_name.c_str(), 1, &DYN_MAP::pose_cb, this );

    //---Output
    _BB_pub = _nh.advertise<geometry_msgs::PolygonStamped>( "/obstacle_BB_x", 0 );

}

void DYN_MAP::pose_cb( const geometry_msgs::PoseStamped pose ) {
    _pos << pose.pose.position.x, pose.pose.position.y,pose.pose.position.z; 
}

void DYN_MAP::odom_cb( const nav_msgs::Odometry odom ) {
    _pos << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;

    _vel << odom.twist.twist.linear.x, odom.twist.twist.linear.y,odom.twist.twist.linear.z;
}

void DYN_MAP::create_BB() {
    ros::Rate r(_rate);
    geometry_msgs::PolygonStamped bb;
    geometry_msgs::Point32 corner1, corner2, corner3, corner4;
    bool first = true;
    geometry_msgs::PolygonStamped emptyBoundingBox;

    while( ros::ok() ) {

        corner1.x = -2.5;
        corner1.y = _pos[1] - _y_bound;
        corner1.z = _pos[2];

        corner2.x = 2.5;
        corner2.y = _pos[1] - _y_bound;
        corner2.z = _pos[2];

        corner3.x = 2.5;
        corner3.y = _pos[1] + _y_bound;
        corner3.z = _pos[2];

        corner4.x = -2.5;
        corner4.y = _pos[1] + _y_bound; 
        corner4.z = _pos[2];

        bb.header.frame_id = "map";
        bb.header.stamp = ros::Time::now();

        if ( first ) {
            bb.polygon.points.push_back(corner1);
            bb.polygon.points.push_back(corner2);
            bb.polygon.points.push_back(corner3);
            bb.polygon.points.push_back(corner4);
            _BB_pub.publish(bb);
            first = false;
        }
        else {
            _BB_pub.publish(emptyBoundingBox);
            bb.polygon.points.pop_back();
            bb.polygon.points.pop_back();
            bb.polygon.points.pop_back();
            bb.polygon.points.pop_back();
            bb.polygon.points.push_back(corner1);
            bb.polygon.points.push_back(corner2);
            bb.polygon.points.push_back(corner3);
            bb.polygon.points.push_back(corner4);
            _BB_pub.publish(bb);
        }


        

        r.sleep();

    }
}

void DYN_MAP::run() {
    boost::thread bb_gen_t( &DYN_MAP::create_BB, this );
    ros::spin();
}

int main( int argc, char** argv ) {

    ros::init(argc, argv, "obstacle_bb");

    DYN_MAP dm;
    dm.run();

    return 0;
}