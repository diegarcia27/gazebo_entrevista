#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointReached.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Int32.h>


mavros_msgs::State current_state;
mavros_msgs::ExtendedState current_extended_state;
mavros_msgs::HomePosition current_home_position;
mavros_msgs::WaypointList waypoints;
mavros_msgs::WaypointReached waypoint_reached;
sensor_msgs::NavSatFix current_global_position;
std_msgs::Int32 current_control_mode;

mavros_msgs::GlobalPositionTarget position_sp;

// Publishers
ros::Publisher position_sp_pub;
ros::Publisher position_control_pub;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void waypoints_cb(const mavros_msgs::WaypointList::ConstPtr& msg){
    waypoints = *msg;
}

void waypoint_reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg){
    waypoint_reached = *msg;
}

void home_position_cb(const mavros_msgs::HomePosition::ConstPtr& msg){
    current_home_position = *msg;
}

void global_postion_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_global_position = *msg;
    // Sanity checks
    if ((waypoints.waypoints.size() < 5) ||
        //  (current_state.mode != "OFFBOARD") ||
         !current_state.armed ||
         (waypoint_reached.wp_seq > waypoints.waypoints.size() - 2))
        return;
    mavros_msgs::Waypoint last_wp = waypoints.waypoints[waypoint_reached.wp_seq];
    mavros_msgs::Waypoint next_wp = waypoints.waypoints[waypoint_reached.wp_seq + 1];
    
    // Calculate the direction in degrees from north
    double lat_diff = next_wp.x_lat - last_wp.x_lat;
    double lon_diff = next_wp.y_long - last_wp.y_long;
    double direction_rad = atan2(lat_diff, lon_diff);
    position_sp.yaw = direction_rad;

    // Set the position setpoint
    position_sp.latitude = next_wp.x_lat;
    position_sp.longitude = next_wp.y_long;
    position_sp.altitude = next_wp.z_alt;
    position_sp.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    position_sp.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX |
                            mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                            mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
                            mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                            mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
                            mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                            // mavros_msgs::GlobalPositionTarget::FORCE |
                            mavros_msgs::GlobalPositionTarget::IGNORE_YAW |
                            mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
    // if (current_control_mode.data == 1){
    //     position_sp_pub.publish(position_sp);
    // }
    position_sp_pub.publish(position_sp);
    position_control_pub.publish(position_sp);
    
}

void control_mode_cb(const std_msgs::Int32::ConstPtr& msg){
    current_control_mode = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_node");
    ros::NodeHandle nh;
    ros::Rate rate(1); // 1 Hz

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, waypoints_cb);
    ros::Subscriber waypoint_reached_sub = nh.subscribe<mavros_msgs::WaypointReached>
            ("offboard_control/waypoint_reached_combined", 10, waypoint_reached_cb);
    ros::Subscriber global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, global_postion_cb);
    ros::Subscriber home_position_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("mavros/home_position/home", 10, home_position_cb);
    ros::Subscriber control_mode_sub = nh.subscribe<std_msgs::Int32>
            ("offboard_control/control_mode", 10, control_mode_cb);

    // Publisher
    position_sp_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
        ("/mavros/setpoint_raw/global", 10);
    position_control_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
        ("offboard_control/position_sp", 10);

    
    while(ros::ok() ){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}