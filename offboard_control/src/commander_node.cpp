
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/HomePosition.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/SetMavFrame.h>
#include <std_msgs/Int32.h>
#include <stdlib.h>
#include <resources.hpp>

// Global variables
mavros_msgs::State current_state;
mavros_msgs::ExtendedState current_extended_state;
mavros_msgs::WaypointList waypoints;
mavros_msgs::WaypointReached waypoint_reached;
mavros_msgs::HomePosition current_home_position;
sensor_msgs::NavSatFix current_global_position;
mavros_msgs::WaypointReached waypoint_reached_combined;
geometry_msgs::PoseStamped current_local_position;
std_msgs::Int32 current_control_mode;


// Global services and publishers
ros::ServiceClient waypoint_set_current;
ros::Publisher wp_reached_combined_pub;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg){
    current_extended_state = *msg;
}

void waypoints_cb(const mavros_msgs::WaypointList::ConstPtr& msg){
    waypoints = *msg;
}

void waypoint_reached_cb(const mavros_msgs::WaypointReached::ConstPtr& msg){
    waypoint_reached = *msg;
    waypoint_reached_combined = *msg;
    wp_reached_combined_pub.publish(waypoint_reached_combined);
}

void home_position_cb(const mavros_msgs::HomePosition::ConstPtr& msg){
    current_home_position = *msg;
}

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_position = *msg;
}
// Callback for global position
// Used to check if the drone has reached the waypoints in offboard mode
void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_global_position = *msg;
    // In offboard mode, we have to manually validate the waypoints
    if( current_state.mode == "OFFBOARD") {
        // Convert latitude and longitude to radians
        double lat1 = current_global_position.latitude;
        double lon1 = current_global_position.longitude;
        double lat2 = waypoints.waypoints[waypoint_reached_combined.wp_seq + 1].x_lat;
        double lon2 = waypoints.waypoints[waypoint_reached_combined.wp_seq + 1].y_long;
        float distance = calculate_position_error(lat1, lon1, lat2, lon2);
        float z_distance = waypoints.waypoints[waypoint_reached_combined.wp_seq + 1].z_alt - current_local_position.pose.position.z;
        distance = sqrt(pow(distance, 2) + pow(z_distance, 2));

        if (distance < acceptance_radius)
        {
            mavros_msgs::WaypointSetCurrent wp_current;
            wp_current.request.wp_seq = waypoint_reached_combined.wp_seq + 2;
            if (waypoint_set_current.call(wp_current) &&  wp_current.response.success) {
                waypoint_reached_combined.wp_seq++ ;
                wp_reached_combined_pub.publish(waypoint_reached_combined);
            }    
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "commander_node");
    ros::NodeHandle nh;
    ros::Rate rate(1); // 1 Hz
    ros::Rate fast_rate(50); // 10 Hz
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>
            ("mavros/extended_state", 10, extended_state_cb);
    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, waypoints_cb);
    ros::Subscriber waypoint_reached_sub = nh.subscribe<mavros_msgs::WaypointReached>
            ("mavros/mission/reached", 10, waypoint_reached_cb);
    ros::Subscriber global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, global_position_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber home_position_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("mavros/home_position/home", 1, home_position_cb);

    //Publishers        
    wp_reached_combined_pub = nh.advertise<mavros_msgs::WaypointReached>
            ("offboard_control/waypoint_reached_combined", 10);
    ros::Publisher control_mode_pub = nh.advertise<std_msgs::Int32>
            ("offboard_control/control_mode", 10);
    // Services
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient setFrame = nh.serviceClient<mavros_msgs::SetMavFrame>
            ("mavros/setpoint_position/mav_frame");
    waypoint_set_current = nh.serviceClient<mavros_msgs::WaypointSetCurrent>
            ("mavros/mission/set_current");
    
    // Set the control mode to OFF
    current_control_mode.data = OffboardControlMode::OFF;
    control_mode_pub.publish(current_control_mode);
    ROS_INFO("Waiting for FCU connection...");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");
    ROS_INFO("Waiting for waypoints...");
    while(ros::ok() && waypoints.waypoints.size() < 5){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Waypoints received: %lu", waypoints.waypoints.size());

    ROS_INFO("Waiting for drone landed");
    while(ros::ok() && current_extended_state.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND){
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Commanding mission mode and arming");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.MISSION";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    while(current_state.mode != "AUTO.MISSION" || !current_state.armed){
        if( current_state.mode != "AUTO.MISSION"){
            ROS_INFO("Commanding mission");
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Mission enabled");
            }
        } else {
            if( !current_state.armed){
                ROS_INFO("Arming vehicle");
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    // Start recording the rosbag
    // int i; i=system("rosbag record -o ~/rosbag/ -a __name:=my_bag &");

    ROS_INFO("Waiting for takeoff and  the first waypoint to be reached...");
    while(ros::ok() && 
          waypoint_reached_combined.wp_seq != 0 &&
          current_extended_state.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR){
        ros::spinOnce();
        rate.sleep();
    }
    // Set the control mode
    current_control_mode.data = OffboardControlMode::THRUST;
    control_mode_pub.publish(current_control_mode);
    // Run until drone lands
    while(ros::ok() && current_extended_state.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND){
        if(waypoint_reached_combined.wp_seq < 2) {
            ROS_INFO("Waypoint %d reached out of %lu. Mission mode", waypoint_reached_combined.wp_seq, waypoints.waypoints.size());
        }
        else if(waypoint_reached_combined.wp_seq < waypoints.waypoints.size() - 2) {
            ROS_INFO("Waypoint %d reached out of %lu. Offboard mode", waypoint_reached_combined.wp_seq, waypoints.waypoints.size());
            if( current_state.mode != "OFFBOARD"){
                offb_set_mode.request.custom_mode = "OFFBOARD";
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("OFFBOARD enabled");
                }
            }
        }
        else {
            ROS_INFO("Going back to mission mode and landing");
            if( current_state.mode != "AUTO.MISSION"){
                offb_set_mode.request.custom_mode = "AUTO.MISSION";
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Mission enabled");
                }
            }
        }
        for (int j = 0; j < 70; j++) {
            ros::spinOnce();
            fast_rate.sleep();
        }

    }
    // Set the control mode
    current_control_mode.data = OffboardControlMode::SHUTDOWN;
    control_mode_pub.publish(current_control_mode);
    // i=system("rosnode kill /my_bag");
    ROS_INFO("Drone landed.");
    ROS_INFO("Shutting down the node...");
    ros::shutdown();
    return 0;
}