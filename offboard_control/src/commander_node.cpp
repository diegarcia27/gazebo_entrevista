
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointReached.h>

mavros_msgs::State current_state;
mavros_msgs::ExtendedState current_extended_state;
mavros_msgs::WaypointList waypoints;
mavros_msgs::WaypointReached waypoint_reached;
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
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "commander_node");
    ros::NodeHandle nh;
    ros::Rate rate(0.5); // 1 Hz
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>
            ("mavros/extended_state", 10, extended_state_cb);
    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, waypoints_cb);
    ros::Subscriber waypoint_reached_sub = nh.subscribe<mavros_msgs::WaypointReached>
            ("mavros/mission/reached", 10, waypoint_reached_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

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

    // Mission mode and arm
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
    // Wait until mission starts
    while(ros::ok() && waypoint_reached.wp_seq != 0){
        ros::spinOnce();
        rate.sleep();
    }
    // Run until drone lands
    while(ros::ok() && current_extended_state.landed_state != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND){
        if(waypoint_reached.wp_seq < 2) {
            ROS_INFO("Waypoint %d reached out of %lu. Mission mode", waypoint_reached.wp_seq, waypoints.waypoints.size());
        }
        else if(waypoint_reached.wp_seq < waypoints.waypoints.size() - 2) {
            ROS_INFO("Waypoint %d reached out of %lu. Offboard mode", waypoint_reached.wp_seq, waypoints.waypoints.size());
        }
        else {
            ROS_INFO("Going back to mission mode and landing");
        }
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Drone landed.");
    return 0;
}