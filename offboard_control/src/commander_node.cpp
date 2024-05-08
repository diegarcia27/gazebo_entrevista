
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

mavros_msgs::State current_state;
mavros_msgs::ExtendedState current_extended_state;
mavros_msgs::WaypointList waypoints;
mavros_msgs::WaypointReached waypoint_reached;
mavros_msgs::HomePosition current_home_position;
sensor_msgs::NavSatFix current_global_position;
mavros_msgs::WaypointReached waypoint_reached_combined;


ros::ServiceClient waypoint_set_current;

ros::Publisher wp_reached_combined;

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
    wp_reached_combined.publish(waypoint_reached_combined);
}

void home_position_cb(const mavros_msgs::HomePosition::ConstPtr& msg){
    current_home_position = *msg;
}

// Callback for global position
// Used to check if the drone has reached the waypoints in offboard mode
void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_global_position = *msg;
    // In offboard mode, we have to manually validate the waypoints
    if( current_state.mode == "OFFBOARD") {
        // Convert latitude and longitude to radians
        double lat1 = current_global_position.latitude * M_PI / 180.0;
        double lon1 = current_global_position.longitude * M_PI / 180.0;
        double lat2 = waypoints.waypoints[waypoint_reached_combined.wp_seq + 1].x_lat * M_PI / 180.0;
        double lon2 = waypoints.waypoints[waypoint_reached_combined.wp_seq + 1].y_long * M_PI / 180.0;

        // Earth radius in meters
        double earth_radius = 6371000.0;

        // Calculate the differences in latitude and longitude
        double delta_lat = lat2 - lat1;
        double delta_lon = lon2 - lon1;

        // Calculate the distance using the haversine formula
        double a = sin(delta_lat / 2) * sin(delta_lat / 2) +
                   cos(lat1) * cos(lat2) *
                   sin(delta_lon / 2) * sin(delta_lon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        float distance = earth_radius * c;
        // distance = sqrt(pow(distance, 2) + pow(, 2));

        ROS_INFO("Distance to next waypoint: %f meters", distance);
        if (distance < 0.4)
        {
            mavros_msgs::WaypointSetCurrent wp_current;
            wp_current.request.wp_seq = waypoint_reached_combined.wp_seq + 2;
            if (waypoint_set_current.call(wp_current) &&  wp_current.response.success) {
                waypoint_reached_combined.wp_seq++ ;
                wp_reached_combined.publish(waypoint_reached_combined);
            }    
        }
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "commander_node");
    ros::NodeHandle nh;
    ros::Rate rate(1); // 1 Hz
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>
            ("mavros/extended_state", 10, extended_state_cb);
    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>
            ("mavros/mission/waypoints", 10, waypoints_cb);
    ros::Subscriber waypoint_reached_sub = nh.subscribe<mavros_msgs::WaypointReached>
            ("mavros/mission/reached", 10, waypoint_reached_cb);
    ros::Subscriber global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 1, global_position_cb);
    ros::Subscriber home_position_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("mavros/home_position/home", 10, home_position_cb);

    //Publishers        
    wp_reached_combined = nh.advertise<mavros_msgs::WaypointReached>
            ("offboard_control/waypoint_reached_combined", 10);
    // Services
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    waypoint_set_current = nh.serviceClient<mavros_msgs::WaypointSetCurrent>
            ("mavros/mission/set_current");

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
    ROS_INFO("Printing waypoints locations:");
    for (int i = 0; i < waypoints.waypoints.size(); i++) {
        ROS_INFO("Waypoint %d: x = %f, y = %f, z = %f", i, waypoints.waypoints[i].x_lat, waypoints.waypoints[i].y_long, waypoints.waypoints[i].z_alt);
    }
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
    while(ros::ok() && waypoint_reached_combined.wp_seq != 0){
        ros::spinOnce();
        rate.sleep();
    }
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
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Drone landed.");
    return 0;
}