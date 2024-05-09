#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>

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

//Parameters
float speed_kp = 2.0;
float max_speed = 5.0;

// Global variables
mavros_msgs::State current_state;
sensor_msgs::NavSatFix current_global_position;
std_msgs::Int32 current_control_mode;
geometry_msgs::PoseStamped current_local_position;
mavros_msgs::GlobalPositionTarget speed_sp;
mavros_msgs::GlobalPositionTarget current_position_sp;

// Publishers
ros::Publisher speed_sp_pub;
ros::Publisher speed_control_pub;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void position_sp_cb(const mavros_msgs::GlobalPositionTarget::ConstPtr& msg){
    current_position_sp = *msg;
}

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_position = *msg;
}

void control_mode_cb(const std_msgs::Int32::ConstPtr& msg){
    current_control_mode = *msg;
    if (current_control_mode.data == -1) {
        ros::shutdown();
    }
}
// Callback for global position
// The offboard control runs in sync with the global position
void global_postion_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_global_position = *msg;
    // Sanity checks
    // Sanity checks
    if (current_control_mode.data < 1||
         !current_state.armed)
        return;
    
    // Convert latitude and longitude to radians
    double lat1 = current_global_position.latitude * M_PI / 180.0;
    double lon1 = current_global_position.longitude * M_PI / 180.0;
    double lat2 = current_position_sp.latitude * M_PI / 180.0;
    double lon2 = current_position_sp.longitude * M_PI / 180.0;

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
    float z_distance = current_position_sp.altitude - current_local_position.pose.position.z;
    // Calculate the direction in degrees from north
    double direction_rad = atan2(delta_lat, delta_lon);
    // Calculate the speed setpoint
    double horizontal_speed = distance * speed_kp;
    if (horizontal_speed > max_speed) {
        horizontal_speed = max_speed;
    }
    double vertical_speed = z_distance * speed_kp;
    if (vertical_speed > max_speed) {
        vertical_speed = max_speed;
    }
    // Set the speed setpoint
    speed_sp.velocity.x = horizontal_speed * cos(direction_rad);
    speed_sp.velocity.y = horizontal_speed * sin(direction_rad);
    speed_sp.velocity.z = vertical_speed;
    speed_sp.yaw = direction_rad;
    speed_sp.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    speed_sp.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_LATITUDE |
                            mavros_msgs::GlobalPositionTarget::IGNORE_LONGITUDE |
                            mavros_msgs::GlobalPositionTarget::IGNORE_ALTITUDE |
                            mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                            mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
                            mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                            mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
    if (current_control_mode.data == 2){
        speed_sp_pub.publish(speed_sp);
    }
    speed_sp_pub.publish(speed_sp);
    speed_control_pub.publish(speed_sp);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_node");
    ros::NodeHandle nh;
    ros::Rate rate(1); // 1 Hz

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, global_postion_cb);
    ros::Subscriber control_mode_sub = nh.subscribe<std_msgs::Int32>
            ("offboard_control/control_mode", 10, control_mode_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber position_sp_sub = nh.subscribe<mavros_msgs::GlobalPositionTarget>
            ("offboard_control/position_sp", 10, position_sp_cb);
    // Publisher
    speed_sp_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
        ("/mavros/setpoint_raw/global", 10);
    speed_control_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
        ("offboard_control/speed_sp", 10);

    
    while(ros::ok() ){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}