#include <ros/ros.h>
#include <ros/console.h>
#include <resources.hpp>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>
#include <tf2/LinearMath/Quaternion.h>


// Global variables
mavros_msgs::State current_state;
std_msgs::Int32 current_control_mode;
geometry_msgs::TwistStamped current_local_speed;
mavros_msgs::GlobalPositionTarget current_speed_sp;
float vx_integral;
float vy_integral;
float vz_integral;
// Publishers
ros::Publisher thrust_control_pub;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void speed_sp_cb(const mavros_msgs::GlobalPositionTarget::ConstPtr& msg){
    current_speed_sp = *msg;
}

void control_mode_cb(const std_msgs::Int32::ConstPtr& msg){
    current_control_mode = *msg;
    if (current_control_mode.data == OffboardControlMode::SHUTDOWN) {
        ros::shutdown();
    }
}
// Callback for global position
// The offboard control runs in sync with the global position
void local_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_local_speed = *msg;
    
    // Sanity checks
    if (current_control_mode.data == OffboardControlMode::OFF||
         !current_state.armed)
        return;
    
    double current_vx = current_local_speed.twist.linear.x;
    double current_vy = current_local_speed.twist.linear.y;
    double current_vz = current_local_speed.twist.linear.z;
    double current_yaw = current_local_speed.twist.angular.z;
    double vx_error = current_speed_sp.velocity.x - current_vx;
    double vy_error = current_speed_sp.velocity.y - current_vy;
    double vz_error = current_speed_sp.velocity.z - current_vz;

    // Update integrals if control loop is working
    if(current_control_mode.data == OffboardControlMode::THRUST &&
        current_state.mode == "OFFBOARD")
    {
        vx_integral += vx_error;
        vy_integral += vy_error;
        vz_integral += vz_error;
    }
    else {
        vx_integral = 0;
        vy_integral = 0;
        vz_integral = 0.708 * thrust_divider / acceleration_ki;
    }
    

    double ax = acceleration_kp * vx_error + acceleration_ki * vx_integral;
    double ay = acceleration_kp * vy_error + acceleration_ki * vy_integral;
    double az = acceleration_kp * vz_error + acceleration_ki * vz_integral;
    mavros_msgs::AttitudeTarget thrust_sp;
    thrust_sp.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                          mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                          mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    float pitch = 0;
    float roll = 0;
    double yaw = current_speed_sp.yaw;
    // Convert euler angles to quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    thrust_sp.orientation.x = q.x();
    thrust_sp.orientation.y = q.y();
    thrust_sp.orientation.z = q.z();
    thrust_sp.orientation.w = q.w();
    thrust_sp.thrust = az / thrust_divider;
    // ROS_INFO("vz:%lf, vz_error: %lf, thrust: %f", current_vz, vz_error, thrust_sp.thrust);
    
    

   
    if (current_control_mode.data == OffboardControlMode::THRUST &&
        current_state.mode == "OFFBOARD"){
        thrust_control_pub.publish(thrust_sp);
    }
    thrust_control_pub.publish(thrust_sp);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thrust_node");
    ros::NodeHandle nh;
    ros::Rate rate(1); // 1 Hz

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber control_mode_sub = nh.subscribe<std_msgs::Int32>
            ("offboard_control/control_mode", 10, control_mode_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/local_position/velocity_local", 10, local_velocity_cb);
    ros::Subscriber speed_sp_sub = nh.subscribe<mavros_msgs::GlobalPositionTarget>
            ("offboard_control/speed_sp", 10, speed_sp_cb);
    // Publisher
    thrust_control_pub = nh.advertise<mavros_msgs::AttitudeTarget>
        ("/mavros/setpoint_raw/attitude", 10);

    
    while(ros::ok() ){
        ros::spin();
    }
    return 0;
}