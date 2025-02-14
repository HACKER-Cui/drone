#include <ros/ros.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <cmath>
#include <gnc_functions.hpp>

ros::Subscriber global_position_sub;
mavros_msgs::GlobalPositionTarget current_target;
std::vector<std::pair<double, double>> waypoints; // 修改航点坐标为 double 类型
int current_waypoint_index = 0;
bool waypoint_reached = false;



void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    double current_lat = msg->latitude; // 将经纬度数据类型修改为 double
    double current_lon = msg->longitude;

    // Calculate distance to current waypoint
    double distance = calculateDistance(waypoints[current_waypoint_index].first,waypoints[current_waypoint_index].second,get_current_lat(),get_current_lon());
     ROS_INFO("lat=%f,lon=%f",waypoints[current_waypoint_index].first,waypoints[current_waypoint_index].second);
     ROS_INFO("distance=%f",distance);
     ROS_INFO("current_lat=%f,currentlon=%f",get_current_lat(),get_current_lon());
    // Check if distance is less than 1m and waypoint hasn't been reached yet
    if (distance < 1.0 && !waypoint_reached) {
        waypoint_reached = true; // Mark waypoint as reached
        if (current_waypoint_index < waypoints.size() - 1) {
            current_waypoint_index++; // Move to the next waypoint
            set_destination_lla(waypoints[current_waypoint_index].first, 
                                    waypoints[current_waypoint_index].second, 
                                    5, 0); // Assuming altitude is 5 and heading is 0
        }
    } else if (distance >= 1.0) {
        waypoint_reached = false; // Reset waypoint reached flag if distance is greater than 1m
    }
}

int main(int argc, char** argv) {

	 // Initialize ROS
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");

    // Initialize control publisher/subscribers
    init_publisher_subscriber(gnc_node);

    // Wait for FCU connection
    wait4connect();

    // Wait for user to switch to mode GUIDED
    wait4start();

    // Create local reference frame
    initialize_local_frame();

    // Request takeoff
    takeoff(5); // Assuming 5 meters altitude for takeoff

    // Additional code for waiting before proceeding
    ROS_INFO("current_lat=%f, current_lon=%f", get_current_lat(), get_current_lon());
    sleep(6);

    // Initialize ROS
    ros::init(argc, argv, "waypoint_publisher");
    ros::NodeHandle nh;

    // Create a publisher for MAVROS global position target
    global_lla_pos_pub_raw = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 10);

    // Subscribe to global position
    global_position_sub = nh.subscribe("/mavros/global_position/global", 10, global_position_cb);

    // Example waypoints
    waypoints = {
        {30.88032109,121.90087247}, // Waypoint 1
        {30.88096129,121.90170704}, // Waypoint 2
        {30.88127188,121.89981633}, // Waypoint 3
        {30.88081013, 121.90038092}, // Waypoint 4
        {30.88063247, 121.90057244}  // Waypoint 5
    };

    // Initialize ROS
    
    // Set initial destination
    set_destination_lla(waypoints[current_waypoint_index].first, 
                            waypoints[current_waypoint_index].second, 
                            5, 0); // Assuming altitude is 5 and heading is 0

    // Spin
    ros::spin();

    return 0;
}
