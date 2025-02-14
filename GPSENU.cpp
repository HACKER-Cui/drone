#include <gnc_functions.hpp>
#include <ros/ros.h>
int counter=0;
//include API 
float wp_lat[8];
float wp_lon[8];
float wp_alt[8];
float wp_heading[8];
int p=0;
int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(2);	
	sleep(6);
	  std::vector<geometry_msgs::Point> waypoints;
    
    waypoints.push_back(gps_to_enu(-35.36134200, 149.16515480, 10.0)); // 第一个航点
    waypoints.push_back(gps_to_enu(-35.36175747, 149.16295178, 10.0)); // 第二个航点
    waypoints.push_back(gps_to_enu(-35.36323964, 149.16306193, 10.0)); // 第三个航点
	ros::Rate rate(2.0);
  for (const auto& waypoint : waypoints) {
        mavros_msgs::PositionTarget local_setpoint_msg;
        local_setpoint_msg.header.stamp = ros::Time::now();
        local_setpoint_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        local_setpoint_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                       mavros_msgs::PositionTarget::IGNORE_VY |
                                       mavros_msgs::PositionTarget::IGNORE_VZ |
                                       mavros_msgs::PositionTarget::IGNORE_AFX |
                                       mavros_msgs::PositionTarget::IGNORE_AFY |
                                       mavros_msgs::PositionTarget::IGNORE_AFZ |
                                       mavros_msgs::PositionTarget::FORCE |
                                       mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        local_setpoint_msg.position = waypoint;

        // 发布航点
       	set_destination(local_setpoint_msg.position.x,local_setpoint_msg.position.y,local_setpoint_msg.position.z,0);
        

        ros::Duration(8.0).sleep(); 
    }
    land();


	return 0;
}