#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <gnc_functions.hpp>




void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ROS_INFO("CB");
	sensor_msgs::LaserScan current_2D_scan;
  	current_2D_scan = *msg;
	float avoidance_vector_x = 0; 
	float avoidance_vector_y = 0;
	bool avoid = false;
	
	for(int i=1; i<current_2D_scan.ranges.size(); i++)
	{
		float d0 = 3; 
		float k = 0.5;

		if(current_2D_scan.ranges[i] < d0 && current_2D_scan.ranges[i] > .35)
		{
			avoid = true;
			

		}
	}
	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180);

	if(avoid)
	{

     ROS_INFO("avoid");


		
	}
	

}

int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle n;
	ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);
	//initialize control publisher/subscribers
	init_publisher_subscriber(n);


	//create local reference frame 
	initialize_local_frame();

	//request takeoff

	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		
		ros::spinOnce();
		rate.sleep();
		
	
	
	}

	return 0;
}

