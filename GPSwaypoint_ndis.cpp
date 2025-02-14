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
	ROS_INFO("counter=%d,p=%d",counter,p);	
	sleep(6);

 	wp_lat[0]=-35.363262;
 	wp_lon[0]=149.165237;
 	wp_alt[0]=2;
 	wp_heading[0]=0;

 	wp_lat[1]=-35.36134200; 
 	wp_lon[1]=149.16515480;
 	wp_alt[1]=2;
 	wp_heading[1]=-90;

 	wp_lat[2]=-35.36175747; 
 	wp_lon[2]=149.16295178;
 	wp_alt[2]=2;
 	wp_heading[2]=0;

 	wp_lat[3]=-35.36323964; 
 	wp_lon[3]=149.16306193; 
 	wp_alt[3]=2;
 	wp_heading[3]=90;

 	wp_lat[4]=-35.363262;
 	wp_lon[4]=149.165237;
 	wp_alt[4]=2;
 	wp_heading[4]=180;

	ros::Rate rate(2.0);	

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		counter++;
		if(counter==20)
		{
			counter=0;
			p++;
		}
		if(p<=4) 
		{
			ROS_INFO("counter=%d,p=%d",counter,p);
			//ROS_INFO("current_lat=%f,current_lon=%f",get_current_lat(),get_current_lon());
			set_destination_lla_raw(wp_lat[p], wp_lon[p], 2.0,wp_heading[p]);
		}
		else 
		{
			land();
		}
	}
}
