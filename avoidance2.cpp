#include <ros/ros.h>
#include <gnc_functions.hpp>
#include <sensor_msgs/LaserScan.h>
#include <random>
bool avoid=false;
int counter = 0;
int avoidnum=0;
int avoidnumold=0;
int avoidnumnew=0;
int count2time=0;
int avoidtimes=0;
int waypointIndex = 0;

//std::default_random_engine generator; // 创建随机数生成器
//std::uniform_real_distribution<float> distribution(-1.0, 1.0); // 创建均匀分布随机数生成器，范围为-1到1

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	sensor_msgs::LaserScan current_2D_scan;
	current_2D_scan=*msg;
	float avoidance_vector_x=0;
	float avoidance_vector_y=0;
	avoid=false;
	avoidnum=0;
	for(int i=1; i<current_2D_scan.ranges.size(); i++)
	{
		float d0 = 3.5; 
		float k = 0.5;

		if(current_2D_scan.ranges[i] < d0 && current_2D_scan.ranges[i] > .35)
		{
			avoid = true;
			avoidnum=1;
			float x = cos(current_2D_scan.angle_increment*i);
			float y = sin(current_2D_scan.angle_increment*i);
			float U = -.5*k*pow(((1/current_2D_scan.ranges[i]) - (1/d0)), 2);	
			avoidance_vector_x = avoidance_vector_x + x*U;
			avoidance_vector_y = avoidance_vector_y + y*U;
		}
	}
	
	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180);
	avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad);
	avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);
	if(avoid)
	{
		avoidtimes=avoidtimes++;
		if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 4)
		{
			avoidance_vector_x = 4 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
			avoidance_vector_y = 4 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
		}
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		//float random_x = distribution(generator); // 在[-1, 1]范围内生成随机数
        //float random_y = distribution(generator); // 在[-1, 1]范围内生成随机数

        //avoidance_vector_x += random_x; // 加入随机扰动
        //avoidance_vector_y += random_y; // 加入随机扰
		//local_move(avoidance_vector_x,avoidance_vector_y, 2, 0);	
		set_destination(avoidance_vector_x + current_pos.x, avoidance_vector_y + current_pos.y+avoidtimes, 2, 0);	
		sleep(8);
	}
	avoidnumnew=avoidnum;
	if(avoidnumnew!=avoidnumold)
	{
		ROS_INFO("out of warn");
		//count2time++;
		counter--;
		if(count2time==2)
		{
			ROS_INFO("counter--");
			
			count2time=0;
			avoidtimes=0;
		}
	}
	avoidnumold=avoidnumnew;
}

int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "avoidance_node");
	ros::NodeHandle n;
	ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);
	//ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, scan_cb);
	//add rest of code here 
	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

	// wait for FCU connection
	wait4connect();

	//wait for user to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(2);

		//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 10;
	nextWayPoint.y = 0;
	nextWayPoint.z = 2;
	nextWayPoint.psi = -90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 10;
	nextWayPoint.y = 10;
	nextWayPoint.z = 2;
	nextWayPoint.psi = -180;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 10;
	nextWayPoint.z = 2;
	nextWayPoint.psi = 90;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 2;
	nextWayPoint.psi = 180;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 2;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);


	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);//0.5 second per while loop

	while(ros::ok())
	{
		//ROS_INFO("avoidnum %d,avoidnumold %d,avoidnumnew %d",avoidnum,avoidnumold,avoidnumnew);
		ros::spinOnce();
		rate.sleep();
	    gnc_api_waypoint nextWaypoint = waypointList[counter];
			if(check_waypoint_reached(.3) == 1)
			{
				if(avoid == false)
				{
					if (counter < waypointList.size())
					{
						set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
						ROS_INFO("Point %d avoid %d", counter,avoid);
						counter++;	
					}else{
						//land after all waypoints are reached
						land();
					}
				}

			}	
		
		
	}
	return 0;
}