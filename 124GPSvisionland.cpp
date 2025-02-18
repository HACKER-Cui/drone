#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <gnc_functions2.hpp>
#include <ros/time.h> 
#include <ros/duration.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#define PI 3.14159265358979323846
using namespace Eigen;
using namespace std;

Eigen::Isometry3d Twa,Tca,Tbc,Twb,Twc;//刚体变换矩阵
Eigen::Matrix3d rotation_matrix_bc,rotation_matrix_wb,rotation_matrix_ca;;
Eigen::Vector3d t_wb,t_ca;
Quaterniond q_wb,q_ca;

ros::Time last_request;//
ros::Time land_request;//

std::vector<std::pair<double, double>> waypoints;
int current_waypoint_index = 0;
bool countonlytime=true;//防止land_request一直更新
int count3=0;
bool count1time=true;//防止count3一次多加
double distancetp=0;
double Azimuth=0;
float deg2rad = (M_PI/180);
double toRadians(double degree) {
    return degree * PI / 180.0;
}

// 将弧度转换为度数
double toDegrees(double radian) {
    return radian * 180.0 / PI;
}

// 计算方位角（方向角）
double calculateAzimuth(double lat1, double lon1, double lat2, double lon2) {
    // 将经纬度从度数转换为弧度
    lat1 = toRadians(lat1);
    lon1 = toRadians(lon1);
    lat2 = toRadians(lat2);
    lon2 = toRadians(lon2);

    double deltaLon = lon2 - lon1;

    // 使用公式计算方向角
    double x = sin(deltaLon) * cos(lat2);
    double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(deltaLon);

    double azimuth = atan2(x, y);  // 结果为弧度

    // 将方位角从弧度转换为度数
    double azimuthDeg = toDegrees(azimuth);

    // 调整结果到 0-360 度范围内
    if (azimuthDeg < 0) {
        azimuthDeg += 360.0;
    }

    return azimuthDeg;
}

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("avoid=%d",avoid);
    ROS_INFO("Azimuth=%f",Azimuth);
    sensor_msgs::LaserScan current_2D_scan;
    current_2D_scan=*msg;
    float avoidance_vector_x=0;
    float avoidance_vector_y=0;

    avoid=false;

    float d0 = 3.5; 
    float k = 0.5;
    for(int i=1; i<current_2D_scan.ranges.size(); i++)
    {
        
        if(current_2D_scan.ranges[i] < d0 && current_2D_scan.ranges[i] >.35)
        {
            avoid = true;
            float x = cos(current_2D_scan.angle_increment*i);
            float y = sin(current_2D_scan.angle_increment*i);
            float U = -.5*k*pow(((1/current_2D_scan.ranges[i]) - (1/d0)), 2);   

            avoidance_vector_x = avoidance_vector_x + x*U;
            avoidance_vector_y = avoidance_vector_y + y*U;
        }
    }
    float ux=cos(Azimuth*deg2rad);
    float uy=sin(Azimuth*deg2rad);
    float Uat=-.5*k*pow(distancetp,2);
    float att_vector_x=Uat*ux;
    float att_vector_y=Uat*uy;

    float current_heading = get_current_heading();
  
    avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad);
    avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);
    
    att_vector_x = att_vector_x*cos((current_heading)*deg2rad) - att_vector_y*sin((current_heading)*deg2rad);
    att_vector_y = att_vector_x*sin((current_heading)*deg2rad) + att_vector_y*cos((current_heading)*deg2rad);
    if(avoid)
    {
        //if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 4)
        //{
        //    avoidance_vector_x = 4 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
        //    avoidance_vector_y = 4 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
        //}
        geometry_msgs::Point current_pos;
        current_pos = get_current_location();

        set_destination(att_vector_x+avoidance_vector_x + current_pos.x,att_vector_y+avoidance_vector_y + current_pos.y, 4, 0);   
    }    
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    t_wb << msg->pose.position.x ,msg->pose.position.y ,msg->pose.position.z;
    //在Eigen库中,四元数的存储顺序是w、x、y、z
    q_wb = Eigen::Quaterniond(msg->pose.orientation.w ,msg->pose.orientation.x ,msg->pose.orientation.y ,msg->pose.orientation.z);
    rotation_matrix_wb = q_wb.toRotationMatrix();
    Twb.linear() = rotation_matrix_wb;
    Twb.translation() = t_wb;
}
void aruco_pos_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
    ROS_INFO("detected");
    if(!msg->markers.empty()) {
        const auto& first_marker = msg->markers[0];
        t_ca << first_marker.pose.pose.position.x, 
               first_marker.pose.pose.position.y, 
               first_marker.pose.pose.position.z;
        q_ca = Eigen::Quaterniond(
            first_marker.pose.pose.orientation.w,
            first_marker.pose.pose.orientation.x,
            first_marker.pose.pose.orientation.y,
            first_marker.pose.pose.orientation.z
        );
    }
    rotation_matrix_ca = q_ca.toRotationMatrix();
    Tca.linear() = rotation_matrix_ca;  // 获得旋转矩阵
    Tca.translation() = t_ca;           // 平移矩阵 Tca 为变换矩阵
	ROS_INFO("x,%f,y,%f,z,%f",t_ca[0],t_ca[1],t_ca[2]);
	
}

void detceter_cb(const std_msgs::Bool Reached)
{
  detect_state=Reached.data;
}

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	init_publisher_subscriber(gnc_node);
  	// wait for FCU connection
	wait4connect();
	//wait for used to switch to mode GUIDED
	wait4start();
	//create local reference frame 
	initialize_local_frame();
	Initialize();
	APMLanding APMLand;
	ros::Subscriber aruco_pose_sub = gnc_node.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 10, aruco_pos_cb);
	ros::Subscriber position_sub = gnc_node.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);  
	APMLand.mavros_setpoint_pos_pub_ = gnc_node.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
	ros::Subscriber detceter = gnc_node.subscribe<std_msgs::Bool>("/ifdetected",10,detceter_cb);
	ros::Subscriber collision_sub = gnc_node.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);
	//ros::Subscriber collision_sub = gnc_node.subscribe<sensor_msgs::LaserScan>("/scan", 1, scan_cb);
	//ros::Subscriber global_position_sub = gnc_node.subscribe("/mavros/global_position/global", 10, global_position_cb);
	takeoff(4);
	ROS_INFO("current_lat=%f, current_lon=%f", get_current_lat(), get_current_lon());
	sleep(3);
	waypoints = {
      //{30.51935833,120.72055278}, // 礼堂前坐标
	  //{30.51905833,120.72055278},
      //{30.51855833,120.72055278}
		{30.5181129, 120.7188859},
		{30.5180430, 120.7188866},
		{30.5180731, 120.7189362},
		//{30.515274005030925, 120.7233008669868}
      //{30.51823611,120.72053888}, // 中间坐标
      //{30.51761963,120.72053888}, // 学院楼前坐标
     // {30.88115641,121.90103634 }, // Waypoint 4
     // {30.88063247, 121.90057244}  // Waypoint 5
    };
    set_destination_lla(waypoints[1].first, 
                            waypoints[1].second, 
                            4, 0);
    
    t_ca << 0, 0, 0;
    int state_flag=0;
	ros::Rate rate(10);
	int counter = 0;
   	
	while(ros::ok())
	{
		ROS_INFO("state_flag=%d",state_flag);
		ros::spinOnce();
		rate.sleep();
		if(state_flag == 0)
		{
				if (current_waypoint_index < waypoints.size()) {
					distancetp = calculateDistance(waypoints[current_waypoint_index].first,waypoints[current_waypoint_index].second,get_current_lat(),get_current_lon());
					Azimuth=calculateAzimuth(get_current_lat(),get_current_lon(),waypoints[current_waypoint_index].first,waypoints[current_waypoint_index].second);
					ROS_INFO("distancetp=%f",distancetp);
					ROS_INFO("current_waypoint_index=%d",current_waypoint_index);
					if (distancetp < 1.0) {

					current_waypoint_index++;
					}
					else{
						set_destination_lla(waypoints[current_waypoint_index].first, 
	                    waypoints[current_waypoint_index].second, 
	                    4, 0);
					}
      			} 
				else{
					 state_flag = 5;
					 ROS_INFO("state_flag = 5");
					 sleep(3);
				} 

		}
		if (state_flag==5)//到达最后一个航点
		{  
         		ROS_INFO("count3=%d",count3);
				if(detect_state==true){//收到二维码的回调函数
					desire_vel_ = APMLand.LandingPidProcess(t_ca,0,desire_pose_,0);
					//APMLand.send_body_posxyz_setpoint(t_ca,desire_yawVel_);
					ROS_INFO("calibrating");
					if(count3>7){
						count3=0;
					}
				
				desire_xyVel_[1] = desire_vel_[0];//机体坐标和相机坐标xy相反
				if (desire_xyVel_[1]>1){
					desire_xyVel_[1]=1;
				}else if (desire_xyVel_[1]<-1){
					desire_xyVel_[1]=-1;
				} 
				desire_xyVel_[0] = desire_vel_[1];
				if (desire_xyVel_[0]>1){
					desire_xyVel_[0]=1;
				}else if (desire_xyVel_[1]<-1){
					desire_xyVel_[0]=-1;
				}
				desire_xyVel_[2] = desire_vel_[2];
				desire_yawVel_ = desire_vel_[3];
				APMLand.send_body_velxyz_setpoint(desire_xyVel_,desire_yawVel_);
				}
			  	else{//搜寻周围航点
						if(count1time){
							count3++;
							count1time=false;
							last_request = ros::Time::now();//更新5秒切点时间戳
						}
						if(count3>7){
							count3=0;
						}
						if(ros::Time::now() - last_request > ros::Duration(5.0)){//每5秒count3加一
							count1time=true;
						}
					ROS_INFO("oh no");
					// switch (count3){
					// 	case 0:
					// 		set_destination(waypointList[2].x,waypointList[2].y,waypointList[2].z, waypointList[2].psi);
					// 	break;
					// }
				}
         		  if((abs(t_ca[0])<0.05)&&(abs(t_ca[1])<0.05)&&detect_state==true)
        		  {
        		  	if(countonlytime){
        		  		land_request = ros::Time::now();
        		  		countonlytime=false;
        		  	}
        		  	if(ros::Time::now() - land_request > ros::Duration(3.0)){
        		  		state_flag = 6;
        		  	}
        		  	
         		  }
		}	 
        if( state_flag == 6)
 		{

		  	ROS_INFO("REACH landing point");
		  	land();
		  	sleep(5);
		}
		
	}
	return 0;
}
