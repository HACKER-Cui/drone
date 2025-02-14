#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gnc_functions.hpp>
#include <ros/time.h> 
#include <ros/duration.h>
#include <std_msgs/Bool.h>
using namespace Eigen;
using namespace std;

Eigen::Isometry3d Twa,Tca,Tbc,Twb,Twc;
Eigen::Matrix3d rotation_matrix_bc,rotation_matrix_wb,rotation_matrix_ca;;
Eigen::Vector3d t_bc,t_wb,t_ca;
Quaterniond q_wb,q_ca;
Eigen::Vector4d desire_vel_;
Eigen::Vector3d desire_xyVel_;
Eigen::Vector3d temp_pos_drone;
Eigen::Vector3d posxyz_target;//期望飞机的空间位置
Eigen::Vector3d velxy_posz_target;//offboard模式下，发送给飞控的期望值
Eigen::Vector3d  ar_pose_;  //降落板相对飞机位置
Eigen::Vector3d  px4_pose_; //接收来自飞控的飞机位置 
Eigen::Vector3d desire_pose_;//期望的飞机相对降落板的位置
ros::Time last_request;//
ros::Time land_request;//
S_PID s_PidXY,s_PidZ,s_PidYaw;
S_PID_ITEM s_PidItemX;
S_PID_ITEM s_PidItemY;
S_PID_ITEM s_PidItemZ;
S_PID_ITEM s_PidItemYaw;
std::vector<std::pair<double, double>> waypoints;
bool detect_state=false;
bool countonlytime=true;//防止land_request一直更新
float desire_yaw_;//期望的飞机相对降落板的偏航角
float desire_yawVel_;
float desire_pose_x,desire_pose_y,desire_pose_z;
int count3=0;

bool count1time=true;//防止count3一次多加
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            t_wb << msg->pose.position.x ,msg->pose.position.y ,msg->pose.position.z;
            //在Eigen库中,四元数的存储顺序是w、x、y、z
            q_wb = Eigen::Quaterniond(msg->pose.orientation.w ,msg->pose.orientation.x ,msg->pose.orientation.y ,msg->pose.orientation.z);
            rotation_matrix_wb = q_wb.toRotationMatrix();
            Twb.linear() = rotation_matrix_wb;
            Twb.translation() = t_wb;
        }
void aruco_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
        	ROS_INFO("detected");
            t_ca << msg->pose.position.x ,msg->pose.position.y ,msg->pose.position.z;
            q_ca = Eigen::Quaterniond(msg->pose.orientation.w ,msg->pose.orientation.x ,msg->pose.orientation.y ,msg->pose.orientation.z);
            rotation_matrix_ca = q_ca.toRotationMatrix();
            Tca.linear() = rotation_matrix_ca;//获得旋转矩阵
            Tca.translation() = t_ca;//平移矩阵  Tca为变换矩阵
        }

void detceter_cb(const std_msgs::Bool Reached)
{
  detect_state=Reached.data;
}


class APMLanding {
public:
	  ros::Publisher mavros_setpoint_pos_pub_;

	   Eigen::Vector4d LandingPidProcess(Eigen::Vector3d &currentPos,float currentYaw,Eigen::Vector3d &expectPos,float expectYaw)
	{
	  Eigen::Vector4d s_PidOut;
		/*X方向的pid控制*///位置式PID控制
		s_PidItemX.difference = expectPos[0] - currentPos[0];//e(k)
		
		s_PidItemX.intergral += s_PidItemX.difference;
		if(s_PidItemX.intergral >= 50)		
			s_PidItemX.intergral = 50;
		else if(s_PidItemX.intergral <= -50) 
			s_PidItemX.intergral = -50;
		s_PidItemX.differential =  s_PidItemX.difference  - s_PidItemX.tempDiffer;//e(k-1)
	  s_PidItemX.tempDiffer = s_PidItemX.difference;

		s_PidOut[0] = (s_PidXY.p*s_PidItemX.difference + s_PidXY.d*s_PidItemX.differential + s_PidXY.i*s_PidItemX.intergral);
		//ROS_INFO("currentPos[0]=%f",currentPos[0]);
		//ROS_INFO("s_PidOut[0]=%f",s_PidOut[0]);
		/*Y方向的pid控制*/
		s_PidItemY.difference = expectPos[1] - currentPos[1];
		s_PidItemY.intergral += s_PidItemY.difference;
		if(s_PidItemY.intergral >= 50)		
			s_PidItemY.intergral = 50;
		else if(s_PidItemY.intergral <= -50) 
			s_PidItemY.intergral = -50;
		s_PidItemY.differential =  s_PidItemY.difference  - s_PidItemY.tempDiffer;
	  s_PidItemY.tempDiffer = s_PidItemY.difference;
		s_PidOut[1] =(s_PidXY.p*s_PidItemY.difference + s_PidXY.d*s_PidItemY.differential + s_PidXY.i*s_PidItemY.intergral);
		
		
		/*Z方向的pid控制*/
		s_PidItemZ.difference = expectPos[2] - currentPos[2];
		s_PidItemZ.intergral += s_PidItemZ.difference;
		if(s_PidItemZ.intergral >= 100)		
			s_PidItemZ.intergral = 100;
		else if(s_PidItemZ.intergral <= -100) 
			s_PidItemZ.intergral = -100;
		s_PidItemZ.differential =  s_PidItemZ.difference  - s_PidItemZ.tempDiffer;
	  s_PidItemZ.tempDiffer = s_PidItemZ.difference;
		s_PidOut[2] = s_PidZ.p*s_PidItemZ.difference + s_PidZ.d*s_PidItemZ.differential + s_PidZ.i*s_PidItemZ.intergral;

		/*Yaw方向的pid控制*/
		s_PidItemYaw.difference =  expectYaw - currentYaw;
		s_PidItemYaw.intergral += s_PidItemYaw.difference;
		if(s_PidItemYaw.intergral >= 100)		
			s_PidItemYaw.intergral = 100;
		else if(s_PidItemYaw.intergral <= -100) 
			s_PidItemYaw.intergral = -100;
		s_PidItemYaw.differential =  s_PidItemYaw.difference  - s_PidItemYaw.tempDiffer;
	  s_PidItemYaw.tempDiffer = s_PidItemYaw.difference;
		s_PidOut[3] = s_PidYaw.p*s_PidItemYaw.difference + s_PidYaw.d*s_PidItemYaw.differential + s_PidYaw.i*s_PidItemYaw.intergral;

		return s_PidOut;
	}

	  void send_body_velxyz_setpoint(const Eigen::Vector3d& vel_sp, float yaw_sp)
	{
	    mavros_msgs::PositionTarget pos_setpoint;
	    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
	    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
	    //Bit 10 should set to 0, means is not force sp
	    pos_setpoint.type_mask = 1 + 2 + 4 +/* 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024/* + 2048*/;
	    pos_setpoint.coordinate_frame = 8;

	    pos_setpoint.velocity.x = vel_sp[0];
	    pos_setpoint.velocity.y = vel_sp[1];
	    //pos_setpoint.velocity.z = vel_sp[2];
	    pos_setpoint.velocity.z = 0;
	    pos_setpoint.yaw_rate = yaw_sp;
	    mavros_setpoint_pos_pub_.publish(pos_setpoint);
	}
	 void send_body_posxyz_setpoint(const Eigen::Vector3d& vel_sp, float yaw_sp)
	{
	    mavros_msgs::PositionTarget pos_setpoint;
	    //Bitmask toindicate which dimensions should be ignored (1 means ignore,0 means not ignore; Bit 10 must set to 0)
	    //Bit 1:x, bit 2:y, bit 3:z, bit 4:vx, bit 5:vy, bit 6:vz, bit 7:ax, bit 8:ay, bit 9:az, bit 10:is_force_sp, bit 11:yaw, bit 12:yaw_rate
	    //Bit 10 should set to 0, means is not force sp
	    pos_setpoint.type_mask = /*1 + 2 + 4 */+ 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024/* + 2048*/;
	    pos_setpoint.coordinate_frame = 8;

	    pos_setpoint.position.x = vel_sp[0];
	    pos_setpoint.position.y = vel_sp[1];
	    //pos_setpoint.velocity.z = vel_sp[2];
	    pos_setpoint.position.z = 0;
	    pos_setpoint.yaw_rate = yaw_sp;
	    mavros_setpoint_pos_pub_.publish(pos_setpoint);
	}
};

void Initialize()
{
  desire_pose_x=0;
  desire_pose_y=0;
  desire_pose_z=3;
  desire_pose_[0] = desire_pose_x;
  desire_pose_[1] = desire_pose_y;
  desire_pose_[2] = desire_pose_z;
  desire_yawVel_ = 0;
  desire_xyVel_[0]  = 0;
  desire_xyVel_[1]  = 0;
  desire_xyVel_[2]  = 0;
  s_PidXY.p=0.6;
  //s_PidXY.d=0.05;
  s_PidXY.d=0.05;
 // s_PidXY.i=0.001;
  s_PidXY.i=0.01;
  s_PidZ.p=0.2;
  detect_state=false;
}

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
	Initialize();
	APMLanding APMLand;
	ros::Subscriber aruco_pose_sub = gnc_node.subscribe<geometry_msgs::PoseStamped>("/aruco/pose", 10, aruco_pos_cb);
	ros::Subscriber position_sub = gnc_node.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);  
	APMLand.mavros_setpoint_pos_pub_ = gnc_node.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
	ros::Subscriber detceter = gnc_node.subscribe<std_msgs::Bool>("/ifdetected",10,detceter_cb);
	takeoff(3);
	waypoints = {
        {30.88113494,121.90081122}, // Waypoint 1
        {30.88128879,121.90057775}, // Waypoint 2
        {30.88116714,121.90026091}, // Waypoint 3
        {30.88115641,121.90103634 }, // Waypoint 4
        {30.88063247, 121.90057244}  // Waypoint 5
    };
	// //specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	nextWayPoint.x = 0;
	nextWayPoint.y = 0;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 0;
	nextWayPoint.y = 1;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	nextWayPoint.x = 1.5;
	nextWayPoint.y = 4;
	nextWayPoint.z = 3;
	nextWayPoint.psi = 0;
	waypointList.push_back(nextWayPoint);
	t_bc <<  0, 0, 0;
    rotation_matrix_bc << 1, 0, 0,
                          0 ,-1, 0,
                          0, 0, -1;

    // Tbc.linear() = rotation_matrix_bc;
    // Tbc.translation() = t_bc;
    t_ca << 0, 0, 0;
    int state_flag=0;
	ros::Rate rate(10);
	int counter = 0;
   	
	while(ros::ok())
	{
		ROS_INFO("state_flag=%d",state_flag);
		//ROS_INFO("Translation TwaVector: %f, %f, %f", Twa.translation().x(), Twa.translation().y(), Twa.translation().z());
		ros::spinOnce();
		rate.sleep();
		if(state_flag == 0)
		{

			if(check_waypoint_reached(.3) == 1)
			{
				if (counter < waypointList.size())
				{
					set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
                    counter++;
					//desire_vel_ = APMLand.LandingPidProcess(t_ca,0,desire_pose_,0);	
					ROS_INFO("counter=%d",counter);
				}else{
					 state_flag = 5;
					 ROS_INFO("state_flag = 5");
					 sleep(3);
				} 
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
				desire_xyVel_[0] = desire_vel_[1];
				desire_xyVel_[2] = desire_vel_[2];
				desire_yawVel_ = desire_vel_[3];
				APMLand.send_body_velxyz_setpoint(desire_xyVel_,desire_yawVel_);
				}
			  	else{//搜寻周围航点
			  		countonlytime=true;	
					//desire_vel_[0] = 0;
					//desire_vel_[1] = 0;
					//desire_vel_[2] = 0;
					//desire_vel_[3] = 0;
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
					switch (count3){
						case 0:
							set_destination(waypointList[2].x,waypointList[2].y,waypointList[2].z, waypointList[2].psi);
						break;
						case 1:
							set_destination(waypointList[2].x-1,waypointList[2].y,waypointList[2].z, waypointList[2].psi);
						break;
						case 2:
							set_destination(waypointList[2].x-1,waypointList[2].y-1,waypointList[2].z, waypointList[2].psi);
						break;
						case 3:
							set_destination(waypointList[2].x,waypointList[2].y-1,waypointList[2].z, waypointList[2].psi);
						break;
						case 4:
							set_destination(waypointList[2].x,waypointList[2].y,waypointList[2].z, waypointList[2].psi);
						break;
						case 5:
						set_destination(waypointList[2].x+1,waypointList[2].y,waypointList[2].z, waypointList[2].psi);
						break;
						case 6:
						set_destination(waypointList[2].x+1,waypointList[2].y+1,waypointList[2].z, waypointList[2].psi);
						break;
						case 7:
						set_destination(waypointList[2].x,waypointList[2].y+1,waypointList[2].z, waypointList[2].psi);
						break;
					}
				}
					
         		//ROS_INFO("Translation TcaVector: %f, %f, %f", Tca.translation().x(), Tca.translation().y(), Tca.translation().z());
         		//ros::Time last_request = ros::Time::now();
         		  if((abs(t_ca[0])<0.05)&&(abs(t_ca[1])<0.05))
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