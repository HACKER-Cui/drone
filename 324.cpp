#include <gnc_functions.hpp>
#include <ros/ros.h>
int counter=0;
//include API 
float wp_lat[8];
float wp_lon[8];
float wp_alt[8];
float wp_heading[8];
int p=0;


#include <geographic_msgs/GeoPoint.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>


float REFERENCE_ALTITUDE=3; // 参考点海拔

// GPS WGS84椭球体参数
const double WGS84_A = 6378137.0; // 长半轴
const double WGS84_B = 6356752.314245; // 短半轴
const double WGS84_F = 1 / 298.257223563; // 扁率
const double WGS84_E = sqrt(2 * WGS84_F - pow(WGS84_F, 2)); // 第一偏心率

geometry_msgs::Point gps_to_enu2(float latitude, float longitude, float altitude) {
    // 转换为弧度
    double lat_rad = latitude * M_PI / 180.0;
    double lon_rad = longitude * M_PI / 180.0;
    double ref_lat_rad = REFERENCE_LATITUDE * M_PI / 180.0;
    double ref_lon_rad = REFERENCE_LONGITUDE * M_PI / 180.0;

    // 计算子点的参数
    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_lon = sin(lon_rad);
    double cos_lon = cos(lon_rad);
    double sin_ref_lat = sin(ref_lat_rad);
    double cos_ref_lat = cos(ref_lat_rad);
    double sin_ref_lon = sin(ref_lon_rad);
    double cos_ref_lon = cos(ref_lon_rad);

    double delta_lon = lon_rad - ref_lon_rad;

    // 计算ENU坐标系下的XYZ坐标
    double x = delta_lon * WGS84_A * cos_lat;
    double y = (latitude - REFERENCE_LATITUDE) * WGS84_A;
    double z = altitude - REFERENCE_ALTITUDE;

    geometry_msgs::Point enu_point;
    enu_point.x = cos_ref_lat * cos_ref_lon * x + cos_ref_lat * sin_ref_lon * y + sin_ref_lat * z;
    enu_point.y = (-sin_ref_lon) * x + cos_ref_lon * y;
    enu_point.z = (-sin_ref_lat) * cos_ref_lon * x + (-sin_ref_lat) * sin_ref_lon * y + cos_ref_lat * z;

    return enu_point;
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

	//request takeoff
	takeoff(5);	
    ROS_INFO("current_lat=%f,current_lon=%f",get_current_lat(),get_current_lon());
	sleep(6);

  REFERENCE_LATITUDE = get_current_lat(); 
  REFERENCE_LONGITUDE = get_current_lon(); 


	  std::vector<geometry_msgs::Point> waypoints;
    // 添加航点，这里示例添加了三个航点
    waypoints.push_back(gps_to_enu2(30.88030592,121.90089630,5)); // 第二个航点
    waypoints.push_back(gps_to_enu2(30.88097500,121.90172176, 5)); // 第三个航点
    waypoints.push_back(gps_to_enu2(30.88097500,121.90172176, 5)); // 第三个航点
	ros::Rate rate(2.0);
  for (const auto& waypoint : waypoints) {
        mavros_msgs::PositionTarget local_setpoint_msg;
        local_setpoint_msg.header.stamp = ros::Time::now();
        local_setpoint_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        local_setpoint_msg.type_mask =mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        local_setpoint_msg.position = waypoint;
        local_setpoint_msg.velocity.x = 0.1;  // 设置X轴方向的速度限制为0.5 m/s
        local_setpoint_msg.velocity.y = 0.1;  // 设置Y轴方向的速度限制为0.5 m/s
        local_setpoint_msg.velocity.z = 0.1;  // 设置Z轴方向的速度限制为0.5 m/s
        local_setpoint_msg.acceleration_or_force.x=0.1;
        local_setpoint_msg.acceleration_or_force.y=0.1;
        local_setpoint_msg.acceleration_or_force.z=0.1;
        // 发布航点
       	set_destination(local_setpoint_msg.position.x,local_setpoint_msg.position.y,local_setpoint_msg.position.z,0);
        // 等待一段时间，然后发布下一个航点
        ros::Duration(12.0).sleep(); // 
    }
    land();


	return 0;
}