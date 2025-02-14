#include <ros/ros.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <cmath>
#include <gnc_functions.hpp>


geometry_msgs::Point gps_to_enu(double latitude, double longitude, double altitude) {
    double x = (longitude - REFERENCE_LONGITUDE) * 111319.9; // 经度变化大约每111319.9米变化1度
    double y = (latitude - REFERENCE_LATITUDE) * 111319.9; // 纬度变化大约每111319.9米变化1度
    double z = altitude; // 高度保持不变

    ROS_INFO("refx=%f,x1=%f",REFERENCE_LONGITUDE,x);
    geometry_msgs::Point enu_point;
    enu_point.x = x;
    enu_point.y = y;
    enu_point.z = z;

    return enu_point;

    }




int main(int argc, char** argv) 
{


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
    REFERENCE_LATITUDE=GPSCoordinate.latitude;
    REFERENCE_LONGITUDE=GPSCoordinate.longitude;
     ROS_INFO("reflat=%f, reflon=%f", GPSCoordinate.latitude, GPSCoordinate.longitude);


    ros::Rate rate(2.0);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
       
   
        
    

    set_destination(5,5,5,0);
    sleep(4);


        sleep(5);
    ROS_INFO("lat=%f,lon=%f",get_current_lat(),get_current_lon());

    geometry_msgs::Point enu_point=gps_to_enu(get_current_lat(),get_current_lon(),5.0);
    ROS_INFO("x2=%f,y=%f",enu_point.x,enu_point.y);
    set_destination(1 + enu_point.x, 1 +enu_point.y, 5, 0);  


    sleep(4);


    }

    return 0;
}
