#include <ros/ros.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <vector>

// 定义GPS起始点的经纬度

// 定义航点结构体
struct Waypoint {
    double latitude;
    double longitude;
    double altitude;
};

// 将GPS坐标转换为ENU坐标系下的XYZ坐标
geometry_msgs::Point gps_to_enu(double latitude, double longitude, double altitude) {
    double x = (longitude - REFERENCE_LONGITUDE) * 111319.9; // 经度变化大约每111319.9米变化1度
    double y = (latitude - REFERENCE_LATITUDE) * 111319.9; // 纬度变化大约每111319.9米变化1度
    double z = altitude; // 高度保持不变

    geometry_msgs::Point enu_point;
    enu_point.x = x;
    enu_point.y = y;
    enu_point.z = z;

    return enu_point;
}

int main(int argc, char** argv) {
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
    float REFERENCE_LATITUDE = get_current_lat(); 
    float REFERENCE_LONGITUDE = get_current_lon(); 
    std::vector<Waypoint> waypoints = {
        {-35.36134200, 149.16515480, 3.0},
        {-35.36175747, 149.16295178, 3.0},
        {-35.36323964, 149.16306193, 3.0}
    };

    size_t current_waypoint_index = 0; // 当前航点索引

    while (ros::ok()) {
        // 将GPS坐标转换为ENU坐标系下的XYZ坐标
        geometry_msgs::Point enu_point = gps_to_enu(gps_latitude, gps_longitude, gps_altitude);

        // 计算当前位置与当前航点的距离
        double distance_to_waypoint = sqrt(pow(enu_point.x - waypoints[current_waypoint_index].latitude, 2) +
                                           pow(enu_point.y - waypoints[current_waypoint_index].longitude, 2) +
                                           pow(enu_point.z - waypoints[current_waypoint_index].altitude, 2));
        if (distance_to_waypoint <= 1.0) {
            current_waypoint_index++;
            if (current_waypoint_index >= waypoints.size()) {
                // 已经达到最后一个航点，可以选择是停止还是继续循环
                break; // 在此处退出循环，或者根据需要执行其他操作
            }
        }

        // 发布到 /mavros/setpoint_raw/local 话题
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
        local_setpoint_msg.position = enu_point;

        local_setpoint_pub.publish(local_setpoint_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
