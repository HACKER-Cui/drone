#include <ros/ros.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <cmath>
#include <gnc_functions.hpp>
#include <sensor_msgs/LaserScan.h>
bool avoid=false;
int avoidnum=0;
int avoidnumold=0;
int avoidnumnew=0;
double distance=0;
float Beta=0;
ros::Subscriber global_position_sub;
mavros_msgs::GlobalPositionTarget current_target;
std::vector<std::pair<double, double>> waypoints; // 修改航点坐标为 double 类型
int current_waypoint_index = 0;
bool waypoint_reached = false;
bool enablelocalmove=false;

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("avoid=%d",avoid);
    sensor_msgs::LaserScan current_2D_scan;
    current_2D_scan=*msg;
    float avoidance_vector_x=0;
    float avoidance_vector_y=0;
    float drag_vector_x=0;
    float drag_vector_y=0;

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
            Beta=current_2D_scan.angle_increment*i;
            float x = cos(current_2D_scan.angle_increment*i);
            float y = sin(current_2D_scan.angle_increment*i);
            float U = -.5*k*pow(((1/current_2D_scan.ranges[i]) - (1/d0)), 2)*distance;   
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
        //if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 4)
        //{
        //    avoidance_vector_x = 4 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
        //    avoidance_vector_y = 4 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
        //}
        geometry_msgs::Point current_pos;
        current_pos = get_current_location();
        drag_vector_x = current_pos.x*cos((current_heading)*deg2rad) - current_pos.y*sin((current_heading)*deg2rad);
        drag_vector_y = current_pos.x*sin((current_heading)*deg2rad) + current_pos.y*cos((current_heading)*deg2rad);


        //local_move(avoidance_vector_x,avoidance_vector_y, 2, 0);  
        set_destination(avoidance_vector_x + current_pos.x, avoidance_vector_y + current_pos.y, 2, 0);   
        enablelocalmove=true;
        sleep(5);


    }    
    avoidnumnew=avoidnum;
    if(avoidnumnew!=avoidnumold)
    {
        ROS_INFO("out of warn");
        //count2time++;
        avoid=false;
    }
    avoidnumold=avoidnumnew;
    if(enablelocalmove)
    {
        local_move(3*sin(Beta+(current_heading)*deg2rad),3*cos(Beta+(current_heading)*deg2rad),2,0);
        sleep(5);
        enablelocalmove=false;
    }
}



void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    double current_lat = msg->latitude; // 将经纬度数据类型修改为 double
    double current_lon = msg->longitude;
    set_destination_lla(waypoints[current_waypoint_index].first, 
                                        waypoints[current_waypoint_index].second, 
                                        2, 0);
    ROS_INFO("wpcb");
    // Calculate distance to current waypoint
     distance = calculateDistance(waypoints[current_waypoint_index].first,waypoints[current_waypoint_index].second,get_current_lat(),get_current_lon());
     //ROS_INFO("lat=%f,lon=%f",waypoints[current_waypoint_index].first,waypoints[current_waypoint_index].second);
    if(avoid==false)
    {
         ROS_INFO("distance=%f",distance);
         ROS_INFO("current_waypoint_index=%d",current_waypoint_index);
         //ROS_INFO("current_lat=%f,currentlon=%f",get_current_lat(),get_current_lon());
        // Check if distance is less than 1m and waypoint hasn't been reached yet
        if (distance < 1.0 && !waypoint_reached) {
            waypoint_reached = true; // Mark waypoint as reached
            if (current_waypoint_index < waypoints.size() - 1) {
                current_waypoint_index++; // Move to the next waypoint
                set_destination_lla(waypoints[current_waypoint_index].first, 
                                        waypoints[current_waypoint_index].second, 
                                        2, 0); // Assuming altitude is 5 and heading is 0
            }
        } else if (distance >= 1.0) {   
            waypoint_reached = false; // Reset waypoint reached flag if distance is greater than 1m
        }
        if(current_waypoint_index>=3)
        {
            land();
            ROS_INFO("land");
        }
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
    takeoff(2); // Assuming 5 meters altitude for takeoff

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
    ros::Subscriber collision_sub = nh.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);
    // Example waypoints
    waypoints = {
        {30.88113494,121.90081122}, // Waypoint 1
        {30.88128879,121.90057775}, // Waypoint 2
        {30.88116714,121.90026091}, // Waypoint 3
        {30.88115641,121.90103634 }, // Waypoint 4
        {30.88063247, 121.90057244}  // Waypoint 5
    };

    // Initialize ROS
    
    // Set initial destination
    set_destination_lla(waypoints[current_waypoint_index].first, 
                            waypoints[current_waypoint_index].second, 
                            2, 0); // Assuming altitude is 5 and heading is 0

    // Spin
    ros::spin();

    return 0;
}
