#include "Fetch_Controller.hpp"

Fetch_Controller::Fetch_Controller(ros::NodeHandle &nh)
{
    nh_ = nh;

    //TODO: initialize a subscriber that is set to the channel "/base_scan". Set its callback function to be Laser_Scan_Callback
    //TODO: initialize a publisher that is set to the channel "/cmd_vel"

    subscriber_ = nh_.subscribe("/base_scan", 1000, &Fetch_Controller::Laser_Scan_Callback,this);
    publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
}

void Fetch_Controller::Laser_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr &msg_laser_scan)
{
    /*TODO: 
    Given the incoming laser scan message, find the minimium distance of the front facing scans
    Hint: The laser scan measuring directly in front of the robot will be the scan at the middle of the array laser scans. 
    So for finding the minimum, we will ONLY consider the 120 laser scans in the middle of the array of laser scans. 
    If the minimum scan in this direction is greater than 1m, drive forward. 
    Otherwise, turn left. 
    */

   // Left most index
   int index = 271;

   geometry_msgs::Twist direction;

    // Checks the middle of the array
    for (int i = index; i <= index + 120; i++) {
      // If the current index is less than 1 m, it will start to turn left
      if (msg_laser_scan->ranges[i]<1) {
        direction.angular.z = 1.0;
      }
      // Otherwise it will go straight
      else {
        direction.linear.x = 0.5;
      }
    }

    publisher_.publish(direction);

}
