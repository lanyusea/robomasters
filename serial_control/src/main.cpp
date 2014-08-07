#include <ros/ros.h>
#include <SerialStream.h>
#include <tld_msgs/BoundingBox.h>
#include <BoundingBox.h>
#include <iostream>
using namespace LibSerial;
using namespace std;

void callback(const tld_msgs::BoundingBox &data) {
    ROS_INFO("I heared the msg");
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"serialNode");
    ros::NodeHandle n;
    tld_msgs::BoundingBox newMsg;

    ros::Subscriber sub = n.subscribe("/tld_tracked_object", 20, &callback);



    return 0;
}
