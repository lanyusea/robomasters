#include <ros/ros.h>
#include <SerialStream.h>
#include <tld_msgs/BoundingBox.h>
#include <BoundingBox.h>
#include <iostream>
using namespace LibSerial;
using namespace std;

void callback(const tld_msgs::BoundingBox &data) {
    //receive msg
    //ROS_INFO("I heared the msg");
    //ROS_INFO("=======detect data=======");
    //ROS_INFO("Center Position: (%d,%d)",data.x,data.y);
    //ROS_INFO("BoundingBox Size: %dx%d",data.width,data.height);
    //ROS_INFO("Detect Confidence: %f",data.confidence);


    //calculate the angle





    //send the angle using serial port

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"serialNode");
    ros::NodeHandle n;
    tld_msgs::BoundingBox newMsg;

    ros::Subscriber sub = n.subscribe("/tld_tracked_object", 20, &callback);

    ros::spin();


    return 0;
}
