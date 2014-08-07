#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
    int channel, rate, imageId;
    std::string topicName;
    char intStr[2];

    ros::init(argc, argv, "image_publisher");
    // when starting this node, add "__name:=" to give it a different name, to avoid name conflict

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param("channel", channel, int(1)); //choose use which device, /dev/videoX
    pnh.param("imageId", imageId, int(0)); //choose the name of published image /gnd_cam/imageX
    pnh.param("rate", rate, int(20));      //image read rate (fps), its limitation is the device max fps

    snprintf(intStr, 2, "%d", imageId);
    std::string str = std::string(intStr);
    topicName = "gnd_cam/image"+str;

    ROS_INFO("The topic name published by this node is: %s", topicName.c_str());

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise(topicName, 5);

    cv_bridge::CvImagePtr cvPtr;
    cv_bridge::CvImage outMsg;
    cv::VideoCapture cap(channel);
    cv::Mat colorImg, grayImg;
    ros::Rate loopRate(rate);

    while (nh.ok()) {
        cap >> colorImg;
        cv::cvtColor(colorImg, grayImg, CV_RGB2GRAY);
        ROS_INFO("image height: %d, image width: %d\n", grayImg.rows, grayImg.cols);
        //outMsg.image = grayImg;
        outMsg.image = colorImg;
        outMsg.header.stamp = ros::Time::now();
        outMsg.encoding = "bgr8";
        /*
            possible encodings, use "rgb8" if publish color image directly
            DO NOT use captain
            using sensor_msgs::image_encodings::RGB8;
            using sensor_msgs::image_encodings::RGBA8;
            using sensor_msgs::image_encodings::BGR8;
            using sensor_msgs::image_encodings::BGRA8;
            using sensor_msgs::image_encodings::MONO8;
            using sensor_msgs::image_encodings::MONO16;
         */

        pub.publish(outMsg.toImageMsg());
        ros::spinOnce();
        loopRate.sleep();
    }
}
