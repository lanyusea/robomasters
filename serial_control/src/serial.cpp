#include <ros/ros.h>
#include <SerialStream.h>
//#include <tld_msgs/BoundingBox.h>
#include <BoundingBox.h>
#include <iostream>
#include <color.h>
using namespace LibSerial;
using namespace std;

SerialStream serial_port;
const int BUF_LEN=16;  //TODO
//const int MSG_LEN=BUF_LEN/4;
//char msgArray[MSG_LEN];
float angle;
int16_t msg;
int8_t msgEnd;
int8_t msgStart;
//int msgArray[BUF_LEN];



void callback(const tld_msgs::BoundingBox &data) {
    //receive msg test
    //ROS_INFO("I heared the msg");
    //ROS_INFO("=======detect data=======");
    //ROS_INFO("Center Position: (%d,%d)",data.x,data.y);
    //ROS_INFO("BoundingBox Size: %dx%d",data.width,data.height);
    //ROS_INFO("Detect Confidence: %f",data.confidence);


    //calculate the angle

    angle = 35 * ( 2 * (float)data.x / 640 -1);


    //send the angle using serial port

    msg = (int16_t)(angle*100);
    msgStart = msg >> 8;
    msgEnd = msg & 0xFF;

    serial_port<< "AA"
        <<55
        <<msgEnd
        <<msgStart
        <<"BB";


    //call tracking



}

int main(int argc, char** argv)
{

    //serial_port init part
    //credit to Paul Yang and his team
    serial_port.Open("/dev/ttyUSB0");//TODO
    if ( ! serial_port.good() )
    {
        std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
            << "Error: Could not open serial port."
            << std::endl ;
        exit(1) ;
    }

    //
    // Set the baud rate of the serial port.
    //
    serial_port.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the baud rate." <<
            std::endl ;
        exit(1) ;
    }
    //
    // Set the number of data bits.
    //
    serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the character size." <<
            std::endl ;
        exit(1) ;
    }
    //
    // Disable parity.
    //
    serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not disable the parity." <<
            std::endl ;
        exit(1) ;
    }
    //
    // Set the number of stop bits.
    //
    serial_port.SetNumOfStopBits( 1 ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not set the number of stop bits."
            << std::endl ;
        exit(1) ;
    }
    //
    // Turn off hardware flow control.
    //
    serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
    if ( ! serial_port.good() )
    {
        std::cerr << "Error: Could not use hardware flow control."
            << std::endl ;
        exit(1) ;
    }

    //ros init part
    ros::init(argc,argv,"serialNode");
    ros::NodeHandle n;
    tld_msgs::BoundingBox newMsg;

    ros::Subscriber sub = n.subscribe("/tld_tracked_object", 1000, &callback);//queue size TODO

    ros::spin();


    return 0;
}
