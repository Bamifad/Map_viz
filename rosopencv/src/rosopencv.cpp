#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub=it.advertise("camera/image",1);
 
  cv::Mat image = cv::imread("/home/toan/Pictures/images/test1.png", cv::IMREAD_COLOR);
    if(image.empty()){
        printf("open error\n"); }
    sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",image).toImageMsg();
    ros::Rate loop_rate(5);
    while(nh.ok()){
        pub.publish(msg);
        ros::spinOnce;
        loop_rate.sleep();
    }
}