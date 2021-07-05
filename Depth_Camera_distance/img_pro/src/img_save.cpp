#include <ros/ros.h>   
#include <image_transport/image_transport.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
#define SHOW_IMAGE

//*******变量*********
cv_bridge::CvImagePtr rgb_ptr;
cv::Mat rgb_pic;

void rgb_Callback(const sensor_msgs::ImageConstPtr& rgb_msg)
{
  try
  {
    #ifdef SHOW_IMAGE
      cv::imshow("rgb_view", cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8)->image);
      cv::waitKey(1);
    #endif
    rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8); 
    rgb_pic = rgb_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono16'.", rgb_msg->encoding.c_str());
  }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_process");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub_rgb = it.subscribe("/camera/rgb/image_raw", 1, rgb_Callback);
  // ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("generated_pc", 1);
  ros::Rate naptime(2); //频率设置
  cout<<"****** ros set ok ******: "<<endl;
  while (ros::ok()) 
  {
    ros::spinOnce(); //allow data update from callback; 
    naptime.sleep(); // wait for remainder of specified period; 
  }
}
