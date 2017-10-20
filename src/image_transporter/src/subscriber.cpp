#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

const std::string WINDOW_NAME = "Subscriber Window";


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow(WINDOW_NAME.c_str(), cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle nh;

  std::string node_name = ros::this_node::getName();
  ROS_INFO("Start node %s", node_name.c_str());

  std::string subscriber_name;
  if (!nh.getParam(node_name + "/subscriber", subscriber_name)) {
    ROS_ERROR("Failed to find parameter %s/%s.", node_name.c_str(), "subscriber");
    return 1;
  }

  cv::namedWindow(WINDOW_NAME.c_str(), CV_WINDOW_NORMAL);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(subscriber_name, 1, imageCallback);
  ros::spin();
  cv::destroyWindow(WINDOW_NAME.c_str());

  ROS_INFO("Exit node %s", node_name.c_str()); 
}
