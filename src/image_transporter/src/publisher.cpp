#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer


const std::string WINDOW_NAME = "Publisher window";


int main(int argc, char** argv)
{
  // ROS Transporter
  ros::init(argc, argv, "cpp_pub");
  ros::NodeHandle nh;

  std::string node_name = ros::this_node::getName();
  ROS_INFO("Start node %s", node_name.c_str());

  std::string publisher_name;
  if (!nh.getParam(node_name + "/publisher", publisher_name)) {
    ROS_ERROR("Failed to find parameter %s/%s.", node_name.c_str(), "publisher");
    return 1;
  }

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(publisher_name, 1);

  // OpenCV window
  cv::namedWindow(WINDOW_NAME, CV_WINDOW_NORMAL);
  
  sensor_msgs::ImagePtr msg;

  int row = 0;
  int col = 0;
  ros::Rate loop_rate(20);
  while (nh.ok()) {
    cv::Mat frame(cv::Size(256, 256), CV_8UC3, cv::Scalar::all(0));

    // 適当な画像を生成
    for (size_t y = 0; y < 256; y++) {
      for (size_t x = 0; x < 256; x++) { 
        frame.at<cv::Vec3b>(y, x)[0] = ((x + col) * 2 + ((y + row) * 7)) % 256;
        frame.at<cv::Vec3b>(y, x)[1] = ((x + col) * 4 + ((y + row) * 5)) % 256;
        frame.at<cv::Vec3b>(y, x)[2] = ((x + col) * 8 + ((y + row) * 3)) % 256;
      }
    }  
    row++; col++;

    if (row > 255) { row = 0; }
    if (col > 255) { col = 0; }

    cv::imshow(WINDOW_NAME, frame);
    int key = cv::waitKey(1) % 256;

    // 's' : send image
    if (key == 115) {
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Exit node %s", node_name.c_str());
}

