#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg)
{
  try
  {
    cv::imshow("A_IN_WINDOW", cv_bridge::toCvShare(tem_msg, "bgr8")->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());
  }
}



int main(int argc, char** argv)
{
  //cv::namedWindow("A_IN_WINDOW", CV_WINDOW_AUTOSIZE);
  ros::init(argc, argv, "image_node_a");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("node_a", 1);
  //image_transport::Subscriber sub = it.subscribe("node_b",1,imageCallback);

  cv::VideoCapture cap("/home/rover/1.avi");
  if(!cap.isOpened())   
  {  
      ROS_INFO("can not opencv video device\n");  
      return 1;  
  }  
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(20);

  while (nh.ok()) {  
    cap >> frame;
    // Check if grabbed frame is actually full with some content  
    if(!frame.empty()) {  
      //cv::resize(frame, frame, cv::Size(), 0.2, 0.2);
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
      pub.publish(msg);   
    }  
  
    ros::spinOnce();  
    loop_rate.sleep();  
  }    

  return 0;
}