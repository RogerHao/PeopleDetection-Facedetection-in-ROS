#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "ros/ros.h"    
#include "image_transport/image_transport.h"    
#include "cv_bridge/cv_bridge.h"    
#include "sensor_msgs/image_encodings.h" 

#include "face_detection.h"

namespace enc = sensor_msgs::image_encodings;
using namespace std;

void detectAndDraw(cv::Mat& frame);
void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg);


void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg)  
{ 
  cv_bridge::CvImagePtr cv_ptr;
  try    
  {    
      /*转化成CVImage*/    
       cv_ptr = cv_bridge::toCvCopy(tem_msg, enc::BGR8);    
       cv::imshow("C_IN_WINDOW",cv_ptr->image);  
       cv::waitKey(1);  
  }    

  catch (cv_bridge::Exception& e)    
  {    
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());        
  }

  detectAndDraw(cv_ptr->image);
}  



void detectAndDraw(cv::Mat& img){
  seeta::FaceDetection detector("/home/rover/catkin_ws/src/fdetection/model/seeta_fd_frontal_v1.0.bin");
  detector.SetMinFaceSize(10);
  detector.SetScoreThresh(2.f);
  detector.SetImagePyramidScaleFactor(0.8f);
  detector.SetWindowStep(4, 4);

  cv::Mat img_gray;

  if (img.channels() != 1)
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
  else
    img_gray = img;

  seeta::ImageData img_data;
  img_data.data = img_gray.data;
  img_data.width = img_gray.cols;
  img_data.height = img_gray.rows;
  img_data.num_channels = 1;


  long t0 = cv::getTickCount();
  std::vector<seeta::FaceInfo> faces = detector.Detect(img_data);
  long t1 = cv::getTickCount();
  double secs = (t1 - t0)/cv::getTickFrequency();

  cv::Rect face_rect;
  int32_t num_face = static_cast<int32_t>(faces.size());

  for (int32_t i = 0; i < num_face; i++) {
    face_rect.x = faces[i].bbox.x;
    face_rect.y = faces[i].bbox.y;
    face_rect.width = faces[i].bbox.width;
    face_rect.height = faces[i].bbox.height;

    cv::rectangle(img, face_rect, CV_RGB(0, 0, 255), 4, 8, 0);
  }
  cv::imshow("C_OUT_WINDOW", img);
}


int main(int argc, char** argv)
{
  cv::namedWindow("C_IN_WINDOW", CV_WINDOW_AUTOSIZE);
  cv::namedWindow("C_OUT_WINDOW", CV_WINDOW_AUTOSIZE);
	ros::init(argc, argv, "image_node_c");
	ros::NodeHandle nhc;
	image_transport::ImageTransport it(nhc);
	image_transport::Publisher pub = it.advertise("node_c", 1);
	image_transport::Subscriber sub = it.subscribe("node_b",1,imageCallback);

  ros::Rate loop_rate(20);

  while (nhc.ok()) {  
    ros::spinOnce();  
    loop_rate.sleep();  
  }

	return 0;
}

