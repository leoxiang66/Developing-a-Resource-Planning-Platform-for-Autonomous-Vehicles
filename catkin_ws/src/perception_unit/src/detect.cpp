#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

cv::Mat my_img, img;
cv::Mat hsv[3], fullImageHSV;   //destination array

const int max_value_H = 360/2;
const int max_value = 255;
const std::string window_capture_name = "Video Capture";
const std::string window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = cv::min(high_H-1, low_H);
    cv::setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = cv::max(high_H, low_H+1);
    cv::setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = cv::min(high_S-1, low_S);
    cv::setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = cv::max(high_S, low_S+1);
    cv::setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = cv::min(high_V-1, low_V);
    cv::setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = cv::max(high_V, low_V+1);
    cv::setTrackbarPos("High V", window_detection_name, high_V);
}

static void onMouse( int event, int x, int y, int, void* )
{
  if( event != cv::EVENT_LBUTTONDOWN )
    return;

  // cv::Vec3b hsv_value=fullImageHSV.at<cv::Vec3b>(x,y);
  // int H=hsv_value.val[0];
  // int S=hsv_value.val[1];
  // int V=hsv_value.val[2];
  // ROS_INFO_STREAM("point h="<<H<<" s="<<S<<" v="<<V);
}

cv::Scalar car[2] = {cv::Scalar(95, 140, 0), cv::Scalar(130, 255, 220)};
cv::Scalar people[2] = {cv::Scalar(0, 220, 0), cv::Scalar(10, 255, 255)}; // (0, 15)
cv::Scalar trees[2] = {cv::Scalar(40, 180, 190), cv::Scalar(50, 205, 200)}; // 
cv::Scalar sinalization[2] = {cv::Scalar(25, 30, 30), cv::Scalar(35, 255, 255)}; // 
cv::Scalar crosswalk[2] = {cv::Scalar(10, 135, 150), cv::Scalar(20, 155, 190)}; // 
cv::Scalar buildings[2] = {cv::Scalar(80, 170, 120), cv::Scalar(100, 190, 150)}; // (0, 60)



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat greyMat, labels, stats, centroids;
    my_img = cv_bridge::toCvShare(msg, "bgr8")->image;
    // cv::cvtColor(my_img, greyMat, CV_BGR2GRAY);
    cv::cvtColor(my_img, fullImageHSV, CV_BGR2HSV);
    cv::split(fullImageHSV, hsv); greyMat = hsv[0];

    // cv::threshold(greyMat, greyMat, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);

    cv::Mat elem1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
    cv::Mat elem2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(30, 30));
    // cv::morphologyEx(greyMat, greyMat, cv::MORPH_CLOSE, elem1);
    cv::inRange(fullImageHSV, car[0], car[1], greyMat);
    cv::erode(greyMat, greyMat, elem1);
    cv::dilate(greyMat, greyMat, elem2);
    // cv::imshow("view", greyMat);
    int nlabl = cv::connectedComponentsWithStats(greyMat, labels, stats, centroids, 8, CV_32S);
    ROS_INFO_STREAM("Total of labels: " << nlabl);
    // std::vector<int> v;
    // v.reserve(nlabl);
    // for (int i = 0; i < nlabl; ++i) v.push_back(i);
    // std::cout << centroids.at<double>(30, 0) << std::endl;
    // std::cout << centroids.at<double>(30, 1) << std::endl;
    for (int n = 1; n < nlabl; ++n)
    {
      // ROS_INFO_STREAM("labels are " << labels.at<short>(i));
      // ROS_INFO_STREAM("centroids para label="<<i<<" with (" << centroids.at<double>(n, 0) << ", " << centroids.at<double>(n, 1) << ")");
   
      int xmin=1000000, xmax=0, ymin=1000000, ymax=0, nmax = 0;
      for(int i = 0; i < labels.rows; i++)
      {
        for(int j = 0; j < labels.cols; j++)
        {
          if(labels.at<int>(i,j) == n)
          {
            if(i < xmin)  xmin = i;
            if(i > xmax)  xmax = i;
            if(j < ymin)  ymin = j;
            if(j > ymax)  ymax = j;
          }
        }
      }

      std::vector<int> center;
      center.push_back((xmin+xmax)/2);
      center.push_back((ymin+ymax)/2);
      // ROS_INFO_STREAM("centroids para label="<<n<<" with (" << (xmin+xmax)/2 << ", " << (ymin+ymax)/2 << ")");
      cv::Rect r = cv::Rect(ymin, xmin, (ymax-ymin), (xmax-xmin));
      cv::rectangle(my_img, r, cv::Scalar(255,0,255), 3, 8, 0);
    }
    // int xmin=1000000, xmax=0, ymin=1000000, ymax=0, nmax = 0;
    // srand (time(NULL));
    // int chosenLabel = rand() % nlabl;
    // ROS_INFO_STREAM("Chosen label: " << chosenLabel);
    // for(int i = 0; i < labels.rows; ++i)
    // {
    //   for(int j = 0; j < labels.cols; ++j)
    //   {
    //     if(labels.at<int>(i,j) == chosenLabel)
    //     {
    //       // std::cout << labels.at<int>(i,j) << " i=" << i << " j=" << j << std::endl;
    //       if(i < xmin)  xmin = i;
    //       if(i > xmax)  xmax = i;
    //       if(j < ymin)  ymin = j;
    //       if(j > ymax)  ymax = j;
    //     }
    //     // if(labels.at<int>(i,j) > nmax)
    //     // {
    //     //   nmax = labels.at<int>(i,j);
    //     //   ROS_INFO_STREAM("found n = " << nmax);
    //     // }
    //   }
    // }
    // std::vector<int> center;
    // center.push_back((xmin+xmax)/2);
    // center.push_back((ymin+ymax)/2);
    // ROS_INFO_STREAM("("<<center[0]<<", "<<center[1]<<")");
    // int width  = xmax-xmin;
    // int height = ymax-ymin;
    // ROS_INFO_STREAM("xmin="<<xmin<<" xmax="<<xmax<<" ymin="<<ymin<<" ymax="<<ymax);

    // cv::Rect r = cv::Rect(ymin, xmin, (ymax-ymin), (xmax-xmin));
    // greyMat.convertTo(labels, CV_8U);
    // equalizeHist( greyMat, my_img );
    // cv::cvtColor(my_img, my_img, CV_GRAY2BGR);
    // my_img = cv_bridge::toCvShare(msg, "bgr8")->image;
    // cv::rectangle(my_img, r, cv::Scalar(255,0,255), 3, 8, 0);
    // cv::imshow("view", labels);
    cv::imshow("view", my_img);
    // cv::Mat frame_threshold;
    // cv::inRange(fullImageHSV, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), frame_threshold);
    // Show the frames
    // imshow(window_capture_name, frame);
    // cv::imshow(window_detection_name, frame_threshold);
    // cv::imwrite("test.png", labels);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  std::string node_name("image_listener");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.setParam("image_transport", "compressed");
  cv::namedWindow("view", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
  // cv::namedWindow(window_detection_name, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
  cv::resizeWindow(window_detection_name, 1080, 720);
  cv::resizeWindow("view", 1080, 720);
  cv::setMouseCallback( "view", onMouse, 0 );

  cv::createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
  cv::createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
  cv::createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
  cv::createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
  cv::createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
  cv::createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("simulator/segmentation_camera", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}