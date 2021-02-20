#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <visualization_msgs/Marker.h>
#include <perception_unit/DetectedObject.h>
#include <perception_unit/DetectedObjectArray.h>

cv::Mat my_img, img;
cv::Mat fullImageHSV;   // destination array

// Struct used to save each labeled object spec
typedef struct labelInfo
{
  double x, y, z;
  int height, width;
  int xmin, ymin;
}LabelInfo;

const std::string window_detection_name = "Object Detection";

double retXFromCamera(double z, double u, double fx, double cx)
{ // calculate the real world x value given the pixel's depth
  double x;
  x = z*(u - cx)/fx;
  return x;
}
double retYFromCamera(double z, double v, double fy, double cy)
{ // calculate the real world y value given the pixel's depth
  double y;
  y = z*(v - cy)/fy;
  return y;
}

cv::Mat depthImg, bgr[3];
static void onMouse( int event, int x, int y, int, void* )
{
  if( event != cv::EVENT_LBUTTONDOWN )
    return;

  cv::Vec3b hsv_value=depthImg.at<cv::Vec3b>(x,y);
  int H=hsv_value.val[0];
  int S=hsv_value.val[1];
  int V=hsv_value.val[2];
  ROS_INFO_STREAM("point h="<<H<<" s="<<S<<" v="<<V);
}

// HSV values used to mask each objects, e.g., as the cars appears as blueish pixels 
cv::Scalar car[2] = {cv::Scalar(95, 140, 0), cv::Scalar(130, 255, 220)};
cv::Scalar people[2] = {cv::Scalar(0, 220, 0), cv::Scalar(10, 255, 255)}; // (0, 15)
cv::Scalar trees[2] = {cv::Scalar(40, 180, 190), cv::Scalar(50, 205, 200)}; // 
cv::Scalar sinalization[2] = {cv::Scalar(25, 30, 30), cv::Scalar(35, 255, 255)}; // 
cv::Scalar crosswalk[2] = {cv::Scalar(10, 135, 150), cv::Scalar(20, 155, 190)}; // 
cv::Scalar buildings[2] = {cv::Scalar(80, 170, 120), cv::Scalar(100, 190, 150)}; // (0, 60)

// Structuring elements used to do morphology operations
cv::Mat el_10_RECT = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
cv::Mat el_15_RECT = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
cv::Mat el_30_ELPS = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(30, 30));
cv::Mat el_60_ELPS = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(60, 60));

// Mask src image using given HSV range values to capture only the object
// Then erode and dilate the pixel to remove oulier pixels and grow the bounding box
void preMorpho(cv::Mat& src, cv::Mat& dst, cv::Scalar range[2], bool erode, bool dilate, cv::Mat el[2])
{
  cv::inRange(src, range[0], range[1], dst);
  if (erode)  cv::erode(dst, dst, el[0]);
  if (dilate) cv::dilate(dst, dst, el[1]);
}

double UPPER_BOUND = 100;
double LOWER_BOUND = .1;

namespace enc = sensor_msgs::image_encodings;

// Inverse function for normalization used by LGSVL Simulator in Depth Image
double inverNormalization(int x)
{
  double y = ((x-255)*(UPPER_BOUND-LOWER_BOUND)/(-255.)) + LOWER_BOUND;
}

// double inverNormalization(int x)
// {
//   double y = ((x-255)*(UPPER_BOUND-LOWER_BOUND)/(-255.)) + LOWER_BOUND;
// }

cv::Mat h1, h2;
void countObj(cv::Mat& src, cv::Mat& labelMat, std::vector<LabelInfo>& vecLabels)
{
  // Use OpenCV's connectedComponents to count and label each objects
  // Then parse the img getting the height and width of the img in pixels
  int connection = 8;
  int nlabl = cv::connectedComponentsWithStats(src, labelMat, h1, h2, connection, CV_32S);
  // ROS_INFO_STREAM("Total of labels: " << nlabl);
  vecLabels.reserve(nlabl);
  for (int n = 1; n < nlabl; n++)
  {
    int xmin=100000, xmax=0, ymin=100000, ymax=0;
    for(int i = 0; i < labelMat.rows; i++)
    {
      for(int j = 0; j < labelMat.cols; j++)
      {
        if(labelMat.at<int>(i,j) == n)
        {
          if(i < xmin)  xmin = i;
          if(i > xmax)  xmax = i;
          if(j < ymin)  ymin = j;
          if(j > ymax)  ymax = j;
        }
      }
    }
    LabelInfo li;
    li.x = (xmax+xmin)/2;
    li.y = (ymax+ymin)/2;
    li.xmin = xmin;
    li.ymin = ymin;
    li.width = (ymax-ymin);
    li.height = (xmax-xmin);
    // ROS_INFO_STREAM(li.xmin);
    // ROS_INFO_STREAM(li.ymin);
    // ROS_INFO_STREAM(li.width);
    // ROS_INFO_STREAM(li.height);
    try {
      // ROS_INFO_STREAM(int(depthImg.at<uchar>((xmax+xmin)/2, (ymax+ymin)/2)));
      li.z = inverNormalization(int(depthImg.at<uchar>((xmax+xmin)/2, (ymax+ymin)/2)));
      // ROS_INFO_STREAM(li.z);
    }
    catch(...) {
      ROS_ERROR("Couldn't get the z inversion");
    }
    vecLabels.push_back(li);
  }
}

ros::Publisher marker_pub;
uint32_t shape = visualization_msgs::Marker::CUBE;
void publishMarker(double x, double y, double z, int id, double height, double width)
{
  // Publish a ROS Marker to know where is each detected object 
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/camera";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  if(width < 0)
    marker.scale.z = -width/2;
  else
    marker.scale.z = width/2;
  if(height < 0)
    marker.scale.y = -height/2;
  else
    marker.scale.y = height/2;
  marker.scale.x = 3;

  marker.color.r = x;
  marker.color.g = x;
  marker.color.b = x;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
}

void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Get depth img to try to calculate the real world position and dimmension
  try
  {
    depthImg = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::split(depthImg, bgr);
    depthImg = bgr[0];
    // ROS_INFO_STREAM(depthImg.type() == CV_8UC3);
    // depthImg.convertTo( depthImg, CV_64F );
    // ROS_INFO_STREAM("Depth pixel in (720, 30) is bgr " << (int(depthImg.at<uchar>(720, 30))) <<" m");
    // ROS_INFO_STREAM("Depth pixel in (720, 30) is bgr " << inverNormalization(int(depthImg.at<uchar>(720, 30))) <<" m");

    double z = inverNormalization(int(depthImg.at<uchar>(720, 30)));
    double x = retXFromCamera(z, (depthImg.rows-720.)/depthImg.rows, 1, 0);
    double y = retYFromCamera(z, (depthImg.cols-30.)/depthImg.cols, 1, 0);
    // ROS_INFO_STREAM("X pixel in (720, 30) is bgr " << x <<" m");
    // ROS_INFO_STREAM("Y pixel in (720, 30) is bgr " << y <<" m");
    // publishMarker(x,y,z);

    // cv::Rect r = cv::Rect(30, 720, 10, 10);
    // cv::rectangle(depthImg, r, cv::Scalar(255,0,255), 3, 8, 0);
    // cv::imshow(window_detection_name, depthImg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Get the Segmented img and trasform from RGB space to HSV space
  try
  {
    my_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::cvtColor(my_img, fullImageHSV, CV_BGR2HSV);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

bool REAL_DIM = false;
ros::Publisher detected_obj_pub;
void publishAllObjt(std::vector<LabelInfo> cars, std::vector<LabelInfo> peoples, std::vector<LabelInfo> treeses,
                    std::vector<LabelInfo> sinals, std::vector<LabelInfo> crosses, std::vector<LabelInfo> builds)
{
  // Insert all the objects in a vector and publish it
  perception_unit::DetectedObjectArray allObjt;
  perception_unit::DetectedObject aux;

  for (int i = 0; i < cars.size(); ++i)
  {
    aux.type = std::string("Car");
    aux.pose.position.z = cars[i].z;
    // If the object is close to the camera then try to calculate the real world dimmesions
    if ((aux.pose.position.z < UPPER_BOUND) && (REAL_DIM))
    {
      aux.pose.position.x = retXFromCamera(cars[i].z, (float)cars[i].x, 1158.03, depthImg.rows/2.);
      aux.pose.position.y = retYFromCamera(cars[i].z, (float)cars[i].y, 1158.03, depthImg.cols/2.);
      aux.height = retXFromCamera(cars[i].z, (float)cars[i].height, 1158.03, depthImg.rows/2.);
      aux.width = retYFromCamera(cars[i].z, (float)cars[i].width, 1158.03, depthImg.cols/2.);
      // ROS_INFO_STREAM(cars[i].x);
      // ROS_INFO_STREAM(cars[i].y);
      // ROS_INFO_STREAM(aux);
    }
    else
    {
      aux.pose.position.x = cars[i].x;
      aux.pose.position.y = cars[i].y;
      aux.height = cars[i].height;
      aux.width = cars[i].width;
    }
    aux.pose.orientation.w = 1;
    // cv::Rect r = cv::Rect(cars[i].ymin, cars[i].xmin, cars[i].width, cars[i].height);
    // cv::rectangle(my_img, r, cv::Scalar(255,0,255), 3, 8, 0);
    // publishMarker(aux.pose.position.x, aux.pose.position.y, 15., i, aux.height, aux.width);
    allObjt.objects.push_back(aux);
  }

  for (int i = 0; i < peoples.size(); ++i)
  {
    aux.type = std::string("Person");
    aux.pose.position.z = peoples[i].z;
    if ((aux.pose.position.z < UPPER_BOUND) && (REAL_DIM))
    {
      aux.pose.position.x = retXFromCamera(peoples[i].z, (depthImg.rows-peoples[i].x)/depthImg.rows, 1, 0);
      aux.pose.position.y = retYFromCamera(peoples[i].z, (depthImg.cols-peoples[i].y)/depthImg.cols, 1, 0);
      aux.height = retXFromCamera(peoples[i].z, (depthImg.rows-peoples[i].height)/depthImg.rows, 1, 0);
      aux.width = retYFromCamera(peoples[i].z, (depthImg.cols-peoples[i].width)/depthImg.cols, 1, 0);
    }
    else
    {
      aux.pose.position.x = peoples[i].x;
      aux.pose.position.y = peoples[i].y;
      aux.height = peoples[i].height;
      aux.width = peoples[i].width;
    }
    aux.pose.orientation.w = 1;
    allObjt.objects.push_back(aux);
  }

  for (int i = 0; i < treeses.size(); ++i)
  {
    aux.type = std::string("Tree");
    aux.pose.position.z = treeses[i].z;
    if ((aux.pose.position.z < UPPER_BOUND) && (REAL_DIM))
    {
      aux.pose.position.x = retXFromCamera(treeses[i].z, (depthImg.rows-treeses[i].x)/depthImg.rows, 1, 0);
      aux.pose.position.y = retYFromCamera(treeses[i].z, (depthImg.cols-treeses[i].y)/depthImg.cols, 1, 0);
      aux.height = retXFromCamera(treeses[i].z, (depthImg.rows-treeses[i].height)/depthImg.rows, 1, 0);
      aux.width = retYFromCamera(treeses[i].z, (depthImg.cols-treeses[i].width)/depthImg.cols, 1, 0);
    }
    else
    {
      aux.pose.position.x = treeses[i].x;
      aux.pose.position.y = treeses[i].y;
      aux.height = treeses[i].height;
      aux.width = treeses[i].width;
    }
    aux.pose.orientation.w = 1;
    allObjt.objects.push_back(aux);
  }

  for (int i = 0; i < sinals.size(); ++i)
  {
    aux.type = std::string("Sinalization");
    aux.pose.position.z = sinals[i].z;
    if ((aux.pose.position.z < UPPER_BOUND) && (REAL_DIM))
    {
      aux.pose.position.x = retXFromCamera(sinals[i].z, (depthImg.rows-sinals[i].x)/depthImg.rows, 1, 0);
      aux.pose.position.y = retYFromCamera(sinals[i].z, (depthImg.cols-sinals[i].y)/depthImg.cols, 1, 0);
      aux.height = retXFromCamera(sinals[i].z, (depthImg.rows-sinals[i].height)/depthImg.rows, 1, 0);
      aux.width = retYFromCamera(sinals[i].z, (depthImg.cols-sinals[i].width)/depthImg.cols, 1, 0);
    }
    else
    {
      aux.pose.position.x = sinals[i].x;
      aux.pose.position.y = sinals[i].y;
      aux.height = sinals[i].height;
      aux.width = sinals[i].width;
    }
    aux.pose.orientation.w = 1;
    allObjt.objects.push_back(aux);
  }

  for (int i = 0; i < crosses.size(); ++i)
  {
    aux.type = std::string("Crosswalk");
    aux.pose.position.z = crosses[i].z;
    if ((aux.pose.position.z < UPPER_BOUND) && (REAL_DIM))
    {
      aux.pose.position.x = retXFromCamera(crosses[i].z, (depthImg.rows-crosses[i].x)/depthImg.rows, 1, 0);
      aux.pose.position.y = retYFromCamera(crosses[i].z, (depthImg.cols-crosses[i].y)/depthImg.cols, 1, 0);
      aux.height = retXFromCamera(crosses[i].z, (depthImg.rows-crosses[i].height)/depthImg.rows, 1, 0);
      aux.width = retYFromCamera(crosses[i].z, (depthImg.cols-crosses[i].width)/depthImg.cols, 1, 0);
    }
    else
    {
      aux.pose.position.x = crosses[i].x;
      aux.pose.position.y = crosses[i].y;
      aux.height = crosses[i].height;
      aux.width = crosses[i].width;
    }
    aux.pose.orientation.w = 1;
    allObjt.objects.push_back(aux);
    // cv::Rect r = cv::Rect(crosses[i].y, crosses[i].x, crosses[i].width, crosses[i].height);
    // cv::rectangle(my_img, r, cv::Scalar(255,0,255), 3, 8, 0);
  }

  for (int i = 0; i < builds.size(); ++i)
  {
    aux.type = std::string("Buildings");
    aux.pose.position.z = builds[i].z;
    if ((aux.pose.position.z < UPPER_BOUND) && (REAL_DIM))
    {
      aux.pose.position.x = retXFromCamera(builds[i].z, (depthImg.rows-builds[i].x)/depthImg.rows, 1, 0);
      aux.pose.position.y = retYFromCamera(builds[i].z, (depthImg.cols-builds[i].y)/depthImg.cols, 1, 0);
      aux.height = retXFromCamera(builds[i].z, (depthImg.rows-builds[i].height)/depthImg.rows, 1, 0);
      aux.width = retYFromCamera(builds[i].z, (depthImg.cols-builds[i].width)/depthImg.cols, 1, 0);
    }
    else
    {
      aux.pose.position.x = builds[i].x;
      aux.pose.position.y = builds[i].y;
      aux.height = builds[i].height;
      aux.width = builds[i].width;
    }
    aux.pose.orientation.w = 1;
    allObjt.objects.push_back(aux);
    // cv::Rect r = cv::Rect(builds[i].y, builds[i].x, builds[i].width, builds[i].height);
    // cv::rectangle(my_img, r, cv::Scalar(255,255,0), 3, 8, 0);
  }
  allObjt.header.stamp = ros::Time::now();
  detected_obj_pub.publish(allObjt);
}

int main(int argc, char **argv)
{
  std::string node_name("obj_detection");
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.setParam("image_transport", "compressed");

  // cv::namedWindow("view", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
  // cv::resizeWindow("view", 1080, 720);

  // cv::namedWindow(window_detection_name, CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
  // cv::resizeWindow(window_detection_name, 1080, 720);
  // cv::setMouseCallback(window_detection_name, onMouse, 0 );

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  detected_obj_pub = pnh.advertise<perception_unit::DetectedObjectArray>("detected_objects", 1);
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("simulator/segmentation_camera", 1, imageCallback);
  image_transport::Subscriber dsub = it.subscribe("simulator/depth_camera", 1, imageDepthCallback);
  
  cv::Mat labels;
  ros::Rate loop_rate(1);
  
  std::vector<LabelInfo> carInfo, peopleInfo, treesInfo;
  std::vector<LabelInfo> sinalInfo, crossInfo, buildInfo;
  cv::Mat carsMat, peopleMat,treesMat;
  cv::Mat sinalMat, crossMat, buildMat;
  cv::Mat els[2];
  while(ros::ok())
  {
    if(my_img.cols > 0)
    {
      // For each type of object use the segmented img to detect and extract its specs

      // cars
      cv::Mat els[2] = {el_15_RECT, el_30_ELPS};
      els[0] = el_15_RECT;
      els[1] = el_60_ELPS;
      preMorpho(fullImageHSV, carsMat, car, true, true, els);
      countObj(carsMat, labels, carInfo);

      // people
      els[1] = el_15_RECT;
      preMorpho(fullImageHSV, peopleMat, people, false, true, els);
      countObj(peopleMat, labels, peopleInfo);

      // trees
      els[0] = el_30_ELPS;
      els[1] = el_30_ELPS;
      preMorpho(fullImageHSV, treesMat, trees, false, true, els);
      countObj(treesMat, labels, treesInfo);

      // sinalization
      els[0] = el_10_RECT;
      els[1] = el_30_ELPS;
      preMorpho(fullImageHSV, sinalMat, sinalization, true, true, els);
      countObj(sinalMat, labels, sinalInfo);

      // crosswalk
      els[0] = el_15_RECT;
      els[1] = el_30_ELPS;
      preMorpho(fullImageHSV, crossMat, crosswalk, true, true, els);
      countObj(crossMat, labels, crossInfo);

      // buildings
      els[0] = el_15_RECT;
      els[1] = el_60_ELPS;
      preMorpho(fullImageHSV, buildMat, buildings, true, true, els);
      countObj(buildMat, labels, buildInfo);

      // Log the detection
      ROS_INFO_STREAM("Detection: ");
      ROS_INFO_STREAM("Cars: " << carInfo.size());
      ROS_INFO_STREAM("People: " << peopleInfo.size());
      ROS_INFO_STREAM("Trees: " << treesInfo.size());
      ROS_INFO_STREAM("Sinaliz: " << sinalInfo.size());
      ROS_INFO_STREAM("Crosswalk: " << crossInfo.size());
      ROS_INFO_STREAM("Buildings: " << buildInfo.size());
      publishAllObjt(carInfo, peopleInfo, treesInfo, sinalInfo, crossInfo, buildInfo);
      
      // cv::imshow("view", my_img);
      // cv::imshow(window_detection_name, peopleMat);
      // cv::waitKey(30);

      // Clean old data
      carInfo.clear();
      peopleInfo.clear();
      treesInfo.clear();
      sinalInfo.clear();
      crossInfo.clear();
      buildInfo.clear();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}