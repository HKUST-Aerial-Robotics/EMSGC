#ifndef EMSGC_TOOLS_UTILS_H
#define EMSGC_TOOLS_UTILS_H

#include <Eigen/Eigen>
#include <iostream>
#include <deque>
#include <ros/ros.h>
#include <dvs_msgs/Event.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <emsgc/container/PerspectiveCamera.h>

using namespace std;
namespace emsgc
{
using namespace container;
namespace tools
{
#define PI 3.1415926;
using EventQueue = std::deque<dvs_msgs::Event>;
using EventVector = std::vector<dvs_msgs::Event>;
using vEigenVector4i = std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i> >;

inline static EventQueue::iterator EventQueue_lower_bound(
  EventQueue& eb, ros::Time& t)
{
  return std::lower_bound(eb.begin(), eb.end(), t,
      [](const dvs_msgs::Event & e, const ros::Time & t) {return e.ts.toSec() < t.toSec();});
}

using TimeStampMaskAssocMap = std::map<double, std::string>;

struct MotionModel
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MotionModel():m_x_(0), m_y_(0), m_s_(0), m_theta_(0){}
  MotionModel(double m_x,double m_y,double m_s,double m_theta):m_x_(m_x), m_y_(m_y), m_s_(m_s), m_theta_(m_theta){}
  double m_x_, m_y_, m_s_, m_theta_;
};

enum MotionModelType
{
  TranslationModel, // 2 DoF
  RotationModel, // 3 DoF
  AffineModel// 4 DoF
};

inline string MotionModeType_to_String(MotionModelType type)
{
  switch(type) {
    case TranslationModel:
      return "TranslationModel";
    case RotationModel:
      return "RotationModel";
    case AffineModel:
      return "AffineModel";
    default:
      return "Invalid type";
  }
}

struct GeneralMotionModel
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GeneralMotionModel(size_t num_dof, double* data):num_dof_(num_dof)
  {
    parameters_ = std::vector<double>(data, data + num_dof);
    if(num_dof == 2)
      mmType_ = TranslationModel;
    else if(num_dof == 3)
      mmType_ = RotationModel;
    else if(num_dof == 4)
      mmType_ = AffineModel;
    else
    {
      std::cout << "The given num_dof is not defined!!!!!!!!!!";
      exit(-1);
    }
  }
  GeneralMotionModel()
  {
    mmType_ = TranslationModel;
    num_dof_ = 2;
    parameters_.push_back(0);
    parameters_.push_back(0);
  }
  void reset(size_t num_dof, double* data)
  {
    num_dof_ = num_dof;
    parameters_ = std::vector<double>(data, data + num_dof);
    if(num_dof == 2)
      mmType_ = TranslationModel;
    else if(num_dof == 3)
      mmType_ = RotationModel;
    else if(num_dof == 4)
      mmType_ = AffineModel;
    else
    {
      std::cout << "The given num_dof is not defined!!!!!!!!!!";
      exit(-1);
    }
  }

  MotionModelType mmType_;
  size_t num_dof_;
  std::vector<double> parameters_;
};

enum {
  VARIANCE_CONTRAST, // 0
  MEAN_SQUARE_CONTRAST
};

// Structure collecting the options for warping the events onto
// a histogram or image: the "image of warped events" (IWE)
struct OptionsWarp
{
  // Whether to use polarity or not in the IWE
  bool use_polarity_ = true;

  // Amounf ot Gaussian blur (in pixels) to make the IWE smoother,
  // and consequently, optimize a smoother objective function
  double blur_sigma_ = 1.0;
};

// Options of the method
struct OptionsMethod
{
  // Sliding Window options
  // Number of events used to synthetize an image of warped events
  int num_events_per_image_ = 15000;
  // Amount of overlap between consecutive packets of events: a number >= 0. (no overlap) and <1.0 (full)
  int num_events_slide_ = 15000;

  // Objective function to be optimized: 0=Variance, 1=RMS, etc.
  int contrast_measure_ = VARIANCE_CONTRAST;

  // Options of the image of warped events
  OptionsWarp opts_warp_;

  // Verbosity / printing level
  unsigned int verbose_ = 0;
};

inline double bilinearIntepolation(
  const Eigen::MatrixXd & img, const Eigen::Vector2d & location, float defaultValue )
{
  Eigen::Vector2f float_indices;
  float_indices << ((float) location[1])-0.5f, ((float) location[0])-0.5f;

  std::pair<int,int> lower_indices( floor(float_indices[0]), floor(float_indices[1]) );
  std::pair<int,int> upper_indices( lower_indices.first + 1, lower_indices.second + 1 );

  if( lower_indices.first < 0 || lower_indices.second < 0 )
    return defaultValue;
  if( upper_indices.first >= img.rows() || upper_indices.second >= img.cols() )
    return defaultValue;

  float w1 = ((float) upper_indices.second) - float_indices[1];
  float w2 = float_indices[1] - ((float) lower_indices.second);

  float lower_y = w1 * img(lower_indices.first,lower_indices.second)
                  + w2 * img(lower_indices.first,upper_indices.second);
  float upper_y = w1 * img(upper_indices.first,lower_indices.second)
                  + w2 * img(upper_indices.first,upper_indices.second);
  float value = ( ((float) upper_indices.first) - float_indices[0] ) * lower_y
                + ( float_indices[0] - ((float) lower_indices.first) ) * upper_y;

  return value;
}

inline int countWords(string& str)
{
  // breaking input into word using string stream
  stringstream s(str); // Used for breaking words
  string word; // to store individual words

  int count = 0;
  while (s >> word)
    count++;
  return count;
}

inline void createDenoisingMask(
  EventQueue::iterator it_begin,
  EventQueue::iterator it_end,
  cv::Mat& mask,
  size_t row,
  size_t col,
  bool bBlur = false)
{
  cv::Mat eventMap;
  eventMap = cv::Mat(cv::Size(col, row), CV_8UC1, cv::Scalar(0));
  while(it_begin!=it_end)
  {
    eventMap.at<uchar>(it_begin->y, it_begin->x) = 255;
    it_begin++;
  }
  if(bBlur)
    cv::medianBlur(eventMap, mask, 3);//TODO: tue ksize
  else
    mask = eventMap;
}

inline void extractDenoisedEvents(
  EventQueue::iterator it_begin,
  EventQueue::iterator it_end,
  std::vector<dvs_msgs::Event> &vEdgeEvents,
  cv::Mat& mask)
{
  size_t numAllEvents = std::distance(it_begin, it_end) + 1;
  vEdgeEvents.reserve(numAllEvents);
  while(it_begin != it_end)
  {
    if(mask.at<uchar>(it_begin->y, it_begin->x) == 255)
      vEdgeEvents.push_back(*it_begin);
    it_begin++;
  }
}

inline void drawEventImage(
  std::vector<dvs_msgs::Event>& vEventsPtr,
  cv::Mat& eventMap,
  size_t width,
  size_t height)
{
  eventMap = cv::Mat(cv::Size(width, height), CV_8UC1, cv::Scalar(0));
  for(size_t i = 0; i < vEventsPtr.size(); i++)
    eventMap.at<uchar>(vEventsPtr[i].y, vEventsPtr[i].x) = 255;
}

}
}

#endif //EMS_TOOLS_UTILS_H
