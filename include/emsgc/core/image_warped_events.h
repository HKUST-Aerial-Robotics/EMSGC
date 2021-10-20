#pragma once

#include <opencv2/core/core.hpp>
#include <dvs_msgs/Event.h>
#include <Eigen/Eigen>

#include <emsgc/tools/utils.h>
#include <emsgc/container/EventQueueMat.h>
#include <emsgc/container/EventMRF.h>

namespace emsgc
{
using namespace tools;
using namespace container;
namespace core
{
void warpEvent(
  MotionModel & model,
  const dvs_msgs::Event& event,
  const double t_ref,
  cv::Point2d* warped_pt );

void warpEvent_GMM(
  GeneralMotionModel &gmm,
  const cv::Point2d& ev_pt,
  const double t_ev,
  const double t_ref,
  cv::Point2d *warped_pt);

bool accumulateWarpedEvent(
  const cv::Point2d& ev_warped_pt,
  const float polarity,
  cv::Mat* image_warped);

size_t computeImageOfWarpedEvents(
  MotionModel & model,
  std::vector<dvs_msgs::Event> &pvEvent_involved,
  const double t_ref,
  const cv::Size& img_size,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp
);

size_t computeImageOfWarpedEvents_GMM(
  GeneralMotionModel & gmm,
  std::vector<dvs_msgs::Event> &pvEvent_involved,
  const double t_ref,
  const cv::Size& img_size,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp
);

size_t computeImageOfWarpedEvents_GMM_undistortion(
  GeneralMotionModel & gmm,
  std::vector<UndistortedEvent> &pvEvent_undistorted,
  const double t_ref,
  const cv::Size& img_size,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp
);

}
}
