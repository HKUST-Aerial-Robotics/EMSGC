#include <emsgc/core/event_motion_segmentation.h>
#include <glog/logging.h>
#include <sstream>

namespace emsgc
{
namespace core
{
void EventMotionSegmentation::optimizeModels_GMM(ros::Time& t_ref)
{
  for(auto& [label, ec] : mEvtClustersGMM_)
  {
    std::vector<UndistortedEvent> vUndistEvents;
    size_t numEvents = ec.dqEvtSites_.size();
    vUndistEvents.reserve(numEvents);
    for(auto it_ev = ec.dqEvtSites_.begin(); it_ev != ec.dqEvtSites_.end(); it_ev++)
      vUndistEvents.push_back(it_ev->ev_undistort_);

    cv::Mat mask = cv::Mat::zeros(cv::Size(img_size_.width, img_size_.height), CV_8UC1);
    mask.setTo(255);

    // Print motion parameters
    LOG(INFO) << "Motion parameters:";
    stringstream res;
    for (int i=0; i <ec.gmm_.num_dof_; ++i){ res << ec.gmm_.parameters_[i] << " "; }
    LOG(INFO) << "  Before optimizeModels_GMM(" << ec.label_ << "): [" << res.str() << "]";

    // Optimize motion parameters
    if(ec.gmm_.num_dof_ == 1)
    {
      gmmFittingUndistortion( vUndistEvents, ec.gmm_.mmType_, mask, t_ref, ec.gmm_, 0.01, 0.01);
    }
    else if(ec.gmm_.num_dof_ == 2)
    {
      gmmFittingUndistortion( vUndistEvents, ec.gmm_.mmType_, mask, t_ref, ec.gmm_, 1, 0.1);
    }
    else if(ec.gmm_.num_dof_ == 3)
    {
      gmmFittingUndistortion( vUndistEvents, ec.gmm_.mmType_, mask, t_ref, ec.gmm_, 0.005, 0.001);
    }
    else if(ec.gmm_.num_dof_ == 4)
    {
      gmmFittingUndistortion( vUndistEvents, ec.gmm_.mmType_, mask, t_ref, ec.gmm_, 0.01, 0.001);
    }
    // Print motion parameters after optimization
    res.str(std::string());
    for (int i=0; i <ec.gmm_.num_dof_; ++i){ res << ec.gmm_.parameters_[i] << " "; }
    LOG(INFO) << "  After  optimizeModels_GMM(" << ec.label_ << "): [" << res.str() << "]";
  }
}

}//namespace core
}//namespace emsgc
