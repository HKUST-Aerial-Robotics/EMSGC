#include <emsgc/core/event_motion_segmentation.h>
#include <glog/logging.h>

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

    if(ec.gmm_.num_dof_ == 1)
    {
      LOG(INFO) << "Before optimizeModels_GMM(" << ec.label_ << "): " << ec.gmm_.parameters_[0];
      gmmFittingUndistortion(
        vUndistEvents, ec.gmm_.mmType_, mask, t_ref,
        ec.gmm_, 0.01, 0.01);
      LOG(INFO) << "After optimizeModels_GMM(" << ec.label_ << "): " << ec.gmm_.parameters_[0];
    }
    if(ec.gmm_.num_dof_ == 2)
    {
      LOG(INFO) << "Before optimizeModels_GMM(" << ec.label_ << "): " << ec.gmm_.parameters_[0] << " " << ec.gmm_.parameters_[1];
      gmmFittingUndistortion( vUndistEvents, ec.gmm_.mmType_, mask, t_ref, ec.gmm_, 1, 0.1);
      LOG(INFO) << "After optimizeModels_GMM(" << ec.label_ << "): " << ec.gmm_.parameters_[0] << " " << ec.gmm_.parameters_[1];
    }

    if(ec.gmm_.num_dof_ == 3)
    {
      LOG(INFO) << "Before optimizeModels_GMM(" << ec.label_ << "): "
                << ec.gmm_.parameters_[0] << " " << ec.gmm_.parameters_[1] << " " << ec.gmm_.parameters_[2];
      gmmFittingUndistortion( vUndistEvents, ec.gmm_.mmType_, mask, t_ref, ec.gmm_, 0.005, 0.001);
      LOG(INFO) << "After optimizeModels_GMM(" << ec.label_ << "): "
                << ec.gmm_.parameters_[0] << " " << ec.gmm_.parameters_[1] << " " << ec.gmm_.parameters_[2];
    }

    if(ec.gmm_.num_dof_ == 4)
    {
      LOG(INFO) << "Before optimizeModels_GMM(" << ec.label_ << "): "
                << ec.gmm_.parameters_[0] << " " << ec.gmm_.parameters_[1] << " " << ec.gmm_.parameters_[2] << " " << ec.gmm_.parameters_[3];
      gmmFittingUndistortion( vUndistEvents, ec.gmm_.mmType_, mask, t_ref, ec.gmm_, 0.01, 0.001);
      LOG(INFO) << "After optimizeModels_GMM(" << ec.label_ << "): "
                << ec.gmm_.parameters_[0] << " " << ec.gmm_.parameters_[1] << " " << ec.gmm_.parameters_[2] << " " << ec.gmm_.parameters_[3];
    }
  }
}

}//namespace core
}//namespace emsgc

