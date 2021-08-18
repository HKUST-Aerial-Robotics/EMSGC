#include <emsgc/core/event_motion_segmentation.h>
#include <glog/logging.h>

namespace emsgc
{
namespace core
{
void EventMotionSegmentation::updateModels()
{
  auto it = mST_vols_.begin();
  for(; it != mST_vols_.end(); it++)
  {
    MotionModel mm = it->second.mm_;
    modelFitting(it->second, mm);
    it->second.mm_ = mm;
  }
}
}//namespace core
}//namespace emsgc