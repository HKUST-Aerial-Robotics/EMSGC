#include <emsgc/core/event_motion_segmentation.h>

namespace emsgc
{
namespace core
{
void EventMotionSegmentation::updateModels()
{
  for(auto it = mST_vols_.begin(); it != mST_vols_.end(); it++)
  {
    MotionModel mm = it->second.mm_;
    modelFitting(it->second, mm);
    it->second.mm_ = mm;
  }
}
}//namespace core
}//namespace emsgc
