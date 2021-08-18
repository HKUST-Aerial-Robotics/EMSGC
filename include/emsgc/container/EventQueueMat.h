#ifndef EMSGC_CONTAINER_EVENTQUEUEMAT_H
#define EMSGC_CONTAINER_EVENTQUEUEMAT_H
#include <emsgc/tools/utils.h>
#include <glog/logging.h>

namespace emsgc
{
using namespace tools;
namespace container
{
class EventQueueMat
{
public:
  EventQueueMat(int width, int height):width_(width), height_(height)
  {
    eqMat_ = std::vector<EventQueue>(width_ * height_, EventQueue());
    eqGlobalHeadID_ = std::vector<size_t>(width_ * height_, 0);
  }

  inline bool getMostRecentEventAfterT(
    const size_t x,
    const size_t y,
    const ros::Time& t,
    dvs_msgs::Event* ev)
  {
    if(!insideImage(x, y))
      return false;

    EventQueue& eq = getEventQueue(x, y);
    if(eq.empty())
      return false;

    for(auto it = eq.begin(); it != eq.end(); ++it)
    {
      const dvs_msgs::Event& e = *it;
      if(e.ts >= t)
      {
        *ev = *it;
        return true;
      }
    }
    return false;
  }

  bool getMostRecentEventBeforeT(
    const size_t x,
    const size_t y,
    const ros::Time& t,
    dvs_msgs::Event* ev)
  {
    if(!insideImage(x, y))
      return false;

    EventQueue& eq = getEventQueue(x, y);
    if(eq.empty())
      return false;

    for(auto it = eq.rbegin(); it != eq.rend(); ++it)
    {
      const dvs_msgs::Event& e = *it;
      if(e.ts <= t)
      {
        *ev = *it;
        return true;
      }
    }
    return false;
  }

  void insertEvent(const dvs_msgs::Event& e)
  {
    if(!insideImage(e.x, e.y))
      return;
    else
    {
      EventQueue& eq = getEventQueue(e.x, e.y);
      auto it_lb = EventQueue_lower_bound(eq, const_cast<ros::Time&>(e.ts));
      if(it_lb == eq.end()) // the new coming event is the latest one in this coordinate
        eq.push_back(e);
      else
      {
        if(it_lb->ts != e.ts) // the new coming one is not the most recent one, but no redundancy is witnessed.
          eq.push_back(e);
      }
    }
  }

  bool insideImage(const size_t x, const size_t y)
  {
    return !(x < 0 || x >= width_ || y < 0 || y >= height_);
  }

  inline EventQueue& getEventQueue(const size_t x, const size_t y)
  {
    return eqMat_[x + width_ * y];
  }

  inline size_t getNumEventAtXY(const size_t x, const size_t y)
  {
    return eqMat_[x + width_ * y].size();
  }

  // Compute the global index of the first (earliest) event at each pixel coordinate.
  // This global index is used for indexing an event in the Spatio-Temporal Volume.
  inline void computeGlobalHeadID()
  {
    size_t counter = 0;
    for(size_t y = 0; y < height_; y++)
      for(size_t x = 0; x < width_ ; x++)
      {
        eqGlobalHeadID_[x + y * width_] = counter;
        counter += getNumEventAtXY(x,y);
      }
  }

  inline size_t getGlobalHeadID(const size_t x, const size_t y)
  {
    return eqGlobalHeadID_[x + y * width_];
  }

  inline size_t getSiteID(dvs_msgs::Event& e)
  {
    // local ID
    EventQueue& eq = getEventQueue(e.x, e.y);
    auto it = EventQueue_lower_bound(eq, e.ts);
    size_t localID = std::distance(eq.begin(), it);
    size_t globalID = getGlobalHeadID(it->x ,it->y) + localID;
    return globalID;
  }

  size_t width_;
  size_t height_;
  std::vector<EventQueue> eqMat_;
  std::vector<size_t> eqGlobalHeadID_;// restores the global ID of the first element at the position (x,y)
};

class ST_Volome
{
  public:
  ST_Volome(EventQueueMat* pEvtQueueMat):pEvtQueueMat_(pEvtQueueMat)
  {}
  virtual ~ST_Volome(){}

  // set spatial dim with an ROI.
  void setSpatialDimension(Eigen::Vector4i& ROI_range)
  {
    int leftUpX = ROI_range[0];
    int leftUpY = ROI_range[1];
    int rightDownX = ROI_range[2];
    int rightDownY = ROI_range[3];

    size_t img_width = pEvtQueueMat_->width_;

    seg_.clear();
    for(int y = leftUpY; y <= rightDownY; y++)
    {
      for(int x = leftUpX; x <= rightDownX; x++)
      {
        size_t index = y * img_width + x;
        seg_.emplace(index, std::make_pair(x,y));
      }
    }
  }

  void setSpatialDimension(std::list<cv::Point2i>& lCoordinates)
  {
    size_t img_width = pEvtQueueMat_->width_;
    seg_.clear();
    auto it = lCoordinates.begin();
    for(;it != lCoordinates.end(); it++)
    {
      int x = it->x;
      int y = it->y;
      size_t index = y * img_width + x;
      seg_.emplace(index, std::make_pair(x,y));
    }
  }

  /* extract sub ST vol from the whole vol.
   * */
  void extractSubVolume()
  {
    std::deque<dvs_msgs::Event> dqEventTmp;
    for(auto it = seg_.begin(); it != seg_.end(); it++)
    {
      size_t x = it->second.first;
      size_t y = it->second.second;

      // trim the temporal dimension
      EventQueue& EvtQueueAtXY = pEvtQueueMat_->getEventQueue(x,y);
      if(EvtQueueAtXY.size() == 0)
        continue;
      auto it_ev_begin = EvtQueueAtXY.begin();
      auto it_ev_end = EvtQueueAtXY.end();
      if(it_ev_begin != it_ev_end)
        dqEventTmp.insert(dqEventTmp.end(), it_ev_begin, it_ev_end);
    }

    // Transfer dq to vector
    events_involved_.clear();
    if(dqEventTmp.size() == 0)
    {
//      LOG(INFO) << events_involved_.size() << " events reside in the ST volume spanning at ["
//                << t_ref_.toSec() << ", " << t_end_.toSec() << "].";
      return;
    }
    events_involved_.reserve(dqEventTmp.size());
    events_involved_.insert(events_involved_.end(), dqEventTmp.begin(), dqEventTmp.end());
//    LOG(INFO) << events_involved_.size() << " events reside in this ST volume.";
  }

  /* Add a voxel
   * */
  void addVoxelAtXY(size_t x, size_t y)
  {
    size_t img_width = pEvtQueueMat_->width_;
    size_t index = y * img_width + x;
    if(seg_.find(index) != seg_.end()) // voxel exists
      return;
    // add element to seg
    seg_.emplace(index, std::make_pair(x,y));
    // trim the temporal dimension
    EventQueue& EqAtXY = pEvtQueueMat_->getEventQueue(x,y);
    auto it_ev_begin = EqAtXY.begin();
    auto it_ev_end = EqAtXY.end();

    // refresh events_involved
    std::vector<dvs_msgs::Event> vEvtTmp;
    vEvtTmp.reserve(events_involved_.size() + std::distance(it_ev_begin, it_ev_end));
    vEvtTmp.insert(vEvtTmp.end(), events_involved_.begin(), events_involved_.end());
    vEvtTmp.insert(vEvtTmp.end(), it_ev_begin, it_ev_end);
    events_involved_.clear();
    events_involved_ = vEvtTmp;
  }

  /* Remove a voxel
   * */
  void removeVoxelAtXY(size_t x, size_t y)
  {
    size_t img_width = pEvtQueueMat_->width_;
    size_t index = y * img_width + x;
    if(seg_.find(index) == seg_.end()) // voxel does not exist
      return;
    seg_.erase(index);

    // refresh events_involved
    extractSubVolume();
  }

  /* Create the mask in the original image plane
   * */
  void createMask()
  {
//    LOG(INFO) << "pEvtQueueMat_'size: " << pEvtQueueMat_->width_ << " "  << pEvtQueueMat_->height_;
    mask_ = cv::Mat::zeros(pEvtQueueMat_->height_, pEvtQueueMat_->width_, CV_8UC1);
//    LOG(INFO) << "mask'size: " << mask_.cols << " " << mask_.rows;
    auto it = seg_.begin();
    while(it != seg_.end())
    {
      size_t x = it->second.first;
      size_t y = it->second.second;
      mask_.at<uchar>(y, x) = 255;
      it++;
    }
  }

  void setModel(MotionModel& mm)
  {
    mm_ = mm;
  }

  std::map<size_t, std::pair<size_t, size_t> > seg_; //<index, coordinate>
  std::map<size_t, float> contribution_;
//  ros::Time t_ref_, t_end_; // temporal dimension
//  std::map<size_t, EventVector> vol_;
  std::vector<dvs_msgs::Event> events_involved_;

  // pointer to the whole volume
  EventQueueMat* pEvtQueueMat_;

  //
  cv::Mat mask_;

  int label_;
  MotionModel mm_;
};

struct UndistortedEvent
{
  UndistortedEvent(){}
  UndistortedEvent(ros::Time& time, double x, double y):
  ts_(time), x_(x), y_(y){}
  ros::Time ts_;
  double x_;
  double y_;
};

}
}

// getSubStVolume
// --> seg's coordinates ==> mask for IWE
// --> involved events ==> compute IWE and contrast

#endif //EMSGC_CONTAINER_EVENTQUEUEMAT_H