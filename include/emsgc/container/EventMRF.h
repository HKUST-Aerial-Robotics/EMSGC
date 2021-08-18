#ifndef EMSGC_CONTAINER_EVENTMRF_H
#define EMSGC_CONTAINER_EVENTMRF_H

#include <glog/logging.h>
#include <emsgc/container/EventQueueMat.h>
#include <emsgc/tools/utils.h>

namespace emsgc
{
using namespace tools;
namespace container
{
struct EventSite
{
  EventSite(int label, size_t siteID,dvs_msgs::Event& ev):
    label_(label),
    siteID_(siteID),
    ev_(ev)
  {}

  inline void setLabel(int label){ label_ = label; }

  int label_;
  size_t siteID_;
  dvs_msgs::Event ev_;
  UndistortedEvent ev_undistort_;
};

struct EventClusterGMM
{
  EventClusterGMM(int label, GeneralMotionModel& gmm):
    label_(label), gmm_(gmm)
  {}
  EventClusterGMM(){}
  int label_;
  GeneralMotionModel gmm_;
  std::deque<EventSite> dqEvtSites_;
};

/* Initialization
 * 1) ST Volume-wise Model:
 * EventQueueMat -> mST_vols { mm, seg };
 *
 * 2) Event Cluster:
 * EventQueueMat -> mEvtClusters { label, mm, evt_sites }
 * a) the label, i.e. the model, can be determined by checking of evt_site is inside the segment; TODO: implement a function for this;
 * b) The data term could be easily calculated from these clusters. (f(mm, evt, TS_negative) -> cost)
 *
 * 3) Graph (neighbour relationship)
 * a) MRF_numNeighbors_ = new int[numSites];
 * b) MRF_neighborsIndexes_ = new int[][];
 * c) MRF_neighborsWeights_ = new int[][];
 * create a function int getSiteID(event, EventQueueMat, EventCountMat)
 * create a int EventCountMat[height][width], stores the number of events at each pixel coordinate
 *
 * 4) optimize L
 * a) compute data terms
 * b) assign smoothness terms
 * c) set label terms
 * d) optimize
 * e) rearrange label (sorting)
 * f) recreate EvtClusters
 *
 * 5) optimize M
 * a) multi variables joint estimation
 *
 * */
}
}
#endif //EMSGC_CONTAINER_EVENTMRF_H