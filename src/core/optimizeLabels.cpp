#include <emsgc/core/event_motion_segmentation.h>
#include <emsgc/core/image_warped_events.h>
#include <emsgc/tools/TicToc.h>
#include <GCoptimization.h>
#include <glog/logging.h>
#include <thread>

#define ROUND(a) ((int)((a)+0.5))
#define NUM_THREAD_DATATERM 4
namespace emsgc
{
namespace core
{
enum DataTermFunctor
{
  UNIQUE_TS_NEGATIVE,
  MULTI_IWE_NEGATIVES
};

struct dataTermJobGMM
{
  size_t id_;
  size_t numLabels_;
  double t_ref_;
  std::vector<EventSite>* pvEventSite_;
  std::map<int, Eigen::MatrixXd>* pmIWE_LookUp_Tables_;
//  std::map<int, EventClusterGMM>* pEventClustersGMM_;
  std::map<int, GeneralMotionModel>* pmGmmPool_;
  int LAMBDA_data_;
  int* data_;
};

void dataTermHyperThreadGMM(dataTermJobGMM& job, double lambda)
{
  size_t numLabels = job.numLabels_;
  size_t id = job.id_;
  double t_ref = job.t_ref_;
  std::vector<EventSite>& vEventSite = *job.pvEventSite_;
//  std::map<int, EventClusterGMM>& mEventClusterGMM = *job.pEventClustersGMM_;
  std::map<int, GeneralMotionModel>& mGmmPool = *job.pmGmmPool_;
//  size_t count = 0;

  for(size_t i = id; i < vEventSite.size(); i+=NUM_THREAD_DATATERM)
  {
    size_t siteID = vEventSite[i].siteID_;
    for(size_t l = 0; l < numLabels; l++)
    {
      GeneralMotionModel& gmm = mGmmPool.find(int(l))->second;
      // compute warped coordinate
      cv::Point2d ev_warped_coordinate;
//      dvs_msgs::Event ev_undist = vEventSite[i].ev_;
//      ev_undist.x = vEventSite[i].ev_coord_undistorted_(0);
//      ev_undist.y = vEventSite[i].ev_coord_undistorted_(1);
//      warpEvent_GMM(gmm, ev_undist, t_ref, &ev_warped_coordinate);
      warpEvent_GMM_undistortion(gmm, vEventSite[i].ev_undistort_, t_ref, &ev_warped_coordinate);
      // get cost
      Eigen::Vector2d ev_coordinate(ev_warped_coordinate.x, ev_warped_coordinate.y);
      double cost = bilinearIntepolation(
        job.pmIWE_LookUp_Tables_->find(l)->second, ev_coordinate, 255.0);
//      LOG(INFO) << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
      // assign cost
      job.data_[siteID * numLabels + l] = ROUND(cost * lambda);
//      count++;
//      LOG(INFO) << "Site ID: " << siteID << ", label: " << l << ", cost: " << job.data_[siteID * numLabels + l];
    }
  }
//  LOG(INFO) << "Thread " << id << " caculate " << count << " data term.";
}

inline int smoothnessTerm(int l1, int l2, int LAMBDA_smooth)
{
  if(l1 == l2)
    return 0;
  else
    return LAMBDA_smooth;
}

bool cmp_EventClusterGMM(
  std::pair<int, EventClusterGMM >& a,
  std::pair<int, EventClusterGMM >& b)
{
  return a.second.dqEvtSites_.size() > b.second.dqEvtSites_.size();
}

void sortEventClustersGMM(
  GCoptimization *gc,
  std::map<size_t, EventSite>& mEventSites,//input
//  std::map<int, EventClusterGMM>& mEventClusterGMMOriginal,//input
  std::map<int, GeneralMotionModel>& mGmmPool,
  std::vector<std::pair<int, EventClusterGMM> >& vEventClusterGMMSorted)//output
{
  std::map<int, EventClusterGMM> mEventClusterGMMUpdated;
  for(size_t id = 0; id < gc->numSites(); id++)
  {
    int label = gc->whatLabel(id);
    GeneralMotionModel& gmm = mGmmPool.find(label)->second; // mEventClusterGMMOriginal[label].gmm_;
    mEventSites.find(id)->second.label_ = label;//TODO: label was used before in the find()!!!
    if(mEventClusterGMMUpdated.find(label) == mEventClusterGMMUpdated.end())
      mEventClusterGMMUpdated.emplace(label, EventClusterGMM(label, gmm));
    mEventClusterGMMUpdated.find(label)->second.dqEvtSites_.push_back(mEventSites.find(id)->second);
  }

  vEventClusterGMMSorted.reserve(mEventClusterGMMUpdated.size());
  for(auto& ec : mEventClusterGMMUpdated)
    vEventClusterGMMSorted.push_back(ec);
  sort(vEventClusterGMMSorted.begin(), vEventClusterGMMSorted.end(), cmp_EventClusterGMM);
}

int EventMotionSegmentation::optimizeLabels_GMM(
  ros::Time& t_ref,
  const int NUM_ITERATIONS)
{
  size_t numLabels = mGmmPool_.size();
  GCoptimizationGeneralGraph* gc = new GCoptimizationGeneralGraph(numSites_, numLabels);
  gc->setAllNeighbors(MRF_numNeighbors_, MRF_neighborsIndexes_, MRF_neighborsWeights_);
  LOG(INFO) << "GCoptimizationGeneralGraph's Info: ";
  LOG(INFO) << "-- # Site: " << numSites_;
  LOG(INFO) << "-- # Label: " << numLabels;
  LOG(INFO) << "-- # Edges: " << numEdges_;

  // initialize the labels
  for(auto& ec : mEvtClustersGMM_)
  {
    int label = ec.first;
//    LOG(INFO) << "initialize cluster: " << label << " " << ec.second.dqEvtSites_.size();
    for(auto& es : ec.second.dqEvtSites_)
      gc->setLabel(es.siteID_, label);
  }
  LOG(INFO) << "There are " << mGmmPool_.size() << " labels before the labeling optimization.";

  // set up data terms
  int* data = new int[numSites_ * numLabels];
  std::vector<EventSite> vEvtSiteTmp;
  vEvtSiteTmp.reserve(numSites_);
  for(auto& ec : mEvtClustersGMM_)
    vEvtSiteTmp.insert(vEvtSiteTmp.end(), ec.second.dqEvtSites_.begin(), ec.second.dqEvtSites_.end());
//  LOG(INFO) << "^^^^^^^^^^^^^^^^^^^^^^^^^ vEvtSiteTmp.size(): " << vEvtSiteTmp.size();
//  LOG(INFO) << "^^^^^^^^^^^^^^^^^^^^^^^^^ mEventSites_.size(): " << mEventSites_.size();
  std::vector<dataTermJobGMM> jobs(NUM_THREAD_DATATERM);
  for(size_t i = 0; i < NUM_THREAD_DATATERM; i++)
  {
    jobs[i].id_ = i;
    jobs[i].numLabels_ = numLabels;
    jobs[i].LAMBDA_data_ = LAMBDA_data_;
    jobs[i].data_ = data;
    jobs[i].pvEventSite_ = &vEvtSiteTmp;
    jobs[i].pmIWE_LookUp_Tables_ = &mIWE_lookup_tables_;
//    jobs[i].pEventClustersGMM_ = &mEvtClustersGMM_;
    jobs[i].pmGmmPool_ = &mGmmPool_;
    jobs[i].t_ref_ = t_ref.toSec();
  }
  std::vector<std::thread> threads;
  for(size_t i = 0; i < NUM_THREAD_DATATERM; i++)
    threads.emplace_back(std::bind(&dataTermHyperThreadGMM, jobs[i], LAMBDA_data_));
  for(auto& thread : threads)
    if(thread.joinable())
      thread.join();
  gc->setDataCost(data);
  LOG(INFO) << "set Data cost -------------";

  // set up smoothness terms individually
  for ( int l1 = 0; l1 < numLabels; l1++ )
    for (int l2 = 0; l2 < numLabels; l2++ )
    {
      int cost = smoothnessTerm(l1, l2, LAMBDA_smooth_);
      gc->setSmoothCost(l1, l2,cost);
    }
  LOG(INFO) << "set smoothness cost -------------";

  /* set label cost*/
  gc->setLabelCost(ROUND(LAMBDA_label_));
  LOG(INFO) << "set label cost -------------";

//  tt.tic();
  // graph cut (TODO: Think about to use expansion or swap method.)
//  LOG(INFO) << "data energy: " << gc->giveDataEnergy();
//  LOG(INFO) << "smooth energy: " << gc->giveSmoothEnergy();
//  LOG(INFO) << "label energy: " << gc->giveLabelEnergy();

  LOG(INFO) << "Before optimization energy is: " << gc->compute_energy();
  gc->expansion(NUM_ITERATIONS);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
  int resulting_energy = gc->compute_energy();
  LOG(INFO) << "After optimization energy is:  " << resulting_energy;

//  LOG(INFO) << "Graph cut (alpha-expansion) iterates " << NUM_ITERATIONS
//            << " times, takes " << tt.toc() / 1000.0 << " sec.";

  std::vector<std::pair<int, EventClusterGMM> > vEventClusterGMMSorted;
  sortEventClustersGMM(gc, mEventSites_, mGmmPool_, vEventClusterGMMSorted);

  LOG(INFO) << "-------------------- sortEventClusters --------------------";
  LOG(INFO) << vEventClusterGMMSorted.size() << " clusters left.";
  for(size_t k = 0; k < vEventClusterGMMSorted.size(); k++)
  {
    LOG(INFO) << "old label: (" << vEventClusterGMMSorted[k].first << ") <--> sites: ("
              << vEventClusterGMMSorted[k].second.dqEvtSites_.size() << ", "
              << vEventClusterGMMSorted[k].second.dqEvtSites_.size() * 100.f / numSites_ << "%).";
  }

  std::map<int, EventClusterGMM> mNewEvtClustersGMM_;
  std::map<int, GeneralMotionModel> mNewGmmPool_;
  for(size_t k = 0; k < vEventClusterGMMSorted.size(); k++)
  {
    EventClusterGMM& ec = vEventClusterGMMSorted[k].second;
    ec.label_ = k;
    mNewEvtClustersGMM_.emplace(k, ec);
    mNewGmmPool_.emplace(k, vEventClusterGMMSorted[k].second.gmm_);
  }
  mEvtClustersGMM_.clear();
  mEvtClustersGMM_ = mNewEvtClustersGMM_;
  mGmmPool_.clear();
  mGmmPool_ = mNewGmmPool_;

  LOG(INFO) << "----------- Re-arrange event clusters -----------";
  for(auto& ec : mEvtClustersGMM_)
  {
    LOG(INFO) << "new label: (" << ec.first << ") <--> sites: ("
              << ec.second.dqEvtSites_.size() << ", "
              << ec.second.dqEvtSites_.size() * 100.f / numSites_ << "%).";
  }

  LOG(INFO) << "----------- Re-arrange GMM pool -----------";
  for(auto& gmm : mGmmPool_)
  {
    LOG(INFO) << "new label: (" << gmm.first << ") <--> Type: ("
              << MotionModeType_to_String(gmm.second.mmType_) << ").";
  }

  delete gc;
  delete [] data;
  return resulting_energy;
}

}//namespace core
}//namespace emsgc