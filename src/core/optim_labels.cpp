#include <emsgc/core/event_motion_segmentation.h>
#include <emsgc/core/contrastFunctor.h>
#include <emsgc/tools/TicToc.h>
#include <GCoptimization.h>
#include <glog/logging.h>
#include <thread>

//#define LAMBDA_data 10
//#define LAMBDA_smooth 5
//#define LAMBDA_label 100
#define ksi 1.0
#define ROI_WIDTH_HALF 3
#define ROI_HEIGHT_HALF 3
#define ROUND(a) ((int)((a)+0.5))

namespace emsgc
{
namespace core
{

inline bool insideImage(
  size_t xcoord, size_t ycoord,
  size_t width, size_t height
)
{
  if(xcoord < 0 || xcoord > width - 1 || ycoord < 0 || ycoord > height - 1)
    return false;
  return true;
}

struct dataTermJob
{
  int label_;
  size_t num_labels_;
  cv::Mat* iwe_;
  size_t width_;
  size_t height_;
  int LAMBDA_data_;
  // output
  int* data_;
};

inline double dataTermHyperThread(dataTermJob& job)
{
  size_t width = job.width_;
  size_t height = job.height_;
  int label = job.label_;
  size_t num_labels = job.num_labels_;
  int LAMBDA_data = job.LAMBDA_data_;

  for(size_t y = 0; y < height; y++)
    for(size_t x = 0; x < width; x++)
    {
      size_t index = x + y * width;
      // compute cost
      ems_prepare::tools::TicToc tt; tt.tic();
      cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);
      int x_leftUp, y_leftUp, x_rightDown, y_rightDown;
      x_leftUp = x - ROI_WIDTH_HALF;
      x_leftUp = x_leftUp >= 0 ? x_leftUp : 0;
      x_rightDown = x + ROI_WIDTH_HALF;
      x_rightDown = x_rightDown <= width - 1 ? x_rightDown : width - 1;

      y_leftUp = y - ROI_HEIGHT_HALF;
      y_leftUp = y_leftUp >= 0 ? y_leftUp : 0;
      y_rightDown = y + ROI_HEIGHT_HALF;
      y_rightDown = y_rightDown <= height - 1 ? y_rightDown : height - 1;

      mask(cv::Rect(x_leftUp, y_leftUp, x_rightDown - x_leftUp, y_rightDown - y_leftUp)) = 255;

      double contrast = computeContrast(*job.iwe_, mask, VARIANCE_CONTRAST);
      /*** cost = lambda_d * exp(-contrast / ksi) ***/
      // assign cost
      if(contrast < 0.01) //TODO: to think about
        job.data_[index * num_labels + label] = 0;
      else
        job.data_[index * num_labels + label] = ROUND(LAMBDA_data * exp(-contrast / ksi));
    }
}

inline double dataTermHyperThread2(dataTermJob& job)
{
  size_t width = job.width_;
  size_t height = job.height_;
  int label = job.label_;
  size_t num_labels = job.num_labels_;
  int LAMBDA_data = job.LAMBDA_data_;

  for(size_t y = 0; y < height; y++)
    for(size_t x = 0; x < width; x++)
    {
      size_t index = x + y * width;
      float contribution = job.iwe_->at<float>(y,x);
      if(contribution < 0.01f)
        job.data_[index * num_labels + label] = 0;
      else
        job.data_[index * num_labels + label] = ROUND(LAMBDA_data * exp(-contribution / ksi));
    }
}

inline int smoothnessTerm(int l1, int l2, int LAMBDA_smooth)
{
  if(l1 == l2)
    return 0;
  else
    return LAMBDA_smooth;
}

bool cmp(
  std::pair<int, std::list<cv::Point2i> >& a,
  std::pair<int, std::list<cv::Point2i> >& b)
{
  return a.second.size() > b.second.size();
}

inline void sortSegAccordingToArea (
  GCoptimization *gc,
  std::vector<std::pair<int, std::list<cv::Point2i> > >& newSegSorted,
  int width,
  int height)
{
  std::map<int, std::list<cv::Point2i> > newSegmentation;
  // collect resulting labels and segments' coordinates
  for(int y = 0; y < height; y++)
    for(int x = 0; x < width; x++)
    {
      int label = gc->whatLabel(x+y*width);
      auto it = newSegmentation.find(label);
      if(it != newSegmentation.end())
        it->second.emplace_back(x,y);
      else
      {
        std::list<cv::Point2i> listTmp;
        listTmp.emplace_back(cv::Point2i(x,y));
        newSegmentation.emplace(label, listTmp);
      }
    }
  // sorting
  for(auto& seg : newSegmentation)
    newSegSorted.push_back(seg);
  sort(newSegSorted.begin(), newSegSorted.end(), cmp);
}

int EventMotionSegmentation::updateLabels_hyperThread(
  const unsigned int NUM_ITERATIONS)
{
  int width = img_size_.width;
  int height = img_size_.height;
  int num_sites = width * height;
  int num_labels = mST_vols_.size();

  // compute data cost array
  int* data = new int[num_sites * num_labels];
  std::vector<dataTermJob> jobs(num_labels);
  for(int i = 0; i < num_labels; i++)
  {
    jobs[i].num_labels_ = num_labels;
    jobs[i].label_ = i;
    jobs[i].data_ = data;
    jobs[i].height_ = height;
    jobs[i].width_ = width;
    jobs[i].iwe_ = &(mIWE_lookup_tables_.find(i)->second);
    jobs[i].LAMBDA_data_ = LAMBDA_data_;
  }
  ems_prepare::tools::TicToc tt;
  std::vector<std::thread> threads;
  for(size_t i = 0; i < num_labels; i++)
    threads.emplace_back(std::bind(&dataTermHyperThread, jobs[i]));
  for(auto& thread : threads)
  {
    if(thread.joinable())
      thread.join();
  }
  LOG(INFO) << "Data cost computation (hyper-thread) takes: " << tt.toc() / 1000.0 << " sec.";

  GCoptimization *gc = new GCoptimizationGridGraph(width, height, num_labels);
  // initialize the labels
  for(auto& it : mST_vols_)
  {
    auto it_site = it.second.seg_.begin();
    for(;it_site != it.second.seg_.end(); it_site++)
      gc->setLabel(it_site->first, it.first);
  }

  // set up data cost
  gc->setDataCost(data);

  // set up smoothness terms individually
  for ( int l1 = 0; l1 < num_labels; l1++ )
    for (int l2 = 0; l2 < num_labels; l2++ )
    {
      int cost = smoothnessTerm(l1,l2, LAMBDA_smooth_);
      gc->setSmoothCost(l1,l2,cost);
    }

  /* set label cost*/
//  gc->setLabelCost(ROUND(LAMBDA_label));

  tt.tic();
  // graph cut (TODO: Think about to use expansion or swap method.)
  LOG(INFO) << "Before optimization energy is " << gc->compute_energy();
  gc->expansion(NUM_ITERATIONS);// run expansion for 2 iterations. For swap use gc->swap(num_iterations);
  int resulting_energy = gc->compute_energy();
  LOG(INFO) << "After optimization energy is " << resulting_energy;

  LOG(INFO) << "Graph cut (alpha-expansion) iterates " << NUM_ITERATIONS
            << " times, takes " << tt.toc() / 1000.0 << " sec.";

//  // assign labeling result
//  for(size_t y = 0; y < height; y++)
//    for(size_t x = 0; x < width; x++)
//      labelMap_.at<uchar>(y,x) = gc->whatLabel(x + y * width);

  //  1. sort the resulting segments according to their area.
  //  2. update mST_vol_ (namely L, M, as well as ST volumes)
  std::vector<std::pair<int, std::list<cv::Point2i> > > newSegSorted;
  sortSegAccordingToArea(gc, newSegSorted, width, height);

  LOG(INFO) << "-------------------- sortSegAccordingToArea --------------------";
  LOG(INFO) << newSegSorted.size() << " segments left.";
  for(auto it_seg = newSegSorted.begin(); it_seg != newSegSorted.end(); it_seg++)
  {
    LOG(INFO) << "label: (" << it_seg->first << ") <--> sites: ("
              << it_seg->second.size() << ", " << it_seg->second.size() * 100.f / num_sites << "%).";
  }

  std::map<int, ST_Volome> mST_vol_tmp;
  double t_ref = mST_vols_.begin()->second.t_ref_.toSec();
  double t_bound = mST_vols_.begin()->second.t_end_.toSec();
  auto it = newSegSorted.begin();
  int label = 0;
//  LOG(INFO) << "-------------------- ++++++++++++++++++++.";
  for(;it != newSegSorted.end(); it++, label++)
  {
    ST_Volome vol(&wholeVolume_);
    vol.setSpatialDimension(it->second);
    vol.setTemporalRange(t_ref, t_bound);
    vol.extractSubVolume();
    vol.createMask();
    vol.mm_ = mST_vols_.find(it->first)->second.mm_; // assign the motion model indexed with the old label.
    vol.label_ = label;
    mST_vol_tmp.emplace(label, vol);
    // assign labelMap_
//    assignLabels(label, it->second);
  }

//  LOG(INFO) << "-------------------- ++++++++++++++++++++.";
  mST_vols_.clear();
  mST_vols_ = mST_vol_tmp;

//  LOG(INFO) << "-------------------- update ST volumes.";

  delete gc;
  delete [] data;
  return resulting_energy;
}

}//namespace core
}//namespace ems

