#ifndef EMSGC_CORE_EVENT_MOTION_SEGMENTATION_H
#define EMSGC_CORE_EVENT_MOTION_SEGMENTATION_H

#include <emsgc/tools/utils.h>
#include <emsgc/tools/TicToc.h>
#include <emsgc/core/image_warped_events.h>
#include "../../../trash/TimeSurface.h"
#include <emsgc/container/EventQueueMat.h>
#include <emsgc/container/EventMRF.h>
#include <emsgc/container/PerspectiveCamera.h>

#include <dvs_msgs/Event.h>

namespace emsgc
{
namespace core
{
using namespace tools;
using namespace container;

enum HybridModelCombination
{
  Only2D,
  Only3D,
  Only4D,
  Both2Dand3D,
  Both2Dand4D,
  Both3Dand4D,
  All2D3D4D
};

struct gmmFittingJob
{
  ST_Volome* pST_vol_;
  ros::Time ts_;
  int labelID_;
  cv::Mat* pMask_;
  std::map<int, GeneralMotionModel>* pmGmmPool_;
};

class EventMotionSegmentation
{
public:
  EventMotionSegmentation();
  virtual ~EventMotionSegmentation();

  // IO
  void loadCalibInfo(const std::string &calibInfoPath, bool bPrintCalibInfo = false);
  void loadBaseOptions(const std::string & option_path);
  void loadAllEventsFromTxt(const std::string & event_txt_path);
  void loadSpatioTemporalVolume(ros::Time t_ref, ros::Time t_end);

  /* Segmentation */
  void applySegmentation_GMM(ros::Time& t_ref, ros::Time& t_end);
  int optimizeLabels_GMM(ros::Time& t_ref, const int NUM_ITERATIONS);
  void optimizeModels_GMM(ros::Time& t_ref);

  /*  ST Volume Sub-Division (for initialization) */
  void divideSTVolume();

  /* General Motion Model (GMM) Fitting and Pool Initialization */
  void initGmmPoolUndistortion_hyperthread(ros::Time& t_ref);
  void createGmmThread(gmmFittingJob& job);
  void initGeneralMotionModelFromUndistortedEvents(
    std::vector<UndistortedEvent>& vUndistEvents,
    cv::Mat& mask,
    ros::Time& t_ref,
    GeneralMotionModel& gmm);

  /* To determine the initial value of GMM by brute force */
  void findBestGmmInRangeBruteForce_2D_FromeUndistortedEvents(
    Eigen::Vector2d& trans2D_range,
    double trans2D_step,
    std::vector<UndistortedEvent>& vEvents,
    cv::Mat& mask,
    ros::Time& t_ref,
    GeneralMotionModel & gmm);
  void findBestGmmInRangeBruteForce_3D_FromeUndistortedEvents(
    Eigen::Matrix<double,6,1>& angular3D_range,
    double angular3D_step,
    std::vector<UndistortedEvent>& vEvents,
    cv::Mat& mask,
    ros::Time& t_ref,
    GeneralMotionModel & gmm);
  void findBestGmmInRangeBruteForce_4D_FromeUndistortedEvents(
    Eigen::Vector4d& trans2D_range,
    Eigen::Vector2d& trans2D_step,
    Eigen::Vector2d& scale_range,
    double scale_step,
    Eigen::Vector2d& theta_range,
    double theta_step,
    std::vector<UndistortedEvent>& vEvents,
    cv::Mat& mask,
    ros::Time& t_ref,
    GeneralMotionModel & gmm);

  double gmmFittingUndistortion(
    std::vector<UndistortedEvent>& vUndistEvents,
    MotionModelType mmType,
    cv::Mat& mask,
    ros::Time& t_ref,
    GeneralMotionModel& gmm,
    double init_step_size,
    double tolerance);

  /* initialization of Event clusters */
  void initEventClustersSitewisely();

  /* Event MRF */
  void createEventGeneralGraph_newInit();
  void initGeneralGraphSites(bool bUndistortion = true);
  void deleteEventGeneralGraph();
  void delaunayTriangulation();
  void findSTNeighbourAtXYt(size_t x, size_t y, ros::Time& t, std::vector<size_t> & vST_neighID);
  void computeDataTermLookUpTables_GMM_undistortion(ros::Time& t_ref, ros::Time& t_end);

  /* utils */
  size_t getEventQueueLength();
  double getTemporalBound(double t_ref);
  void recycleData();

  /* IO */
  void drawIWE_undistorted(GeneralMotionModel& gmm, ros::Time& t_ref, cv::Mat& raw_iwe);
  void saveSparseLabelMap(ros::Time& t_ref, OptionsWarp& opts_warp, std::string& saveBaseDir, std::string& fileName);
  void saveIWE(cv::Mat& iwe, std::string& saveBaseDir, std::string& fileName);

  /****************************************************************************/
  /****************************************************************************/
  // data
  PerspectiveCamera::Ptr pCam_;
  EventQueue events_;// all events
  EventQueueMat stVolume_;
  std::map<int, ST_Volome> mST_vols_;
  std::map<int, EventClusterGMM> mEvtClustersGMM_;
  std::map<size_t, EventSite> mEventSites_;
  std::map<int, GeneralMotionModel> mGmmPool_;// newly added <LabelID, GMM>.
  std::map<int, GeneralMotionModel> mGmmPoolLastReserved_;

  std::map<double, vEigenVector4i, std::less<double>,
    Eigen::aligned_allocator<std::pair<const double, vEigenVector4i> > > ts_bbs_assoc_;
  OptionsMethod opts_;
  cv::Size img_size_;
  size_t numSites_;
  size_t numEdges_;

  // used only for ems_continuous
  double ts_begin_, ts_end_, ts_step_;

  cv::Mat tsMap_;
  Eigen::MatrixXd tsNegMap_;

  std::map<int, Eigen::MatrixXd> mIWE_lookup_tables_;

  // MRF
  int *MRF_numNeighbors_;
  int **MRF_neighborsIndexes_;
  int **MRF_neighborsWeights_;
  std::map<size_t, std::list<size_t> > mEdges_; //<2DMap_index, list<2DMap_index> >
  std::vector<size_t> v2dNbVertices_;// stores all neighbouring 2D vertexID in

  // parameters
  double LAMBDA_data_;
  double LAMBDA_smooth_;
  double LAMBDA_label_;
  size_t BracketRow_;
  size_t BracketCol_;
  size_t num_iteration_outer_loop_;
  size_t num_iteration_label_;

  HybridModelCombination hmc_;

  bool bDisplayResult_;
  bool bSaveResult_;
  size_t advanceNum_;

  double Affine_trans2d_max_, Affine_scale_max_, Affine_theta_max_;
  double InPlaneRotation_theta_max_;
  int division_exponent_;

  size_t num_IMO_GT_;

  // result
  cv::Mat labelMap_;
};
}
}

#endif //EMSGC_CORE_EVENT_MOTION_SEGMENTATION_H