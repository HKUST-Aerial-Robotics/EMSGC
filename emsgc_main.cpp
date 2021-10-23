#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>

#include <iostream>
#include <glog/logging.h>

#include <emsgc/core/event_motion_segmentation.h>
#include <emsgc/tools/visualizer.h>
#include <emsgc/tools/TicToc.h>
#include <filesystem>

#define EMS_LOG

using namespace emsgc;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "emsgc");

  // parse input variables (pass via a shell script)
  // load dirs
  if(argc != 5)
  {
    LOG(ERROR) << "Wrong input. The correct command is as follows: \n";
    LOG(ERROR) << "./ems path_to_calib path_to_cfg path_to_event path_to_save_result";
    exit(-1);
  }
  std::string calib_dir(argv[1]);
  std::string option_dir(argv[2]);
  std::string raw_event_dir(argv[3]);
  std::string result_dir(argv[4]);
  LOG(INFO) << "******************************************************************";
  std::string result_raw_iwe_dir(result_dir + "/raw_IWEs");
  std::filesystem::create_directories(result_raw_iwe_dir);
  std::string result_seg_iwe_dir(result_dir + "/seg_IWEs");
  std::filesystem::create_directories(result_seg_iwe_dir);
  std::string result_seg_label_dir(result_dir + "/seg_labels");
  std::filesystem::create_directories(result_seg_label_dir);
  //std::system(("ls -l " + result_dir).c_str());

  LOG(INFO) << "******************************************************************";
  LOG(INFO) << "**********************  I/O Directories  *************************";
  LOG(INFO) << "calib_dir: " << calib_dir;
  LOG(INFO) << "option_dir: " << option_dir;
  LOG(INFO) << "raw_event_dir: " << raw_event_dir;
  LOG(INFO) << "result_dir: " << result_dir;
  LOG(INFO) << "result_raw_iwe_dir: " << result_raw_iwe_dir;
  LOG(INFO) << "result_seg_iwe_dir: " << result_seg_iwe_dir;
  LOG(INFO) << "result_seg_label_dir: " << result_seg_label_dir;
  LOG(INFO) << "******************************************************************";

  // create EMS object
  core::EventMotionSegmentation ems;

  // load calibration
  ems.loadCalibInfo(calib_dir, true);
  LOG(INFO) << "Loaded calibration information.";

  // load option (parameters)
  ems.loadBaseOptions(option_dir);

  // load all events (in the sequence) to an EventQueue (deque)
  ems.loadAllEventsFromTxt(raw_event_dir);
  LOG(INFO) << "Loaded " << ems.getEventQueueLength() << " events in total.";

  // For computational efficiency analysis
  TicToc tt, tt2;
  double t_initStGraph;// time to generate the S-T graph
  double t_initGMMPool, t_initPartitionStVolumeHierarchy, t_initGmmPoolUndistortion, t_initEventClustersSitewisely;
  double t_applySegmentation;

  // Outer Loop (all gt mask)
  size_t frameID = 0;
  for(double t_seg = ems.ts_begin_ + ems.advanceNum_ * ems.ts_step_;
    t_seg < ems.ts_end_;
    t_seg += ems.ts_step_, frameID++)
  {
    LOG(INFO) << "*********************************************";
    LOG(INFO) << "*****************   EMSGC   *****************";
    LOG(INFO) << "*********************************************";
    LOG(INFO) << "This is event-window # " << frameID;

    ros::Time t_ref(t_seg);
    ros::Time t_end(ems.getTemporalBound(t_ref.toSec()));
    LOG(INFO) << "The involved events occur within [" << t_ref.toNSec() << "," << t_end.toNSec() << "] nano sec.";

    ems.loadSpatioTemporalVolume(t_ref, t_end, true);
    LOG(INFO) << "Loaded Spatio-Temporal Volume.";

    /* Algorithm Summary:
    * 1. Initialize siteID (vertex ID in MRF) -> mEventSites_
    * 2. Perform delaunay triangulation (edges in MRF) -> mEdges_
    * 3. Create spatio-temporal graph (a general MRF) -> MRF_XXX[]
    * 4. Init the general motion model (GMM) pool
    * 5. Init mEvtClusters
    * 6. Apply segmentation
    * 7. Delete general graph and other containers.
    * * */

    tt.tic();
    ems.initGeneralGraphSites(true); /* 1. Initialize siteID -> mEventSites_ */
    LOG(INFO) << "Init the general graph sites.";

    ems.delaunayTriangulation(); /* 2. Delaunay triangulation -> mEdges_ */
    LOG(INFO) << "Delaunay Triangulation.";

    ems.createEventGeneralGraph_newInit(); /* 3. Create general graph -> MRF_XXX[] */
    LOG(INFO) << "Create the Event General Graph.";
    t_initStGraph = tt.toc();

    tt.tic();
    tt2.tic();
    ems.divideSTVolume(); /* 4.1 Generate sub divisions on the original ST volume -> mST_vols_ */
    t_initPartitionStVolumeHierarchy = tt2.toc();
    LOG(INFO) << "Init S-T sub volumes.";

    tt2.tic();
    ems.initGmmPoolUndistortion_hyperthread(t_ref); /* 4.2 GMM fitting -> mGmmPool_ */
    t_initGmmPoolUndistortion = tt2.toc();
    LOG(INFO) << "Init the GMM pool.";

    tt2.tic();
    ems.initEventClustersSitewisely(); /* 5. Init event clusters -> mEvntClusters_ */
    t_initEventClustersSitewisely = tt2.toc();
    LOG(INFO) << "Init event clusters site-wisely.";
    t_initGMMPool = tt.toc();

    // apply segmentation
    tt.tic();
    ems.applySegmentation_GMM(t_ref, t_end); /* 6. apply segmentation */
    t_applySegmentation = tt.toc();
    LOG(INFO) << "Apply segmentation.";

#ifdef EMS_LOG
    LOG(INFO) << "******************************************";
    LOG(INFO) << "**********   Computation Cost   **********";
    LOG(INFO) << "******************************************";
    LOG(INFO) << "initStGraphp: " << t_initStGraph / 1000.0 << " s.";
    LOG(INFO) << "initModelPool: " << t_initGMMPool / 1000.0 << " s.";
    LOG(INFO) << "--- divideSTVolume: " << t_initPartitionStVolumeHierarchy / 1000.0 << " s.";
    LOG(INFO) << "--- initGmmPoolUndistortion: " << t_initGmmPoolUndistortion / 1000.0 << " s.";
    LOG(INFO) << "--- initEventClustersSitewisely: " << t_initEventClustersSitewisely / 1000.0 << " s.";
    LOG(INFO) << "applySegmentation: " << t_applySegmentation / 1000.0 << " s.";
    LOG(INFO) << "Total: " << (t_initStGraph + t_initGMMPool + t_applySegmentation) / 1000.0 << " s.";
    LOG(INFO) << "******************************************";
    LOG(INFO) << "******************************************\n\n";
#endif

    /***** Visualization and Saving Results *****/
    cv::Mat raw_iwe, labeled_iwe;

    // display IWE without motion compensation
    GeneralMotionModel gmm0;
    ems.drawIWE_undistorted(gmm0,t_ref, raw_iwe);
    std::ostringstream ss;
    ss << std::setw(4) << std::setfill('0') << frameID;
    std::string iwe_name(ss.str() + ".png");
    if(ems.bSaveResult_)
      cv::imwrite(result_raw_iwe_dir + "/" + iwe_name, raw_iwe);
    if(ems.bDisplayResult_)
      cv::imshow("IWE without Compensation", raw_iwe);

    // display labeled IWE (compensated accordingly)
    //drawColorIWE_with_EventClustersGMM(
    drawColorIWE_with_EventClustersGMM_HSV(
      ems.mEvtClustersGMM_, t_ref, ems.opts_.opts_warp_, ems.img_size_.width, ems.img_size_.height, labeled_iwe);
    if(ems.bSaveResult_)
      cv::imwrite(result_seg_iwe_dir + "/" + iwe_name, labeled_iwe);
    if(ems.bDisplayResult_)
    {
      cv::imshow("Labeled IWE (GMM)", labeled_iwe);
      cv::waitKey(0);
    }

    // save seg label txt
    if(ems.bSaveResult_)
    {
      std::string fileName(ss.str() + ".txt");
      ems.saveSparseLabelMap(t_ref, ems.opts_.opts_warp_, result_seg_label_dir, fileName);
    }

    /* 7. recycle MRF related space and other containers */
    ems.deleteEventGeneralGraph();
    ems.recycleData();
  }

  std::cout << "\n\n";
  LOG(INFO) << raw_event_dir << " has been processed. Exiting now ...\n\n";
  return 1;
}
