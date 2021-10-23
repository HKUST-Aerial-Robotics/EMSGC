#include <emsgc/core/event_motion_segmentation.h>
#include <emsgc/tools/visualizer.h>
#include <yaml-cpp/yaml.h>
#include <glog/logging.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>
#include <delaunator.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <thread>

namespace emsgc
{
namespace core
{
EventMotionSegmentation::EventMotionSegmentation()
  :stVolume_(EventQueueMat(0,0)),
   pCam_(new PerspectiveCamera())
{}

EventMotionSegmentation::~EventMotionSegmentation()
{}

void EventMotionSegmentation::loadCalibInfo(const std::string &calibInfoPath, bool bPrintCalibInfo)
{
  const std::string cam_calib_dir(calibInfoPath + "/calib.yaml");
  YAML::Node CamCalibInfo = YAML::LoadFile(cam_calib_dir);

  // load camera calibration
  size_t width = CamCalibInfo["image_width"].as<int>();
  size_t height = CamCalibInfo["image_height"].as<int>();
  std::string cameraName = CamCalibInfo["camera_name"].as<std::string>();
  std::string distortion_model = CamCalibInfo["distortion_model"].as<std::string>();
  std::vector<double> vD, vK;
  vD = CamCalibInfo["distortion_coefficients"]["data"].as< std::vector<double> >();
  vK = CamCalibInfo["camera_matrix"]["data"].as< std::vector<double> >();
  pCam_->setIntrinsicParameters(width,height,cameraName,distortion_model,vD,vK);
  if(bPrintCalibInfo)
  {
    LOG(INFO) << "================== Calib Info ===============";
    LOG(INFO) << "Camera Calibration" << std::endl;
    LOG(INFO) << "--image width : " << pCam_->width_;
    LOG(INFO) << "--image height: " << pCam_->height_;
    LOG(INFO) << "--distortion model: " << pCam_->distortion_model_;
    LOG(INFO) << "--distortion coefficients: " << pCam_->D_.transpose();
    LOG(INFO) << "=============================================" << std::endl;
  }
}

void EventMotionSegmentation::loadBaseOptions(const std::string & option_path)
{
  YAML::Node baseOption = YAML::LoadFile(option_path);
  if(baseOption.IsNull())
  {
    LOG(ERROR) << "Option file not found!";
    exit(-1);
  }

  LOG(INFO) << "******************************************************************";
  LOG(INFO) << "*********************  Algorithm Parameters  *********************";
  // Image size. Copied from calibration information
  CHECK_GT(pCam_->width_, 0);
  CHECK_GT(pCam_->height_, 0);
  img_size_.width = pCam_->width_;
  img_size_.height = pCam_->height_;

  // Number of event per segmentation
  opts_.num_events_per_image_ = baseOption["num_events_per_image"].as<int>();
  LOG(INFO) << "Found parameter: num_events_per_image = " << opts_.num_events_per_image_;

  // Objective function parameters
  opts_.contrast_measure_ = (!baseOption["contrast_measure"].IsDefined() ?
    0 : baseOption["contrast_measure"].as<int>());
  LOG(INFO) << "Found parameter: contrast_measure = " << opts_.contrast_measure_;

  // Event warping parameters
  opts_.opts_warp_.use_polarity_ = (!baseOption["use_polarity"].IsDefined() ?
    false : baseOption["use_polarity"].as<bool>());
  LOG(INFO) << "Found parameter: use_polarity = " << ((opts_.opts_warp_.use_polarity_) ? "true" : "false" );

  opts_.opts_warp_.blur_sigma_ = (!baseOption["gaussian_smoothing_sigma"].IsDefined() ?
    1. : baseOption["gaussian_smoothing_sigma"].as<double>());
  LOG(INFO) << "Found parameter: gaussian_smoothing_sigma = " << opts_.opts_warp_.blur_sigma_ << " pixels";

  // Verbosity / printing level
  // Set the verbosity by running:  env GLOG_v=1 roslaunch emsgc car.launch

  // optimization parameters
  LAMBDA_data_ = baseOption["LAMBDA_data"].as<double>();//data term
  LOG(INFO) << "Found parameter: LAMBDA_data = " << LAMBDA_data_;

  LAMBDA_smooth_ = baseOption["LAMBDA_smooth"].as<double>();//Potts model
  LOG(INFO) << "Found parameter: LAMBDA_smooth = " << LAMBDA_smooth_;

  LAMBDA_label_ = baseOption["LAMBDA_label"].as<double>();//MDL term
  LOG(INFO) << "Found parameter: LAMBDA_label = " << LAMBDA_label_;

  BracketRow_ = baseOption["BracketRow"].as<unsigned int>();
  LOG(INFO) << "Found parameter: BracketRow = " << BracketRow_;

  BracketCol_ = baseOption["BracketCol"].as<unsigned int>();
  LOG(INFO) << "Found parameter: BracketCol = " << BracketCol_;

  num_iteration_outer_loop_ = baseOption["NUM_ITER_SEGMENTATION"].as<unsigned int>();
  LOG(INFO) << "Found parameter: NUM_ITER_SEGMENTATION = " << num_iteration_outer_loop_;

  num_iteration_label_ = baseOption["NUM_ITER_LABELING"].as<unsigned int>();
  LOG(INFO) << "Found parameter: NU_ITER_LABELING = " << num_iteration_label_;

  hmc_ = (HybridModelCombination)baseOption["HybridModelCombination"].as<unsigned int>();
  LOG(INFO) << "Found parameter: HybridModelCombination = " << hmc_;

  bDisplayResult_ = baseOption["DisplayResult"].as<bool>();
  LOG(INFO) << "Found parameter: DisplayResult = " << (bDisplayResult_ ? "True" : "False");
  bSaveResult_ = baseOption["SaveResult"].as<bool>();
  LOG(INFO) << "Found parameter: SaveResult = " << (bSaveResult_ ? "True" : "False");
  advanceNum_ = baseOption["advanceNum"].as<unsigned int>();
  LOG(INFO) << "Found parameter: advanceNum = " << advanceNum_;

  // TODO: remove unused
  Affine_trans2d_max_ = !baseOption["Affine_trans2d_max"].IsDefined() ? 1000 : baseOption["Affine_trans2d_max"].as<double>();
  LOG(INFO) << "Found parameter: Affine_trans2d_max = " << Affine_trans2d_max_;
  Affine_scale_max_   = !baseOption["Affine_scale_max"].IsDefined() ? 0.5 : baseOption["Affine_scale_max"].as<double>();
  LOG(INFO) << "Found parameter: Affine_scale_max = " << Affine_scale_max_;
  Affine_theta_max_   = !baseOption["Affine_theta_max"].IsDefined() ? 1.5 : baseOption["Affine_theta_max"].as<double>();
  LOG(INFO) << "Found parameter: Affine_theta_max = " << Affine_theta_max_;
  InPlaneRotation_theta_max_ = !baseOption["InPlaneRotation_theta_max"].IsDefined() ? 50 : baseOption["InPlaneRotation_theta_max"].as<double>();
  LOG(INFO) << "Found parameter: InPlaneRotation_theta_max = " << InPlaneRotation_theta_max_;
  division_exponent_  = !baseOption["division_exponent"].IsDefined() ? 5 : baseOption["division_exponent"].as<int>();
  LOG(INFO) << "Found parameter: division_exponent = " << division_exponent_;
  num_IMO_GT_ = !baseOption["num_IMO_GT"].IsDefined() ? 1 : baseOption["num_IMO_GT"].as<size_t>();
  LOG(INFO) << "Found parameter: num_IMO_GT = " << num_IMO_GT_;

  ts_begin_ = !baseOption["timestamp_begin"].IsDefined() ? 1 : baseOption["timestamp_begin"].as<double>();
  LOG(INFO) << "Found parameter: ts_begin = " << ts_begin_;
  ts_end_ = !baseOption["timestamp_end"].IsDefined() ? 1 : baseOption["timestamp_end"].as<double>();
  LOG(INFO) << "Found parameter: ts_end = " << ts_end_;
  ts_step_ = !baseOption["timestamp_step"].IsDefined() ? 1 : baseOption["timestamp_step"].as<double>();
  LOG(INFO) << "Found parameter: ts_step = " << ts_step_;
  LOG(INFO) << "******************************************************************";
}

void EventMotionSegmentation::loadAllEventsFromTxt(const std::string & event_txt_path)
{
  std::ifstream event_file(event_txt_path);
  if(!event_file.is_open())
  {
    std::cout << "Raw event file is not opened!" << std::endl;
    exit(-1);
  }
  std::string event_message_line;
  while(std::getline(event_file, event_message_line))
  {
    std::stringstream ss(event_message_line);
    double ts;
    int x, y, polarity;
    ss >> std::fixed >> ts >> x >> y >> polarity;
    dvs_msgs::Event e;
    e.ts = ros::Time(ts);
    e.x = x;
    e.y = y;
    e.polarity = polarity > 0 ? true : false;
    events_.push_back(e);
  }
}

void EventMotionSegmentation::loadSpatioTemporalVolume(
  ros::Time t_ref, ros::Time t_end, bool bDenoising)
{
  stVolume_ = EventQueueMat(img_size_.width, img_size_.height);
  auto it_ev_begin = EventQueue_lower_bound(events_, t_ref);
  auto it_ev_end = EventQueue_lower_bound(events_, t_end);
  numSites_ = 0;
  numEdges_ = 0;
  if(bDenoising)
  {
    cv::Mat mask;
    std::vector<dvs_msgs::Event> vEdgeEvents;
    tools::createDenoisingMask(it_ev_begin, it_ev_end, mask, img_size_.height, img_size_.width, false);
    cv::medianBlur(mask, mask, 1);
    tools::extractDenoisedEvents(it_ev_begin, it_ev_end, vEdgeEvents, mask);
    LOG(INFO) << "Event denoising Finshed... " << vEdgeEvents.size() << " events are preserved.";
    for(size_t i = 0; i < vEdgeEvents.size(); i++)
      stVolume_.insertEvent(vEdgeEvents[i]);
  }
  else
  {
    while(it_ev_begin != it_ev_end)
    {
      stVolume_.insertEvent(*it_ev_begin);
      it_ev_begin++;
    }
  }
  stVolume_.computeGlobalHeadID();
}

size_t EventMotionSegmentation::getEventQueueLength()
{
  return events_.size();
}

double EventMotionSegmentation::getTemporalBound(double t_ref)
{
  ros::Time ts_ref(t_ref);
  auto it_ev_begin = EventQueue_lower_bound(events_, ts_ref);

  size_t num_events_in_between = std::distance(it_ev_begin, events_.end());
  if(num_events_in_between >= opts_.num_events_per_image_)
    num_events_in_between = opts_.num_events_per_image_;

  auto it_ev_end = it_ev_begin;
  std::advance(it_ev_end, num_events_in_between - 1);
  LOG(INFO) << "-------------num_events_in_between: " << num_events_in_between;
  double t_end = it_ev_end->ts.toSec();
  return t_end;
}

void EventMotionSegmentation::delaunayTriangulation()
{
  // create a EventQueueCounter
  size_t width = img_size_.width;
  size_t height = img_size_.height;
  std::vector<double> coords;
  coords.reserve(width * height * 2);

  for(size_t y = 0; y < height; y++)
    for(size_t x = 0; x < width; x++)
      if(stVolume_.getNumEventAtXY(x,y) > 0)
      {
        coords.push_back(x);
        coords.push_back(y);
      }

  // triangulation happens here
  delaunator::Delaunator d(coords);

  // push edges to container, which will be used for constructing MRF_neighborsIndexes_
  mEdges_.clear();
  std::set<size_t> setVerticesTMP;
  v2dNbVertices_.clear();
  v2dNbVertices_.reserve(d.triangles.size() * 3);
  for(size_t i = 0; i < d.triangles.size(); i+=3)
  {
    cv::Point2i p1, p2, p3;
    p1.x = (int)d.coords[2 * d.triangles[i]];
    p1.y = (int)d.coords[2 * d.triangles[i] + 1];
    p2.x = (int)d.coords[2 * d.triangles[i + 1]];
    p2.y = (int)d.coords[2 * d.triangles[i + 1] + 1];
    p3.x = (int)d.coords[2 * d.triangles[i + 2]];
    p3.y = (int)d.coords[2 * d.triangles[i + 2] + 1];

    size_t ID1 = p1.x + p1.y * width;
    size_t ID2 = p2.x + p2.y * width;
    size_t ID3 = p3.x + p3.y * width;

    // used by the new initialization strategy (created on 23 Oct 2020)
    if(setVerticesTMP.find(ID1) == setVerticesTMP.end())
    {
      setVerticesTMP.insert(ID1);
      v2dNbVertices_.push_back(ID1);
    }
    if(setVerticesTMP.find(ID2) == setVerticesTMP.end())
    {
      setVerticesTMP.insert(ID2);
      v2dNbVertices_.push_back(ID2);
    }
    if(setVerticesTMP.find(ID3) == setVerticesTMP.end())
    {
      setVerticesTMP.insert(ID3);
      v2dNbVertices_.push_back(ID3);
    }

    auto it = mEdges_.find(ID1);
    if(it == mEdges_.end())
    {
      mEdges_.emplace(ID1, std::list<size_t>());
      mEdges_.find(ID1)->second.push_back(ID2);
      mEdges_.find(ID1)->second.push_back(ID3);
    }
    else
    {
      it->second.push_back(ID2);
      it->second.push_back(ID3);
    }

    it = mEdges_.find(ID2);
    if(it == mEdges_.end())
    {
      mEdges_.emplace(ID2, std::list<size_t>());
      mEdges_.find(ID2)->second.push_back(ID1);
      mEdges_.find(ID2)->second.push_back(ID3);
    }
    else
    {
      it->second.push_back(ID1);
      it->second.push_back(ID3);
    }

    it = mEdges_.find(ID3);
    if(it == mEdges_.end())
    {
      mEdges_.emplace(ID3, std::list<size_t>());
      mEdges_.find(ID3)->second.push_back(ID1);
      mEdges_.find(ID3)->second.push_back(ID2);
    }
    else
    {
      it->second.push_back(ID1);
      it->second.push_back(ID2);
    }
  }
}

void EventMotionSegmentation::divideSTVolume()
{
  mST_vols_.clear();
  double mean_num_evt_overall = 0;
  int labelID = 0;
  for(size_t h = 1; h <= BracketRow_; h++)
  {
    // Divide the image plane into M x N brackets evenly.
    size_t bracketWidth = img_size_.width / h;
    size_t bracketHeight = img_size_.height / h;
    size_t bracketWidth_mod = img_size_.width % h;
    size_t bracketHeight_mod = img_size_.height % h;

    for(size_t r = 0; r < h; r++)
    {
      for(size_t c = 0; c < h; c++)
      {
        // localize ROI
        size_t x_leftUp = c * bracketWidth;
        size_t y_leftUp = r * bracketHeight;
        size_t x_rightDown, y_rightDown;
        if(r == h - 1)
          y_rightDown = (r+1) * bracketHeight - 1 + bracketHeight_mod;
        else
          y_rightDown = (r+1) * bracketHeight - 1;

        if(c == h - 1)
          x_rightDown = (c+1) * bracketWidth - 1 + bracketWidth_mod;
        else
          x_rightDown = (c+1) * bracketWidth - 1;

        Eigen::Vector4i ROI(x_leftUp, y_leftUp, x_rightDown, y_rightDown);
        // create sub ST volume
        ST_Volome vol(&stVolume_);
        vol.setSpatialDimension(ROI);
        vol.extractSubVolume();
        vol.createMask();

        if(h == 1)
        {
          mean_num_evt_overall = vol.events_involved_.size() * 1.0 / vol.seg_.size();
          mST_vols_.emplace(labelID, vol);
          labelID++;
        }
        else
        {
          if(vol.events_involved_.size() * 1.0 / vol.seg_.size() >= 0.75 * mean_num_evt_overall)
          {
            mST_vols_.emplace(labelID, vol);
            labelID++;
          }
        }
      }
    }
  }
}

void EventMotionSegmentation::initEventClustersSitewisely()
{
  mEvtClustersGMM_.clear();
  for(auto& [siteID, es] : mEventSites_)
  {
    int label = rand() % mGmmPool_.size();
    es.label_ = label;
    auto it_ec = mEvtClustersGMM_.find(label);
    if(it_ec == mEvtClustersGMM_.end())
    {
      EventClusterGMM ec(label, mGmmPool_.find(label)->second);
      ec.dqEvtSites_.push_back(es);
      mEvtClustersGMM_.emplace(label, ec);
    }
    else
      it_ec->second.dqEvtSites_.push_back(es);
  }
}

void EventMotionSegmentation::applySegmentation_GMM(ros::Time& t_ref, ros::Time& t_end)
{
  LOG(INFO) << "Apply segmentation... ...";
  TicToc tt;
  int labeling_result = 0;
  for(int i = 0; i < num_iteration_outer_loop_; i++)
  {
    if(mEvtClustersGMM_.size() == 1)
      break;
    LOG(INFO) << "**********************************************************";
    LOG(INFO) << "The " << i + 1 << "-th iteration.";
//    computeDataTermLookUpTables_GMM(t_ref, t_end);
    computeDataTermLookUpTables_GMM_undistortion(t_ref, t_end);

    tt.tic();
    int cost = optimizeLabels_GMM(t_ref, num_iteration_label_);
    if(cost == labeling_result)
      break;
    else
      labeling_result = cost;
    LOG(INFO) << "-- (" << i + 1 << "-th iter) Discrete Optimization on Labels takes: " << tt.toc() / 1000.0 << " s.";

    tt.tic();
    optimizeModels_GMM(t_ref);
    LOG(INFO) << "-- (" << i + 1 << "-th iter) Continuous Optimization on Models takes: " << tt.toc() << " ms.";
    LOG(INFO) << "**********************************************************\n\n";
  }
}

void EventMotionSegmentation::createEventGeneralGraph_newInit()
{
  // Regarding MRF (define the neighbourhood relationship)
  //  1) SiteID *numNeighbors (numNeighbors[i] returns the number of neighbours that the node i has)
  //  2) SiteID **neighborsIndexes (neighborsIndexes[i] is an array that stores all SiteIDs of the node i's neighbours)
  //  3) EnergyTermType **neighborsWeights (neighborsWeights[i] is an array that stores all weights between node i and its
  //     neighbours. The order of the neighbours is identical to that stored in neighborsIndexes[i])

  MRF_numNeighbors_ = new int[numSites_];
  MRF_neighborsIndexes_ = new int*[numSites_];
  MRF_neighborsWeights_ = new int*[numSites_];

  LOG(INFO) << "Allocate space for arrays (createEventGeneralGraph_newInit) ----: "
            << numSites_ << " " << mEventSites_.size();
  if(numSites_ != mEventSites_.size())
  {
    LOG(INFO) << "mEventSite's size is not correct !!!";
    exit(-1);
  }
  size_t width = img_size_.width;

  for(auto& [label, es] : mEventSites_)
  {
    size_t x = es.ev_.x;
    size_t y = es.ev_.y;
    ros::Time t = es.ev_.ts;
    size_t siteID = es.siteID_;// MRF site ID
    size_t coordIndex = x + y * width;// index in the image plane (ID = x + y * width)

    auto it_edges = mEdges_.find(coordIndex);
    if(it_edges == mEdges_.end())
    {
      LOG(INFO) << "A coordinate is missing in Delaunay's triangulation.";
      LOG(INFO) << "x: " << x;
      LOG(INFO) << "y: " << y;
      exit(-1);
    }

    size_t numEdgesLinkedToThisCoord = it_edges->second.size();
    std::vector<size_t> vST_neighID;
    vST_neighID.reserve(2 * numEdgesLinkedToThisCoord + 2); // horizontal nb (2 * numEdgesLinkedToThisCoord) + vertical nb (2)

    // horizontal nb
    for(auto& nbIndex : it_edges->second)
    {
      size_t nb_x = nbIndex % width;
      size_t nb_y = nbIndex / width;
      findSTNeighbourAtXYt(nb_x, nb_y,t,vST_neighID);
    }
    // vertical nb
    findSTNeighbourAtXYt(x, y,t,vST_neighID);

    if(vST_neighID.size() == 0)
    {
      LOG(INFO) << "!!!!!!!!!!!!!!!!!!!!!! Something wrong !!!!!!!!!!!!!!!!!!!";
      LOG(INFO) << "x: " << x << ", y: " << y;
      LOG(INFO) << "number of events at (x,y): " << stVolume_.getNumEventAtXY(x,y);
      LOG(INFO) << "number of edges in 2D D-Tri: " << mEdges_.find(coordIndex)->second.size();
      exit(-1);
    }

    MRF_numNeighbors_[siteID] = vST_neighID.size();
    MRF_neighborsIndexes_[siteID] = new int[vST_neighID.size()];
    MRF_neighborsWeights_[siteID] = new int[vST_neighID.size()];
    for(size_t k = 0; k < vST_neighID.size(); k++)
    {
      MRF_neighborsIndexes_[siteID][k] = vST_neighID[k];
      MRF_neighborsWeights_[siteID][k] = 1;
      numEdges_++;
    }
  }
}

void EventMotionSegmentation::initGeneralGraphSites(bool bUndistortion)
{
  mEventSites_.clear();
  size_t width = stVolume_.width_;
  size_t height = stVolume_.height_;
  std::set<size_t> coordIdSet;
  for(size_t y = 0; y < height; y++)
    for(size_t x = 0; x < width; x++)
    {
      EventQueue &eq = stVolume_.getEventQueue(x,y);
      for(auto & e : eq)
      {
        size_t siteID = stVolume_.getSiteID(e);
        if (coordIdSet.find(siteID) != coordIdSet.end())
        {
          LOG(INFO) << "Found redundant Site ID."
                    << "-- Site ID: " << siteID << "\n"
                    << "-- Event: " << e.x << " " << e.y << " " << e.ts << " ";
          LOG(INFO) << "The original event with the same ID is: "
                    << mEventSites_.find(siteID)->second.ev_.x << " "
                    << mEventSites_.find(siteID)->second.ev_.y << " "
                    << mEventSites_.find(siteID)->second.ev_.ts;
          continue;
        }
        else
          numSites_++;

        int label = 0;// we assign all sites with label 0 at this early stage.
        EventSite es(label, siteID, e);
        if(bUndistortion)
        {
          Eigen::Vector2d undistortCoord = pCam_->getUndistortedCoordinate(e.x, e.y); // set the undistorted coordinates
          es.ev_undistort_ = UndistortedEvent(e.ts, undistortCoord(0), undistortCoord(1));
        }
        else
          es.ev_undistort_ = UndistortedEvent(e.ts, e.x, e.y);

        mEventSites_.emplace(siteID, es);
        coordIdSet.insert(siteID);
      }
    }
}

void EventMotionSegmentation::deleteEventGeneralGraph()
{
  delete MRF_numNeighbors_;
  for(size_t i = 0; i < numSites_; i++)
  {
    delete [] MRF_neighborsIndexes_[i];
    delete [] MRF_neighborsWeights_[i];
  }
  delete [] MRF_neighborsIndexes_;
  delete [] MRF_neighborsWeights_;

  numEdges_ = 0;
  numSites_ = 0;

  mEdges_.clear();
}

void EventMotionSegmentation::findSTNeighbourAtXYt(
  size_t x,
  size_t y,
  ros::Time& t,
  std::vector<size_t> & vST_neighID)
{
  EventQueue& eq = stVolume_.getEventQueue(x,y);
  dvs_msgs::Event e1, e2;
  if(stVolume_.getMostRecentEventBeforeT(x,y,t,&e1))
  {
    size_t nb1_ID = stVolume_.getSiteID(e1);
    vST_neighID.push_back(nb1_ID);
  }
  if(stVolume_.getMostRecentEventAfterT(x,y,t,&e2))
  {
    size_t nb2_ID = stVolume_.getSiteID(e2);
    vST_neighID.push_back(nb2_ID);
  }
}

// Generate the normalized IWEs (negative) corresponding to each gmm in the motion model pool.
void EventMotionSegmentation::computeDataTermLookUpTables_GMM_undistortion(
  ros::Time& t_ref, ros::Time& t_end)
{
  std::vector<UndistortedEvent> vUndistEvents;
  vUndistEvents.reserve(mEventSites_.size());
  for(auto& [SiteID, es] : mEventSites_)
    vUndistEvents.push_back(es.ev_undistort_);

  // 1. Compute all IWEs.
  // 2. Find the IWE who has the maximum "intensity" value.
  // 3. Normalize all IWEs w.r.t the maximum "intensity" value.
  // 4. Compute the IWEs negative and push them back to mIWE_lookup_tables_

  mIWE_lookup_tables_.clear();
  std::map<int, cv::Mat> mIWE_lookup_tables_TMP;
  int ID_maxValue;
  double maxValue = 0.0;
  cv::Point maxCoord;
  for(auto& [label, gmm] : mGmmPool_)
  {
    cv::Mat iwe;
    computeImageOfWarpedEvents_GMM_undistortion(
      gmm, vUndistEvents, t_ref.toSec(), img_size_, &iwe, opts_.opts_warp_);
    double localMin, localMax;
    cv::Point localMinCoord, localMaxCoord;
    cv::minMaxLoc(iwe, &localMin, &localMax, &localMinCoord, &localMaxCoord);
    if(localMax >= maxValue)
    {
      maxValue = localMax;
      ID_maxValue = label;
      maxCoord = localMaxCoord;
    }
    mIWE_lookup_tables_TMP.emplace(label, iwe);
  }

  cv::Mat iwe_with_max_value = mIWE_lookup_tables_TMP.find(ID_maxValue)->second;
//  LOG(INFO) << "^^^^^^^^^^^^Max value is: " << maxValue;
//  LOG(INFO) << "^^^^^^^^^^^^Max coord is: " << maxCoord;
//  LOG(INFO) << "^^^^^^^^^^^^Max ID is: " << ID_maxValue;

  for(auto& [label, iwe] : mIWE_lookup_tables_TMP)
  {
    cv::Mat concatIWEs;
    cv::hconcat(iwe, iwe_with_max_value, concatIWEs);
    cv::normalize(concatIWEs, concatIWEs, 0.0, 255.0, cv::NORM_MINMAX, CV_32FC1);
    concatIWEs = 255.0 - concatIWEs;
    cv::Mat iwe_crop = concatIWEs(cv::Rect(0,0,img_size_.width, img_size_.height));

    Eigen::MatrixXd iwe_eigen;
    cv::cv2eigen(iwe_crop, iwe_eigen);
    mIWE_lookup_tables_.emplace(label, iwe_eigen);
  }
}

// Initialize the GMM pool (hyper-thread version)
void EventMotionSegmentation::initGmmPoolUndistortion_hyperthread(ros::Time& t_ref)
{
  int labelID = 0;
  mGmmPool_.clear();
  size_t numVols = mST_vols_.size();

  // load job
  std::vector<gmmFittingJob> jobs;
  jobs.reserve(numVols);
  for(auto& [labelID, vol] : mST_vols_)
  {
    gmmFittingJob job;
    job.pST_vol_ = &vol;
    job.ts_ = t_ref;
    job.labelID_ = labelID;
    job.pMask_ = &vol.mask_;
    job.pmGmmPool_ = &mGmmPool_;
    jobs.push_back(job);
  }

  // joint thread
  std::vector<std::thread> threads;
  threads.reserve(numVols);
  for(size_t i = 0; i < numVols; i++)
    threads.emplace_back(std::bind(&EventMotionSegmentation::createGmmThread, this, jobs[i]));
  for(auto& thread:threads)
    if(thread.joinable())
      thread.join();
  LOG(INFO) << "InitGmmPools generates " << mGmmPool_.size() << " models.";
}

void EventMotionSegmentation::createGmmThread(gmmFittingJob& job)
{
  ST_Volome &vol = *job.pST_vol_;
  cv::Mat &mask = *job.pMask_;
  ros::Time& t_ref = job.ts_;
  int labelID = job.labelID_;
  std::map<int, GeneralMotionModel>& mGmmPool = *job.pmGmmPool_;// TODO: to see if this is necessary

  std::vector<UndistortedEvent> vUndistortedEvents;
  vUndistortedEvents.reserve(vol.events_involved_.size());
  for(auto& e:vol.events_involved_)
  {
    Eigen::Vector2d undistortCoord = pCam_->getUndistortedCoordinate(e.x, e.y);
    vUndistortedEvents.push_back(UndistortedEvent(e.ts, undistortCoord(0), undistortCoord(1)));
  }

  switch(hmc_)
  {
    case Only4D:
    {
      double init_step_size_4D = 1;
      double tolerance_4D = 0.1;
      double data4D[4] = {0, 0, 0, 0};
      GeneralMotionModel gmm4D(4, data4D);
      initGeneralMotionModelFromUndistortedEvents(
        vUndistortedEvents,
        mask,
        t_ref,
        gmm4D);
      gmmFittingUndistortion(
        vUndistortedEvents,
        AffineModel,
        mask,
        t_ref,
        gmm4D,
        init_step_size_4D,
        tolerance_4D);
      mGmmPool.emplace(labelID, gmm4D);
      break;
    }
    case Both3Dand4D:
    {
      double init_step_size_3D = 0.01;
      double tolerance_3D = 0.01;
      double data3D[3] = {0, 0, 0};
      GeneralMotionModel gmm3D(3, data3D);
      initGeneralMotionModelFromUndistortedEvents(
        vUndistortedEvents,
        mask,
        t_ref,
        gmm3D);
      gmmFittingUndistortion(
        vUndistortedEvents,
        RotationModel,
        mask,
        t_ref,
        gmm3D,
        init_step_size_3D,
        tolerance_3D);
      mGmmPool.emplace(2 * labelID, gmm3D);

      double init_step_size_4D = 1;
      double tolerance_4D = 0.1;
      double data4D[4] = {0, 0, 0, 0};
      GeneralMotionModel gmm4D(4, data4D);
      initGeneralMotionModelFromUndistortedEvents(
        vUndistortedEvents,
        mask,
        t_ref,
        gmm4D);
      gmmFittingUndistortion(
        vUndistortedEvents,
        AffineModel,
        mask,
        t_ref,
        gmm4D,
        init_step_size_4D,
        tolerance_4D);
      mGmmPool.emplace(2 * labelID + 1, gmm4D);
      break;
    }
    default:
    {
      LOG(INFO) << "The given HybridModelCombination value is not supported in this exemplary release.";
      exit(-1);
    }
  }
//  // 4D Affine
//  if(hmc_ == Only4D)
//  {
//
//  }
  // 3D rotation + 4D affine
//  if(hmc_ == Both3Dand4D)
//  {
//
//  }
}

void EventMotionSegmentation::initGeneralMotionModelFromUndistortedEvents(
  std::vector<UndistortedEvent>& vUndistEvents,
  cv::Mat& mask,
  ros::Time& t_ref,
  GeneralMotionModel& gmm)
{
  if(gmm.mmType_ == TranslationModel)
  {
    const double trans2d_max = 1000.0;
    std::vector<double> trans2D_steps = {trans2d_max, trans2d_max/2, trans2d_max/4, trans2d_max/8, trans2d_max/16, trans2d_max/32};

    GeneralMotionModel gmm_opt;
    double data[2] = {0,0};
    gmm_opt.reset(2, data);
    for(size_t i = 1; i<trans2D_steps.size(); i++)
    {
      const double prev_step = trans2D_steps[i-1];
      const double step = trans2D_steps[i];
      Eigen::Vector2d trans2D_range(gmm_opt.parameters_[0] - prev_step, gmm_opt.parameters_[0] + prev_step);
      findBestGmmInRangeBruteForce_2D_FromeUndistortedEvents(
        trans2D_range, step, vUndistEvents, mask, t_ref, gmm_opt);
    }
    gmm = gmm_opt;
  }

  if(gmm.mmType_ == RotationModel)
  {
    const double angular3D_max = 2 / 57.3;
    std::vector<double> angular3D_steps = {
      angular3D_max, angular3D_max/2, angular3D_max/4, angular3D_max/8, angular3D_max/16, angular3D_max/32};

    GeneralMotionModel gmm_opt;
    double data[3] = {0,0,0};
    gmm_opt.reset(3, data);
    for(size_t i = 1; i<angular3D_steps.size(); i++)
    {
      const double prev_step = angular3D_steps[i-1];
      const double step = angular3D_steps[i];
      Eigen::Matrix<double,6,1> angular3D_range;
      angular3D_range << gmm_opt.parameters_[0] - prev_step,
        gmm_opt.parameters_[0] + prev_step,
        gmm_opt.parameters_[1] - prev_step,
        gmm_opt.parameters_[1] + prev_step,
        gmm_opt.parameters_[2] - prev_step,
        gmm_opt.parameters_[2] + prev_step;

      findBestGmmInRangeBruteForce_3D_FromeUndistortedEvents(
        angular3D_range, step, vUndistEvents, mask,t_ref, gmm_opt);
    }
    gmm = gmm_opt;
  }

  if(gmm.mmType_ == AffineModel)
  {
    const double trans2d_max = Affine_trans2d_max_;
    std::vector<double> trans2D_steps;
    trans2D_steps.reserve(division_exponent_ + 1);
    for(size_t i = 0;i <= division_exponent_;i++)
      trans2D_steps.push_back(trans2d_max / pow(2, i));

    const double scale_max = Affine_scale_max_;
    std::vector<double> scale_steps;
    scale_steps.reserve(division_exponent_ + 1);
    for(size_t i = 0;i <= division_exponent_;i++)
      scale_steps.push_back(scale_max / pow(2, i));

    const double theta_max = Affine_theta_max_;
    std::vector<double> theta_steps;
    theta_steps.reserve(division_exponent_ + 1);
    for(size_t i = 0;i <= division_exponent_;i++)
      theta_steps.push_back(theta_max / pow(2, i));

    GeneralMotionModel gmm_opt;
    double data[4] = {0,0,0,0};
    gmm_opt.reset(4, data);
    for(size_t i = 1; i<trans2D_steps.size(); i++)
    {
      const double prev_step = trans2D_steps[i-1];
      const double step = trans2D_steps[i];
      Eigen::Vector4d trans2D_range(
        gmm_opt.parameters_[0] - prev_step, gmm_opt.parameters_[0] + prev_step,
        gmm_opt.parameters_[1] - prev_step, gmm_opt.parameters_[1] + prev_step);
      Eigen::Vector2d trans2D_step(step, step);

      const double prev_scale_step = scale_steps[i-1];
      const double scale_step = scale_steps[i];

      const double prev_theta_step = theta_steps[i-1];
      const double theta_step = theta_steps[i];

      Eigen::Vector2d scale_range(gmm_opt.parameters_[2] - prev_scale_step, gmm_opt.parameters_[2] + prev_scale_step);
      Eigen::Vector2d theta_range(gmm_opt.parameters_[3] - prev_theta_step, gmm_opt.parameters_[3] + prev_theta_step);
      findBestGmmInRangeBruteForce_4D_FromeUndistortedEvents(
        trans2D_range, trans2D_step,
        scale_range, scale_step,
        theta_range, theta_step,
        vUndistEvents, mask, t_ref, gmm_opt);
    }
    gmm = gmm_opt;
  }
}

void EventMotionSegmentation::drawIWE_undistorted(
  GeneralMotionModel& gmm,
  ros::Time& t_ref,
  cv::Mat& iwe)
{
  OptionsWarp opts_warp_display = opts_.opts_warp_;
  opts_warp_display.blur_sigma_ = 0.;
  cv::Size img_size(img_size_.width, img_size_.height);

  std::vector<UndistortedEvent> vUndistEvents;
  vUndistEvents.reserve(numSites_);
  for(auto& [SiteID, es] : mEventSites_)
  {
    vUndistEvents.push_back(es.ev_undistort_);
  }
  computeImageOfWarpedEvents_GMM_undistortion(
    gmm,vUndistEvents, t_ref.toSec(), img_size, &iwe, opts_warp_display);

  if (opts_warp_display.use_polarity_)
  {
    // Visualize the image of warped events with the zero always at the mean grayscale level
    const float bmax = 5.0;
    iwe = (255.0/(2.0*bmax)) * (iwe + bmax);
  }
  else
  {
    // Scale the image to full range [0,255]
    cv::normalize(iwe, iwe, 0.0, 255.0, cv::NORM_MINMAX, CV_32FC1);
    iwe = 255.0 - iwe; // invert "color": dark events over white background
  }
  iwe.convertTo(iwe, CV_8UC1);
}

void EventMotionSegmentation::saveSparseLabelMap(
  ros::Time& t_ref,
  OptionsWarp& opts_warp,
  std::string& saveBaseDir,
  std::string& fileName)
{
  std::string savePath(saveBaseDir + "/" + fileName);
  ofstream ofile;
  ofile.open(savePath);
  if(!ofile.is_open())
  {
    LOG(INFO) << "The given file is not opened.";
    LOG(INFO) << savePath;
    exit(-1);
  }

  OptionsWarp opts_warp_display = opts_warp;
  opts_warp_display.blur_sigma_ = 0.;

  for(auto it = mEvtClustersGMM_.begin(); it != mEvtClustersGMM_.end(); it++)
  {
    // draw each sub IWE
    auto & ec = it->second;
    std::vector<UndistortedEvent> vUndistortedEvents;
    vUndistortedEvents.reserve(ec.dqEvtSites_.size());
    for(auto& es : ec.dqEvtSites_)
      vUndistortedEvents.push_back(es.ev_undistort_);

    cv::Mat iweROI;
    computeImageOfWarpedEvents_GMM_undistortion(
      ec.gmm_,
      vUndistortedEvents,
      t_ref.toSec(), img_size_, &iweROI, opts_warp_display);

    //cv::normalize(iweROI, iweROI, 0.0, 255.0, cv::NORM_MINMAX, CV_32FC1);

    // record sparse labeled coordinates
    for(size_t y = 0; y < img_size_.height; y++)
      for(size_t x = 0; x < img_size_.width; x++)
      {
        float intensity = iweROI.at<float>(y ,x);
        if(intensity > 2.f) // Denoising: If at least two events (or equivalent contribution) is accumulated here, we regard it as a valid result
        {
          ofile << x << " " << y << " " << it->first << std::endl; // undistorted label map
        }
      }
  }
  ofile.close();
}

void EventMotionSegmentation::recycleData()
{
  mST_vols_.clear();
  mEvtClustersGMM_.clear();
  mEventSites_.clear();
  mGmmPool_.clear();
  mIWE_lookup_tables_.clear();
  mEdges_.clear();
  v2dNbVertices_.clear();
}

}//core
}//ems
