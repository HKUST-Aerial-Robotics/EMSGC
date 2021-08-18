#include <emsgc/core/image_warped_events.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace emsgc
{
using namespace container;
namespace core
{
void warpEvent(
  MotionModel &model,
  const dvs_msgs::Event &event,
  const double t_ref,
  cv::Point2d *warped_pt)
{
  // compute the optical flow velocity using the motion model
  Eigen::Vector2d trans2D;
  trans2D << model.m_x_,
             model.m_y_;
  Eigen::Matrix2d rotation;
  rotation << cos(model.m_theta_), -sin(model.m_theta_),
              sin(model.m_theta_),  cos(model.m_theta_);
  Eigen::Vector2d pt(event.x, event.y);
  Eigen::Vector2d vel = trans2D + (1 + model.m_s_) * rotation * pt - pt;
  // warp (the minus sign is because vel is defined from t_ref to t_k.
  Eigen::Vector2d displacement = (event.ts.toSec() - t_ref) * vel;
  warped_pt->x = event.x - displacement(0);
  warped_pt->y = event.y - displacement(1);
}

void warpEvent_GMM(
  GeneralMotionModel &gmm,
  const dvs_msgs::Event &event,
  const double t_ref,
  cv::Point2d *warped_pt)
{
  // compute the optical flow velocity using the motion model
  if(gmm.mmType_ == TranslationModel)
  {
    Eigen::Vector2d vel;
    vel << gmm.parameters_[0];
           gmm.parameters_[1];

    // warp (the minus sign is because vel is defined from t_ref to t_k.
    Eigen::Vector2d displacement = (event.ts.toSec() - t_ref) * vel;
    warped_pt->x = event.x - displacement(0);
    warped_pt->y = event.y - displacement(1);
  }

  if(gmm.mmType_ == RotationModel)
  {
    Eigen::Vector3d w;
    w << gmm.parameters_[0],
         gmm.parameters_[1],
         gmm.parameters_[2];
    Eigen::Vector3d ecoord_3D_rotated, ecoord_3d;
    ecoord_3d << event.x,
                 event.y,
                 1;
    ecoord_3D_rotated = ecoord_3d + (event.ts.toSec() - t_ref) * (w.cross(ecoord_3d));
    warped_pt->x = ecoord_3D_rotated(0) / ecoord_3D_rotated(2);
    warped_pt->y = ecoord_3D_rotated(1) / ecoord_3D_rotated(2);
  }

  if(gmm.mmType_ == AffineModel)
  {
    Eigen::Vector2d vel;
    Eigen::Vector2d trans2D;
    trans2D << gmm.parameters_[0],
               gmm.parameters_[1];
    Eigen::Matrix2d rotation;
    double theta = gmm.parameters_[3];
    rotation << cos(theta), -sin(theta),
                sin(theta),  cos(theta);
    double scale = gmm.parameters_[2];
    Eigen::Vector2d pt(event.x, event.y);
    vel = trans2D + (1 + scale) * rotation * pt - pt;

    // warp (the minus sign is because vel is defined from t_ref to t_k.
    Eigen::Vector2d displacement = (event.ts.toSec() - t_ref) * vel;
    warped_pt->x = event.x - displacement(0);
    warped_pt->y = event.y - displacement(1);
  }
}

void warpEvent_GMM_undistortion(
  GeneralMotionModel &gmm,
  const UndistortedEvent &uev,
  const double t_ref,
  cv::Point2d *warped_pt)
{
  // compute the optical flow velocity using the motion model
  if(gmm.mmType_ == TranslationModel)
  {
    Eigen::Vector2d vel;
    vel << gmm.parameters_[0];
    gmm.parameters_[1];

    // warp (the minus sign is because vel is defined from t_ref to t_k.
    Eigen::Vector2d displacement = (uev.ts_.toSec() - t_ref) * vel;
    warped_pt->x = uev.x_ - displacement(0);
    warped_pt->y = uev.y_ - displacement(1);
  }

  if(gmm.mmType_ == RotationModel)
  {
    Eigen::Vector3d w;
    w << gmm.parameters_[0],
         gmm.parameters_[1],
         gmm.parameters_[2];
    Eigen::Vector3d ecoord_3D_rotated, ecoord_3d;
    ecoord_3d << uev.x_,
                 uev.y_,
                  1;
    ecoord_3D_rotated = ecoord_3d - (uev.ts_.toSec() - t_ref) * (w.cross(ecoord_3d));
    warped_pt->x = ecoord_3D_rotated(0) / ecoord_3D_rotated(2);
    warped_pt->y = ecoord_3D_rotated(1) / ecoord_3D_rotated(2);
  }

  if(gmm.mmType_ == AffineModel)
  {
    Eigen::Vector2d vel;
    Eigen::Vector2d trans2D;
    trans2D << gmm.parameters_[0],
               gmm.parameters_[1];
    Eigen::Matrix2d rotation;
    double theta = gmm.parameters_[3];
    rotation << cos(theta), -sin(theta),
      sin(theta),  cos(theta);
    double scale = gmm.parameters_[2];
    Eigen::Vector2d pt(uev.x_, uev.y_);
    vel = trans2D + (1 + scale) * rotation * pt - pt;

    // warp (the minus sign is because vel is defined from t_ref to t_k.
    Eigen::Vector2d displacement = (uev.ts_.toSec() - t_ref) * vel;
    warped_pt->x = uev.x_ - displacement(0);
    warped_pt->y = uev.y_ - displacement(1);
  }
}

bool accumulateWarpedEvent(
  const dvs_msgs::Event &event,
  const int img_width,
  const int img_height,
  const cv::Point2d &ev_warped_pt,
  cv::Mat *image_warped,
  const OptionsWarp &optsWarp)
{
  const float polarity = (optsWarp.use_polarity_) ? 2.f * static_cast<float>(event.polarity) - 1.f : 1.f;

  // Accumulate warped events, using bi-linear voting (polarity or count)
  const int xx = ev_warped_pt.x, yy = ev_warped_pt.y;

  // if warped point is within the image, accumulate its contribution
  // to neighbouring four pixels.
  if (1 <= xx && xx < img_width - 2 && 1 <= yy && yy < img_height - 2)
  {
    // Accumulate warped events on the IWE
    // FILL IN ...  image_warped (4 pixels)
    /* (x1,y1) --- (x2,y1) */
    /*    |           |    */
    /*    |           |    */
    /* (x1,y2) --- (x2,y2) */
    size_t x1 = xx;//std::floor(ev_warped_pt.x);
    size_t y1 = yy;//std::floor(ev_warped_pt.y);
    size_t x2 = xx+1;//std::ceil(ev_warped_pt.x);
    size_t y2 = yy+1;//std::ceil(ev_warped_pt.y);

    double delta_x = ev_warped_pt.x - x1;
    double delta_y = ev_warped_pt.y - y1;

    image_warped->at<float>(y1, x1) += polarity * (1 - delta_x) * (1 - delta_y);
    image_warped->at<float>(y1, x2) += polarity * delta_x * (1 - delta_y);
    image_warped->at<float>(y2, x1) += polarity * (1 - delta_x) * delta_y;
    image_warped->at<float>(y2, x2) += polarity * delta_x * delta_y;
    return true;
  }
  return false;
}

bool accumulateWarpedEvent_Undistortion(
  const int img_width,
  const int img_height,
  const cv::Point2d& ev_warped_pt,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp)
{
  // Accumulate warped events, using bi-linear voting (polarity or count)
  const int xx = ev_warped_pt.x, yy = ev_warped_pt.y;

  // if warped point is within the image, accumulate its contribution
  // to neighbouring four pixels.
  if (1 <= xx && xx < img_width - 2 && 1 <= yy && yy < img_height - 2)
  {
    // Accumulate warped events on the IWE
    // FILL IN ...  image_warped (4 pixels)
    /* (x1,y1) --- (x2,y1) */
    /*    |           |    */
    /*    |           |    */
    /* (x1,y2) --- (x2,y2) */
    size_t x1 = xx;//std::floor(ev_warped_pt.x);
    size_t y1 = yy;//std::floor(ev_warped_pt.y);
    size_t x2 = xx+1;//std::ceil(ev_warped_pt.x);
    size_t y2 = yy+1;//std::ceil(ev_warped_pt.y);

    double delta_x = ev_warped_pt.x - x1;
    double delta_y = ev_warped_pt.y - y1;

    image_warped->at<float>(y1, x1) += (1 - delta_x) * (1 - delta_y);
    image_warped->at<float>(y1, x2) += delta_x * (1 - delta_y);
    image_warped->at<float>(y2, x1) += (1 - delta_x) * delta_y;
    image_warped->at<float>(y2, x2) += delta_x * delta_y;
    return true;
  }
  return false;
}

size_t computeImageOfWarpedEvents(
  MotionModel &model,
  std::vector<dvs_msgs::Event> &pvEvent_involved,
  const double t_ref,
  const cv::Size &img_size,
  cv::Mat *image_warped,
  const OptionsWarp &optsWarp)
{
  const int width = img_size.width;
  const int height = img_size.height;

  // Create image of warped events (IWE)
  *image_warped = cv::Mat::zeros(height, width, CV_32FC1);

  // Loop through all events
  size_t numAccumulation = 0;
  for (const dvs_msgs::Event &ev : pvEvent_involved)
  {
    // Warp event according to candidate flow and accumulate on the IWE
    cv::Point2d warped_pt;
    warpEvent(model, ev, t_ref, &warped_pt);
    if(accumulateWarpedEvent(ev, width, height, warped_pt, image_warped, optsWarp))
      numAccumulation++;
  }

  // Smooth the IWE (to spread the votes)
  if (optsWarp.blur_sigma_ > 0)
  {
    cv::GaussianBlur(*image_warped, *image_warped,
                     cv::Size2d(optsWarp.blur_sigma_, optsWarp.blur_sigma_), optsWarp.blur_sigma_);
  }

  return numAccumulation;
}

size_t computeImageOfWarpedEvents_GMM(
  GeneralMotionModel& gmm,
  std::vector<dvs_msgs::Event> &pvEvent_involved,
  const double t_ref,
  const cv::Size &img_size,
  cv::Mat *image_warped,
  const OptionsWarp &optsWarp)
{
  const int width = img_size.width;
  const int height = img_size.height;

  // Create image of warped events (IWE)
  *image_warped = cv::Mat::zeros(height, width, CV_32FC1);

  // Loop through all events
  size_t numAccumulation = 0;
  for (const dvs_msgs::Event &ev : pvEvent_involved)
  {
    // Warp event according to candidate flow and accumulate on the IWE
    cv::Point2d warped_pt;
    warpEvent_GMM(gmm, ev, t_ref, &warped_pt);
    if(accumulateWarpedEvent(ev, width, height, warped_pt, image_warped, optsWarp))
      numAccumulation++;
  }

  // Smooth the IWE (to spread the votes)
  if (optsWarp.blur_sigma_ > 0)
  {
    cv::GaussianBlur(*image_warped, *image_warped,
                     cv::Size2d(optsWarp.blur_sigma_, optsWarp.blur_sigma_), optsWarp.blur_sigma_);
  }

  return numAccumulation;
}

size_t computeImageOfWarpedEvents_GMM_undistortion(
  GeneralMotionModel & gmm,
  std::vector<UndistortedEvent> &pvEvent_undistorted,
  const double t_ref,
  const cv::Size& img_size,
  cv::Mat* image_warped,
  const OptionsWarp& optsWarp
)
{
  const int width = img_size.width;
  const int height = img_size.height;

  // Create image of warped events (IWE)
  *image_warped = cv::Mat::zeros(height, width, CV_32FC1);

  // Loop through all events
  size_t numAccumulation = 0;
  for (const UndistortedEvent &uev : pvEvent_undistorted)
  {
    // Warp event according to candidate flow and accumulate on the IWE
    cv::Point2d warped_pt;
    warpEvent_GMM_undistortion(gmm, uev, t_ref, &warped_pt);
    if(accumulateWarpedEvent_Undistortion(width, height, warped_pt, image_warped, optsWarp))
      numAccumulation++;
  }

  // Smooth the IWE (to spread the votes)
  if (optsWarp.blur_sigma_ > 0)
  {
    cv::GaussianBlur(*image_warped, *image_warped,
                     cv::Size2d(optsWarp.blur_sigma_, optsWarp.blur_sigma_), optsWarp.blur_sigma_);
  }

  return numAccumulation;
}

}

}