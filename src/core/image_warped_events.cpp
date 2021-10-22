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
  trans2D << model.m_x_, model.m_y_;
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
  const cv::Point2d& ev_pt,
  const double t_ev,
  const double t_ref,
  cv::Point2d *warped_pt)
{
  const double dt = t_ev - t_ref;

  // compute the optical flow velocity using the motion model
  if(gmm.mmType_ == TranslationModel)
  {
    // warp (the minus sign is because vel is defined from t_ref to t_k.
    warped_pt->x = ev_pt.x - gmm.parameters_[0]*dt;
    warped_pt->y = ev_pt.y - gmm.parameters_[1]*dt;
  }

  if(gmm.mmType_ == RotationModel)
  {
    Eigen::Vector3d w;
    w << gmm.parameters_[0], gmm.parameters_[1], gmm.parameters_[2];
    Eigen::Vector3d ev_3D;
    ev_3D << ev_pt.x, ev_pt.y, 1;
    Eigen::Vector3d ev_3D_rotated = ev_3D + dt * (w.cross(ev_3D));
    warped_pt->x = ev_3D_rotated(0) / ev_3D_rotated(2);
    warped_pt->y = ev_3D_rotated(1) / ev_3D_rotated(2);
  }

  if(gmm.mmType_ == AffineModel)
  {
    Eigen::Vector2d vel, trans2D;
    trans2D << gmm.parameters_[0], gmm.parameters_[1];
    Eigen::Matrix2d rotation;
    const double theta = gmm.parameters_[3];
    rotation << cos(theta), -sin(theta),
                sin(theta),  cos(theta);
    const double scale = gmm.parameters_[2];
    Eigen::Vector2d pt(ev_pt.x, ev_pt.y);
    vel = trans2D + (1 + scale) * rotation * pt - pt;

    // warp (the minus sign is because vel is defined from t_ref to t_k.
    Eigen::Vector2d displacement = dt * vel;
    warped_pt->x = ev_pt.x - displacement(0);
    warped_pt->y = ev_pt.y - displacement(1);
  }
}

bool accumulateWarpedEvent(
  const cv::Point2d& ev_warped_pt,
  const float polarity,
  cv::Mat* image_warped)
{
  // Accumulate warped events, using bi-linear voting (polarity or count)
  const int xx = ev_warped_pt.x,
            yy = ev_warped_pt.y;

  // if warped point is within the image, accumulate its contribution
  // to its four neighbouring pixels.
  if (1 <= xx && xx < image_warped->cols-2 && 1 <= yy && yy < image_warped->rows-2)
  {
    // Accumulate warped events on the IWE
    // image_warped (4 pixels)
    /* (xx,yy)   --- (xx+1,yy)   */
    /*    |              |       */
    /*    |              |       */
    /* (xx,yy+1) --- (xx+1,yy+1) */

    const float dx = ev_warped_pt.x - xx,
                dy = ev_warped_pt.y - yy;

    image_warped->at<float>(yy  ,xx  ) += polarity*(1.f-dx)*(1.f-dy);
    image_warped->at<float>(yy  ,xx+1) += polarity*      dx*(1.f-dy);
    image_warped->at<float>(yy+1,xx  ) += polarity*(1.f-dx)*dy;
    image_warped->at<float>(yy+1,xx+1) += polarity*      dx*dy;
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
  // Create image of warped events (IWE)
  *image_warped = cv::Mat::zeros(img_size, CV_32FC1);

  // Loop through all events
  size_t numAccumulation = 0;
  for (const dvs_msgs::Event &ev : pvEvent_involved)
  {
    // Warp event according to candidate motion...
    cv::Point2d ev_warped_pt;
    warpEvent(model, ev, t_ref, &ev_warped_pt);
    // ...and accumulate on the IWE
    const float polarity = (optsWarp.use_polarity_) ? 2.f * static_cast<float>(ev.polarity) - 1.f : 1.f;
    if( accumulateWarpedEvent(ev_warped_pt, polarity, image_warped) )
      numAccumulation++;
  }

  // Smooth the IWE (to spread the votes)
  if (optsWarp.blur_sigma_ > 0)
    cv::GaussianBlur(*image_warped, *image_warped, cv::Size2d(0,0), optsWarp.blur_sigma_);

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
  // Create image of warped events (IWE)
  *image_warped = cv::Mat::zeros(img_size, CV_32FC1);

  // Loop through all events
  size_t numAccumulation = 0;
  for (const dvs_msgs::Event &ev : pvEvent_involved)
  {
    // Warp event according to candidate motion...
    cv::Point2d ev_warped_pt;
    cv::Point2d ev_pt(ev.x,ev.y);
    warpEvent_GMM(gmm, ev_pt, ev.ts.toSec(), t_ref, &ev_warped_pt);
    // ...and accumulate on the IWE
    const float polarity = (optsWarp.use_polarity_) ? 2.f * static_cast<float>(ev.polarity) - 1.f : 1.f;
    if( accumulateWarpedEvent(ev_warped_pt, polarity, image_warped) )
      numAccumulation++;
  }

  // Smooth the IWE (to spread the votes)
  if (optsWarp.blur_sigma_ > 0)
    cv::GaussianBlur(*image_warped, *image_warped, cv::Size2d(0,0), optsWarp.blur_sigma_);

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
  // Create image of warped events (IWE)
  *image_warped = cv::Mat::zeros(img_size, CV_32FC1);

  // Loop through all events
  size_t numAccumulation = 0;
  for (const UndistortedEvent &ev : pvEvent_undistorted)
  {
    // Warp event according to candidate motion...
    cv::Point2d ev_warped_pt;
    cv::Point2d ev_pt(ev.x_,ev.y_);
    warpEvent_GMM(gmm, ev_pt, ev.ts_.toSec(), t_ref, &ev_warped_pt);
    // ...and accumulate on the IWE
    if( accumulateWarpedEvent(ev_warped_pt, 1.f, image_warped) )
      numAccumulation++;
  }

  // Smooth the IWE (to spread the votes)
  if (optsWarp.blur_sigma_ > 0)
    cv::GaussianBlur(*image_warped, *image_warped, cv::Size2d(0,0), optsWarp.blur_sigma_);

  return numAccumulation;
}

}

}
