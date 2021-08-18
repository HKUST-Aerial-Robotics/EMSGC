#include <emsgc/container/PerspectiveCamera.h>
#include <opencv2/core/eigen.hpp>
#include <glog/logging.h>

namespace emsgc
{
namespace container
{
PerspectiveCamera::PerspectiveCamera() {}
PerspectiveCamera::~PerspectiveCamera() {}

void PerspectiveCamera::setIntrinsicParameters(
  size_t width,
  size_t height,
  std::string &cameraName,
  std::string &distortion_model,
  std::vector<double> &vD,
  std::vector<double> &vK)
{
  width_ = width;
  height_ = height;
  cameraName_ = cameraName;
  distortion_model_ = distortion_model;
  D_ = Eigen::Matrix<double,4,1>(vD.data());
  K_ = Eigen::Matrix<double,3,3,Eigen::RowMajor>(vK.data());
  preComputeUndistortedCoordinate();
}

void PerspectiveCamera::preComputeUndistortedCoordinate()
{
  precomputed_undistorted_points_ = Eigen::Matrix2Xd(2, height_ * width_);
  cv::Mat_<cv::Point2f> RawCoordinates(1, width_ * height_);
  for (int y = 0; y < height_; y++)
  {
    for (int x = 0; x < width_; x++)
    {
      int index = y * width_ + x;
      RawCoordinates(index) = cv::Point2f((float) x, (float) y);
    }
  }

  cv::Mat_<cv::Point2f> UndistortedCoordinates(1, height_ * width_);
  cv::Mat cvKmat(3, 3, CV_64F);
  cv::Mat cvDistCoeff(1, 4, CV_64F);
  cv::eigen2cv(K_, cvKmat);
  cv::eigen2cv(D_, cvDistCoeff);
  if (distortion_model_ == "plumb_bob" || distortion_model_ == "radtan")
  {
    cv::undistortPoints(RawCoordinates, UndistortedCoordinates, cvKmat, cvDistCoeff, cv::Mat::eye(3,3,CV_64F), cvKmat);
  }
  else
  {
    std::cout << "wrong distortion model is provided." << std::endl;
    exit(-1);
  }
//  LOG(INFO) << "newKmat: \n" << newKmat;
  for (size_t i = 0; i < height_ * width_; i++)
  {
    precomputed_undistorted_points_.col(i) = Eigen::Matrix<double, 2, 1>(
      UndistortedCoordinates(i).x,
      UndistortedCoordinates(i).y);
  }
}

Eigen::Matrix<double, 2, 1>
PerspectiveCamera::getUndistortedCoordinate(int xcoor, int ycoor)
{
  size_t index = ycoor * width_ + xcoor;
  return precomputed_undistorted_points_.block<2, 1>(0, index);
}

}
}