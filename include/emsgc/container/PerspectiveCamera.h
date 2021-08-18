#ifndef EMSGC_CONTAINER_PERSPECTIVECAMERA_H
#define EMSGC_CONTAINER_PERSPECTIVECAMERA_H

#include <string>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>

namespace emsgc
{
namespace container
{
class PerspectiveCamera
{
  public:
  PerspectiveCamera();
  virtual ~PerspectiveCamera();

  using Ptr = std::shared_ptr<PerspectiveCamera>;

  void setIntrinsicParameters(
    size_t width, size_t height,
    std::string &cameraName,
    std::string &distortion_model,
    std::vector<double> &vD,
    std::vector<double> &vK);

  void preComputeUndistortedCoordinate();

  Eigen::Matrix<double, 2, 1> getUndistortedCoordinate(int xcoor, int ycoor);

  public:
  size_t width_, height_;
  std::string cameraName_;
  std::string distortion_model_;
  Eigen::Matrix<double, 4, 1> D_;
  Eigen::Matrix3d K_;
  Eigen::Matrix2Xd precomputed_undistorted_points_;
};
}
}

#endif //EMSGC_CONTAINER_PERSPECTIVECAMERA_H