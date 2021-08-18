#ifndef EMSGC_CORE_CONTRASTFUNCTOR_H
#define EMSGC_CORE_CONTRASTFUNCTOR_H

#include<emsgc/core/image_warped_events.h>
#include<emsgc/core/event_motion_segmentation.h>
#include<emsgc/core/numerical_deriv.h>
#include<emsgc/tools/utils.h>

#include <glog/logging.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>
//#include <gsl/gsl_blas.h>

#include <opencv2/highgui/highgui.hpp>

namespace emsgc
{
using namespace tools;
namespace core
{
inline double contrast_MeanSquare(const cv::Mat &image) //TODO: support mask operation
{
  // Compute mean square value of the image
  double contrast = cv::norm(image, cv::NORM_L2SQR) / static_cast<double>(image.rows * image.cols);
  //LOG(INFO) << "mean square = " << contrast;
  return contrast;
}

inline double contrast_Variance(const cv::Mat &image, const cv::Mat &mask)
{
  // Compute variance of the image
  double contrast;
  cv::Mat mean, stdVar;
  cv::meanStdDev(image, mean, stdVar, mask);
  contrast = stdVar.at<double>(0, 0) * stdVar.at<double>(0, 0);
  return contrast;
}

inline double computeContrast(
  const cv::Mat &image,
  const cv::Mat &mask,
  const int contrast_measure
)
{
  // Branch according to contrast measure
  double contrast;
  switch (contrast_measure)
  {
    case MEAN_SQUARE_CONTRAST:contrast = contrast_MeanSquare(image);
      break;
    default:contrast = contrast_Variance(image, mask);
      break;
  }

  return contrast;
}

/**
 * @brief Auxiliary data structure for optimization algorithm
 */
typedef struct
{
  std::vector<dvs_msgs::Event> *pEvents_involved;
  std::vector<UndistortedEvent> *pUndistortedEvents;
  double t_ref;
  cv::Size *pImg_size;
  cv::Mat *pMask;
  OptionsMethod *opts;
  size_t num_dof;
} AuxdataBestFlow; // TODO: change variables to be ada

/**
 * @brief Main function used by optimization algorithm.
 * Maximize contrast, or equivalently, minimize (-contrast)
 */
inline double contrast_f_numerical(const gsl_vector *v, void *adata)
{
  // Extract auxiliary data of cost function
  AuxdataBestFlow *pAux_data = (AuxdataBestFlow *) adata;

  // Parameter vector (from GSL to OpenCV)
  MotionModel mm(
    gsl_vector_get(v, 0),
    gsl_vector_get(v, 1),
    gsl_vector_get(v, 2),
    gsl_vector_get(v, 3));

  // Compute cost
  double contrast;
  cv::Mat image_warped;
  computeImageOfWarpedEvents(mm, *(pAux_data->pEvents_involved), pAux_data->t_ref, *(pAux_data->pImg_size),
                             &image_warped, pAux_data->opts->opts_warp_);
  contrast = computeContrast(image_warped, *(pAux_data->pMask), VARIANCE_CONTRAST);
  return -contrast;
}

inline double contrast_f_numerical_GMM(const gsl_vector *v, void *adata)
{
  // Extract auxiliary data of cost function
  AuxdataBestFlow *pAux_data = (AuxdataBestFlow *) adata;

  // Parameter vector (from GSL to OpenCV)
  GeneralMotionModel gmm;
  size_t numDoF = pAux_data->num_dof;
  if(numDoF == 2)
  {
    double data[2] = {gsl_vector_get(v, 0), gsl_vector_get(v, 1)};
    gmm.reset(numDoF, data);
  }
  if(numDoF == 3)
  {
    double data[3] = {gsl_vector_get(v, 0), gsl_vector_get(v, 1), gsl_vector_get(v, 2)};
    gmm.reset(numDoF, data);
  }
  if(numDoF == 4)
  {
    double data[4] = {gsl_vector_get(v, 0), gsl_vector_get(v, 1), gsl_vector_get(v, 2), gsl_vector_get(v, 3)};
    gmm.reset(numDoF, data);
  }

  // Compute cost
  double contrast;
  cv::Mat image_warped;
  computeImageOfWarpedEvents_GMM(gmm, *(pAux_data->pEvents_involved), pAux_data->t_ref, *(pAux_data->pImg_size),
                             &image_warped, pAux_data->opts->opts_warp_);
  contrast = computeContrast(image_warped, *(pAux_data->pMask), VARIANCE_CONTRAST);
  return -contrast;
}

inline double contrast_f_numerical_GMM_undistortion(const gsl_vector *v, void *adata)
{
  // Extract auxiliary data of cost function
  AuxdataBestFlow *pAux_data = (AuxdataBestFlow *) adata;

  // Parameter vector (from GSL to OpenCV)
  GeneralMotionModel gmm;
  size_t numDoF = pAux_data->num_dof;
  if(numDoF == 1)
  {
    double data[1] = {gsl_vector_get(v, 0)};
    gmm.reset(numDoF, data);
  }
  if(numDoF == 2)
  {
    double data[2] = {gsl_vector_get(v, 0), gsl_vector_get(v, 1)};
    gmm.reset(numDoF, data);
  }
  if(numDoF == 3)
  {
    double data[3] = {gsl_vector_get(v, 0), gsl_vector_get(v, 1), gsl_vector_get(v, 2)};
    gmm.reset(numDoF, data);
  }
  if(numDoF == 4)
  {
    double data[4] = {gsl_vector_get(v, 0), gsl_vector_get(v, 1), gsl_vector_get(v, 2), gsl_vector_get(v, 3)};
    gmm.reset(numDoF, data);
  }

  // Compute cost
  double contrast;
  cv::Mat image_warped;
  computeImageOfWarpedEvents_GMM_undistortion(gmm, *(pAux_data->pUndistortedEvents), pAux_data->t_ref, *(pAux_data->pImg_size),
                                 &image_warped, pAux_data->opts->opts_warp_);
  contrast = computeContrast(image_warped, *(pAux_data->pMask), VARIANCE_CONTRAST);
  return -contrast;
}

inline void contrast_fdf_numerical(const gsl_vector *v, void *adata, double *f, gsl_vector *df)
{
  // Finite difference approximation
  *f = vs_gsl_Gradient_ForwardDiff(v, adata, contrast_f_numerical, df, 1e0);
}

inline void contrast_fdf_numerical_GMM(const gsl_vector *v, void *adata, double *f, gsl_vector *df)
{
  // Finite difference approximation
  *f = vs_gsl_Gradient_ForwardDiff(v, adata, contrast_f_numerical_GMM, df, 1e0);
}

inline void contrast_fdf_numerical_GMM_undistortion(const gsl_vector *v, void *adata, double *f, gsl_vector *df)
{
  // Finite difference approximation
  *f = vs_gsl_Gradient_ForwardDiff(v, adata, contrast_f_numerical_GMM_undistortion, df, 1e0);
}

inline void contrast_df_numerical(const gsl_vector *v, void *adata, gsl_vector *df)
{
  double cost;
  contrast_fdf_numerical(v, adata, &cost, df);
}

inline void contrast_df_numerical_GMM(const gsl_vector *v, void *adata, gsl_vector *df)
{
  double cost;
  contrast_fdf_numerical_GMM(v, adata, &cost, df);
}

inline void contrast_df_numerical_GMM_undistortion(const gsl_vector *v, void *adata, gsl_vector *df)
{
  double cost;
  contrast_fdf_numerical_GMM_undistortion(v, adata, &cost, df);
}

}
}
#endif //EMSGC_CORE_CONTRASTFUNCTOR_H
