#pragma once

#include<emsgc/core/image_warped_events.h>
#include<emsgc/core/event_motion_segmentation.h>
#include<emsgc/core/numerical_deriv.h>
#include<emsgc/tools/utils.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>

#include <glog/logging.h>

namespace emsgc
{
using namespace tools;
namespace core
{
inline double contrast_MeanSquare(const cv::Mat &image) //TODO: support mask operation
{
  // Compute mean square value of the image
  double contrast = cv::norm(image, cv::NORM_L2SQR) / static_cast<double>(image.rows * image.cols);
  VLOG(5) << "mean square = " << contrast;
  return contrast;
}

inline double contrast_Variance(const cv::Mat &image, const cv::Mat &mask)
{
  // Compute variance of the image
  cv::Mat mean, stddev;
  cv::meanStdDev(image, mean, stddev, mask);
  double contrast = stddev.at<double>(0,0) * stddev.at<double>(0,0);
  VLOG(5) << "mean = " << mean.at<double>(0,0) << "  std = " << stddev.at<double>(0,0);
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
    case MEAN_SQUARE_CONTRAST:
      contrast = contrast_MeanSquare(image);
      break;
    default:
      contrast = contrast_Variance(image, mask);
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
  // Extract Auxiliary data of cost function
  AuxdataBestFlow *pAdata = (AuxdataBestFlow *) adata;

  // Set motion model using current parameter vector ("operating point")
  MotionModel mm(
    gsl_vector_get(v, 0),
    gsl_vector_get(v, 1),
    gsl_vector_get(v, 2),
    gsl_vector_get(v, 3));

  // Compute cost
  cv::Mat iwe;
  computeImageOfWarpedEvents(mm, *(pAdata->pEvents_involved),
                            pAdata->t_ref, *(pAdata->pImg_size),
                            &iwe, pAdata->opts->opts_warp_);
  double contrast = computeContrast(iwe, *(pAdata->pMask), pAdata->opts->contrast_measure_);
  return -contrast;
}

inline double contrast_f_numerical_GMM(const gsl_vector *v, void *adata)
{
  // Extract Auxiliary data of cost function
  AuxdataBestFlow *pAdata = (AuxdataBestFlow *) adata;

  // Set motion model using current parameter vector ("operating point")
  GeneralMotionModel gmm;
  size_t num_dof = pAdata->num_dof;
  double* param_vec = new double[num_dof];
  for(size_t j=0; j < num_dof; ++j)
    param_vec[j] = gsl_vector_get(v, j);
  gmm.reset(num_dof, param_vec);

  // Compute cost
  cv::Mat iwe;
  computeImageOfWarpedEvents_GMM(gmm, *(pAdata->pEvents_involved),
                            pAdata->t_ref, *(pAdata->pImg_size),
                            &iwe, pAdata->opts->opts_warp_);
  double contrast = computeContrast(iwe, *(pAdata->pMask), pAdata->opts->contrast_measure_);
  delete[] param_vec;
  return -contrast;
}

inline double contrast_f_numerical_GMM_undistortion(const gsl_vector *v, void *adata)
{
  // Extract Auxiliary data of cost function
  AuxdataBestFlow *pAdata = (AuxdataBestFlow *) adata;

  // Set motion model using current parameter vector ("operating point")
  GeneralMotionModel gmm;
  size_t num_dof = pAdata->num_dof;
  double* param_vec = new double[num_dof];
  for(size_t j=0; j < num_dof; ++j)
    param_vec[j] = gsl_vector_get(v, j);
  gmm.reset(num_dof, param_vec);

  // Compute cost
  cv::Mat iwe;
  computeImageOfWarpedEvents_GMM_undistortion(gmm, *(pAdata->pUndistortedEvents),
                            pAdata->t_ref, *(pAdata->pImg_size),
                            &iwe, pAdata->opts->opts_warp_);
  double contrast = computeContrast(iwe, *(pAdata->pMask), pAdata->opts->contrast_measure_);
  VLOG(4) << "contrast_f_numerical_GMM_undistortion: contrast = " << contrast;
  delete[] param_vec;
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
