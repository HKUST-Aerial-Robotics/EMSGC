#include<emsgc/core/image_warped_events.h>
#include<emsgc/core/event_motion_segmentation.h>
#include<emsgc/core/numerical_deriv.h>
#include<emsgc/core/contrastFunctor.h>
#include<emsgc/tools/utils.h>

#include <glog/logging.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>

#include <opencv2/highgui/highgui.hpp>

namespace emsgc
{
using namespace tools;
namespace core
{
void EventMotionSegmentation::findBestGmmInRangeBruteForce_2D_FromeUndistortedEvents(
  Eigen::Vector2d& trans2D_range,
  double trans2D_step,
  std::vector<UndistortedEvent>& vEvents,
  cv::Mat& mask,
  ros::Time& t_ref,
  GeneralMotionModel & gmm)
{
  //Auxiliary data for the cost function
  AuxdataBestFlow oAuxdata;
  oAuxdata.pUndistortedEvents = &vEvents;
  oAuxdata.t_ref = t_ref.toSec();
  oAuxdata.pImg_size = &img_size_;
  oAuxdata.pMask = &mask;
  oAuxdata.opts = &opts_;
  oAuxdata.num_dof = 2;

  gsl_vector *gmm_gsl = gsl_vector_alloc (oAuxdata.num_dof);

  // Brute-force search
  const double min_mx = trans2D_range(0), max_mx = trans2D_range(1);
  const double min_my = trans2D_range(0), max_my = trans2D_range(1);
  const double step_mx = trans2D_step, step_my = trans2D_step;

  VLOG(1) << "Trans2D parameters search:";
  VLOG(1) << "mx: " << min_mx << " -> " << max_mx << " step: " << step_mx;
  VLOG(1) << "my: " << min_my << " -> " << max_my << " step: " << step_my;

  // brute-force search in the trans2D dimensions.
  double minimum_cost = 0.;
  double opt_mx = 0., opt_my = 0.;
  for(double mx = min_mx; mx <= max_mx; mx += step_mx)
  {
    for(double my = min_my; my <= max_my; my += step_my)
    {
      gsl_vector_set(gmm_gsl, 0, mx);
      gsl_vector_set(gmm_gsl, 1, my);

      const double cost = contrast_f_numerical_GMM_undistortion(gmm_gsl, &oAuxdata);

      VLOG(2) << "gmm (2D): [" << mx << ", " << my << "], cost: " << cost;

      if(cost < minimum_cost)
      {
        minimum_cost = cost;
        opt_mx = mx;
        opt_my = my;
      }
    }
  }
  VLOG(1) << "Best trans2D parameters: " << opt_mx << " , " << opt_my;

  gmm.parameters_.clear();
  gmm.parameters_.push_back(opt_mx);
  gmm.parameters_.push_back(opt_my);
}

void EventMotionSegmentation::findBestGmmInRangeBruteForce_3D_FromeUndistortedEvents(
  Eigen::Matrix<double,6,1>& angular3D_range,
  double angular3D_step,
  std::vector<UndistortedEvent>& vEvents,
  cv::Mat& mask,
  ros::Time& t_ref,
  GeneralMotionModel & gmm)
{
  //Auxiliary data for the cost function
  AuxdataBestFlow oAuxdata;
  oAuxdata.pUndistortedEvents = &vEvents;
  oAuxdata.t_ref = t_ref.toSec();
  oAuxdata.pImg_size = &img_size_;
  oAuxdata.pMask = &mask;
  oAuxdata.opts = &opts_;
  oAuxdata.num_dof = 3;

  gsl_vector *gmm_gsl = gsl_vector_alloc (oAuxdata.num_dof);

  // Brute-force search
  const double min_w1 = angular3D_range(0), max_w1 = angular3D_range(1);
  const double min_w2 = angular3D_range(2), max_w2 = angular3D_range(3);
  const double min_w3 = angular3D_range(4), max_w3 = angular3D_range(5);
  const double step_w = angular3D_step;

  VLOG(1) << "Angular3D parameters search:";
  VLOG(1) << "w1: " << min_w1 << " -> " << max_w1 << " stepw1: " << step_w;
  VLOG(1) << "w2: " << min_w2 << " -> " << max_w2 << " stepw2: " << step_w;
  VLOG(1) << "w3: " << min_w3 << " -> " << max_w3 << " stepw3: " << step_w;

  // brute-force search in the trans2D dimensions.
  double minimum_cost = 0.;
  double opt_w1 = 0., opt_w2 = 0., opt_w3 = 0.;
  for(double w1 = min_w1; w1 <= max_w1; w1 += step_w)
  {
    for(double w2 = min_w2; w2 <= max_w2; w2 += step_w)
    {
      for(double w3 = min_w3; w3 <= max_w3; w3 += step_w)
      {
        gsl_vector_set(gmm_gsl, 0, w1);
        gsl_vector_set(gmm_gsl, 1, w2);
        gsl_vector_set(gmm_gsl, 2, w3);

        const double cost = contrast_f_numerical_GMM_undistortion(gmm_gsl, &oAuxdata);

        VLOG(2) << "gmm (3D): [" << w1 << ", " << w2 << ", " << w3 << "], cost: " << cost;

        if(cost < minimum_cost)
        {
          minimum_cost = cost;
          opt_w1 = w1;
          opt_w2 = w2;
          opt_w3 = w3;
        }
      }
    }
  }
  VLOG(1) << "Best Angular3D parameters: " << opt_w1 << " , " << opt_w2 << " , " << opt_w3;

  gmm.parameters_.clear();
  gmm.parameters_.push_back(opt_w1);
  gmm.parameters_.push_back(opt_w2);
  gmm.parameters_.push_back(opt_w3);
}

void EventMotionSegmentation::findBestGmmInRangeBruteForce_4D_FromeUndistortedEvents(
  Eigen::Vector4d& trans2D_range,
  Eigen::Vector2d& trans2D_step,
  Eigen::Vector2d& scale_range,
  double scale_step,
  Eigen::Vector2d& theta_range,
  double theta_step,
  std::vector<UndistortedEvent>& vUndistEvents,
  cv::Mat& mask,
  ros::Time& t_ref,
  GeneralMotionModel & gmm)
{
  //Auxiliary data for the cost function
  AuxdataBestFlow oAuxdata;
  oAuxdata.pUndistortedEvents = &vUndistEvents;
  oAuxdata.t_ref = t_ref.toSec();
  oAuxdata.pImg_size = &img_size_;
  oAuxdata.pMask = &mask;
  oAuxdata.opts = &opts_;
  oAuxdata.num_dof = 4;

  gsl_vector *gmm_gsl = gsl_vector_alloc (oAuxdata.num_dof);

  // Brute-force search
  const double min_mx = trans2D_range(0), max_mx = trans2D_range(1),
               min_my = trans2D_range(2), max_my = trans2D_range(3);
  const double step_mx = trans2D_step(0), step_my = trans2D_step(1);
  const double min_ms = scale_range(0), max_ms = scale_range(1);
  const double min_mTheta = theta_range(0), max_mTheta = theta_range(1);

  VLOG(1) << "Affine4D parameters search:";
  VLOG(1) << "mx: " << min_mx << " -> " << max_mx << " step: " << step_mx;
  VLOG(1) << "my: " << min_my << " -> " << max_my << " step: " << step_my;
  VLOG(1) << "m_scale: " << min_ms << " -> " << max_ms << " step: " << scale_step;
  VLOG(1) << "m_theta: " << min_mTheta << " -> " << max_mTheta << " step: " << theta_step;

  // 1. brute-force search in the trans2D dimensions.
  double minimum_cost = 0.;
  double opt_mx = 0., opt_my = 0., opt_scale = 0., opt_theta = 0.;
  for(double mx = min_mx; mx <= max_mx; mx += step_mx)
  {
    for(double my = min_my; my <= max_my; my += step_my)
    {
      for(double m_scale = min_ms; m_scale <= max_ms; m_scale += scale_step)
      {
        for (double m_theta = min_mTheta; m_theta <= max_mTheta; m_theta += theta_step)
        {
          gsl_vector_set(gmm_gsl, 0, mx);
          gsl_vector_set(gmm_gsl, 1, my);
          gsl_vector_set(gmm_gsl, 2, m_scale); // scale
          gsl_vector_set(gmm_gsl, 3, m_theta); // theta

          const double cost = contrast_f_numerical_GMM_undistortion(gmm_gsl, &oAuxdata);

          VLOG(2) << "gmm (4D): [" << mx << ", " << my << ", " << m_scale << ", " << m_theta << "], cost: " << cost;

          if(cost < minimum_cost)
          {
            minimum_cost = cost;
            opt_mx = mx;
            opt_my = my;
            opt_scale = m_scale;
            opt_theta = m_theta;
          }
        }
      }
    }
  }
  VLOG(1) << "Best Affine 4D parameters: " << opt_mx << " , " << opt_my
          << " " << opt_scale << " , " << opt_theta;

  gmm.parameters_.clear();
  gmm.parameters_.push_back(opt_mx);
  gmm.parameters_.push_back(opt_my);
  gmm.parameters_.push_back(opt_scale);
  gmm.parameters_.push_back(opt_theta);
}

double EventMotionSegmentation::gmmFittingUndistortion(
  std::vector<UndistortedEvent>& vUndistEvents,
  MotionModelType mmType,
  cv::Mat& mask,
  ros::Time& t_ref,
  GeneralMotionModel& gmm,
  double init_step_size,
  double tolerance)
{
  //Solver/minimizer type (algorithm):
  const gsl_multimin_fdfminimizer_type *solver_type;
  solver_type = gsl_multimin_fdfminimizer_conjugate_fr; // Fletcher-Reeves conjugate gradient algorithm

  //Routines to compute the cost function and its derivatives
  gsl_multimin_function_fdf solver_info;
  int numDoF = 0;
  switch (mmType)
  {
    case TranslationModel:
    {
      numDoF = 2;
      break;
    }
    case RotationModel:
    {
      numDoF = 3;
      break;
    }
    case AffineModel:
    {
      numDoF = 4;
      break;
    }
    default:
    {
      LOG(INFO) << "The given mmType is not supported in this exemplary release.";
      exit(-1);
    }
  }

  //Auxiliary data for the cost function
  AuxdataBestFlow oAuxdata;
  oAuxdata.pUndistortedEvents = &vUndistEvents;
  oAuxdata.t_ref = t_ref.toSec();
  oAuxdata.pImg_size = &img_size_;
  oAuxdata.pMask = &mask;
  oAuxdata.opts = &opts_;
  oAuxdata.num_dof = numDoF;

  const int num_params = numDoF;
  solver_info.n = num_params; // Size of the parameter vector
  solver_info.f = contrast_f_numerical_GMM_undistortion; // Cost function
  solver_info.df = contrast_df_numerical_GMM_undistortion; // Gradient of cost function
  solver_info.fdf = contrast_fdf_numerical_GMM_undistortion; // Cost and gradient functions
  solver_info.params = &oAuxdata; // Auxiliary data

  //Initial parameter vector
  gsl_vector *vx = gsl_vector_alloc (num_params);
  if (!(2<= num_params && num_params <=4))
  {
      LOG(INFO) << "The num_params: " << num_params << " is not defined !!!!!!!!!!!!";
      exit(-1);
  }
  for (int j=0; j <num_params; ++j){ gsl_vector_set(vx, j, gmm.parameters_[j]); }

  //Initialize solver
  gsl_multimin_fdfminimizer *solver = gsl_multimin_fdfminimizer_alloc (solver_type, num_params);
  const double initial_step_size = init_step_size; // TODO: ask GG
  double tol = tolerance; // TODO: ask GG

  gsl_multimin_fdfminimizer_set (solver, &solver_info, vx, initial_step_size, tol);

  const double initial_cost = solver->f;

  //ITERATE
  const int num_max_line_searches = 50;
  int status;
  const double epsabs_grad = 1e-3, tolfun = 1e-2;
  double cost_new = 1e9, cost_old = 1e9;
  size_t iter = 0;
  VLOG(2) << "Optimization. Solver type = " << solver_type->name;
  if (VLOG_IS_ON(2))
  {
    stringstream res;
    for (int j=0; j <num_params; ++j){ res << gsl_vector_get(solver->x, j) << " "; }
    VLOG(2) << "iter=" << std::setw(3) << iter << " vec =["
            << res.str() << "]  cost=" << std::setprecision(8) << solver->f;
  }

  do
  {
    iter++;
    cost_old = cost_new;
    status = gsl_multimin_fdfminimizer_iterate (solver);
    //status == GLS_SUCCESS (0) means that the iteration reduced the function value

    if (VLOG_IS_ON(2))
    {
      stringstream res;
      for (int j=0; j <num_params; ++j){ res << gsl_vector_get(solver->x, j) << " "; }
      VLOG(2) << "iter=" << std::setw(3) << iter << " vec =["
              << res.str() << "]  cost=" << std::setprecision(8) << solver->f;
    }

    if (status == GSL_SUCCESS)
    {
      //Test convergence due to stagnation in the value of the function
      cost_new = gsl_multimin_fdfminimizer_minimum(solver);
      if ( fabs( 1-cost_new/(cost_old+1e-7) ) < tolfun )
      {
        VLOG(3) << "progress tolerance reached.";
        break;
      }
      else
        status = GSL_CONTINUE;
    }

    //Test convergence due to absolute norm of the gradient
    if (GSL_SUCCESS == gsl_multimin_test_gradient (solver->gradient, epsabs_grad))
    {
      VLOG(3) << "gradient tolerance reached.";
      break;
    }

    if (status != GSL_CONTINUE)
    {
      // The iteration was not successful (did not reduce the function value)
      VLOG(3) << "stopped iteration; status = " << status;
      VLOG_IF(3, (GSL_ENOPROG == status)) << "iteration is not making progress towards solution";
      break;
    }
  }
  while (status == GSL_CONTINUE && iter < num_max_line_searches);

  //SAVE RESULTS (best motion parameters)
  //Convert from GSL to output format
  gsl_vector *final_x = gsl_multimin_fdfminimizer_x(solver);
  for (int j=0; j <num_params; ++j){ gmm.parameters_[j] = final_x->data[j]; }
  const double final_cost = gsl_multimin_fdfminimizer_minimum(solver);

  VLOG(1) << "--- Initial cost = " << std::fixed << std::setprecision(8) << initial_cost;
  VLOG(1) << "--- Final cost   = " << std::fixed << std::setprecision(8) << final_cost;

  //Release memory used during optimization
  gsl_multimin_fdfminimizer_free (solver);
  gsl_vector_free (vx);

  return final_cost;
}

}
}
