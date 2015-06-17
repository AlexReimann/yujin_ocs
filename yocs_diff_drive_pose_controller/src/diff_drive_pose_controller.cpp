#include "yocs_diff_drive_pose_controller/diff_drive_pose_controller.hpp"

#include <cmath>
#include <ros/console.h>

namespace yocs
{

DiffDrivePoseController::DiffDrivePoseController(std::string name, double v_max, double w_max, double dist_thres,
                                                 double orient_thres, double dist_eps, double orient_eps, double k_1,
                                                 double k_2, double beta, double lambda, double v_min, double w_min)
{
  name_ = name;
  r_ = 0.0;
  v_ = 0.0;
  w_ = 0.0;
  delta_ = 0.0;
  theta_ = 0.0;
  cur_ = 0.0;
  pose_reached_ = false;

  v_min_ = v_min;
  v_max_ = v_max;
  w_min_ = w_min;
  w_max_ = w_max;

  dist_thres_ = dist_thres;
  orient_thres_ = orient_thres;
  dist_eps_ = dist_eps;
  orient_eps_ = orient_eps;

  k_1_ = k_1;
  k_2_ = k_2;
  beta_ = beta;
  lambda_ = lambda;
}

void DiffDrivePoseController::setInput(double distance_to_goal, double delta, double theta)
{
  r_ = distance_to_goal;
  delta_ = delta;
  theta_ = theta;
}

void DiffDrivePoseController::spinOnce()
{
  calculateControls();
}

void DiffDrivePoseController::calculateControls()
{
  cur_ = (-1 / r_)
      * (k_2_ * (delta_ - std::atan(-k_1_ * theta_)) + (1 + (k_1_ / (1 + std::pow((k_1_ * theta_), 2)))) * sin(delta_));
  v_ = v_max_ / (1 + beta_ * std::pow(std::abs(cur_), lambda_));

  // bounds for v
  if (v_ < 0.0)
  {
    if (v_ > -v_min_)
    {
      v_ = -v_min_;
    }
    else if (v_ < -v_max_)
    {
      v_ = -v_max_;
    }
  }
  else
  {
    if (v_ < v_min_)
    {
      v_ = v_min_;
    }
    else if (v_ > v_max_)
    {
      v_ = v_max_;
    }
  }

  w_ = cur_ * v_; // unbounded for now

  // pose reached thresholds
  if (r_ <= dist_thres_)
  {
    v_ = 0;
    if (std::abs(delta_ - theta_) <= orient_thres_)
    {
      w_ = 0;
    }
  }

  // check, if pose has been reached
  if ((r_ <= dist_thres_) && (std::abs(delta_ - theta_) <= orient_thres_))
  {
    if (!pose_reached_)
    {
      pose_reached_ = true;
      ROS_INFO_STREAM("Pose reached. [" << name_ << "]");
    }
  }
  else if ((r_ > (dist_thres_ + dist_eps_)) || (std::abs(delta_ - theta_) > (orient_thres_ + orient_eps_)))
  {
    if (pose_reached_)
    {
      pose_reached_ = false;
      ROS_INFO_STREAM("Tracking new goal pose. [" << name_ << "]");
    }
  }
}

void DiffDrivePoseController::getControlOutput(double& v, double& w)
{
  v = v_;
  w = w_;
}

bool DiffDrivePoseController::goalIsReached()
{
  return pose_reached_;
}

} /* end namespace */
