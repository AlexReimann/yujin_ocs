#include "yocs_diff_drive_pose_controller/controller.hpp"

namespace yocs
{

bool DiffDrivePoseController::init()
{
  enable_controller_subscriber_ = nh_.subscribe("enable", 10, &DiffDrivePoseController::enableCB, this);
  disable_controller_subscriber_ = nh_.subscribe("disable", 10, &DiffDrivePoseController::disableCB, this);
  control_velocity_subscriber_ = nh_.subscribe("control_max_vel", 10, &DiffDrivePoseController::controlMaxVelCB, this);
  command_velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("command_velocity", 10);
  pose_reached_publisher_ = nh_.advertise<std_msgs::Bool>("pose_reached", 10);

  // retrieve configuration parameters
  base_frame_name_ = "base_footprint";
  if(!nh_.getParam("base_frame_name", base_frame_name_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'base_frame_name' from parameter server! Using default '"
                    << base_frame_name_ << "'. [" << name_ <<"]");
  }
  goal_frame_name_ = "base_goal_pose";
  if(!nh_.getParam("goal_frame_name", goal_frame_name_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'goal_frame_name' from parameter server! Using default '"
                    << goal_frame_name_ << "'. [" << name_ <<"]");
  }
  v_ = 0.0;
  v_min_ = 0.01;
  if(!nh_.getParam("v_min", v_min_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'v_min' from parameter server! Using default '"
                    << v_min_ << "'. [" << name_ <<"]");
  }
  v_max_ = 0.5;
  if(!nh_.getParam("v_max", v_max_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'v_max' from parameter server! Using default '"
                    << v_max_ << "'. [" << name_ <<"]");
  }
  w_min_ = 0.01;
  if(!nh_.getParam("w_min", w_min_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'w_min' from parameter server! Using default '"
                    << w_min_ << "'. [" << name_ <<"]");
  }
  w_max_ = M_PI / 4 * v_max_;
  if(!nh_.getParam("w_max", w_max_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'w_max' from parameter server! Using default '"
                    << w_max_ << "'. [" << name_ <<"]");
  }
  k_1_ = 1.0;
  if(!nh_.getParam("k_1", k_1_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'k_1' from parameter server! Using default '"
                    << k_1_ << "'. [" << name_ <<"]");
  }
  k_2_ = 3.0;
  if(!nh_.getParam("k_2", k_2_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'k_2' from parameter server! Using default '"
                    << k_2_ << "'. [" << name_ <<"]");
  }
  beta_ = 0.4;
  if(!nh_.getParam("beta", beta_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'beta' from parameter server! Using default '"
                    << beta_ << "'. [" << name_ <<"]");
  }
  lambda_ = 2.0;
  if(!nh_.getParam("lambda", lambda_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'lambda' from parameter server! Using default '"
                    << lambda_ << "'. [" << name_ <<"]");
  }
  dist_thres_ = 0.01;
  if(!nh_.getParam("dist_thres", dist_thres_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'dist_thres' from parameter server! Using default '"
                    << dist_thres_ << "'. [" << name_ <<"]");
  }
  orient_thres_ = 0.02;
  if(!nh_.getParam("orient_thres", orient_thres_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'orient_thres' from parameter server! Using default '"
                    << orient_thres_ << "'. [" << name_ <<"]");
  }
  dist_eps_ = dist_eps_ * 0.2;
  if(!nh_.getParam("dist_eps", dist_eps_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'dist_eps' from parameter server! Using default '"
                    << dist_eps_ << "'. [" << name_ <<"]");
  }
  orient_eps_ = orient_thres_ * 0.2;
  if(!nh_.getParam("orient_eps", orient_eps_))
  {
    ROS_WARN_STREAM("Couldn't retrieve parameter 'orient_eps' from parameter server! Using default '"
                    << orient_eps_ << "'. [" << name_ <<"]");
  }
  pose_reached_ = false;
  ROS_DEBUG_STREAM("Controller initialised with the following parameters: [" << name_ <<"]");
  ROS_DEBUG_STREAM("base_frame_name = " << base_frame_name_ <<", goal_frame_name = "
                   << goal_frame_name_ << " [" << name_ <<"]");
  ROS_DEBUG_STREAM("v_max = " << v_max_ <<", k_1 = " << k_1_ << ", k_2 = " << k_2_ << ", beta = " << beta_
                   << ", lambda = " << lambda_ << ", dist_thres = " << dist_thres_
                   << ", orient_thres = " << orient_thres_ <<" [" << name_ <<"]");
  return true;
};

void DiffDrivePoseController::spinOnce()
{
  if (this->getState())
  {
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Controller spinning. [" << name_ <<"]");
    // determine pose difference in polar coordinates
    if (!getPoseDiff())
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Getting pose difference failed. Skipping control loop. [" << name_ <<"]");
      return;
    }
    // determine controller output (v, w)
    getControlOutput();
    // set control output (v, w)
    setControlOutput();
    // Logging
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Current state: [" << name_ <<"]");
    ROS_DEBUG_STREAM_THROTTLE(1.0, "r = " << r_ << ", theta = " << theta_ << ", delta = " << delta_
                                   << " [" << name_ <<"]");
    ROS_DEBUG_STREAM_THROTTLE(1.0, "cur = " << cur_ << ", v = " << v_ << ", w = " << w_ << " [" << name_ <<"]");
  }
  else
  {
    ROS_DEBUG_STREAM_THROTTLE(3.0, "Controller is disabled. Idling ... [" << name_ <<"]");
  }
};

bool DiffDrivePoseController::getPoseDiff()
{
  // use tf to get information about the goal pose relative to the base
  try
  {
    tf_listener_.lookupTransform(base_frame_name_, goal_frame_name_, ros::Time(0), tf_goal_pose_rel_);
  }
  catch (tf::TransformException const &ex)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Couldn't get transform from base to goal pose! [" << name_ <<"]");
    ROS_DEBUG_STREAM_THROTTLE(1.0, "tf error: " << ex.what());
    return false;
  }

  // determine distance to goal
  r_ = std::sqrt(std::pow(tf_goal_pose_rel_.getOrigin().getX(), 2)
                 + std::pow(tf_goal_pose_rel_.getOrigin().getY(), 2));
  // determine orientation of r relative to the base frame
  delta_ = std::atan2(-tf_goal_pose_rel_.getOrigin().getY(), tf_goal_pose_rel_.getOrigin().getX());
  // determine orientation of r relative to the goal frame
  // helper: theta = tf's orientation + delta
  theta_ = tf::getYaw(tf_goal_pose_rel_.getRotation()) + delta_;

  return true;
};

void DiffDrivePoseController::getControlOutput()
{
  cur_ = (-1 / r_) * (k_2_ * (delta_ - std::atan(-k_1_ * theta_))
         + (1 + (k_1_ / (1 + std::pow((k_1_ * theta_), 2)))) * sin(delta_));
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
      ROS_INFO_STREAM("Pose reached. [" << name_ <<"]");
      std_msgs::Bool bool_msg;
      bool_msg.data = true;
      pose_reached_publisher_.publish(bool_msg);
    }
  }
  else if ((r_ > (dist_thres_ + dist_eps_)) || (std::abs(delta_ - theta_) > (orient_thres_ + orient_eps_)))
  {
    if (pose_reached_)
    {
      pose_reached_ = false;
      ROS_INFO_STREAM("Tracking new goal pose. [" << name_ <<"]");
    }
  }
};

void DiffDrivePoseController::setControlOutput()
{
  geometry_msgs::TwistPtr cmd_vel(new geometry_msgs::Twist());
  if (!pose_reached_)
  {
    cmd_vel->linear.x = v_;
    cmd_vel->angular.z = w_;
  }
  command_velocity_publisher_.publish(cmd_vel);
};

void DiffDrivePoseController::controlMaxVelCB(const std_msgs::Float32ConstPtr msg)
{
  v_max_ = msg->data;
  ROS_INFO_STREAM("Maximum linear control velocity has been set to " << v_max_ << ". [" << name_ << "]");
};

void DiffDrivePoseController::enableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->enable())
  {
    ROS_INFO_STREAM("Controller has been enabled. [" << name_ << "]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already enabled. [" << name_ <<"]");
  }
};

void DiffDrivePoseController::disableCB(const std_msgs::EmptyConstPtr msg)
{
  if (this->disable())
  {
    ROS_INFO_STREAM("Controller has been disabled. [" << name_ <<"]");
  }
  else
  {
    ROS_INFO_STREAM("Controller was already disabled. [" << name_ <<"]");
  }
};

} // namespace yocs
