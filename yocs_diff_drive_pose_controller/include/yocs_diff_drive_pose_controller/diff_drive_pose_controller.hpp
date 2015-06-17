#ifndef YOCS_DIFF_DRIVE_POSE_CONTROLLER_HPP_
#define YOCS_DIFF_DRIVE_POSE_CONTROLLER_HPP_

#include <string>
#include <yocs_controllers/default_controller.hpp>

namespace yocs
{

/**
 * @brief A controller for driving a differential drive base to a pose goal
 *         or along a path specified by multiple poses.
 *
 * This controller implements a control law drives a differental drive base towards a planar pose goal,
 * i.e. 2D position (x,y) + 1D orientation (theta). It also allows path following by specifying multiple pose goals.
 * The control law contains a transition strategy, which insures that the base moves through each pose and transitions
 * smoothly to the next pose goal.
 *
 * This controller is an implementation of control law based on the following work:
 * @inproceedings{DBLP:conf/icra/ParkK11,
 *   author    = {Jong Jin Park and
 *                Benjamin Kuipers},
 *   title     = {A smooth control law for graceful motion of differential
 *                wheeled mobile robots in 2D environment},
 *   booktitle = {ICRA},
 *   year      = {2011},
 *   pages     = {4896-4902},
 *   ee        = {http://dx.doi.org/10.1109/ICRA.2011.5980167},
 *   crossref  = {DBLP:conf/icra/2011},
 *   bibsource = {DBLP, http://dblp.uni-trier.de}
 * }
 *
 * This controller can be enabled/disabled.
 */
class DiffDrivePoseController : public Controller
{
public:
  DiffDrivePoseController(std::string name, double v_max, double w_max, double dist_thres = 0.01, double orient_thres =
                              0.02,
                          double dist_eps = 0.01 * 0.2, double orient_eps = 0.02 * 0.2, double k_1 = 1.0, double k_2 =
                              3.0,
                          double beta = 0.4, double lambda = 2.0, double v_min = 0.01, double w_min = 0.01);

  virtual ~DiffDrivePoseController()
  {

  }

  /**
   * @brief unused, overwrite if inherited (and needed)
   * @return true
   */
  virtual bool init()
  {
    return true;
  }

  /**
   * @brief Set input of controller. Should be called before each spinOnce
   * @param distance_to_goal distance to goal [m]
   * @param delta heading of the robot [rad]
   * @param theta angle difference between heading and goal [m]
   */
  virtual void setInput(double distance_to_goal, double delta, double theta);

  /**
   * @brief Execute one controller step
   */
  virtual void spinOnce();

  /**
   * @brief Get controller result / output after spinning
   * @param v linear velocity out variable
   * @param w angular velocity out variable
   */
  virtual void getControlOutput(double& v, double& w);

  virtual bool goalIsReached();

protected:
  /**
   * @brief Calculates the controls with the set variables (speed, goal etc.)
   */
  virtual void calculateControls();

protected:
  std::string name_;

  // variables and constants for the control law
  /// distance to pose goal [m]
  double r_;
  /// direction of the pose goal [rad]
  double delta_;
  /// linear base velocity [m/s]
  double theta_;
  /// current heading of the base [rad]
  double v_;
  /// minimum linear base velocity [m/s]
  double v_min_;
  /// maximum linear base velocity [m/s]
  double v_max_;
  /// angular base velocity [rad/s]
  double w_;
  /// minimum angular base velocity [rad/s]
  double w_min_;
  /// maximum angular base velocity [rad/s]
  double w_max_;
  /// path to goal curvature
  double cur_;
  /// constant factor determining the ratio of the rate of change in theta to the rate of change in r
  double k_1_;
  /// constant factor applied to the heading error feedback
  double k_2_;
  /**
   * constant factor for the curvature-based velocity rule
   * determines how fast the velocity drops when the curvature increases
   */
  double beta_;
  /**
   * constant factor for the curvature-based velocity rule
   * determines the sharpness of the curve: higher lambda -> bigger drop in short term, smaller in the long term
   */
  double lambda_;
  /// lower bound for the distance (v = 0)
  double dist_thres_;
  /// lower bound for the orientation (w = 0)
  double orient_thres_;
  /// True, if pose has been reached (v == 0, w == 0)
  bool pose_reached_;
  /// Error in distance above which pose is considered different
  double dist_eps_;
  /// Error in orientation above which pose is considered different
  double orient_eps_;
};

} /* end namespace */

#endif
