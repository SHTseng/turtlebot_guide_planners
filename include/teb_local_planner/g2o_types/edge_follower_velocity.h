#ifndef EDGE_FOLLOWER_VELOCITY_H
#define EDGE_FOLLOWER_VELOCITY_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>

#include <geometry_msgs/Twist.h>

namespace teb_local_planner
{


/**
 * @class EdgeFollowerVelocity
 * @brief Edge defining the cost function for limiting the translational and rotational velocity.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [v,omega]^T ) \cdot weight \f$. \n
 * \e v is calculated using the difference quotient and the position parts of both poses. \n
 * \e omega is calculated using the difference quotient of both yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational velocity and
 * the second one the rotational velocity.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeFollowerVelocity : public BaseTebMultiEdge<2, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeFollowerVelocity()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocity()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

    const Eigen::Vector2d deltaS = conf2->estimate().position() - conf1->estimate().position();

    double dist = deltaS.norm();
    const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
    if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
    {
        double radius =  dist/(2*sin(angle_diff/2));
        dist = fabs( angle_diff * radius ); // actual arg length!
    }
    double vel = dist / deltaT->estimate();

//     vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) + deltaS[1]*sin(conf1->theta())); // consider direction
//    vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction

    const double omega = angle_diff / deltaT->estimate();

    _error[0] = penaltyBoundToInterval(vel, follower_pose_.position().norm(), cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
  }

  void setFollowerVelocity(const geometry_msgs::Twist& follower_vel)
  {
    follower_pose_.position() << follower_vel.linear.x, follower_vel.linear.y;
    follower_pose_.theta() = follower_vel.angular.z;
    ROS_INFO_STREAM(follower_vel.linear.x << " " << follower_vel.linear.y);
  }

protected:

  PoseSE2 follower_pose_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace

#endif // EDGE_FOLLOWER_VELOCITY_H
