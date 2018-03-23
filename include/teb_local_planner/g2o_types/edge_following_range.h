#ifndef EDGE_FOLLOWING_RANGE_H
#define EDGE_FOLLOWING_RANGE_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/robot_footprint_model.h>

namespace teb_local_planner
{

class EdgeFollowingRange : public BaseTebUnaryEdge<2, const PoseSE2*, VertexPose>
{
public:

  EdgeFollowingRange() : t_(0.0)
  {
    _measurement = NULL;
  }

  EdgeFollowingRange(double t) : t_(t){}

  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig(), setViaPoint() on EdgeViaPoint()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

//    double dist = robot_model_->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, t_);
//    double dist = hypot(bandpt->pose().x()- (_measurement->position().x()+t_*follower_vel_->position().x()),
//                        bandpt->pose().y()- (_measurement->position().y()+t_*follower_vel_->position().y()));
    double dist = bandpt->pose().x()- (_measurement->position().x()+t_*follower_vel_->position().x());
    _error[0] = (bandpt->position() - _measurement->position()).norm();
//    _error[0] = penaltyBoundToInterval(bandpt->position().x() - _measurement->x(), 1.2, 2.0, cfg_->optim.penalty_epsilon);
//    _error[1] = penaltyBoundToInterval(bandpt->position().y() - _measurement->y(), -0.3, 0.3, cfg_->optim.penalty_epsilon);

    _error[0] = penaltyBoundFromBelow(dist, 1.5, cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundFromBelow(dist, 1.8, 0.0);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeFollowingRange::computeError() _error[0]=%f\n",_error[0]);
  }

  void setFollowerPose(const PoseSE2* follower_pose)
  {
    _measurement = follower_pose;
  }

  void setFollowerVel(const PoseSE2* follower_vel)
  {
    follower_vel_ = follower_vel;
  }

  /**
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param follower 2D position vector containing the position of the obstacle
   */
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, const PoseSE2* follower_pose)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = follower_pose;
  }

protected:

  const PoseSE2* follower_vel_;
  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  double t_; //!< Estimated time until current pose is reached

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // end namespace

#endif // EDGE_FOLLOWING_RANGE_H
