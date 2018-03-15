#ifndef EDGE_FOLLOWING_RANGE_H
#define EDGE_FOLLOWING_RANGE_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>

namespace teb_local_planner
{

class EdgeFollowingRange : public BaseTebUnaryEdge<1, const Eigen::Vector2d*, VertexPose>
{
public:

  EdgeFollowingRange()
  {
    _measurement = NULL;
  }

  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig(), setViaPoint() on EdgeViaPoint()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
//    _error[0] = (bandpt->position() - *_measurement).norm();
    _error[0] = penaltyBoundToInterval((bandpt->position() - *_measurement).norm(), 1.2, 2.0, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeFollowingRange::computeError() _error[0]=%f\n",_error[0]);
  }

  void setFollowerPose(const Eigen::Vector2d* follower_pose)
  {
    _measurement = follower_pose;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // end namespace

#endif // EDGE_FOLLOWING_RANGE_H
