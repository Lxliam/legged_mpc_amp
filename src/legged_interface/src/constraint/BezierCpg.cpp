/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
******************************************************************************/

#include "legged_interface/constraint/BezierCpg.h"

#include <algorithm>
#include <cassert>

namespace ocs2 {
namespace legged_robot {

BezierCpg::BezierCpg(Node liftOff, scalar_t midHeight, Node touchDown, scalar_t roundness)
    : midTime_((liftOff.time + touchDown.time) / 2) {
  roundness = std::max<scalar_t>(0.0, std::min<scalar_t>(0.8, roundness));
  const scalar_t topHalfDuration = 0.25 * roundness * (touchDown.time - liftOff.time);
  topStartTime_ = midTime_ - topHalfDuration;
  topEndTime_ = midTime_ + topHalfDuration;

  leftSegment_.reset(new BezierSegment(liftOff, Node{topStartTime_, midHeight, 0.0}, 0.0, 0.0));
  if (topEndTime_ > topStartTime_) {
    topSegment_.reset(new BezierSegment(Node{topStartTime_, midHeight, 0.0}, Node{topEndTime_, midHeight, 0.0}, 0.0, 0.0));
  }
  rightSegment_.reset(new BezierSegment(Node{topEndTime_, midHeight, 0.0}, touchDown, 0.0, 0.0));
}

scalar_t BezierCpg::position(scalar_t time) const {
  if (time < topStartTime_) {
    return leftSegment_->position(time);
  }
  if (topSegment_ && time < topEndTime_) {
    return topSegment_->position(time);
  }
  return rightSegment_->position(time);
}

scalar_t BezierCpg::velocity(scalar_t time) const {
  if (time < topStartTime_) {
    return leftSegment_->velocity(time);
  }
  if (topSegment_ && time < topEndTime_) {
    return topSegment_->velocity(time);
  }
  return rightSegment_->velocity(time);
}

scalar_t BezierCpg::acceleration(scalar_t time) const {
  if (time < topStartTime_) {
    return leftSegment_->acceleration(time);
  }
  if (topSegment_ && time < topEndTime_) {
    return topSegment_->acceleration(time);
  }
  return rightSegment_->acceleration(time);
}

BezierCpg::BezierSegment::BezierSegment(Node start, Node end, scalar_t startRoundness, scalar_t endRoundness) {
  assert(start.time < end.time);
  t0_ = start.time;
  dt_ = end.time - start.time;

  startRoundness = std::max<scalar_t>(0.0, std::min<scalar_t>(1.0, startRoundness));
  endRoundness = std::max<scalar_t>(0.0, std::min<scalar_t>(1.0, endRoundness));

  const scalar_t cubicP1 = start.position + start.velocity * dt_ / 3.0;
  const scalar_t cubicP2 = end.position - end.velocity * dt_ / 3.0;

  const scalar_t quinticP2 = 0.3 * start.position + 0.6 * cubicP1 + 0.1 * cubicP2;
  const scalar_t quinticP3 = 0.1 * cubicP1 + 0.6 * cubicP2 + 0.3 * end.position;
  const scalar_t roundP2 = start.position + 2.0 * (end.position - start.position) / 5.0;
  const scalar_t roundP3 = start.position + 3.0 * (end.position - start.position) / 5.0;

  p0_ = start.position;
  p1_ = start.position + start.velocity * dt_ / 5.0;
  p2_ = (1.0 - startRoundness) * quinticP2 + startRoundness * roundP2;
  p3_ = (1.0 - endRoundness) * quinticP3 + endRoundness * roundP3;
  p4_ = end.position - end.velocity * dt_ / 5.0;
  p5_ = end.position;
}

scalar_t BezierCpg::BezierSegment::position(scalar_t time) const {
  const scalar_t s = normalizedTime(time);
  const scalar_t oneMinusS = 1.0 - s;
  return oneMinusS * oneMinusS * oneMinusS * oneMinusS * oneMinusS * p0_ +
         5.0 * oneMinusS * oneMinusS * oneMinusS * oneMinusS * s * p1_ +
         10.0 * oneMinusS * oneMinusS * oneMinusS * s * s * p2_ +
         10.0 * oneMinusS * oneMinusS * s * s * s * p3_ + 5.0 * oneMinusS * s * s * s * s * p4_ +
         s * s * s * s * s * p5_;
}

scalar_t BezierCpg::BezierSegment::velocity(scalar_t time) const {
  const scalar_t s = normalizedTime(time);
  const scalar_t oneMinusS = 1.0 - s;
  return (5.0 * oneMinusS * oneMinusS * oneMinusS * oneMinusS * (p1_ - p0_) +
          20.0 * oneMinusS * oneMinusS * oneMinusS * s * (p2_ - p1_) +
          30.0 * oneMinusS * oneMinusS * s * s * (p3_ - p2_) +
          20.0 * oneMinusS * s * s * s * (p4_ - p3_) + 5.0 * s * s * s * s * (p5_ - p4_)) /
         dt_;
}

scalar_t BezierCpg::BezierSegment::acceleration(scalar_t time) const {
  const scalar_t s = normalizedTime(time);
  const scalar_t oneMinusS = 1.0 - s;
  return (20.0 * oneMinusS * oneMinusS * oneMinusS * (p2_ - 2.0 * p1_ + p0_) +
          60.0 * oneMinusS * oneMinusS * s * (p3_ - 2.0 * p2_ + p1_) +
          60.0 * oneMinusS * s * s * (p4_ - 2.0 * p3_ + p2_) + 20.0 * s * s * s * (p5_ - 2.0 * p4_ + p3_)) /
         (dt_ * dt_);
}

scalar_t BezierCpg::BezierSegment::normalizedTime(scalar_t time) const {
  return (time - t0_) / dt_;
}

}  // namespace legged_robot
}  // namespace ocs2
