/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.
******************************************************************************/

#pragma once

#include <memory>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace legged_robot {

class BezierCpg {
 public:
  struct Node {
    scalar_t time;
    scalar_t position;
    scalar_t velocity;
  };

  BezierCpg(Node liftOff, scalar_t midHeight, Node touchDown, scalar_t roundness = 0.0);

  scalar_t position(scalar_t time) const;

  scalar_t velocity(scalar_t time) const;

  scalar_t acceleration(scalar_t time) const;

 private:
  class BezierSegment {
   public:
    BezierSegment(Node start, Node end, scalar_t startRoundness, scalar_t endRoundness);

    scalar_t position(scalar_t time) const;

    scalar_t velocity(scalar_t time) const;

    scalar_t acceleration(scalar_t time) const;

   private:
    scalar_t normalizedTime(scalar_t time) const;

    scalar_t t0_;
    scalar_t dt_;
    scalar_t p0_;
    scalar_t p1_;
    scalar_t p2_;
    scalar_t p3_;
    scalar_t p4_;
    scalar_t p5_;
  };

  scalar_t midTime_;
  scalar_t topStartTime_;
  scalar_t topEndTime_;
  std::unique_ptr<BezierSegment> leftSegment_;
  std::unique_ptr<BezierSegment> topSegment_;
  std::unique_ptr<BezierSegment> rightSegment_;
};

}  // namespace legged_robot
}  // namespace ocs2
