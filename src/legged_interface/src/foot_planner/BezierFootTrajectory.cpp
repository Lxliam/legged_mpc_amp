/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "legged_interface/foot_planner/BezierFootTrajectory.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace legged_robot {
namespace {

scalar_t smoothStep(scalar_t x) {
  return x * x * (3.0 - 2.0 * x);
}

scalar_t smoothStepDerivative(scalar_t x) {
  return 6.0 * x * (1.0 - x);
}

}  // namespace

BezierFootTrajectorySample evaluateBezierFootTrajectory(scalar_t phase, scalar_t duration,
                                                        const BezierFootTrajectoryConfig& config) {
  const scalar_t s = std::max<scalar_t>(0.0, std::min<scalar_t>(1.0, phase));
  const scalar_t oneMinusS = 1.0 - s;
  scalar_t ramp = 1.0;
  scalar_t rampDerivativeWithTime = 0.0;
  if (config.rampTime > 0.0 && duration > 0.0) {
    const scalar_t swingTime = s * duration;
    const scalar_t rampPhase = std::max<scalar_t>(0.0, std::min<scalar_t>(1.0, swingTime / config.rampTime));
    ramp = smoothStep(rampPhase);
    rampDerivativeWithTime = smoothStepDerivative(rampPhase) / config.rampTime;
  }
  const scalar_t halfStep = 0.5 * config.stepLength;
  const scalar_t controlX = config.bezierControlXRatio * halfStep;
  const scalar_t controlHeight = config.bezierControlHeightRatio * config.arcHeight;

  vector3_t p0 = vector3_t::Zero();
  vector3_t p1 = vector3_t::Zero();
  vector3_t p2 = vector3_t::Zero();
  vector3_t p3 = vector3_t::Zero();
  p0.x() = -halfStep;
  p1.x() = -controlX;
  p2.x() = controlX;
  p3.x() = halfStep;
  p1.z() = controlHeight;
  p2.z() = controlHeight;

  const vector3_t derivativeWithPhase = 3.0 * oneMinusS * oneMinusS * (p1 - p0) + 6.0 * oneMinusS * s * (p2 - p1) +
                                        3.0 * s * s * (p3 - p2);

  BezierFootTrajectorySample sample;
  const vector3_t position = oneMinusS * oneMinusS * oneMinusS * p0 + 3.0 * oneMinusS * oneMinusS * s * p1 +
                             3.0 * oneMinusS * s * s * p2 + s * s * s * p3;
  sample.positionInBase = ramp * position;
  if (duration > 0.0) {
    sample.velocityInBase = ramp * derivativeWithPhase / duration + rampDerivativeWithTime * position;
  }
  return sample;
}

BezierFootTrajectoryConfig loadBezierFootTrajectoryConfig(const std::string& fileName, const std::string& fieldName,
                                                          bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  const std::string prefix = fieldName + ".";
  BezierFootTrajectoryConfig config;

  loadData::loadPtreeValue(pt, config.enabled, prefix + "enabled", verbose);
  loadData::loadPtreeValue(pt, config.stepLength, prefix + "stepLength", verbose);
  loadData::loadPtreeValue(pt, config.arcHeight, prefix + "arcHeight", verbose);
  loadData::loadPtreeValue(pt, config.rampTime, prefix + "rampTime", verbose);
  loadData::loadPtreeValue(pt, config.bezierControlXRatio, prefix + "bezierControlXRatio", verbose);
  loadData::loadPtreeValue(pt, config.bezierControlHeightRatio, prefix + "bezierControlHeightRatio", verbose);
  loadData::loadPtreeValue(pt, config.positionWeights.x(), prefix + "positionWeightX", verbose);
  loadData::loadPtreeValue(pt, config.positionWeights.y(), prefix + "positionWeightY", verbose);
  loadData::loadPtreeValue(pt, config.positionWeights.z(), prefix + "positionWeightZ", verbose);
  loadData::loadPtreeValue(pt, config.velocityWeights.x(), prefix + "velocityWeightX", verbose);
  loadData::loadPtreeValue(pt, config.velocityWeights.y(), prefix + "velocityWeightY", verbose);
  loadData::loadPtreeValue(pt, config.velocityWeights.z(), prefix + "velocityWeightZ", verbose);

  return config;
}

}  // namespace legged_robot
}  // namespace ocs2
