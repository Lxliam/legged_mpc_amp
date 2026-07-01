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

#pragma once

#include <memory>
#include <string>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "legged_interface/SwitchedModelReferenceManager.h"
#include "legged_interface/foot_planner/BezierFootTrajectory.h"

namespace ocs2 {
namespace legged_robot {

class FootTrajectoryTrackingCost final : public StateInputCost {
 public:
  using Config = BezierFootTrajectoryConfig;

  FootTrajectoryTrackingCost(const SwitchedModelReferenceManager& referenceManager,
                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t footIndex,
                             CentroidalModelInfo info, vector3_t nominalFootOffsetInBase, Config config);

  FootTrajectoryTrackingCost* clone() const override { return new FootTrajectoryTrackingCost(*this); }

  bool isActive(scalar_t time) const override;

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                    const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const vector_t& input,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 private:
  FootTrajectoryTrackingCost(const FootTrajectoryTrackingCost& rhs);

  vector3_t getPositionReference(scalar_t time, const TargetTrajectories& targetTrajectories) const;

  vector3_t getVelocityReference(scalar_t time, const TargetTrajectories& targetTrajectories) const;

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const size_t footIndex_;
  const CentroidalModelInfo info_;
  const vector3_t nominalFootOffsetInBase_;
  const Config config_;
};

FootTrajectoryTrackingCost::Config loadFootTrajectoryTrackingCostConfig(const std::string& fileName,
                                                                        const std::string& fieldName,
                                                                        bool verbose = true);

}  // namespace legged_robot
}  // namespace ocs2
