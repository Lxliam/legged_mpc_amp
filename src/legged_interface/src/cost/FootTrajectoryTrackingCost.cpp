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

#include "legged_interface/cost/FootTrajectoryTrackingCost.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace legged_robot {
namespace {

matrix_t diagonalMatrix(const vector3_t& weights) {
  return weights.asDiagonal();
}

void addGaussNewtonTerm(ScalarFunctionQuadraticApproximation& cost, const VectorFunctionLinearApproximation& residual,
                        const matrix_t& weight) {
  if (weight.isZero(0.0)) {
    return;
  }

  cost.f += 0.5 * residual.f.dot(weight * residual.f);
  cost.dfdx.noalias() += residual.dfdx.transpose() * weight * residual.f;
  cost.dfdu.noalias() += residual.dfdu.transpose() * weight * residual.f;
  cost.dfdxx.noalias() += residual.dfdx.transpose() * weight * residual.dfdx;
  cost.dfduu.noalias() += residual.dfdu.transpose() * weight * residual.dfdu;
  cost.dfdux.noalias() += residual.dfdu.transpose() * weight * residual.dfdx;
}

}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FootTrajectoryTrackingCost::FootTrajectoryTrackingCost(const SwitchedModelReferenceManager& referenceManager,
                                                       const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                       size_t footIndex, CentroidalModelInfo info,
                                                       vector3_t nominalFootOffsetInBase, Config config)
    : referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      footIndex_(footIndex),
      info_(std::move(info)),
      nominalFootOffsetInBase_(std::move(nominalFootOffsetInBase)),
      config_(std::move(config)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FootTrajectoryTrackingCost::FootTrajectoryTrackingCost(const FootTrajectoryTrackingCost& rhs)
    : referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      footIndex_(rhs.footIndex_),
      info_(rhs.info_),
      nominalFootOffsetInBase_(rhs.nominalFootOffsetInBase_),
      config_(rhs.config_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool FootTrajectoryTrackingCost::isActive(scalar_t time) const {
  return config_.enabled && !referenceManagerPtr_->getContactFlags(time)[footIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t FootTrajectoryTrackingCost::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                              const TargetTrajectories& targetTrajectories,
                                              const PreComputation& /*preComp*/) const {
  const vector3_t positionResidual = endEffectorKinematicsPtr_->getPosition(state).front() - getPositionReference(time, targetTrajectories);
  const vector3_t velocityResidual =
      endEffectorKinematicsPtr_->getVelocity(state, input).front() - getVelocityReference(time, targetTrajectories);

  return 0.5 * positionResidual.dot(config_.positionWeights.asDiagonal() * positionResidual) +
         0.5 * velocityResidual.dot(config_.velocityWeights.asDiagonal() * velocityResidual);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation FootTrajectoryTrackingCost::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
    const PreComputation& /*preComp*/) const {
  auto cost = ScalarFunctionQuadraticApproximation::Zero(state.size(), input.size());

  const matrix_t positionWeight = diagonalMatrix(config_.positionWeights);
  const matrix_t velocityWeight = diagonalMatrix(config_.velocityWeights);

  const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
  VectorFunctionLinearApproximation positionResidual = positionApprox;
  positionResidual.f.noalias() -= getPositionReference(time, targetTrajectories);
  addGaussNewtonTerm(cost, positionResidual, positionWeight);

  const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
  VectorFunctionLinearApproximation velocityResidual = velocityApprox;
  velocityResidual.f.noalias() -= getVelocityReference(time, targetTrajectories);
  addGaussNewtonTerm(cost, velocityResidual, velocityWeight);

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector3_t FootTrajectoryTrackingCost::getPositionReference(scalar_t time, const TargetTrajectories& targetTrajectories) const {
  const auto desiredState = targetTrajectories.getDesiredState(time);
  const auto basePose = centroidal_model::getBasePose(desiredState, info_);
  const matrix3_t baseToWorld = getRotationMatrixFromZyxEulerAngles(vector3_t(basePose.tail<3>()));

  const scalar_t phase = referenceManagerPtr_->getSwingTrajectoryPlanner()->getSwingPhase(footIndex_, time);
  const scalar_t duration = referenceManagerPtr_->getSwingTrajectoryPlanner()->getSwingDuration(footIndex_, time);
  const auto bezier = evaluateBezierFootTrajectory(phase, duration, config_);
  vector3_t footOffsetInBase = nominalFootOffsetInBase_;
  footOffsetInBase.x() += bezier.positionInBase.x();
  vector3_t reference = basePose.head<3>() + baseToWorld * footOffsetInBase;
  reference.z() = basePose.z() + nominalFootOffsetInBase_.z() + bezier.positionInBase.z();
  return reference;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector3_t FootTrajectoryTrackingCost::getVelocityReference(scalar_t time, const TargetTrajectories& targetTrajectories) const {
  const auto desiredState = targetTrajectories.getDesiredState(time);
  const auto basePose = centroidal_model::getBasePose(desiredState, info_);
  const matrix3_t baseToWorld = getRotationMatrixFromZyxEulerAngles(vector3_t(basePose.tail<3>()));

  const scalar_t phase = referenceManagerPtr_->getSwingTrajectoryPlanner()->getSwingPhase(footIndex_, time);
  const scalar_t duration = referenceManagerPtr_->getSwingTrajectoryPlanner()->getSwingDuration(footIndex_, time);

  vector3_t reference = vector3_t::Zero();
  if (duration > 0.0) {
    const auto bezier = evaluateBezierFootTrajectory(phase, duration, config_);
    vector3_t velocityInBase = vector3_t::Zero();
    velocityInBase.x() = bezier.velocityInBase.x();
    reference = baseToWorld * velocityInBase;
    reference.z() = bezier.velocityInBase.z();
  }
  return reference;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FootTrajectoryTrackingCost::Config loadFootTrajectoryTrackingCostConfig(const std::string& fileName, const std::string& fieldName,
                                                                        bool verbose) {
  return loadBezierFootTrajectoryConfig(fileName, fieldName, verbose);
}

}  // namespace legged_robot
}  // namespace ocs2
