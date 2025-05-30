/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/soft_constraint/StateInputSoftErrBoxConstraint.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftErrBoxConstraint::BoxConstraint::BoxConstraint(const BoxConstraint& other)
    : index(other.index), lowerBound(other.lowerBound), upperBound(other.upperBound), penaltyPtr(other.penaltyPtr->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftErrBoxConstraint::BoxConstraint& StateInputSoftErrBoxConstraint::BoxConstraint::operator=(const BoxConstraint& other) {
  this->index = other.index;
  this->lowerBound = other.lowerBound;
  this->upperBound = other.upperBound;
  this->penaltyPtr.reset(other.penaltyPtr->clone());
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftErrBoxConstraint::StateInputSoftErrBoxConstraint(std::vector<BoxConstraint> stateBoxConstraints,
                                                         std::vector<BoxConstraint> inputBoxConstraints)
    : stateBoxConstraints_(std::move(stateBoxConstraints)), inputBoxConstraints_(std::move(inputBoxConstraints)), offset_(0.0) {
  sortByIndex(stateBoxConstraints_);
  sortByIndex(inputBoxConstraints_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputSoftErrBoxConstraint* StateInputSoftErrBoxConstraint::clone() const {
  return new StateInputSoftErrBoxConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateInputSoftErrBoxConstraint::isActive(scalar_t time) const {
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputSoftErrBoxConstraint::sortByIndex(std::vector<BoxConstraint>& boxConstraints) const {
  std::sort(boxConstraints.begin(), boxConstraints.end(),
            [](const BoxConstraint& lhs, const BoxConstraint& rhs) { return lhs.index < rhs.index; });
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputSoftErrBoxConstraint::initializeOffset(scalar_t time, const vector_t& state, const vector_t& input) {
  offset_ = -getValue(time, state, state, stateBoxConstraints_) - getValue(time, input, input, inputBoxConstraints_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputSoftErrBoxConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
                                               const PreComputation& preComp) const {
  return getValue(time, state, targetTrajectories.getDesiredState(time), stateBoxConstraints_) + getValue(time, input, targetTrajectories.getDesiredInput(time), inputBoxConstraints_) + offset_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateInputSoftErrBoxConstraint::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                            const vector_t& input,
                                                                                            const TargetTrajectories& targetTrajectories,
                                                                                            const PreComputation& preComp) const {
  auto cost = ScalarFunctionQuadraticApproximation::Zero(state.size(), input.size());
  fillQuadraticApproximation(time, state, targetTrajectories.getDesiredState(time), stateBoxConstraints_, cost.f, cost.dfdx, cost.dfdxx);
  fillQuadraticApproximation(time, input, targetTrajectories.getDesiredInput(time), inputBoxConstraints_, cost.f, cost.dfdu, cost.dfduu);
  cost.f += offset_;
  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateInputSoftErrBoxConstraint::getValue(scalar_t t, const vector_t& h, const vector_t& target, const std::vector<BoxConstraint>& boxConstraints) const {
  scalar_t f = scalar_t(0.0);
  for (const auto& boxConstraint : boxConstraints) {
    const auto i = boxConstraint.index;
    const auto& p = *boxConstraint.penaltyPtr;
    const scalar_t upperBoundDistance = target(i) + boxConstraint.upperBound - h(i);
    const scalar_t lowerBoundDistance = h(i) - (target(i) + boxConstraint.lowerBound);

    f += p.getValue(t, lowerBoundDistance) + p.getValue(t, upperBoundDistance);
  }
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateInputSoftErrBoxConstraint::fillQuadraticApproximation(scalar_t t, const vector_t& h, const vector_t& target,
                                                             const std::vector<BoxConstraint>& boxConstraints, scalar_t& value,
                                                             vector_t& firstDerivative, matrix_t& secondDerivative) const {
  for (const auto& boxConstraint : boxConstraints) {
    const auto i = boxConstraint.index;
    const auto& p = *boxConstraint.penaltyPtr;
    const scalar_t upperBoundDistance = target(i) + boxConstraint.upperBound - h(i);
    const scalar_t lowerBoundDistance = h(i) - (target(i) + boxConstraint.lowerBound);

    value += p.getValue(t, lowerBoundDistance) + p.getValue(t, upperBoundDistance);
    firstDerivative(i) += p.getDerivative(t, lowerBoundDistance) - p.getDerivative(t, upperBoundDistance);
    secondDerivative(i, i) += p.getSecondDerivative(t, lowerBoundDistance) + p.getSecondDerivative(t, upperBoundDistance);
  }
}

}  // namespace ocs2
