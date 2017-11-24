/**
    @file
    @author  Stanislas Brossette
    @author  Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once
#include <cmath>

namespace humoto {
namespace wpg05 {
/// @brief Task requiring the CoM to be inside of a rectangle approximating the
/// kinematic reachable area for a biped robot in the form of { lb < Ax < ub }
class HUMOTO_LOCAL TaskKinematicsRectangle : public humoto::TaskALU {
protected:
#define HUMOTO_CONFIG_ENTRIES HUMOTO_CONFIG_PARENT_CLASS(TaskALU)
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

  /// @brief Sets the defaults
  void setDefaults();

  /// @brief Finalizes the class initialization
  void finalize();

private:
  /// @brief min and max heights of the polygons
  double min_height_, max_height_;
  /// @brief width along x and y of the polygons
  double width_x_, width_y_;

  /// @brief Vectors containing the bounds for the CoM
  Eigen::VectorXd lbCoM_, ubCoM_;

public:
  /// @brief Default constructor
  ///
  /// @param gain gain of the task
  TaskKinematicsRectangle(const double gain = 1)
      : TaskALU("TaskKinematicsRectangle", gain) {
    setDefaults();
  }

  /// @brief Forms the matrices A and b to represent the task
  ///
  /// @param sol_structure structure of the problems solution
  /// @param model_base model (can be downcasted dynamically to a specific model
  /// type if necessary)
  /// @param control_problem control_problem (can be downcasted dynamically to a
  /// specific problem type if necessary)
  void form(const humoto::SolutionStructure &sol_structure,
            const humoto::Model &model_base,
            const humoto::ControlProblem &control_problem);
};

void TaskKinematicsRectangle::setDefaults() {
  TaskALU::setDefaults();
  min_height_ = 0.5;
  max_height_ = 1.0;
  width_x_ = 0.5;
  width_y_ = 0.5;
  setGain(1);
}

void TaskKinematicsRectangle::finalize() { TaskALU::finalize(); }

void TaskKinematicsRectangle::form(
    const humoto::SolutionStructure &sol_structure,
    const humoto::Model &model_base,
    const humoto::ControlProblem &control_problem) {
  // Downcast the control problem into a simpleMPC type
  const humoto::wpg05::MPCVerticalMotion &mpc =
      dynamic_cast<const humoto::wpg05::MPCVerticalMotion &>(control_problem);

  min_height_ = mpc.pbParams().kinematicLimitZmin_;
  max_height_ = mpc.pbParams().kinematicLimitZmax_;
  width_x_ = mpc.pbParams().kinematicLimitXSpan_;
  width_y_ = mpc.pbParams().kinematicLimitYSpan_;

  // Initialize the matrices A and b
  Eigen::MatrixXd &A = getA();
  Eigen::VectorXd &ub = getUpperBounds();
  Eigen::VectorXd &lb = getLowerBounds();

  long nHorizon = mpc.getPreviewHorizonLength();
  // Setup the CoM bounds along the preview horizon
  if (lbCoM_.size() != nHorizon * 3)
    lbCoM_.resize(3 * nHorizon);
  if (ubCoM_.size() != nHorizon * 3)
    ubCoM_.resize(3 * nHorizon);
  for (long i = 0; i < nHorizon; ++i) {
    double x = 0.5 * (mpc.stepPlan().xMin()(mpc.currentStepIndex() + i) +
                      mpc.stepPlan().xMax()(mpc.currentStepIndex() + i));
    double y = 0.5 * (mpc.stepPlan().yMin()(mpc.currentStepIndex() + i) +
                      mpc.stepPlan().yMax()(mpc.currentStepIndex() + i));
    lbCoM_(3 * i + 0) = x - width_x_ / 2.0;
    ubCoM_(3 * i + 0) = x + width_x_ / 2.0;
    lbCoM_(3 * i + 1) = y - width_y_ / 2.0;
    ubCoM_(3 * i + 1) = y + width_y_ / 2.0;
    lbCoM_(3 * i + 2) =
        mpc.stepPlan().z()(mpc.currentStepIndex() + i) + min_height_;
    ubCoM_(3 * i + 2) =
        mpc.stepPlan().z()(mpc.currentStepIndex() + i) + max_height_;
  }

  // selection matrix that selects only the position terms
  etools::SelectionMatrix posSelector(3, 0);

  // Compute the A and b matrices
  A.noalias() = getGain() * (posSelector * mpc.Uu());
  ub.noalias() =
      getGain() * (ubCoM_ - posSelector * mpc.Ux() * mpc.currentState());
  lb.noalias() =
      getGain() * (lbCoM_ - posSelector * mpc.Ux() * mpc.currentState());
}
} // namespace wpg05
} // namespace humoto
