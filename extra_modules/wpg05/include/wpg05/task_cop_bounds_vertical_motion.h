/**
    @file
    @author  Stanislas Brossette
    @author  Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
namespace wpg05
{
/// @brief Task describing the CoP location in the sustentation polygon in the form { l < Ax < u}
class HUMOTO_LOCAL TaskCoPBoundsVerticalMotion : public humoto::TaskALU
{
  private:
    /// @brief Lower bounds for the CoP along the preview horizon
    Eigen::VectorXd zBoundsLow_;
    /// @brief Upper bounds for the CoP along the preview horizon
    Eigen::VectorXd zBoundsHigh_;

  protected:
#define HUMOTO_CONFIG_ENTRIES HUMOTO_CONFIG_PARENT_CLASS(TaskALU)
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

    /// @brief Sets the defaults
    void setDefaults()
    {
        TaskALU::setDefaults();
        setGain(1);
    }

    /// @brief Finalizes the class initialization
    void finalize() { TaskALU::finalize(); }

  public:
    /// @brief Default constructor
    ///
    /// @param gain gain of the task
    TaskCoPBoundsVerticalMotion(const double gain = 1)
        : TaskALU("TaskCoPBoundsVerticalMotion", gain)
    {
    }

    /// @brief Forms the matrices A, l and u to represent the task
    ///
    /// @param sol_structure structure of the problems solution
    /// @param model_base model (can be downcasted dynamically to a specific model type if
    /// necessary)
    /// @param control_problem control_problem (can be downcasted dynamically to a specific problem
    /// type if necessary)
    void form(const humoto::SolutionStructure &sol_structure, const humoto::Model &model_base,
              const humoto::ControlProblem &control_problem)
    {
        // Downcast the control problem into a simpleMPC type
        const humoto::wpg05::MPCVerticalMotion &mpc =
            dynamic_cast<const humoto::wpg05::MPCVerticalMotion &>(control_problem);

        // Initialize the matrices A and b
        Eigen::MatrixXd &A = getA();
        Eigen::VectorXd &l = getLowerBounds();
        Eigen::VectorXd &u = getUpperBounds();

        // Setup the bounds along the preview horizon
        if (zBoundsHigh_.size() != 6 * (long)mpc.getPreviewHorizonLength())
            zBoundsHigh_.resize(6 * mpc.getPreviewHorizonLength());
        if (zBoundsLow_.size() != 6 * (long)mpc.getPreviewHorizonLength())
            zBoundsLow_.resize(6 * mpc.getPreviewHorizonLength());

        for (std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
        {
            zBoundsLow_(6 * i + 0) = mpc.stepPlan().xMin()(mpc.currentStepIndex() + i);
            zBoundsLow_(6 * i + 1) = mpc.stepPlan().yMin()(mpc.currentStepIndex() + i);
            zBoundsLow_(6 * i + 2) = mpc.stepPlan().z()(mpc.currentStepIndex() + i) +
                                     mpc.zetaMin() * mpc.pbParams().g_;
            zBoundsLow_(6 * i + 3) = mpc.stepPlan().xMin()(mpc.currentStepIndex() + i);
            zBoundsLow_(6 * i + 4) = mpc.stepPlan().yMin()(mpc.currentStepIndex() + i);
            zBoundsLow_(6 * i + 5) = -humoto::g_infinity;

            zBoundsHigh_(6 * i + 0) = mpc.stepPlan().xMax()(mpc.currentStepIndex() + i);
            zBoundsHigh_(6 * i + 1) = mpc.stepPlan().yMax()(mpc.currentStepIndex() + i);
            zBoundsHigh_(6 * i + 2) = humoto::g_infinity;
            zBoundsHigh_(6 * i + 3) = mpc.stepPlan().xMax()(mpc.currentStepIndex() + i);
            zBoundsHigh_(6 * i + 4) = mpc.stepPlan().yMax()(mpc.currentStepIndex() + i);
            zBoundsHigh_(6 * i + 5) = mpc.stepPlan().z()(mpc.currentStepIndex() + i) +
                                      mpc.zetaMax() * mpc.pbParams().g_;
        }

        // Compute the A, l and u matrices
        A.noalias() = getGain() * mpc.Ou();
        l.noalias() = getGain() * (-mpc.Ox() * mpc.currentState() + zBoundsLow_);
        u.noalias() = getGain() * (-mpc.Ox() * mpc.currentState() + zBoundsHigh_);
    };
};
}
}
