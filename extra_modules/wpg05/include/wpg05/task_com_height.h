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
/// @brief Task describing the CoM height being as close as possible to a reference height in the
/// form of { Ax = b }
class HUMOTO_LOCAL TaskCoMHeight : public humoto::TaskAB
{
  protected:
#define HUMOTO_CONFIG_ENTRIES HUMOTO_CONFIG_PARENT_CLASS(TaskAB)
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

    /// @brief Sets the defaults
    void setDefaults()
    {
        TaskAB::setDefaults();
        setGain(1);
    }

    /// @brief Finalizes the class initialization
    void finalize() { TaskAB::finalize(); }

  public:
    /// @brief Default constructor
    ///
    /// @param gain gain of the task
    TaskCoMHeight(const double gain = 1) : TaskAB("TaskCoMHeight", gain) {}

    /// @brief Forms the matrices A and b to represent the task
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
        Eigen::VectorXd &b = getB();

        // Setup the reference velocity along the preview horizon
        Eigen::VectorXd cHeightRef;
        if (cHeightRef.size() != (long)mpc.getPreviewHorizonLength())
            cHeightRef.resize(mpc.getPreviewHorizonLength());
        for (std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
        {
            cHeightRef(i) =
                mpc.stepPlan().z()(mpc.currentStepIndex() + i) + mpc.pbParams().comHeightRef_;
        }

        etools::SelectionMatrix posZselector(9, 6);
        // Compute the A and b matrices
        A.noalias() = getGain() * (posZselector * mpc.Uu());
        b.noalias() = getGain() * (cHeightRef - posZselector * mpc.Ux() * mpc.currentState());
    };
};
}
}
