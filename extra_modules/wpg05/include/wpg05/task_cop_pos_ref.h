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
/// @brief Task describing the CoP position following a reference value in the form of { Ax = b }
class HUMOTO_LOCAL TaskCoPPosRef : public humoto::TaskAB
{
  private:
    /// @brief Reference trajectory of the CoP
    Eigen::VectorXd zRef_;
    Eigen::MatrixXd xySelectorBlock_, xySelector_;

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
    TaskCoPPosRef(const double gain = 1) : TaskAB("TaskCoPPosRef", gain)
    {
        xySelectorBlock_.resize(4, 6);
        xySelectorBlock_.setZero();
        xySelectorBlock_(0, 0) = 1;
        xySelectorBlock_(1, 1) = 1;
        xySelectorBlock_(2, 3) = 1;
        xySelectorBlock_(3, 4) = 1;
    }

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
      std::cout << "\nform CoP ref task" << std::endl;
        // Downcast the control problem into a MPCVerticalMotion type
        const humoto::wpg05::MPCVerticalMotion &mpc =
            dynamic_cast<const humoto::wpg05::MPCVerticalMotion &>(control_problem);

        // Initialize the matrices A and b
        Eigen::MatrixXd &A = getA();
        Eigen::VectorXd &b = getB();

        long nHorizon = mpc.getPreviewHorizonLength();
        // Compute the full Selector matrix only once
        if (xySelector_.rows() != 4 * nHorizon || xySelector_.cols() != 6 * nHorizon)
        {
            xySelector_.resize(4 * nHorizon, 6 * nHorizon);
            xySelector_.setZero();
            for (long i = 0; i < nHorizon; ++i)
            {
                xySelector_.block(4 * i, 6 * i, 4, 6) = xySelectorBlock_;
            }
        }

        // Setup the reference trajectory along the preview horizon (it is the middle of the bounds)
        if (zRef_.size() != 4 * nHorizon) zRef_.resize(4 * nHorizon);

        for (long i = 0; i < nHorizon; ++i)
        {
            double xMin = mpc.stepPlan().xMin()(mpc.currentStepIndex() + i);
            double xMax = mpc.stepPlan().xMax()(mpc.currentStepIndex() + i);
            double yMin = mpc.stepPlan().yMin()(mpc.currentStepIndex() + i);
            double yMax = mpc.stepPlan().yMax()(mpc.currentStepIndex() + i);
            zRef_(4 * i) = 0.5 * (xMin + xMax);
            zRef_(4 * i + 1) = 0.5 * (yMin + yMax);
            zRef_(4 * i + 2) = 0.5 * (xMin + xMax);
            zRef_(4 * i + 3) = 0.5 * (yMin + yMax);
        }
        std::cout << "zRef_: " << zRef_.transpose() << std::endl;

        // Compute the A and b matrices
        A.noalias() = getGain() * xySelector_ * mpc.Ou();
        b.noalias() = getGain() * (-xySelector_ * mpc.Ox() * mpc.currentState() + zRef_);
        std::cout << "form CoP ref task DONE" << std::endl;
    };
};
}
}
