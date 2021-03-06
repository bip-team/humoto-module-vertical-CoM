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
#include "task_kinematic.h.in"

namespace humoto
{
namespace wpg05
{
/// @brief Task requiring the CoM to be inside of a polygon approximating the
/// kinematic reachable area for a biped robot in the form of { b < Ax } The
/// constraint polygon is described in the file task_kinematic.h.in that can be
/// generated by using the computeKinematicTask.py script
class HUMOTO_LOCAL TaskKinematicsPolygon : public humoto::TaskAL
{
  protected:
#define HUMOTO_CONFIG_ENTRIES HUMOTO_CONFIG_PARENT_CLASS(TaskAL)
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

    /// @brief Sets the defaults
    void setDefaults()
    {
        TaskAL::setDefaults();
        filename_ = "task_kinematic.yaml";
        setGain(1);
    }

    /// @brief Finalizes the class initialization
    void finalize() { TaskAL::finalize(); }

  private:
    Eigen::MatrixXd A_;
    Eigen::VectorXd b_;
    Eigen::MatrixXd ABlocks_;
    Eigen::VectorXd bBlocks_;
    std::string filename_;

  public:
    /// @brief Default constructor
    ///
    /// @param gain gain of the task
    TaskKinematicsPolygon(const double gain = 1) : TaskAL("TaskKinematicsPolygon", gain) {}

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
        Eigen::IOFormat fmt(5, 0, ", ", ",\n", "[", "]", "[", "]");
        // Downcast the control problem into a simpleMPC type
        const humoto::wpg05::MPCVerticalMotion &mpc =
            dynamic_cast<const humoto::wpg05::MPCVerticalMotion &>(control_problem);

        GeneratedKinematicConstraint cstr;
        long N = (long)mpc.getPreviewHorizonLength();
        long cRows = cstr.A.rows();
        long cCols = cstr.A.cols();
        ABlocks_.resize(2 * N * cRows, N * cCols);
        bBlocks_.resize(2 * N * cRows);
        ABlocks_.setZero();
        bBlocks_.setZero();

        Eigen::Vector3d s2a = mpc.pbParams().soleToAnkle_;
        Eigen::Vector3d com2rHip = mpc.pbParams().comToRightHip_;
        Eigen::Vector3d com2lHip = mpc.pbParams().comToLeftHip_;
        const FootTraj &rFootTraj = mpc.rightFootTraj();
        const FootTraj &lFootTraj = mpc.leftFootTraj();

        for (long i = 0; i < N; ++i)
        {
            // Right leg feasibility
            ABlocks_.block(2 * i * cRows, i * cCols, cRows, cCols) = cstr.A;
            bBlocks_.segment(2 * i * cRows, cRows) =
                cstr.b - cstr.A * (com2rHip - rFootTraj(mpc.currentStepIndex() + 1 + i) - s2a);
            // Left leg feasibility
            ABlocks_.block((2 * i + 1) * cRows, i * cCols, cRows, cCols) = cstr.A;
            bBlocks_.segment((2 * i + 1) * cRows, cRows) =
                cstr.b - cstr.A * (com2lHip - lFootTraj(mpc.currentStepIndex() + 1 + i) - s2a);
        }

        // Initialize the matrices A and b
        Eigen::MatrixXd &A = getA();
        Eigen::VectorXd &b = getLowerBounds();
        A.setZero();
        b.setZero();

        etools::SelectionMatrix posSelector(3, 0);

        // Compute the A and b matrices
        A.noalias() = getGain() * ABlocks_ * (posSelector * mpc.Uu());
        b.noalias() =
            getGain() * (bBlocks_ - ABlocks_ * (posSelector * mpc.Ux() * mpc.currentState()));
    }

    static void computeAndLogHighestFeasibleZ(const humoto::wpg05::MPCVerticalMotion &mpc,
                                              const Eigen::Vector3d &CoM, Logger &logger_)
    {
        double maxZ = computeHighestFeasibleZ(mpc, CoM);
        mpc.logger().addHighestFeasibleZ(maxZ, mpc.currentStepIndex());
    }

    static double computeHighestFeasibleZ(const humoto::wpg05::MPCVerticalMotion &mpc,
                                          const Eigen::Vector3d &CoM)
    {
        humoto::wpg05::GeneratedKinematicConstraint genCstr;
        Eigen::Vector3d s2a = mpc.pbParams().soleToAnkle_;
        Eigen::Vector3d com2rHip = mpc.pbParams().comToRightHip_;
        Eigen::Vector3d com2lHip = mpc.pbParams().comToLeftHip_;
        size_t stepIndex = mpc.currentStepIndex();

        Eigen::Vector3d rAnkle = mpc.rightFootTraj()(stepIndex) + s2a;
        Eigen::Vector3d lAnkle = mpc.leftFootTraj()(stepIndex) + s2a;
        Eigen::Vector3d rHip = CoM + com2rHip;
        Eigen::Vector3d lHip = CoM + com2lHip;
        Eigen::Vector3d rLeg = rHip - rAnkle;
        Eigen::Vector3d lLeg = lHip - lAnkle;
        rLeg.z() = std::sqrt(genCstr.radius_ * genCstr.radius_ - rLeg.x() * rLeg.x() -
                             rLeg.y() * rLeg.y());
        lLeg.z() = std::sqrt(genCstr.radius_ * genCstr.radius_ - lLeg.x() * lLeg.x() -
                             lLeg.y() * lLeg.y());
        Eigen::Vector3d CoMrLeg = rAnkle + rLeg - com2rHip;
        Eigen::Vector3d CoMlLeg = lAnkle + lLeg - com2lHip;
        double maxCoMHeight = std::min(CoMrLeg.z(), CoMlLeg.z());
        return maxCoMHeight;
    }
};
}
}
