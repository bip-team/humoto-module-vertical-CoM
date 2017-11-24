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
/// @brief Class handling the control problem, it is responsible for updating the model and its
/// state with the control found as solution of the optimization problem (on previous iteration)
class HUMOTO_LOCAL SimpleMPC : public humoto::MPC
{
  private:
    // Matrices to update the state
    // x_{k_1} = A*x_k + B*u_k
    // y_{k_1} = D*x_k + E*u_k
    etools::Matrix9 A_;
    etools::Matrix9x3 B_;
    etools::Matrix2x9 D_;
    etools::Matrix2x3 E_;

    // Condensed update matrices
    // v_x = Ux*x_0 + Uu*v_u
    // v_y = Ox*x_0 + Ou*v_u
    Eigen::MatrixXd Ux_;
    Eigen::MatrixXd Uu_;
    Eigen::MatrixXd Ox_;
    Eigen::MatrixXd Ou_;

    /// @brief Current state of the model
    etools::Vector9 currentState_;
    /// @brief reference to the parameters of the problem
    const ProblemParameters& pbParams_;
    /// @brief Matrix to select velocities out of the state vector
    etools::SelectionMatrix velocity_selector_;
    /// @brief Plan for the steps to take (predefined)
    StepPlan stepPlan_;
    /// @brief Current step index
    size_t currentStepIndex_;
    /// @brief logger
    Logger logger_;

    /// @brief Computes the A matrix
    /// x_{k_1} = A*x_k + B*u_k
    ///
    /// @return The A matrix
    etools::Matrix9 computeA()
    {
        double t = pbParams_.t_;
        Eigen::Matrix3d Ablock;
        Ablock.setIdentity();
        Ablock(0, 1) = t;
        Ablock(0, 2) = t * t / 2.0;
        Ablock(1, 2) = t;
        A_.setIdentity();
        A_.block(0, 0, 3, 3) = Ablock;
        A_.block(3, 3, 3, 3) = Ablock;
        A_.block(6, 6, 3, 3) = Ablock;
        return A_;
    }

    /// @brief Computes the B matrix
    /// x_{k_1} = A*x_k + B*u_k
    ///
    /// @return The B matrix
    etools::Matrix9x3 computeB()
    {
        double t = pbParams_.t_;
        Eigen::Vector3d Bblock;
        Bblock(0) = t * t * t / 6.0;
        Bblock(1) = t * t / 2.0;
        Bblock(2) = t;

        B_.setZero();
        B_.block(0, 0, 3, 1) = Bblock;
        B_.block(3, 1, 3, 1) = Bblock;
        B_.block(6, 2, 3, 1) = Bblock;
        return B_;
    }

    /// @brief Computes the D matrix
    /// y_{k_1} = D*x_k + E*u_k
    ///
    /// @return The D matrix
    etools::Matrix2x9 computeD()
    {
        double t = pbParams_.t_;
        D_.setZero();
        D_(0, 0) = 1;
        D_(0, 1) = t;
        D_(0, 2) = t * t / 2.0 - pbParams_.h_CoM_ / pbParams_.g_;
        D_(1, 3) = 1;
        D_(1, 4) = t;
        D_(1, 5) = t * t / 2.0 - pbParams_.h_CoM_ / pbParams_.g_;
        return D_;
    }

    /// @brief Computes the E matrix
    /// y_{k_1} = D*x_k + E*u_k
    ///
    /// @return The E matrix
    etools::Matrix2x3 computeE()
    {
        double t = pbParams_.t_;
        E_.setZero();
        E_(0, 0) = t * t * t / 6 - pbParams_.h_CoM_ * t / pbParams_.g_;
        E_(1, 1) = t * t * t / 6 - pbParams_.h_CoM_ * t / pbParams_.g_;
        return E_;
    }

  public:
    /// @brief Main constructor of the MPC problem, based on the problems parameters
    ///
    /// @param pbParam problems parameters
    SimpleMPC(const ProblemParameters &pbParam)
        : pbParams_(pbParam), velocity_selector_(3, 1),
          stepPlan_(pbParams_.leftStepsParameters_,
                    pbParams_.rightStepsParameters_, pbParams_.t_),
          currentStepIndex_(0),
          logger_(pbParams_.t_, stepPlan_, stepPlan_.rightFoot(),
                  stepPlan_.leftFoot(), pbParams_) {
      // compute all the A, B, D, E matrices
      computeA();
      computeB();
      computeD();
      computeE();
      // condense the A, B, D, E matrices to get the Ux, Uu, Ox and Ou matrices
      condenseTimeInvariant(Ux_, Uu_, pbParams_.nHorizon_, A_, B_);
      condenseOutput(Ox_, Ou_, D_, E_, Ux_, Uu_);
    }

    /// @brief Getter for pbParams
    const ProblemParameters& pbParams() const { return pbParams_; }
    /// @brief Getter for velocity_selector
    const etools::SelectionMatrix& velocity_selector() const { return velocity_selector_; }
    /// @brief Getter for Uu
    const Eigen::MatrixXd& Uu() const { return Uu_; }
    /// @brief Getter for Ux
    const Eigen::MatrixXd& Ux() const { return Ux_; }
    /// @brief Getter for Ou
    const Eigen::MatrixXd& Ou() const { return Ou_; }
    /// @brief Getter for Ox
    const Eigen::MatrixXd& Ox() const { return Ox_; }
    /// @brief Getter for currentState
    const etools::Vector9& currentState() const { return currentState_; }
    /// @brief Getter for logger
    const Logger& logger() const { return logger_; }
    /// @brief Getter for stepPlan
    const StepPlan& stepPlan() const { return stepPlan_; }

    ///@brief Update control problem from model
    ///
    ///@param[in] model     model of the system
    ///@param[in] problem_parameters
    ///
    ///@return ControlProblemStatus::OK/ControlProblemStatus::STOPPED
    ControlProblemStatus::Status update(
        const humoto::wpg05::Model& model,
        const humoto::wpg05::ProblemParameters& problem_parameters)
    {
        sol_structure_.reset();
        // Add a variable of size 3*n called JERK_VARIABLE_ID to the structure of the solution
        sol_structure_.addSolutionPart("JERK_VARIABLE_ID",
                                       problem_parameters.nHorizon_ * 3);

        currentState_ = model.state_.getStateVector();

        return (ControlProblemStatus::OK);
    }

    ///@brief Get next model state.
    /// Computes Xˆ_k+1 = A.Xˆ_k + B.dddX_k
    ///
    ///@param[in] solution  solution
    ///@param[in] model model
    ///
    ///@return next model state.
    humoto::wpg05::ModelState getNextModelState(const humoto::Solution& solution,
                                                  const humoto::wpg05::Model& model)
    {
        humoto::wpg05::ModelState state;

        currentState_ = model.state_.getStateVector();

        etools::Vector9 newState;
        newState = A_ * currentState_ + B_ * solution.x_.segment(0, 3);
        state.updateFromVector(newState);

        // Add the new state and control to the logger
        logger_.addStateAndControl(newState, solution.x_.segment(0, 3));

        std::cout << "currentStepIndex: " << currentStepIndex_ << std::endl;
        currentStepIndex_++;

        return (state);
    }

    /// @brief Getter for PreviewHorizonLength
    size_t getPreviewHorizonLength() const { return pbParams_.nHorizon_; }
    /// @brief Getter for currentStepIndex
    size_t currentStepIndex() const { return currentStepIndex_; }

    /// @brief Log
    ///
    /// @param[in,out] logger logger
    /// @param[in] parent parent
    /// @param[in] name name
    void log(humoto::Logger& logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
             const LogEntryName& parent = LogEntryName(),
             const std::string& name = "simple_mpc") const
    {
        LogEntryName subname = parent;
        subname.add(name);
    }
};
}
}

