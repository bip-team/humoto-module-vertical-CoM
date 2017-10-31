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
class HUMOTO_LOCAL MPCVerticalMotion : public humoto::MPC
{
  private:
    // timestep
    double t_;

    // Value of zeta at previous iteration
    double zeta_;

    /// @brief Lower bound for zeta
    double zetaMin_;
    /// @brief Upper bound for zeta
    double zetaMax_;

    // Matrices used to compute A and B
    Eigen::Vector3d Bblock_;
    Eigen::Matrix3d Ablock_;

    // Matrices to update the state
    // x_{k_1} = A*x_k + B*u_k
    // y_{k} = C*x_k
    // y_{k_1} = D*x_k + E*u_k
    etools::Matrix9 A_;
    etools::Matrix9x3 B_;
    etools::Matrix6x9 C_;
    etools::Matrix6x9 D_;
    etools::Matrix6x3 E_;

    // Condensed update matrices
    // v_x = Ux*x_0 + Uu*v_u
    // v_y = Ox*x_0 + Ou*v_u
    Eigen::MatrixXd Ux_;
    Eigen::MatrixXd Uu_;
    Eigen::MatrixXd Ox_;
    Eigen::MatrixXd Ou_;

    /// @brief Current state of the model
    etools::Vector9 current_state_;
    /// @brief reference to the parameters of the problem
    const ProblemParameters& pb_params_;
    /// @brief Matrix to select velocities out of the state vector
    etools::SelectionMatrix velocity_selector_;
    /// @brief Plan for the steps to take (predefined)
    StepPlan step_plan_;
    /// @brief Current step index
    size_t current_step_index_;
    /// @brief logger
    Logger logger_;

    Eigen::Matrix3d computeAblock()
    {
        Ablock_.setIdentity();
        Ablock_(0, 1) = t_;
        Ablock_(0, 2) = t_ * t_ / 2.0;
        Ablock_(1, 2) = t_;
        return Ablock_;
    }
    Eigen::Vector3d computeBblock()
    {
        Bblock_(0) = t_ * t_ * t_ / 6.0;
        Bblock_(1) = t_ * t_ / 2.0;
        Bblock_(2) = t_;
        return Bblock_;
    }

    /// @brief Computes the A matrix
    /// x_{k_1} = A*x_k + B*u_k
    ///
    /// @return The A matrix
    etools::Matrix9 computeA()
    {
        A_.setZero();
        A_.block(0, 0, 3, 3) = Ablock_;
        A_.block(3, 3, 3, 3) = Ablock_;
        A_.block(6, 6, 3, 3) = Ablock_;
        return A_;
    }

    /// @brief Computes the B matrix
    /// x_{k_1} = A*x_k + B*u_k
    ///
    /// @return The B matrix
    etools::Matrix9x3 computeB()
    {
        B_.setZero();
        B_.block(0, 0, 3, 1) = Bblock_;
        B_.block(3, 1, 3, 1) = Bblock_;
        B_.block(6, 2, 3, 1) = Bblock_;
        return B_;
    }

    /// @brief Computes the C matrix
    /// z_{k} = C*x_k
    ///
    /// @return The C matrix
    etools::Matrix6x9 computeC()
    {
        Eigen::Vector3d Cblock;
        Cblock(0) = 1;
        Cblock(1) = 0;
        Cblock(2) = -zetaMin_;
        C_.setZero();
        C_.block(0, 0, 1, 3) = Cblock.transpose();
        C_.block(1, 3, 1, 3) = Cblock.transpose();
        C_.block(2, 6, 1, 3) = Cblock.transpose();
        Cblock(2) = -zetaMax_;
        C_.block(3, 0, 1, 3) = Cblock.transpose();
        C_.block(4, 3, 1, 3) = Cblock.transpose();
        C_.block(5, 6, 1, 3) = Cblock.transpose();
        return C_;
    }

    /// @brief Computes the D matrix
    /// y_{k_1} = D*x_k + E*u_k
    ///
    /// @return The D matrix
    etools::Matrix6x9 computeD()
    {
        D_ = C_ * A_;
        return D_;
    }

    /// @brief Computes the E matrix
    /// y_{k_1} = D*x_k + E*u_k
    ///
    /// @return The E matrix
    etools::Matrix6x3 computeE()
    {
        E_ = C_ * B_;
        return E_;
    }

  public:
    /// @brief Main constructor of the MPC problem, based on the problems parameters
    ///
    /// @param pbParam problems parameters
    MPCVerticalMotion(const ProblemParameters& pbParam)
        : pb_params_(pbParam),
          velocity_selector_(3, 1),
          step_plan_(pb_params_.leftStepsParameters_, pb_params_.rightStepsParameters_,
                     pb_params_.t_),
          current_step_index_(0),
          logger_(pb_params_.t_, step_plan_, pb_params_)
    {
        t_ = pb_params_.t_;
        zeta_ = pb_params_.zetaZero_;
        zetaMin_ = zeta_ - pb_params_.zetaSpan_ / 2;
        zetaMax_ = zeta_ + pb_params_.zetaSpan_ / 2;
        // compute all the A, B, D, E matrices
        computeAblock();
        computeBblock();
        computeA();
        computeB();
        computeC();
        computeD();
        computeE();
        // condense the A, B, D, E matrices to get the Ux, Uu, Ox and Ou matrices
        condenseTimeInvariant(Ux_, Uu_, pb_params_.n_, A_, B_);
        condenseOutput(Ox_, Ou_, D_, E_, Ux_, Uu_);
    }

    /// @brief Getter for pbParams
    const ProblemParameters& pbParams() const { return pb_params_; }
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
    const etools::Vector9& currentState() const { return current_state_; }
    /// @brief Getter for logger
    const Logger& logger() const { return logger_; }
    /// @brief Getter for stepPlan
    const StepPlan& stepPlan() const { return step_plan_; }

    /// @brief Getter for zetaMin
    const double& zetaMin() const { return zetaMin_; }
    /// @brief Getter for zetaMax
    const double& zetaMax() const { return zetaMax_; }

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
        sol_structure_.addSolutionPart("JERK_VARIABLE_ID", problem_parameters.n_ * 3);

        current_state_ = model.state_.getStateVector();

        // The following matrices need to be recomputed because zetaMin and zetaMax changed
        computeC();
        computeD();
        computeE();
        condenseOutput(Ox_, Ou_, D_, E_, Ux_, Uu_);

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

        current_state_ = model.state_.getStateVector();

        etools::Vector9 newState;
        newState = A_ * current_state_ + B_ * solution.x_.segment(0, 3);
        state.updateFromVector(newState);

        zeta_ = (state.position_(2) - step_plan_.z()(currentStepIndex())) /
                (state.acceleration_(2) + pb_params_.g_);

        // Add the new state and control to the logger
        logger_.addStateAndControl(newState, solution.x_.segment(0, 3), zeta_);

        std::cout << "currentStepIndex: " << current_step_index_ << std::endl;
        current_step_index_++;

        return (state);
    }

    /// @brief Getter for PreviewHorizonLength
    size_t getPreviewHorizonLength() const { return pb_params_.n_; }
    /// @brief Getter for currentStepIndex
    size_t currentStepIndex() const { return current_step_index_; }

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

