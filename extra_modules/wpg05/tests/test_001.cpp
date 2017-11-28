/**
    @file
    @author  Stanislas Brossette
    @author  Alexander Sherikov

    @copyright 2017 INRIA. Licensed under the Apache License, Version 2.0. (see
    LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

#define HUMOTO_GLOBAL_LOGGER_ENABLED

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
#include "humoto/humoto.h"
#include "humoto/qpoases.h"
#include "humoto/wpg05.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);

/// @brief Implementation and resolution of the problem described in test_000.yaml

int main(int argc, char *argv[])
{
    // All the execution is in a try scope in order to be able to catch exceptions
    std::string config_folder = "extra_modules/wpg05/tests/";
    std::string file_name = "test_001.yaml";
    if (argc >= 2) file_name = argv[1];
    std::string config_file_name = config_folder + file_name;

    // yaml configuration file reader
    std::cout << "Read config" << std::endl;
    humoto::config::yaml::Reader config_reader(config_file_name);

    // optimization problem (a stack of tasks / hierarchy)
    humoto::OptimizationProblem opt_problem;

    // parameters of the solver
    std::cout << "Solver parameters" << std::endl;
    humoto::qpoases::SolverParameters solver_parameters(config_reader);

    // Actual solver (initialized with solver_parameters)
    std::cout << "Solver" << std::endl;
    humoto::qpoases::Solver solver(solver_parameters);

    // Structure that will contain the solutions
    humoto::qpoases::Solution solution;

    // Problems parameters
    std::cout << "Problem parameters" << std::endl;
    humoto::wpg05::ProblemParameters problem_parameters(config_reader);

    // model and state representing the controlled system
    std::cout << "Model" << std::endl;
    humoto::wpg05::ModelState model_state(config_reader);
    humoto::wpg05::Model model(model_state);

    // control problem, which is used to construct an optimization problem
    std::cout << "MPC" << std::endl;
    humoto::wpg05::MPCVerticalMotion mpc(problem_parameters, model_state);

    // Populate the optimization problem
    std::cout << "Hierarchy" << std::endl;
    setupHierarchy_v1(opt_problem, problem_parameters);

    try
    {
        humoto::wpg05::TaskKinematicsPolygon::computeAndLogHighestFeasibleZ(
            mpc, model_state.position_, mpc.logger());
        for (unsigned int i = 0; i < problem_parameters.nIterations_; ++i)
        {
            // if update fails, exit
            if (mpc.update(model, problem_parameters) != humoto::ControlProblemStatus::OK)
            {
                break;
            }
            // form the optimization problem with the current state of the model
            opt_problem.form(solution, model, mpc);
            // Solve the problem and put the result in solution
            solver.solve(solution, opt_problem);

            {
                humoto::HierarchyLevel hlvl0, hlvl1;
                hlvl0 = opt_problem[0];
                hlvl1 = opt_problem[1];

                humoto::constraints::ContainerAL ineqCstr;
                const humoto::SolutionStructure &sol_str = mpc.getSolutionStructure();
                hlvl0.getInequalityConstraints(ineqCstr, sol_str);
                Eigen::MatrixXd A = ineqCstr.getA();
                Eigen::VectorXd l = ineqCstr.getLowerBounds();

                Eigen::VectorXd diff = A * solution.x_ - l;
                for (long i = 0; i < diff.rows(); ++i)
                    if (diff[i] < -1e-4)
                        std::cout << "Violated Cstr " << i << ": " << diff[i] << std::endl;
            }

            // Update the model and its state
            model_state = mpc.getNextModelState(solution, model);
            model.updateState(model_state);

            // Check kin cstr nonlinear
            {
                humoto::wpg05::TaskKinematicsPolygon::computeAndLogHighestFeasibleZ(
                    mpc, model_state.position_, mpc.logger());
                humoto::wpg05::GeneratedKinematicConstraint generatedCstr;
                Eigen::Vector3d s2a = mpc.pbParams().soleToAnkle_;
                Eigen::Vector3d com2rHip = mpc.pbParams().comToRightHip_;
                Eigen::Vector3d com2lHip = mpc.pbParams().comToLeftHip_;
                const humoto::wpg05::FootTraj &rFootTraj = mpc.rightFootTraj();
                const humoto::wpg05::FootTraj &lFootTraj = mpc.leftFootTraj();
                Eigen::Vector3d CoM = model_state.position_;
                Eigen::Vector3d rA2rHip =
                    (CoM + com2rHip) - (rFootTraj(mpc.currentStepIndex()) + s2a);
                Eigen::Vector3d lA2lHip =
                    (CoM + com2lHip) - (lFootTraj(mpc.currentStepIndex()) + s2a);

                if (rA2rHip.norm() >= generatedCstr.radius_ ||
                    lA2lHip.norm() >= generatedCstr.radius_ ||
                    rA2rHip.z() <= generatedCstr.height_ || rA2rHip.z() <= generatedCstr.height_)
                {
                    std::cout << "CHECK KINEMATIC CSTR NONLINEAR:" << std::endl;
                    std::cout << "mpc.currentStepIndex(): " << mpc.currentStepIndex() << std::endl;
                }
                if (rA2rHip.norm() >= generatedCstr.radius_)
                    std::cout << "rA2rHip.norm(): " << rA2rHip.norm() << "\n" << std::endl;
                if (lA2lHip.norm() >= generatedCstr.radius_)
                    std::cout << "lA2lHip.norm(): " << lA2lHip.norm() << "\n" << std::endl;
                if (rA2rHip.z() <= generatedCstr.height_)
                    std::cout << "rA2rHip.z(): " << rA2rHip.z() << "\n" << std::endl;
                if (rA2rHip.z() <= generatedCstr.height_)
                    std::cout << "lA2lHip.z(): " << lA2lHip.z() << "\n" << std::endl;
            }
        }
    }
    catch (const std::exception &e)
    {
        // Log the caught exceptions
        HUMOTO_LOG_RAW(e.what());
    }
    // Write the python file plotFile.py
    mpc.logger().plot();

    // Execute the command { python plotFile.py } in terminal
    std::string command = "python3 plotFile.py";
    system(command.c_str());

    // All Done
    HUMOTO_LOG_RAW("Done.");
    return (0);
}
