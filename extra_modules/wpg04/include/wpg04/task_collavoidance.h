/**
    @file
    @author  Matteo Ciocca
    @author  Jan Michalczyk

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace wpg04
    {
        class HUMOTO_LOCAL TaskCollAvoidance : public humoto::TaskAL
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskAL)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                void setDefaults()
                {
                    TaskAL::setDefaults();
                }


                void finalize()
                {
                    TaskAL::finalize();
                }


            public:
                TaskCollAvoidance() : TaskAL("TaskCollAvoidance")
                {

                }


                // @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::wpg04::MPCforWPG& mpc = dynamic_cast<const humoto::wpg04::MPCforWPG &>(control_problem);
                    const humoto::wpg04::Model& model = dynamic_cast<const humoto::wpg04::Model&>(model_base);
                    std::size_t number_of_constraints_per_obstacle = mpc.getPreviewHorizonLength();
                    std::size_t number_of_obstacles = model.obstacles_.size();
                    std::size_t number_of_constraints = number_of_constraints_per_obstacle * number_of_obstacles;
                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getLowerBounds();

                    if (!model.obstacles_.empty())
                    {
                        A.resize(number_of_constraints, sol_structure.getNumberOfVariables());
                        b.resize(number_of_constraints);

                        for (std::size_t i = 0; i < number_of_obstacles; ++i)
                        {
                            A.block(i * number_of_constraints_per_obstacle, 0, number_of_constraints_per_obstacle, sol_structure.getNumberOfVariables()) = model.obstacles_[i]->getA();
                            b.segment(i * number_of_constraints_per_obstacle, number_of_constraints_per_obstacle) = model.obstacles_[i]->getBounds();
                        }
                    }
                    else
                    {
                        A.resize(number_of_constraints_per_obstacle, sol_structure.getNumberOfVariables());
                        b.resize(number_of_constraints_per_obstacle);
                        A.setZero();
                        b.setZero();
                    }
                };


                /// @copydoc humoto::TaskBase::guessActiveSet
                void guessActiveSet(const humoto::SolutionStructure &sol_structure,
                                    const humoto::Model &model_base,
                                    const humoto::ControlProblem &control_problem)
                {
                    const humoto::wpg04::MPCforWPG& mpc = dynamic_cast<const humoto::wpg04::MPCforWPG &>(control_problem);
                    const humoto::wpg04::Model& model = dynamic_cast<const humoto::wpg04::Model&>(model_base);
                    std::size_t number_of_obstacles = model.obstacles_.size();
                    std::size_t number_of_constraints_per_obstacle = mpc.getPreviewHorizonLength();
                    std::size_t number_of_constraints = number_of_constraints_per_obstacle * number_of_obstacles;

                    if (getActualActiveSet().size() == 0)
                    {
                        getActiveSetGuess().initialize(number_of_constraints, ConstraintActivationType::INACTIVE);
                    }
                    else
                    {
                        HUMOTO_ASSERT((getActualActiveSet().size() == number_of_constraints),
                                      "The number of obstacles is not supposed to change.");

                        getActiveSetGuess().initialize(number_of_constraints, ConstraintActivationType::INACTIVE);

                        humoto::ActiveSetConstraints tmp;
                        tmp.initialize(number_of_constraints_per_obstacle - 1);

                        for (std::size_t i = 0; i < number_of_obstacles; ++i)
                        {
                            humoto::Location from(i * number_of_constraints_per_obstacle + 1, number_of_constraints_per_obstacle - 1);
                            tmp.copyFrom(getActualActiveSet(), from);

                            humoto::Location to(i * number_of_constraints_per_obstacle, number_of_constraints_per_obstacle - 1);
                            getActiveSetGuess().copyTo(to, tmp);
                        }
                    }
                }
        };
    }
}
