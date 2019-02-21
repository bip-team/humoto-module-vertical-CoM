/**
    @file
    @author  Alexander Sherikov
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
        /**
         * @brief 
         */
        class HUMOTO_LOCAL TaskCoMPosBounds : public humoto::TaskALU
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskALU)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS

            protected:
                void setDefaults()
                {
                    TaskALU::setDefaults();
                }


            public:
                TaskCoMPosBounds() : TaskALU("TaskCoMPosBounds")
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(const humoto::SolutionStructure &sol_structure,
                          const humoto::Model &model_base,
                          const humoto::ControlProblem &control_problem)
                {
                    const humoto::wpg04::MPCforWPG& mpc = dynamic_cast <const humoto::wpg04::MPCforWPG &> (control_problem);
                    std::size_t number_of_constraints = mpc.getPreviewHorizonLength()*2;
                    std::size_t number_of_variables = sol_structure.getNumberOfVariables();

                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &lb = getLowerBounds();
                    Eigen::VectorXd &ub = getUpperBounds();

                    Eigen::VectorXd lb_aux;
                    Eigen::VectorXd ub_aux;
                    Eigen::VectorXd sp;

                    // dC = Sp * X + sp
                    // dF = Vfp * X + vfp

                    A.resize(number_of_constraints, number_of_variables);
                    lb_aux.resize(number_of_constraints);
                    ub_aux.resize(number_of_constraints);
                    lb.resize(number_of_constraints);
                    ub.resize(number_of_constraints);

                    for (std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
                    {
                        lb_aux.segment(i*2, 2) = mpc.preview_horizon_.getCoMBounds(i).col(0);
                        ub_aux.segment(i*2, 2) = mpc.preview_horizon_.getCoMBounds(i).col(1);
                    }

                    // lb - sp + vfp =< (Sp - Vfp) * X =< lb - sp + vfp
                    lb.noalias() = lb_aux - mpc.position_selector_ * mpc.s_ + mpc.vfp_;
                    ub.noalias() = ub_aux - mpc.position_selector_ * mpc.s_ + mpc.vfp_;
                    A.noalias()  = mpc.position_selector_ * mpc.S_ - mpc.Vfp_; // Sp - Vfp
                }


                /// @copydoc humoto::TaskBase::guessActiveSet
                void guessActiveSet(const humoto::SolutionStructure &sol_structure,
                                    const humoto::Model &model_base,
                                    const humoto::ControlProblem &control_problem)
                {

                    const humoto::wpg04::MPCforWPG& mpc = dynamic_cast <const humoto::wpg04::MPCforWPG &> (control_problem);
                    std::size_t number_of_constraints = mpc.getPreviewHorizonLength()*2;
        
                    if (getActualActiveSet().size() == 0)
                    {
                        getActiveSetGuess().initialize(number_of_constraints, ConstraintActivationType::INACTIVE);
                    }
                    else
                    {
                        HUMOTO_ASSERT((getActualActiveSet().size() == number_of_constraints),
                                      "The number of CoP variables is not supposed to change.");
        
                        getActiveSetGuess() = getActualActiveSet();
                        getActiveSetGuess().shift(2, ConstraintActivationType::INACTIVE);
                    }
                }
        };
    }
}
