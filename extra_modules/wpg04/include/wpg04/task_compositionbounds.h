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

                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd lb_aux;
                    Eigen::VectorXd ub_aux;
                    etools::Vector2 lb_aux_segment;
                    etools::Vector2 ub_aux_segment;
                    Eigen::VectorXd &lb = getLowerBounds();
                    Eigen::VectorXd &ub = getUpperBounds();
                    Eigen::VectorXd sv;

                    // dC = Sv * X + sv
                    // dC = [c^x_1 c^y_1 c^x_2 c^y_2 ...]
                    sv = mpc.velocity_selector_ * mpc.s_; /*sv*/

                    lb_aux_segment << mpc.preview_horizon_.intervals_[1].com_vel_bound_x_(0),
                    mpc.preview_horizon_.intervals_[1].com_vel_bound_y_(0);

                    ub_aux_segment << mpc.preview_horizon_.intervals_[1].com_vel_bound_x_(1),
                    mpc.preview_horizon_.intervals_[1].com_vel_bound_y_(1);

                    lb_aux.resize(number_of_constraints);
                    ub_aux.resize(number_of_constraints);
                    lb.resize(number_of_constraints);
                    ub.resize(number_of_constraints);

                    for (std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
                    {
                        //lb.segment(i*2, 2) = mpc.preview_horizon_.getCoMVelBounds(i).col(0);
                        //ub.segment(i*2, 2) = mpc.preview_horizon_.getCoMVelBounds(i).col(1);
                        lb_aux.segment(i*2, 2) = lb_aux_segment;
                        ub_aux.segment(i*2, 2) = ub_aux_segment;
                    }

                    // lb - sv =< Sv * X =< lb - sv
                    lb.noalias() = lb_aux - sv;
                    ub.noalias() = ub_aux - sv;
                    A.noalias()  = mpc.velocity_selector_ * mpc.S_; // Sv

                    // for (std::size_t i = 0; i < loc_var.length_/2; ++i)
                    // {
                    //     I[i*2]     = loc_var.offset_ + i*2;
                    //     I[i*2+1]   = loc_var.offset_ + i*2 + 1;
                    //
                    //     lb.segment(i*2, 2) = mpc.preview_horizon_.getCoMVelBounds(i).col(0);
                    //     ub.segment(i*2, 2) = mpc.preview_horizon_.getCoMVelBounds(i).col(1);
                    // }
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
        };
    }
}
