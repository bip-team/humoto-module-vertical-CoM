/**
    @file
    @author  Matteo Ciocca
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
         * @brief [task_cvel.m]
         */
        class HUMOTO_LOCAL TaskCoMPosition: public humoto::TaskAB
        {
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_PARENT_CLASS(TaskAB)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS


            protected:
                void setDefaults()
                {
                    TaskAB::setDefaults();
                    setGain(0.707106781186548);
                }


                void finalize()
                {
                    TaskAB::finalize();
                }


            public:
                TaskCoMPosition(const double gain = 0.7106781186548) : TaskAB("TaskCoMPosition", gain)
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::wpg04::MPCforWPG  &mpc = dynamic_cast <const humoto::wpg04::MPCforWPG &> (control_problem);


                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getB();

                    Eigen::VectorXd cpos_ref;

                    cpos_ref.resize(mpc.getPreviewHorizonLength()*2);

                    for (std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
                    {
                      cpos_ref.segment(i*2, 2) = mpc.preview_horizon_.intervals_[i].cpos_ref_;
                    }

                    A.noalias() = getGain() * (mpc.position_selector_ * mpc.S_); // Sp
                    b.noalias() = -getGain() * (mpc.position_selector_ * mpc.s_ /*sp*/ - cpos_ref);

                };
        };
    }
}
