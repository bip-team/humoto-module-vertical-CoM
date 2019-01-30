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
                    const humoto::wpg04::MPCforWPG& mpc =
                           dynamic_cast<const humoto::wpg04::MPCforWPG &>(control_problem);
                    
                    const humoto::wpg04::Model&   model =
                                     dynamic_cast<const humoto::wpg04::Model&>(model_base);
                    
                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getLowerBounds();

                    if(!model.obstacles_.empty())
                    {
                        A.resize(model.obstacles_.size() * mpc.getPreviewHorizonLength(),
                                                               sol_structure.getNumberOfVariables());
                        b.resize(model.obstacles_.size() * mpc.getPreviewHorizonLength());

                        for(std::size_t i = 0; i < model.obstacles_.size(); ++i)
                        {
                            A.block(i * mpc.getPreviewHorizonLength(), 0, 
                                    mpc.getPreviewHorizonLength(), sol_structure.getNumberOfVariables()) = model.obstacles_[i]->getA();
                            
                            b.segment(i * mpc.getPreviewHorizonLength(), mpc.getPreviewHorizonLength())  = model.obstacles_[i]->getBounds();
                        }
                    }
                    else
                    {
                        A.resize(mpc.getPreviewHorizonLength(), sol_structure.getNumberOfVariables());
                        b.resize(mpc.getPreviewHorizonLength());
                        A.setZero();
                        b.setZero();
                    }
                };
        };
    }
}
