/**
    @file
    @author Nestor Bohorquez
    @copyright 2014-2019 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)
    @brief
*/

#pragma once

namespace humoto
{
    namespace wpg04
    {
        /**
         * @brief Wall obstacle class.
         */
        class HUMOTO_LOCAL Wall : public humoto::obstacle_avoidance::ObstacleBase
        {
            public:
                /**
                 * @brief Constructor.
                 *
                 * @param[in] position
                 * @param[in] rpy
                 */
                Wall(const etools::Vector3& position,
                     const etools::Vector3& rpy) : ObstacleBase(position, rpy)
                {
                }


                /**
                 * @brief Default constructor.
                 */
                Wall()
                {
                }


                /**
                 * @brief Initialize constraints.
                 *
                 * @param[in] control_problem
                 */
                void resetConstraints(const humoto::ControlProblem& control_problem)
                {
                    const humoto::wpg04::MPCforWPG& mpc =
                           dynamic_cast<const humoto::wpg04::MPCforWPG&>(control_problem);

                    A_.resize(mpc.getPreviewHorizonLength(), mpc.getNumberOfDecisionVariables());
                    ub_.resize(mpc.getPreviewHorizonLength());
                    A_.setZero();
                    ub_.setZero();
                }


                /**
                 * @brief Update constraints.
                 *
                 * @param[in] control_problem
                 * @param[in] old_solution
                 * @param[in] safety_margin
                 */
                void updateConstraints(const humoto::ControlProblem& control_problem,
                                       const humoto::Solution&       solution, 
                                       const double                  safety_margin)
                {
                    const humoto::wpg04::MPCforWPG& mpc =
                           dynamic_cast<const humoto::wpg04::MPCforWPG&>(control_problem);

                    // Get the Cartesian normal vector from the RPY orientation of the wall
                    etools::Vector3 normal;
                    normal << cos(rpy_(2))*cos(rpy_(1)), sin(rpy_(2))*cos(rpy_(1)), sin(rpy_(1));

                    // normal matrix
                    Eigen::MatrixXd N(mpc.getPreviewHorizonLength(), 2 * mpc.getPreviewHorizonLength());
                    // safety distance
                    Eigen::VectorXd D(mpc.getPreviewHorizonLength());
                    // wall position
                    Eigen::VectorXd O(2 * mpc.getPreviewHorizonLength());

                    // condensation of the position of the wall. Only valid
                    // because the wall is considered static.
                    for(std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
                    {
                        // this condensation is 
                        O.segment(i * 2, 2) << position_(0), position_(1);
                        // the safety distance might vary including uncertanties
                        D(i) = safety_margin;
                    }

                    // all zeros inside the N matrix
                    N.setZero();

                    // fill the N matrix with the normal vectors and the safety
                    // distance
                    for(std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
                    {
                        // a block is composed by the normal (x,y) coordinates
                        N.block(i, i * 2, 1, 2) << normal(0), normal(1);
                    }

                    // N * Sp
                    A_.noalias() = N * (mpc.position_selector_ * mpc.S_);
                    // D + N * (O - sp)
                    ub_.noalias() = D + N * (O - mpc.position_selector_ * mpc.s_);
                }
        };
    }
}
