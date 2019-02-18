/**
    @file
    @author Jan Michalczyk
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
         * @brief Circle obstacle class.
         */
        class HUMOTO_LOCAL Circle : public humoto::obstacle_avoidance::ObstacleBase
        {

            #define HUMOTO_CONFIG_SECTION_ID "Circle"
            #define HUMOTO_CONFIG_CONSTRUCTOR Circle
            #define HUMOTO_CONFIG_ENTRIES \
                HUMOTO_CONFIG_SCALAR_(radius) \
                HUMOTO_CONFIG_PARENT_CLASS(humoto::obstacle_avoidance::ObstacleBase)
            #include HUMOTO_CONFIG_DEFINE_ACCESSORS

            public:
                double radius_;

            public:
                /**
                 * @brief Constructor.
                 *
                 * @param[in] position
                 * @param[in] rpy
                 */
                Circle(const etools::Vector3& position,
                       const etools::Vector3& rpy,
                       const double           radius) : ObstacleBase(position, rpy), radius_(radius)
                {
                }


                /**
                 * @brief Default constructor.
                 */
                Circle()
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
                                       const humoto::Solution&       old_solution,
                                       const double                  safety_margin)
                {
                    const humoto::wpg04::MPCforWPG& mpc =
                           dynamic_cast<const humoto::wpg04::MPCforWPG&>(control_problem);

                    /*
                    const humoto::qpoases::Solution& previous_solution =
                           dynamic_cast<const humoto::qpoases::Solution&>(old_solution);
                    */

                    // previous CoM position
                    Eigen::VectorXd prev_com_pos(2 * mpc.getPreviewHorizonLength());
                    // previous obstacle position
                    Eigen::VectorXd prev_M(2 * mpc.getPreviewHorizonLength());
                    // normal matrix
                    Eigen::MatrixXd n(mpc.getPreviewHorizonLength(), 2 * mpc.getPreviewHorizonLength());
                    // safety distance
                    Eigen::VectorXd D(mpc.getPreviewHorizonLength());
                    // current obstacle position
                    Eigen::VectorXd M(2 * mpc.getPreviewHorizonLength());

                    // condensation of the previous CoM position
                    prev_com_pos  = mpc.position_selector_ * mpc.prev_S_ * old_solution.get_x() + mpc.position_selector_ * mpc.prev_s_;

                    // condensation of the previous state of the obstacle
                    // in the static case also the current state of the obstacle
                    for(std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
                    {
                        prev_M.segment(i * 2, 2) << position_(0),position_(1);

                        // this condensation is only valid in the case that the
                        // obstacle is considered static.
                        M.segment(i * 2, 2) << prev_M.segment(i * 2, 2);
                    }

                    // all zeros inside the n matrix
                    n.setZero();

                    // fill the n matrix with the normal vectors and the safety
                    // distance
                    for(std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
                    {
                        // a block is composed by the normal (x,y) coordinates
                        n.block(i, i * 2, 1, 2) << (prev_com_pos(i*2) - prev_M(i*2)) / (prev_com_pos.segment(i * 2, 2) - prev_M.segment(i * 2, 2)).norm(),
                                                  (prev_com_pos(i*2+1) - prev_M(i*2+1)) / (prev_com_pos.segment(i * 2, 2) - prev_M.segment(i * 2, 2)).norm();

                        // the safety distance might vary including uncertanties
                        D(i) = safety_margin + radius_;
                    }

                    // n * Sp
                    A_.noalias() = n * (mpc.position_selector_ * mpc.S_);
                    // D + n * (M - sp)
                    ub_.noalias() = D + n * (M - mpc.position_selector_ * mpc.s_);
                }
        };
    }
}
