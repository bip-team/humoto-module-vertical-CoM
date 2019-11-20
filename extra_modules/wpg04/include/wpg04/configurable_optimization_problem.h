/**
    @file
    @author Jan Michalczyk
    @author Alexander Sherikov
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
         * @brief Class representing the hierarchy of the problem
         */
        class HUMOTO_LOCAL ConfigurableOptimizationProblem
            :   public humoto::ConfigurableOptimizationProblem
        {
            protected:
                /**
                 * @brief Fill map with all pointers to all tasks for given module
                 */
                humoto::TaskSharedPointer getTask(const std::string &string_id) const
                {
                    if (string_id == "TaskCoMVelocity")
                    {
                        return (humoto::TaskSharedPointer(new humoto::wpg04::TaskCoMVelocity));
                    }
                    if (string_id == "TaskCoPPosition")
                    {
                        return (humoto::TaskSharedPointer(new humoto::wpg04::TaskCoPPosition));
                    }
                    if (string_id == "TaskCoPVelocity")
                    {
                        return (humoto::TaskSharedPointer(new humoto::wpg04::TaskCoPVelocity));
                    }
                    if (string_id == "TaskFootstepBounds")
                    {
                        return (humoto::TaskSharedPointer(new humoto::wpg04::TaskFootstepBounds));
                    }
                    if (string_id == "TaskCoPPositionBounds")
                    {
                        return (humoto::TaskSharedPointer(new humoto::wpg04::TaskCoPPositionBounds));
                    }
                    if (string_id == "TaskTerminalConstraint")
                    {
                        return (humoto::TaskSharedPointer(new humoto::wpg04::TaskTerminalConstraint));
                    }
                    if (string_id == "TaskCollisionAvoidance")
                    {
                        return (humoto::TaskSharedPointer(new humoto::wpg04::TaskCollisionAvoidance));
                    }
                    if (string_id == "TaskCoMPosition")
                    {
                        return (humoto::TaskSharedPointer(new humoto::wpg04::TaskCoMPosition));
                    }
                    if (string_id == "TaskCoMVelocityBounds")
                    {
                        return (humoto::TaskSharedPointer(new humoto::wpg04::TaskCoMVelocityBounds));
                    }
                    if (string_id == "TaskCoMPositionBounds")
                    {
                        return (humoto::TaskSharedPointer(new humoto::wpg04::TaskCoMPositionBounds));
                    }

                    return(humoto::ConfigurableOptimizationProblem::getTask(string_id));
                }
        };
    }
}
