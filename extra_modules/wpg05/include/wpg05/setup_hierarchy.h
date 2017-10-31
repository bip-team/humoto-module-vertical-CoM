/**
    @file
    @author  Stanislas Brossette
    @author  Alexander Sherikov
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

/// @brief Setup the hierarchy of tasks of the problem for a walk on horizontal floor with
/// predefined steps
///
/// @param opt_problem The optimization problem on which tasks will be added
/// @param params The problem's parameters (for the tasks gains)
void setupHierarchy_v0(humoto::OptimizationProblem& opt_problem,
                       const humoto::wpg05::ProblemParameters& params)
{
    // tasks, which are used in the control problem
    humoto::TaskSharedPointer task_cop_bounds(
        new humoto::wpg05::TaskCoPBounds(params.gainTaskCoPBounds_));
    humoto::TaskSharedPointer task_cop_pos_ref(
        new humoto::wpg05::TaskCoPPosRef(params.gainTaskCoPPosRef_));
    humoto::TaskSharedPointer task_com_velocity(
        new humoto::wpg05::TaskCoMVelocity(params.gainTaskVelocity_));
    humoto::TaskSharedPointer task_min_jerk(new humoto::TaskZeroVariables(params.gainTaskMinJerk_));

    // reset the optimization problem
    opt_problem.reset(2);

    // push tasks into the stack/hierarchy
    opt_problem.pushTask(task_cop_bounds, 0);
    opt_problem.pushTask(task_cop_pos_ref, 1);
    opt_problem.pushTask(task_com_velocity, 1);
    opt_problem.pushTask(task_min_jerk, 1);
}

/// @brief Setup the hierarchy of tasks of the problem for a walk up and down stairs with vertical
/// motion of the CoM with predefined steps
///
/// @param opt_problem The optimization problem on which tasks will be added
/// @param params The problem's parameters (for the tasks gains)
void setupHierarchy_v1(humoto::OptimizationProblem& opt_problem,
                       const humoto::wpg05::ProblemParameters& params)
{
    // tasks, which are used in the control problem
    humoto::TaskSharedPointer task_cop_bounds(
        new humoto::wpg05::TaskCoPBoundsVerticalMotion(params.gainTaskCoPBounds_));
    humoto::TaskSharedPointer task_com_height(
        new humoto::wpg05::TaskCoMHeight(params.gainTaskCoMHeight_));
    humoto::TaskSharedPointer task_cop_pos_ref(
        new humoto::wpg05::TaskCoPPosRef(params.gainTaskCoPPosRef_));
    humoto::TaskSharedPointer task_com_velocity(
        new humoto::wpg05::TaskCoMVelocity(params.gainTaskVelocity_));
    humoto::TaskSharedPointer task_min_jerk(new humoto::TaskZeroVariables(params.gainTaskMinJerk_));
    humoto::TaskSharedPointer task_kinematics_rectangle(
        new humoto::wpg05::TaskKinematicsRectangle(params.gainTaskKinematicsRectangle_));

    // reset the optimization problem
    opt_problem.reset(2);

    // push tasks into the stack/hierarchy
    opt_problem.pushTask(task_cop_bounds, 0);
    opt_problem.pushTask(task_kinematics_rectangle, 0);
    opt_problem.pushTask(task_com_height, 1);
    opt_problem.pushTask(task_cop_pos_ref, 1);
    opt_problem.pushTask(task_com_velocity, 1);
    opt_problem.pushTask(task_min_jerk, 1);
}
