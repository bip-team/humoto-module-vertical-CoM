/**
    @file
    @author  Stanislas Brossette
    @author  Alexander Sherikov

    @copyright 2017 INRIA. Licensed under the Apache License, Version 2.0. (see
    LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief This file regroups all the necessary includes for this project
*/

#pragma once

#include "humoto/walking.h"

#include "wpg05/problem_parameters.h"
#include "wpg05/step_plan.h"
#include "wpg05/model_state.h"
#include "wpg05/model.h"
#include "wpg05/logger.h"
//#include "wpg05/simple_mpc.h"
#include "wpg05/mpc_vertical_motion.h"
#include "wpg05/task_com_height.h"
#include "wpg05/task_com_velocity.h"
//#include "wpg05/task_cop_bounds.h"
#include "wpg05/task_cop_bounds_vertical_motion.h"
#include "wpg05/task_cop_pos_ref.h"
#include "wpg05/task_kinematic.h"
#include "wpg05/task_kinematic_rectangle.h"
#include "wpg05/setup_hierarchy.h"
