/**
    @file
    @author  Stanislas Brossette
    @author  Alexander Sherikov

    @copyright 2017 INRIA. Licensed under the Apache License, Version 2.0. (see
    LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
namespace wpg05
{
/// @brief Class containing the parameters of the problem. It can be configured through yaml file
class HUMOTO_LOCAL ProblemParameters : public humoto::config::RelaxedConfigurableBase
{
  public:
    /// @brief Gravity constant
    double g_;
    /// @brief height of center of mass
    double h_CoM_;

    /*****************************************
     *  ZETA PARAMETERS FOR VERTICAL MOTION  *
     *****************************************/
    /// @brief Initial value of Zeta { zeta = (c_z - p_z) / (ddc_z + g_z) }
    double zetaZero_;
    /// @brief Span in which zeta can be: { zeta_k - zetaSpan/2 < zeta_{k+1} < zeta_k + zetaSpan/2}
    double zetaSpan_;

    /***************************************
     *  TIME PARAMETERS OF THE SIMULATION  *
     ***************************************/
    /// @brief length of one time step
    double t_;
    size_t sampling_time_ms_;
    size_t subsampling_time_ms_;
    size_t tds_sampling_time_ms_;

    /// @brief number of time steps in horizon
    size_t nHorizon_;
    size_t preview_horizon_length_;

    /// @brief total number of iterations to reach endTime_
    size_t nIterations_;
    /// @brief End time of the control
    double endTime_;

    /******************************************
     *  REFERENCE HEIGHT AND VELOCITY OF CoM  *
     ******************************************/
    /// @brief target com speed
    etools::Vector3 comVelRef_;
    /// @brief target com height
    double comHeightRef_;

    /****************
     *  STEP PLANS  *
     ****************/
    /// @brief parameters of the right stepping plan
    std::vector<std::vector<double> > leftStepsParameters_;
    /// @brief parameters of the right stepping plan
    std::vector<std::vector<double> > rightStepsParameters_;
    /// @brief height of the foot trajectory between steps
    double stepHeight_;

    /***********************
     *  CONSTRAINTS GAINS  *
     ***********************/
    /// @brief gain of the CoM velocity task
    double gainTaskVelocity_;
    /// @brief gain of the CoM Height task
    double gainTaskCoMHeight_;
    /// @brief gain of the Min Jerk task
    double gainTaskMinJerk_;
    /// @brief gain of the CoP bounds task
    double gainTaskCoPBounds_;
    /// @brief gain of the CoP position reference task
    double gainTaskCoPPosRef_;
    /// @brief gain of the kinematics rectangle task
    double gainTaskKinematicsRectangle_;
    /// @brief gain of the kinematics polygon task
    double gainTaskKinematicsPolygon_;

    /***************************************************
     *  KINEMATIC DIMENSIONS FOR RECTANGLE CONSTRAINT  *
     ***************************************************/
    /// @brief Minimum height of the CoM in kinematic task
    double kinematicLimitZmin_;
    /// @brief Maximum height of the CoM in kinematic task
    double kinematicLimitZmax_;
    /// @brief Width of the reachable X of the CoM in kinematic task
    double kinematicLimitXSpan_;
    /// @brief Width of the reachable Y of the CoM in kinematic task
    double kinematicLimitYSpan_;

    /***********************************************
     *  KINEMATIC PARAMETERS DESCRIBING THE ROBOT  *
     ***********************************************/
    /// @brief Vector from sole center to ankle
    Eigen::Vector3d soleToAnkle_;
    /// @brief Vector from CoM to right hip
    Eigen::Vector3d comToRightHip_;
    /// @brief Vector from CoM to left hip
    Eigen::Vector3d comToLeftHip_;
    /// @brief Width of foot along the X direction (forward)
    double footXwidth_;
    /// @brief Width of foot along the Y direction (sideway)
    double footYwidth_;

    /*******************
    *  MISCELLANEOUS  *
    *******************/
    int verbose_;


  protected:
/// Those macros define the necessary tools to read the variables from a yaml configuration file
#define HUMOTO_CONFIG_SECTION_ID "ProblemParameters"
#define HUMOTO_CONFIG_ENTRIES                          \
    HUMOTO_CONFIG_SCALAR_(g)                           \
    HUMOTO_CONFIG_SCALAR_(h_CoM)                       \
    HUMOTO_CONFIG_SCALAR_(zetaZero)                    \
    HUMOTO_CONFIG_SCALAR_(zetaSpan)                    \
    HUMOTO_CONFIG_SCALAR_(t)                           \
    HUMOTO_CONFIG_SCALAR_(preview_horizon_length)      \
    HUMOTO_CONFIG_SCALAR_(sampling_time_ms)            \
    HUMOTO_CONFIG_SCALAR_(subsampling_time_ms)         \
    HUMOTO_CONFIG_SCALAR_(tds_sampling_time_ms)        \
    HUMOTO_CONFIG_SCALAR_(nHorizon)                    \
    HUMOTO_CONFIG_SCALAR_(endTime)                     \
    HUMOTO_CONFIG_COMPOUND_(comVelRef)                 \
    HUMOTO_CONFIG_SCALAR_(comHeightRef)                \
    HUMOTO_CONFIG_COMPOUND_(leftStepsParameters)       \
    HUMOTO_CONFIG_COMPOUND_(rightStepsParameters)      \
    HUMOTO_CONFIG_SCALAR_(stepHeight)                  \
    HUMOTO_CONFIG_SCALAR_(gainTaskVelocity)            \
    HUMOTO_CONFIG_SCALAR_(gainTaskCoPPosRef)           \
    HUMOTO_CONFIG_SCALAR_(gainTaskMinJerk)             \
    HUMOTO_CONFIG_SCALAR_(gainTaskKinematicsRectangle) \
    HUMOTO_CONFIG_SCALAR_(gainTaskKinematicsPolygon)   \
    HUMOTO_CONFIG_SCALAR_(kinematicLimitZmin)          \
    HUMOTO_CONFIG_SCALAR_(kinematicLimitZmax)          \
    HUMOTO_CONFIG_SCALAR_(kinematicLimitXSpan)         \
    HUMOTO_CONFIG_SCALAR_(kinematicLimitYSpan)         \
    HUMOTO_CONFIG_SCALAR_(gainTaskCoPBounds)           \
    HUMOTO_CONFIG_COMPOUND_(soleToAnkle)               \
    HUMOTO_CONFIG_COMPOUND_(comToRightHip)             \
    HUMOTO_CONFIG_COMPOUND_(comToLeftHip)              \
    HUMOTO_CONFIG_SCALAR_(footXwidth)                  \
    HUMOTO_CONFIG_SCALAR_(footYwidth)                  \
    HUMOTO_CONFIG_SCALAR_(verbose)
#define HUMOTO_CONFIG_CONSTRUCTOR ProblemParameters
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

  public:
    /// @brief Default constructor
    ProblemParameters() { setDefaults(); }

    /// @brief Finalizes the configuration of this class after all other parameters have been set
    void finalize() { nIterations_ = endTime_ / t_; }

    /// @brief Sets the default parameters of the walk
    void setDefaults()
    {
        g_ = humoto::g_gravitational_acceleration;
        h_CoM_ = 0.8;
        // zetaMin_ = -10;
        // zetaMax_ = 10;
        zetaZero_ = 0;
        zetaSpan_ = 20;
        t_ = 0.005;
        nHorizon_ = 100;
        endTime_ = 10;
        stepHeight_ = 0.2;
        comVelRef_(0) = 0.1;
        comVelRef_(1) = 0;
        comVelRef_(2) = 0;
        gainTaskVelocity_ = 100;
        gainTaskCoMHeight_ = 100;
        gainTaskCoPBounds_ = 100;
        gainTaskCoPPosRef_ = 100;
        gainTaskMinJerk_ = 100;
        footXwidth_ = 0.2;
        footYwidth_ = 0.1;
        verbose_ = 0;
        setDefaultStepParameters();
    }

    /// @brief Sets the default values of the stepping plan
    void setDefaultStepParameters()
    {
        // Create the default arrays of step parameters (x,y,z,tMin,tMax)
        double l0[] = {0, 0.10, 0.0, 0.0, 2.00};
        double r0[] = {0, -0.10, 0.0, 0.0, 2.99};
        double l1[] = {0.2, 0.10, 0.0, 2.91, 3.99};
        double r1[] = {0.4, -0.10, 0.0, 3.91, 4.99};
        double l2[] = {0.6, 0.10, 0.0, 4.91, 5.99};
        double r2[] = {0.8, -0.10, 0.0, 5.91, 7.99};
        double l3[] = {1.0, 0.10, 0.0, 7.91, 8.99};
        double r3[] = {1.3, -0.10, 0.0, 8.91, 30.0};
        double l4[] = {1.6, 0.10, 0.0, 9.91, 30.0};
        // Populate step plan parameters vectors with parameters
        leftStepsParameters_.push_back(std::vector<double>(l0, l0 + 5));
        rightStepsParameters_.push_back(std::vector<double>(r0, r0 + 5));
        leftStepsParameters_.push_back(std::vector<double>(l1, l1 + 5));
        rightStepsParameters_.push_back(std::vector<double>(r1, r1 + 5));
        leftStepsParameters_.push_back(std::vector<double>(l2, l2 + 5));
        rightStepsParameters_.push_back(std::vector<double>(r2, r2 + 5));
        leftStepsParameters_.push_back(std::vector<double>(l3, l3 + 5));
        rightStepsParameters_.push_back(std::vector<double>(r3, r3 + 5));
        leftStepsParameters_.push_back(std::vector<double>(l4, l4 + 5));
    }

    void print() const
    {
    std::cout << "g_: " << g_ << std::endl;
    std::cout << "h_CoM_: " << h_CoM_ << std::endl;

    /*****************************************
     *  ZETA PARAMETERS FOR VERTICAL MOTION  *
     *****************************************/
    std::cout << "zetaZero_: " << zetaZero_ << std::endl;
    std::cout << "zetaSpan_: " << zetaSpan_ << std::endl;

    /***************************************
     *  TIME PARAMETERS OF THE SIMULATION  *
     ***************************************/
    std::cout << "t_: " << t_ << std::endl;
    std::cout << "sampling_time_ms_: " << sampling_time_ms_ << std::endl;
    std::cout << "subsampling_time_ms_: " << subsampling_time_ms_ << std::endl;
    std::cout << "tds_sampling_time_ms_: " << tds_sampling_time_ms_ << std::endl;

    std::cout << "nHorizon_: " << nHorizon_ << std::endl;
    std::cout << "preview_horizon_length_: " << preview_horizon_length_ << std::endl;

    std::cout << "nIterations_: " << nIterations_ << std::endl;
    std::cout << "endTime_: " << endTime_ << std::endl;

    /******************************************
     *  REFERENCE HEIGHT AND VELOCITY OF CoM  *
     ******************************************/
    std::cout << "comVelRef_: " << comVelRef_.transpose() << std::endl;
    std::cout << "comHeightRef_: " << comHeightRef_ << std::endl;

    /****************
     *  STEP PLANS  *
     ****************/
    std::cout << "Left steps parameters:" << std::endl;
    for(size_t i = 0; i < leftStepsParameters_.size(); ++i)
    {
      for(size_t j = 0; j < leftStepsParameters_.at(i).size(); ++j)
        std::cout << leftStepsParameters_.at(i).at(j) << ", ";
      std::cout << std::endl;
    }
    std::cout << "Right steps parameters:" << std::endl;
    for(size_t i = 0; i < rightStepsParameters_.size(); ++i)
    {
      for(size_t j = 0; j < rightStepsParameters_.at(i).size(); ++j)
        std::cout << rightStepsParameters_.at(i).at(j) << ", ";
      std::cout << std::endl;
    }

    std::cout << "stepHeight_: " << stepHeight_ << std::endl;

    /***********************
     *  CONSTRAINTS GAINS  *
     ***********************/
    std::cout << "gainTaskCoPBounds_: " << gainTaskCoPBounds_ << std::endl;
    std::cout << "gainTaskKinematicsPolygon_: " << gainTaskKinematicsPolygon_ << std::endl;
    std::cout << "gainTaskCoMHeight_: " << gainTaskCoMHeight_ << std::endl;
    std::cout << "gainTaskVelocity_: " << gainTaskVelocity_ << std::endl;
    std::cout << "gainTaskMinJerk_: " << gainTaskMinJerk_ << std::endl;
    std::cout << "gainTaskCoPPosRef_: " << gainTaskCoPPosRef_ << std::endl;

    /***************************************************
     *  KINEMATIC DIMENSIONS FOR RECTANGLE CONSTRAINT  *
     ***************************************************/
    std::cout << "kinematicLimitZmin_: " << kinematicLimitZmin_ << std::endl;
    std::cout << "kinematicLimitZmax_: " << kinematicLimitZmax_ << std::endl;
    std::cout << "kinematicLimitXSpan_: " << kinematicLimitXSpan_ << std::endl;
    std::cout << "kinematicLimitYSpan_: " << kinematicLimitYSpan_ << std::endl;

    /***********************************************
     *  KINEMATIC PARAMETERS DESCRIBING THE ROBOT  *
     ***********************************************/
    std::cout << "soleToAnkle_: " << soleToAnkle_.transpose() << std::endl;
    std::cout << "comToRightHip_: " << comToRightHip_.transpose() << std::endl;
    std::cout << "comToLeftHip_: " << comToLeftHip_.transpose() << std::endl;
    std::cout << "footXwidth_: " << footXwidth_ << std::endl;
    std::cout << "footYwidth_: " << footYwidth_ << std::endl;

    /*******************
    *  MISCELLANEOUS  *
    *******************/
    std::cout << "verbose_: " << verbose_ << std::endl;
    }
};
}
}

