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
/// @brief Class containing the model of the system to be controlled
class HUMOTO_LOCAL ModelState : public humoto::ModelState,
                                public humoto::config::RelaxedConfigurableBase
{
  //TODO this class should only contain a PointMassState instead of position, velocity and acceleration
  protected:
/// Those macros define the necessary tools to read the variables from a yaml configuration file
#define HUMOTO_CONFIG_SECTION_ID "ModelState"
#define HUMOTO_CONFIG_ENTRIES         \
    HUMOTO_CONFIG_COMPOUND_(position) \
    HUMOTO_CONFIG_COMPOUND_(velocity) \
    HUMOTO_CONFIG_COMPOUND_(acceleration)
#define HUMOTO_CONFIG_CONSTRUCTOR ModelState
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

  public:
    /// @brief State of the CoM
    etools::Vector3 position_;
    etools::Vector3 velocity_;
    etools::Vector3 acceleration_;

  public:
    /// @brief Default constructor
    ModelState() { setDefaults(); }
    ModelState(const etools::Vector9& vec) { updateFromVector(vec); }

    /// @brief Sets the default values of the model state
    void setDefaults()
    {
        position_ << 0, 0, 0.8;
        velocity_ << 0, 0, 0;
        acceleration_ << 0, 0, 0;
    }

    double x() const { return position_(0); }
    double y() const { return position_(1); }
    double z() const { return position_(2); }
    double dx() const { return velocity_(0); }
    double dy() const { return velocity_(1); }
    double dz() const { return velocity_(2); }
    double ddx() const { return acceleration_(0); }
    double ddy() const { return acceleration_(1); }
    double ddz() const { return acceleration_(2); }

    /// @brief Getter for position
    const etools::Vector3& position() const { return position_; }
    /// @brief Getter for velocity
    const etools::Vector3& velocity() const { return velocity_; }
    /// @brief Getter for acceleration
    const etools::Vector3& acceleration() const { return acceleration_; }

    /// @brief Returns the current state in vector form:
    /// [x; dx; ddx; y; dy; ddy; z; dz; ddz]
    etools::Vector9 getStateVector() const
    {
        etools::Vector9 currentState;
        currentState(0) = position_(0);
        currentState(1) = velocity_(0);
        currentState(2) = acceleration_(0);
        currentState(3) = position_(1);
        currentState(4) = velocity_(1);
        currentState(5) = acceleration_(1);
        currentState(6) = position_(2);
        currentState(7) = velocity_(2);
        currentState(8) = acceleration_(2);
        return currentState;
    }

    /// @brief updates the position, velocity and acceleration fields from a 9-vector
    void updateFromVector(const etools::Vector9 &vec)
    {
        position_(0) = vec(0);
        velocity_(0) = vec(1);
        acceleration_(0) = vec(2);
        position_(1) = vec(3);
        velocity_(1) = vec(4);
        acceleration_(1) = vec(5);
        position_(2) = vec(6);
        velocity_(2) = vec(7);
        acceleration_(2) = vec(8);
    }

    /// @brief Log
    ///
    /// @param[in,out] logger logger
    /// @param[in] parent parent
    /// @param[in] name name
    void log(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
             const LogEntryName &parent = LogEntryName(),
             const std::string &name = "model_state") const
    {
        // LogEntryName subname = parent;
        // subname.add(name);
        // com_state_.log(logger, subname, "com_state");
    }
};
}
}

