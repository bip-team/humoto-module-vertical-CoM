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
/// @brief Wrapper class that handles the model state and its updates
class HUMOTO_LOCAL Model : public humoto::Model
{
  public:
    /// @brief state of the model
    humoto::wpg05::ModelState state_;

  public:
    /// @brief Default constructor
    Model() {}
    /// @brief Constructor with model State to copy
    Model(const humoto::ModelState &mState) { updateState(mState); }

    /// @brief Update model state
    ///
    /// @param[in] model_state model state
    void updateState(const humoto::ModelState &model_state)
    {
        const humoto::wpg05::ModelState &newState =
            dynamic_cast<const humoto::wpg05::ModelState &>(model_state);
        state_.position_ = newState.position_;
        state_.velocity_ = newState.velocity_;
        state_.acceleration_ = newState.acceleration_;
    }

    /// @brief Log
    ///
    /// @param[in, out] logger logger
    /// @param[in] parent parent
    /// @param[in] name name
    void log(humoto::Logger &logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
             const LogEntryName &parent = LogEntryName(), const std::string &name = "model") const
    {
        LogEntryName subname = parent;
        subname.add(name);

        state_.log(logger, subname, "state");
    }
};
}
}
