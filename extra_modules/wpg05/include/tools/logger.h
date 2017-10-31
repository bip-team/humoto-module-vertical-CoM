/**
    @file
    @author  Stanislas Brossette

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>

namespace humoto
{
namespace wpg05
{
/// @brief Class used to log and display the CoM and CoP history
class HUMOTO_LOCAL Logger
{
  public:
    /// @brief Constructor
    Logger(double timeStep, const StepPlan& stepPlan, const ProblemParameters& pbParams)
        : rightFootTraj_(stepPlan.rightFoot()),
          leftFootTraj_(stepPlan.leftFoot()),
          timeStep_(timeStep),
          pbParams_(pbParams)
    {
        xMin_ = stepPlan.xMin();
        xMax_ = stepPlan.xMax();
        yMin_ = stepPlan.yMin();
        yMax_ = stepPlan.yMax();
        zMin_ = stepPlan.z();
        zMax_ = stepPlan.z();
        gravity_ = pbParams_.g_;
        for (long i = 0; i < stepPlan.z().size(); ++i)
        {
            zMin_(i) += (pbParams_.zetaZero_ - pbParams_.zetaSpan_ / 2) * gravity_;
            zMax_(i) += (pbParams_.zetaZero_ + pbParams_.zetaSpan_ / 2) * gravity_;
        }
    }

    /// @brief Adds a single state and control to the history
    ///
    /// @param state state
    /// @param control control
    void addStateAndControl(const etools::Vector9& state, const etools::Vector3& control)
    {
        addStateAndControl(state, control, pbParams_.zetaZero_);
    }
    void addStateAndControl(const etools::Vector9& state, const etools::Vector3& control,
                            const double& zeta)
    {
        Eigen::Vector3d position, velocity, acceleration, cop, copMin, copMax;
        position << state(0), state(3), state(6);
        velocity << state(1), state(4), state(7);
        acceleration << state(2), state(5), state(8);
        positionsCoM_.push_back(position);
        velocitiesCoM_.push_back(velocity);
        accelerationsCoM_.push_back(acceleration);
        jerksCoM_.push_back(control);
        double zetaMin = zeta - pbParams_.zetaSpan_ / 2;
        double zetaMax = zeta + pbParams_.zetaSpan_ / 2;
        copMin(0) = position(0) - zetaMin * acceleration(0);
        copMin(1) = position(1) - zetaMin * acceleration(1);
        copMin(2) = position(2) - zetaMin * (acceleration(2) + gravity_);
        cop(0) = position(0) - zeta * acceleration(0);
        cop(1) = position(1) - zeta * acceleration(1);
        cop(2) = position(2) - zeta * (acceleration(2) + gravity_);
        copMax(0) = position(0) - zetaMax * acceleration(0);
        copMax(1) = position(1) - zetaMax * acceleration(1);
        copMax(2) = position(2) - zetaMax * (acceleration(2) + gravity_);
        positionsCoPMin_.push_back(copMin);
        positionsCoP_.push_back(cop);
        positionsCoPMax_.push_back(copMax);
    }

    Eigen::MatrixXd getPositionsAsMatrix() const { return toMatrix(positionsCoM_); }
    Eigen::MatrixXd getVelocitiesAsMatrix() const { return toMatrix(velocitiesCoM_); }
    Eigen::MatrixXd getAccelerationsAsMatrix() const { return toMatrix(accelerationsCoM_); }
    Eigen::MatrixXd getJerksAsMatrix() const { return toMatrix(jerksCoM_); }
    Eigen::MatrixXd getCoPMinsAsMatrix() const { return toMatrix(positionsCoPMin_); }
    Eigen::MatrixXd getCoPsAsMatrix() const { return toMatrix(positionsCoP_); }
    Eigen::MatrixXd getCoPMaxsAsMatrix() const { return toMatrix(positionsCoPMax_); }

    /// @brief Transforms a list of vector3 into a matrix
    ///
    /// @param vec list of vector3
    ///
    /// @return Matrix concatenation of the vectors
    Eigen::MatrixXd toMatrix(const std::vector<Eigen::Vector3d>& vec) const
    {
        Eigen::MatrixXd mat;
        mat.resize(vec.size(), 3);
        for (size_t i = 0; i < vec.size(); ++i) mat.row(i) << vec[i].transpose();
        return mat;
    }

    /// @brief Getter for size
    size_t size() const { return positionsCoM_.size(); }

    /// @brief Prints the history of the CoM
    void print() const
    {
        std::cout << "Position History:" << std::endl;
        std::cout << toMatrix(positionsCoM_) << std::endl;
        std::cout << "Velocity History:" << std::endl;
        std::cout << toMatrix(velocitiesCoM_) << std::endl;
        std::cout << "Acceleration History:" << std::endl;
        std::cout << toMatrix(accelerationsCoM_) << std::endl;
        std::cout << "Jerk History:" << std::endl;
        std::cout << toMatrix(jerksCoM_) << std::endl;
    }

    /// @brief Writes a python file that can later be executed to display the history of the system
    void plot() const
    {
        Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
        std::ofstream logFile("plotFile.py");

        logFile << "import matplotlib as mpl\n";
        logFile << "from mpl_toolkits.mplot3d import Axes3D\n";
        logFile << "import numpy as np\n";
        logFile << "import matplotlib.pyplot as plt\n";
        logFile << "t = np.arange(0.," << (double)size() * (double)timeStep_ << ", " << timeStep_
                << ")\n\n";

        Eigen::MatrixXd positions(toMatrix(positionsCoM_));
        Eigen::MatrixXd positionsCoPMin(toMatrix(positionsCoPMin_));
        Eigen::MatrixXd positionsCoPMax(toMatrix(positionsCoPMax_));
        Eigen::MatrixXd positionsCoP(toMatrix(positionsCoP_));
        Eigen::MatrixXd velocities(toMatrix(velocitiesCoM_));
        Eigen::MatrixXd accelerations(toMatrix(accelerationsCoM_));
        Eigen::MatrixXd jerks(toMatrix(jerksCoM_));

        logFile << "x = np.array(" << positions.col(0).transpose().format(cleanFmt) << ")\n";
        logFile << "y = np.array(" << positions.col(1).transpose().format(cleanFmt) << ")\n";
        logFile << "z = np.array(" << positions.col(2).transpose().format(cleanFmt) << ")\n";
        logFile << "xCoPMin = np.array(" << positionsCoPMin.col(0).transpose().format(cleanFmt)
                << ")\n";
        logFile << "yCoPMin = np.array(" << positionsCoPMin.col(1).transpose().format(cleanFmt)
                << ")\n";
        logFile << "zCoPMin = np.array(" << positionsCoPMin.col(2).transpose().format(cleanFmt)
                << ")\n";
        logFile << "xCoP = np.array(" << positionsCoP.col(0).transpose().format(cleanFmt) << ")\n";
        logFile << "yCoP = np.array(" << positionsCoP.col(1).transpose().format(cleanFmt) << ")\n";
        logFile << "zCoP = np.array(" << positionsCoP.col(2).transpose().format(cleanFmt) << ")\n";
        logFile << "xCoPMax = np.array(" << positionsCoPMax.col(0).transpose().format(cleanFmt)
                << ")\n";
        logFile << "yCoPMax = np.array(" << positionsCoPMax.col(1).transpose().format(cleanFmt)
                << ")\n";
        logFile << "zCoPMax = np.array(" << positionsCoPMax.col(2).transpose().format(cleanFmt)
                << ")\n";
        logFile << "xMin = np.array(" << xMin_.transpose().format(cleanFmt) << ")\n";
        logFile << "yMin = np.array(" << yMin_.transpose().format(cleanFmt) << ")\n";
        logFile << "zMin = np.array(" << zMin_.transpose().format(cleanFmt) << ")\n";
        logFile << "xMax = np.array(" << xMax_.transpose().format(cleanFmt) << ")\n";
        logFile << "yMax = np.array(" << yMax_.transpose().format(cleanFmt) << ")\n";
        logFile << "zMax = np.array(" << zMax_.transpose().format(cleanFmt) << ")\n";
        logFile << "dx = np.array(" << velocities.col(0).transpose().format(cleanFmt) << ")\n";
        logFile << "dy = np.array(" << velocities.col(1).transpose().format(cleanFmt) << ")\n";
        logFile << "dz = np.array(" << velocities.col(2).transpose().format(cleanFmt) << ")\n";
        logFile << "ddx = np.array(" << accelerations.col(0).transpose().format(cleanFmt) << ")\n";
        logFile << "ddy = np.array(" << accelerations.col(1).transpose().format(cleanFmt) << ")\n";
        logFile << "ddz = np.array(" << accelerations.col(2).transpose().format(cleanFmt) << ")\n";
        logFile << "dddx = np.array(" << jerks.col(0).transpose().format(cleanFmt) << ")\n";
        logFile << "dddy = np.array(" << jerks.col(1).transpose().format(cleanFmt) << ")\n";
        logFile << "dddz = np.array(" << jerks.col(2).transpose().format(cleanFmt) << ")\n";

        logFile << "time = np.array(" << rightFootTraj_.t_.transpose().format(cleanFmt) << ")\n";
        logFile << "rFootX = np.array(" << rightFootTraj_.x_.transpose().format(cleanFmt) << ")\n";
        logFile << "rFootY = np.array(" << rightFootTraj_.y_.transpose().format(cleanFmt) << ")\n";
        logFile << "rFootZ = np.array(" << rightFootTraj_.z_.transpose().format(cleanFmt) << ")\n";

        logFile << "lFootX = np.array(" << leftFootTraj_.x_.transpose().format(cleanFmt) << ")\n";
        logFile << "lFootY = np.array(" << leftFootTraj_.y_.transpose().format(cleanFmt) << ")\n";
        logFile << "lFootZ = np.array(" << leftFootTraj_.z_.transpose().format(cleanFmt) << ")\n";


        logFile << "time = np.array(" << rightFootTraj_.t_.transpose().format(cleanFmt) << ")\n";
        logFile << "trajRFootX = np.array(" << rightFootTraj_.x_.transpose().format(cleanFmt) << ")\n";
        logFile << "trajRFootY = np.array(" << rightFootTraj_.y_.transpose().format(cleanFmt) << ")\n";
        logFile << "trajRFootZ = np.array(" << rightFootTraj_.z_.transpose().format(cleanFmt) << ")\n";
        logFile << "trajLFootX = np.array(" << leftFootTraj_.x_.transpose().format(cleanFmt) << ")\n";
        logFile << "trajLFootY = np.array(" << leftFootTraj_.y_.transpose().format(cleanFmt) << ")\n";
        logFile << "trajLFootZ = np.array(" << leftFootTraj_.z_.transpose().format(cleanFmt) << ")\n";

        /**************************
        *  PLOT FOOT TRAJECTORY  *
        **************************/
        //logFile << "plt.plot(time, trajRFootX, 'r', label='RFootX')\n";
        //logFile << "plt.plot(time, trajRFootY, 'g', label='RFootY')\n";
        //logFile << "plt.plot(time, trajRFootZ, 'b', label='RFootZ')\n";
        //logFile << "plt.legend()\n";
        //logFile << "plt.show()\n";
        //logFile << "plt.plot(time, trajLFootX, 'r', label='LFootX')\n";
        //logFile << "plt.plot(time, trajLFootY, 'g', label='LFootY')\n";
        //logFile << "plt.plot(time, trajLFootZ, 'b', label='LFootZ')\n";
        //logFile << "plt.legend()\n";
        //logFile << "plt.show()\n";

        /***********************************
        *  PLOT COM AND COP TRAJECTORIES  *
        ***********************************/
        logFile << "f, (ax1, ax2, ax3) = plt.subplots(3, sharex=False, sharey=False)\n";
        logFile << "ax1.plot(t, x, 'r-', label='xCoM')\n";
        logFile << "ax2.plot(t, y, 'b-', label='yCoM')\n";
        logFile << "ax3.plot(t, z, 'g-', label='zCoM')\n";
        logFile << "ax1.plot(t, xMin[0:len(t)], 'r^', label='xCoMMin')\n";
        logFile << "ax2.plot(t, yMin[0:len(t)], 'b^', label='yCoMMin')\n";
        logFile << "ax3.plot(t, zMin[0:len(t)], 'g^', label='zCoMMin')\n";
        logFile << "ax1.plot(t, xMax[0:len(t)], 'rv', label='xCoMMax')\n";
        logFile << "ax2.plot(t, yMax[0:len(t)], 'bv', label='yCoMMax')\n";
        logFile << "ax3.plot(t, zMax[0:len(t)], 'gv', label='zCoMMax')\n";
        logFile << "ax1.plot(t, xCoPMin, 'r--', label='xCoPMin')\n";
        logFile << "ax2.plot(t, yCoPMin, 'b--', label='yCoPMin')\n";
        logFile << "ax3.plot(t, zCoPMin, 'g--', label='zCoPMin')\n";
        logFile << "ax1.plot(t, xCoP, 'r,', label='xCoP')\n";
        logFile << "ax2.plot(t, yCoP, 'b,', label='yCoP')\n";
        logFile << "ax3.plot(t, zCoP, 'g,', label='zCoP')\n";
        logFile << "ax1.plot(t, xCoPMax, 'r-.', label='xCoPMax')\n";
        logFile << "ax2.plot(t, yCoPMax, 'b-.', label='yCoPMax')\n";
        logFile << "ax3.plot(t, zCoPMax, 'g-.', label='zCoPMax')\n";
        logFile << "ax1.set_xlabel('Time (s)')\n";
        logFile << "ax2.set_xlabel('Time (s)')\n";
        logFile << "ax3.set_xlabel('Time (s)')\n";
        logFile << "ax1.set_ylabel('x (m)')\n";
        logFile << "ax2.set_ylabel('y (m)')\n";
        logFile << "ax3.set_ylabel('z (m)')\n";
        logFile << "ax1.legend(loc='upper center', shadow=True)\n";
        logFile << "ax2.legend(loc='upper center', shadow=True)\n";
        logFile << "ax3.legend(loc='upper center', shadow=True)\n";
        logFile << "plt.axis('auto')\n";
        logFile << "plt.show()\n";

        /************************************
        *  PLOT ALL TRAJECTORIES TOGETHER  *
        ************************************/
        logFile << "fig = plt.figure()\n";
        logFile << "ax = fig.gca(projection='3d')\n";
        logFile << "ax.plot(lFootX, lFootY, lFootZ, 'r', label='Left foot')\n";
        logFile << "ax.plot(rFootX, rFootY, rFootZ, 'b', label='Right foot')\n";
        logFile << "ax.plot(x, y, z, 'g', label='CoM')\n";
        logFile << "ax.plot(xCoP, yCoP, zCoP, 'y', label='CoP')\n";
        logFile << "ax.legend()\n";
        logFile << "plt.show()\n";
        logFile.close();
    }

  private:
    std::vector<Eigen::Vector3d> positionsCoM_;
    std::vector<Eigen::Vector3d> positionsCoPMin_;
    std::vector<Eigen::Vector3d> positionsCoP_;
    std::vector<Eigen::Vector3d> positionsCoPMax_;
    std::vector<Eigen::Vector3d> velocitiesCoM_;
    std::vector<Eigen::Vector3d> accelerationsCoM_;
    std::vector<Eigen::Vector3d> jerksCoM_;
    Eigen::VectorXd xMin_;
    Eigen::VectorXd xMax_;
    Eigen::VectorXd yMin_;
    Eigen::VectorXd yMax_;
    Eigen::VectorXd zMin_;
    Eigen::VectorXd zMax_;
    FootTraj rightFootTraj_, leftFootTraj_;
    double timeStep_;
    const ProblemParameters& pbParams_;
    double gravity_;
};
} /* wpg05 */
} /* humoto */
