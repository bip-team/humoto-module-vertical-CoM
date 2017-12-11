/**
    @file
    @author  Stanislas Brossette

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once
#include <Eigen/Core>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace humoto
{
namespace wpg05
{

struct Polynomial3D
{
    double t0;
    etools::Vector3 x0;
    etools::Vector7 ax;
    etools::Vector7 ay;
    etools::Vector7 az;

    void setConstant(double tBegin, etools::Vector3 pos)
    {
      t0 = tBegin;
      x0 = pos;
      ax.setZero();
      ay.setZero();
      az.setZero();
    }

    void plotBetween(double T0, double T1) const
    {
        Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(100, T0, T1);
        Eigen::VectorXd trajX, trajY, trajZ;
        trajX.resize(t.size());
        trajY.resize(t.size());
        trajZ.resize(t.size());
        applyPolynomial(trajX, ax, t, x0[0]);
        applyPolynomial(trajY, ay, t, x0[1]);
        applyPolynomial(trajZ, az, t, x0[2]);
        Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
        std::ofstream logFile("plotPolynomial.py");

        logFile << "import matplotlib as mpl\n";
        logFile << "from mpl_toolkits.mplot3d import Axes3D\n";
        logFile << "import numpy as np\n";
        logFile << "import matplotlib.pyplot as plt\n";
        logFile << "time = np.array(" << t.transpose().format(cleanFmt) << ")\n";
        logFile << "trajX = np.array(" << trajX.transpose().format(cleanFmt) << ")\n";
        logFile << "trajY = np.array(" << trajY.transpose().format(cleanFmt) << ")\n";
        logFile << "trajZ = np.array(" << trajZ.transpose().format(cleanFmt) << ")\n";
        logFile << "plt.plot(time, trajX, 'r', label='X')\n";
        logFile << "plt.plot(time, trajY, 'g', label='Y')\n";
        logFile << "plt.plot(time, trajZ, 'b', label='Z')\n";
        logFile << "plt.legend()\n";
        logFile << "plt.show()\n";
        logFile.close();
        std::string command = "python3 plotPolynomial.py";
        system(command.c_str());
    }

    /// @brief Applies the polynomial defined by coeff to the list of values t and store the result
    /// in res
    void applyPolynomial(Eigen::Ref<Eigen::VectorXd> res, const Eigen::VectorXd& coeff,
                         Eigen::VectorXd t, double x0) const
    {
        // Vector t - t0
        for (long i = 0; i < t.size(); ++i)
        {
            t[i] = t[i] - t0;
        }
        Eigen::VectorXd tmp = Eigen::VectorXd::Constant(t.size(), 1.0);
        Eigen::MatrixXd M(t.size(), coeff.size());
        for (long i = 0; i < coeff.size(); ++i)
        {
            M.col(i) = tmp;
            tmp = tmp.cwiseProduct(t);
        }
        res = M * coeff + Eigen::VectorXd::Constant(t.size(), x0);
    }

    Eigen::Vector3d getPositionAt(double t) const
    {
      etools::Vector7 variables;
      t = t-t0;
      variables[0] = 1;
      variables[1] = variables[0] * t;
      variables[2] = variables[1] * t;
      variables[3] = variables[2] * t;
      variables[4] = variables[3] * t;
      variables[5] = variables[4] * t;
      variables[6] = variables[5] * t;

      Eigen::Vector3d res(ax.dot(variables), ay.dot(variables), az.dot(variables));
      return res;
    }
    etools::Vector6 velCoeff(const etools::Vector7& A) const
    {
      etools::Vector6 diffA;
      diffA[0] = A[1];
      diffA[1] = 2*A[2];
      diffA[2] = 3*A[3];
      diffA[3] = 4*A[4];
      diffA[4] = 5*A[5];
      diffA[5] = 6*A[6];
      return diffA;
    }

    Eigen::Vector3d getVelocityAt(double t) const
    {
      etools::Vector6 variables;
      t = t-t0;
      variables[0] = 1;
      variables[1] = variables[0] * t;
      variables[2] = variables[1] * t;
      variables[3] = variables[2] * t;
      variables[4] = variables[3] * t;
      variables[5] = variables[4] * t;

      etools::Vector6 dax = velCoeff(ax);
      etools::Vector6 day = velCoeff(ay);
      etools::Vector6 daz = velCoeff(az);
      Eigen::Vector3d res(dax.dot(variables), day.dot(variables), daz.dot(variables));
      return res;
    }

    etools::Vector5 accCoeff(const etools::Vector7& A) const
    {
      etools::Vector5 ddiffA;
      ddiffA[0] = 2*A[2];
      ddiffA[1] = 6*A[3];
      ddiffA[2] = 12*A[4];
      ddiffA[3] = 20*A[5];
      ddiffA[4] = 30*A[6];
      return ddiffA;
    }

    Eigen::Vector3d getAccelerationAt(double t) const
    {
      etools::Vector5 variables;
      t = t-t0;
      variables[0] = 1;
      variables[1] = variables[0] * t;
      variables[2] = variables[1] * t;
      variables[3] = variables[2] * t;
      variables[4] = variables[3] * t;

      etools::Vector5 ddax = accCoeff(ax);
      etools::Vector5 dday = accCoeff(ay);
      etools::Vector5 ddaz = accCoeff(az);
      Eigen::Vector3d res(ddax.dot(variables), dday.dot(variables), ddaz.dot(variables));
      return res;
    }
};

struct TrajPiece
{
    TrajPiece(Polynomial3D p, double tBegin, double tEnd) : p_(p), tBegin_(tBegin), tEnd_(tEnd) {}
    Polynomial3D p_;
    double tBegin_;
    double tEnd_;
};

struct FootTraj
{
    Eigen::VectorXd x_, y_, z_, t_;
    std::vector<bool> supportFoot_;
    std::vector<TrajPiece> trajPieces_;

    void resize(int s)
    {
        x_.resize(s);
        y_.resize(s);
        z_.resize(s);
        t_.resize(s);
        supportFoot_.resize(s);
        x_.setZero();
        y_.setZero();
        z_.setZero();
        t_.setZero();
    }

    Eigen::Vector3d operator()(long i) const
    {
        Eigen::Vector3d pos(x_(i), y_(i), z_(i));
        return pos;
    }

    void eval(humoto::rigidbody::RigidBodyState& foot_state, const double time_s) const
    {
        //First we find the correct trajPiece for this time
        size_t index = 0;
        while (!(trajPieces_[index].tBegin_ <= time_s && trajPieces_[index].tEnd_ > time_s))
        {
            index++;
        }

        //Then we compute the quantities through the polynomial3D
        foot_state.position_ = trajPieces_[index].p_.getPositionAt(time_s);
        std::cout << "foot_state.position_: " << foot_state.position_.transpose() << std::endl;
        foot_state.velocity_ = trajPieces_[index].p_.getVelocityAt(time_s);
        std::cout << "foot_state.velocity_: " << foot_state.velocity_.transpose() << std::endl;
        foot_state.acceleration_ = trajPieces_[index].p_.getAccelerationAt(time_s);
        std::cout << "foot_state.acceleration_: " << foot_state.acceleration_.transpose() << std::endl;

        foot_state.rpy_(humoto::AngleIndex::YAW) = 0;
        foot_state.angular_velocity_(humoto::AngleIndex::YAW) = 0;
        foot_state.angular_acceleration_(humoto::AngleIndex::YAW) = 0;
    }

    bool isSupportAt(double time) const
    {
        size_t index = 0;
        while (!(t_[index] <= time && t_[index + 1] > time)) index++;

        if (supportFoot_[index] && supportFoot_[index + 1])
            return true;
        else
            return false;
    }

    void print() const
    {
      std::cout << "print footTraj" << std::endl;
      for(size_t i=0; i< supportFoot_.size() ; ++i)
      {
          std::cout << "t: " << t_[i] << ", pos[" << x_[i] << ", " << y_[i] << ", " << z_[i]
                    << "], support: " << supportFoot_[i] << std::endl;
      }
    }
};


class HUMOTO_LOCAL Step
{
  public:
    /// @brief Constructor for step
    ///
    /// @param x x
    /// @param y y
    /// @param z z
    /// @param tMin starting time of the step
    /// @param tMax end time of the step
    Step(double x, double y, double z, double tMin, double tMax)
        : x_(x), y_(y), z_(z), tMin_(tMin), tMax_(tMax)
    {
    }

    /// @brief Construstor for the step from a vector
    ///
    /// @param v vector containing [ x, y, z, tMin, tMax]
    Step(const std::vector<double> v)
        : x_(v.at(0)), y_(v.at(1)), z_(v.at(2)), tMin_(v.at(3)), tMax_(v.at(4))
    {
    }

    void print() const
    {
        std::cout << "[" << pos().transpose() << ", " << tMin_ << ", " << tMax_ << "]" << std::endl;
    }

    /// @brief Getter for x
    const double& x() const { return x_; }
    /// @brief Getter for y
    const double& y() const { return y_; }
    /// @brief Getter for z
    const double& z() const { return z_; }
    /// @brief Getter for position {x, y, z}
    const Eigen::Vector3d pos() const { return Eigen::Vector3d(x_, y_, z_); }
    /// @brief Getter for tMin
    const double& tMin() const { return tMin_; }
    /// @brief Getter for tMax
    const double& tMax() const { return tMax_; }

  private:
    double x_;
    double y_;
    double z_;
    double tMin_;
    double tMax_;
};

class HUMOTO_LOCAL StepPlan
{
    /*! \enum phase
     *  LSS: Left Single Support
     *  RSS: Left Single Support
     *  DS: Double Support
     *
     *  Enumerates the different possible phases of the walk
     */
    enum Phase
    {
        LSS,
        RSS,
        DS
    };

  public:
    /// @brief Default constructor
    StepPlan(){};

    /// @brief Constructor from list of parameters
    ///
    /// @param leftStepsParameters parameters for all the left foot steps
    /// @param rightStepsParameters parameters for all the right foot steps
    /// @param T timestep
    /// @param hStep Additional height reached above the middle point of the
    /// straight line between
    /// @param footXwidth Width of foot along the X direction (forward)
    /// @param footYwidth Width of foot along the Y direction (sideway)
    StepPlan(const std::vector<std::vector<double> >& leftStepsParameters,
             const std::vector<std::vector<double> >& rightStepsParameters, double T,
             double hStep = 0.4, double footXwidth = 0.2, double footYwidth = 0.1);

    /// @brief Computes the values of xMin, xMax, yMin, yMax, z (Sustentation polygon) for each time
    /// step
    ///
    /// @param leftSteps list of left foot steps
    /// @param rightSteps list of right foot steps
    void computePlan(std::vector<Step> leftSteps, std::vector<Step> rightSteps);

    /// @brief Computes the plan for steps contained in the fields leftSteps_ and rightSteps_
    void computePlan() { computePlan(leftSteps_, rightSteps_); }

    /// @brief Applies the polynomial defined by coeff to the list of values t and store the result
    /// in res
    void applyPolynomial(Eigen::Ref<Eigen::VectorXd> res, const Eigen::VectorXd& coeff,
                         const Eigen::Ref<Eigen::VectorXd> t);

    /// @brief Computes the foot trajectory between prevStep and nextStep that goes as high as hStep
    /// on the middle point of the trajectory
    ///
    /// @param prevStep previous step
    /// @param nextStep next step
    /// @param hStep Additional height reached above the middle point of the straight line between
    /// prevStep and nextStep
    Polynomial3D computeFeetTrajectory(const Step& prevStep, const Step& nextStep,
                                       const double& hStep);

    void computeFullFeetTrajectory(FootTraj& footTraj, std::vector<Step> steps, const double dt);

    /// @brief Getter for xMin
    const Eigen::VectorXd& xMin() const { return xMin_; }
    /// @brief Getter for xMax
    const Eigen::VectorXd& xMax() const { return xMax_; }
    /// @brief Getter for yMin
    const Eigen::VectorXd& yMin() const { return yMin_; }
    /// @brief Getter for yMax
    const Eigen::VectorXd& yMax() const { return yMax_; }

    /// @brief Getter for z
    const Eigen::VectorXd& z() const { return z_; }

    /// @brief Getter for position of support foot
    /// {x0,y0,z0,x1,y1,z1,...xN,yN,zN} (middle of both foot in DS phase)
    const Eigen::VectorXd& pos() const { return pos_; }

    /// @brief Getter for tMax
    const double& tMax() const { return tMax_; }

    /// @brief Getter for rightFoot
    const FootTraj& rightFoot() const { return rightFoot_; }
    /// @brief Getter for leftFoot
    const FootTraj& leftFoot() const { return leftFoot_; }

    /// @brief Getter for leftSteps
    const std::vector<Step>& leftSteps() const { return leftSteps_; }

    /// @brief Getter for rightSteps
    const std::vector<Step>& rightSteps() const { return rightSteps_; }

    void print() const
    {
        std::cout << "Left foot steps" << std::endl;
        for (size_t i = 0; i < leftSteps_.size(); ++i)
        {
            leftSteps_.at(i).print();
        }
        std::cout << "right foot steps" << std::endl;
        for (size_t i = 0; i < rightSteps_.size(); ++i)
        {
            rightSteps_.at(i).print();
        }
    }

  private:
    /// @brief List of left foot steps
    std::vector<Step> leftSteps_;
    /// @brief List of right foot steps
    std::vector<Step> rightSteps_;

    /// @brief Trajectory of the right foot
    FootTraj rightFoot_;
    /// @brief Trajectory of the left foot
    FootTraj leftFoot_;

    /// @brief Evolution of the x and y bounds of the support polygon
    Eigen::VectorXd xMin_, xMax_, yMin_, yMax_;

    /// @brief Evolution of the altitude of the single support foot
    Eigen::VectorXd z_;

    /// @brief Evolution of the position of the support foot
    /// {x0,y0,z0,x1,y1,z1,...xN,yN,zN} (middle of both foot in DS phase)
    Eigen::VectorXd pos_;

    double tMax_;
    double T_;

    /// @brief Additional height reached by foot trajectory due to parabolic movement
    double heightSteps_;

    /// @brief Width of foot along X
    double stepXWidth_;
    /// @brief Width of foot along Y
    double stepYWidth_;
};

StepPlan::StepPlan(const std::vector<std::vector<double> >& leftStepsParameters,
                   const std::vector<std::vector<double> >& rightStepsParameters, double T,
                   double hStep, double footXwidth, double footYwidth)
    : T_(T), heightSteps_(hStep), stepXWidth_(footXwidth), stepYWidth_(footYwidth)
{
    std::cout << "Ctor StepPlan we are here" << std::endl;
    for (size_t i = 0; i < leftStepsParameters.size(); ++i)
    {
        HUMOTO_ASSERT(leftStepsParameters.at(i).size() == 5,
                      "[Config] each step parameter must be a size 5 vector]")
        leftSteps_.push_back(Step(leftStepsParameters.at(i)));
    }
    for (size_t i = 0; i < rightStepsParameters.size(); ++i)
    {
        HUMOTO_ASSERT(rightStepsParameters.at(i).size() == 5,
                      "[Config] each step parameter must be a size 5 vector]")
        rightSteps_.push_back(Step(rightStepsParameters.at(i)));
    }
    computePlan();
}

Polynomial3D StepPlan::computeFeetTrajectory(const Step& prevStep, const Step& nextStep,
                                             const double& hStep)
{
    Polynomial3D res;
    res.x0 = prevStep.pos();
    res.t0 = prevStep.tMax();

    etools::Vector3 X0(0.0, 0.0, 0.0);
    etools::Vector3 X1(nextStep.x() - prevStep.x(), nextStep.y() - prevStep.y(),
                       nextStep.z() - prevStep.z());
    etools::Vector3 Xmid = 0.5 * (X1 + X0) + etools::Vector3(0, 0, hStep);
    // T0 and its powers
    double T0 = 0.0;
    double T02 = T0 * T0;
    double T03 = T02 * T0;
    double T04 = T03 * T0;
    double T05 = T04 * T0;
    double T06 = T05 * T0;

    // T1 and its powers
    double T1 = nextStep.tMin() - prevStep.tMax();
    double T12 = T1 * T1;
    double T13 = T12 * T1;
    double T14 = T13 * T1;
    double T15 = T14 * T1;
    double T16 = T15 * T1;

    // Tmid and its powers
    double Tm = T1 / 2;
    double Tm2 = Tm * Tm;
    double Tm3 = Tm2 * Tm;
    double Tm4 = Tm3 * Tm;
    double Tm5 = Tm4 * Tm;
    double Tm6 = Tm5 * Tm;

    // std::cout << "\n\nComputeFeetTraj between:\n[" << X0.transpose() << "] at t
    // = " << T0
    //          << "\nand [" << X1.transpose() << "] at t = " << T1 << "\nGoing
    //          through ["
    //          << Xmid.transpose() << "] at t = " << Tm << std::endl;

    etools::Vector7 bx, by, bz;
    etools::Matrix7 M;
    bx << X0.x(), 0, 0, X1.x(), 0, 0, Xmid.x();
    by << X0.y(), 0, 0, X1.y(), 0, 0, Xmid.y();
    bz << X0.z(), 0, 0, X1.z(), 0, 0, Xmid.z();
    M.row(0) << 1, T0, T02, T03, T04, T05, T06;
    M.row(1) << 0, 1, 2 * T0, 3 * T02, 4 * T03, 5 * T04, 6 * T05;
    M.row(2) << 0, 0, 2, 6 * T0, 12 * T02, 20 * T03, 30 * T04;
    M.row(3) << 1, T1, T12, T13, T14, T15, T16;
    M.row(4) << 0, 1, 2 * T1, 3 * T12, 4 * T13, 5 * T14, 6 * T15;
    M.row(5) << 0, 0, 2, 6 * T1, 12 * T12, 20 * T13, 30 * T14;
    M.row(6) << 1, Tm, Tm2, Tm3, Tm4, Tm5, Tm6;
    res.ax = M.colPivHouseholderQr().solve(bx);
    res.ay = M.colPivHouseholderQr().solve(by);
    res.az = M.colPivHouseholderQr().solve(bz);
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    return res;
}

void StepPlan::computeFullFeetTrajectory(FootTraj& footTraj, std::vector<Step> steps,
                                         const double dt)
{
    size_t iStep = 0;
    size_t iTimeStep = 0;
    double currentTime = 0.0;

    Eigen::VectorXd time(footTraj.x_.size());

    for (long i = 0; i < time.size(); ++i)
    {
        time[i] = i * dt;
    }

    footTraj.t_ = time;
    Polynomial3D p0;
    p0.setConstant(0, steps[iStep].pos());
    footTraj.trajPieces_.push_back(TrajPiece(p0, 0, steps.at(iStep).tMax()));

    while (currentTime < steps.at(iStep).tMax())
    {
        footTraj.x_[iTimeStep] = steps[iStep].x();
        footTraj.y_[iTimeStep] = steps[iStep].y();
        footTraj.z_[iTimeStep] = steps[iStep].z();
        footTraj.supportFoot_[iTimeStep] = true;
        iTimeStep++;
        currentTime += dt;
    }

    for (size_t iStep = 0; iStep < steps.size() - 1; iStep++)
    {
        // We first handle the phase where the foot is in the air
        double durationFlight = steps.at(iStep + 1).tMin() - steps.at(iStep).tMax();
        size_t nTimeSteps = (size_t)(durationFlight / dt);
        Polynomial3D p = computeFeetTrajectory(steps.at(iStep), steps.at(iStep + 1), heightSteps_);

        footTraj.trajPieces_.push_back(TrajPiece(p, steps.at(iStep).tMax(), steps.at(iStep+1).tMin()));

        p.applyPolynomial(footTraj.x_.segment(iTimeStep, nTimeSteps), p.ax,
                          time.segment(iTimeStep, nTimeSteps), p.x0[0]);
        p.applyPolynomial(footTraj.y_.segment(iTimeStep, nTimeSteps), p.ay,
                          time.segment(iTimeStep, nTimeSteps), p.x0[1]);
        p.applyPolynomial(footTraj.z_.segment(iTimeStep, nTimeSteps), p.az,
                          time.segment(iTimeStep, nTimeSteps), p.x0[2]);
        for (size_t i = 0; i < nTimeSteps; ++i)
        {
            footTraj.supportFoot_[iTimeStep + i] = false;
        }

        iTimeStep += nTimeSteps;
        currentTime += nTimeSteps * dt;

        Polynomial3D pConstant;
        pConstant.setConstant(steps.at(iStep + 1).tMin(), steps.at(iStep+1).pos());
        footTraj.trajPieces_.push_back(TrajPiece(pConstant, steps.at(iStep + 1).tMin(), steps.at(iStep + 1).tMax()));

        // Then the phase with foot on the ground
        while (currentTime < steps.at(iStep + 1).tMax())
        {
            footTraj.x_[iTimeStep] = steps[iStep + 1].x();
            footTraj.y_[iTimeStep] = steps[iStep + 1].y();
            footTraj.z_[iTimeStep] = steps[iStep + 1].z();
            footTraj.supportFoot_[iTimeStep] = true;
            iTimeStep++;
            currentTime += dt;
        }
    }
}

void StepPlan::computePlan(std::vector<Step> leftSteps, std::vector<Step> rightSteps)
{
    std::cout << "ComputePlan" << std::endl;
    Phase currentPhase;
    leftSteps_ = leftSteps;
    rightSteps_ = rightSteps;
    tMax_ = leftSteps[0].tMax();

    // Find tMax
    for (size_t i = 0; i < leftSteps_.size(); ++i)
        if (leftSteps_[i].tMax() > tMax_) tMax_ = leftSteps_[i].tMax();
    for (size_t i = 0; i < rightSteps_.size(); ++i)
        if (rightSteps_[i].tMax() > tMax_) tMax_ = rightSteps_[i].tMax();

    int nTimeSteps = tMax_ / T_;
    Eigen::VectorXd time(nTimeSteps);
    for (long i = 0; i < nTimeSteps; i++)
    {
        time(i) = i * T_;
    }

    xMin_.resize(nTimeSteps);
    xMax_.resize(nTimeSteps);
    xMin_.setZero();
    xMax_.setZero();
    yMin_.resize(nTimeSteps);
    yMax_.resize(nTimeSteps);
    yMin_.setZero();
    yMax_.setZero();
    z_.resize(nTimeSteps);
    z_.setZero();
    pos_.resize(3 * nTimeSteps);

    rightFoot_.resize(nTimeSteps + 1);
    leftFoot_.resize(nTimeSteps + 1);
    computeFullFeetTrajectory(rightFoot_, rightSteps, T_);
    computeFullFeetTrajectory(leftFoot_, leftSteps, T_);

    for (long iTimeStep = 0; iTimeStep < nTimeSteps; iTimeStep++)
    {
        bool RightFootOnGround = false;
        bool LeftFootOnGround = false;
        double currentTime = time(iTimeStep);

        size_t iStepLeft = 0;
        size_t iStepRight = 0;

        for (size_t i = 0; i < leftSteps_.size(); i++)
        {
            if (leftSteps_[i].tMin() <= currentTime && currentTime < leftSteps_[i].tMax())
            {
                currentPhase = LSS;
                RightFootOnGround = true;
                iStepLeft = i;
                break;
            }
        }

        for (size_t i = 0; i < rightSteps_.size(); i++)
        {
            if (rightSteps_[i].tMin() <= currentTime && currentTime < rightSteps_[i].tMax())
            {
                currentPhase = RSS;
                LeftFootOnGround = true;
                iStepRight = i;
                break;
            }
        }

        if (RightFootOnGround && LeftFootOnGround)
        {
            currentPhase = DS;
        }

        if (currentPhase == LSS)
        {
            xMin_[iTimeStep] = leftSteps_[iStepLeft].x() - stepXWidth_ / 2;
            xMax_[iTimeStep] = leftSteps_[iStepLeft].x() + stepXWidth_ / 2;
            yMin_[iTimeStep] = leftSteps_[iStepLeft].y() - stepYWidth_ / 2;
            yMax_[iTimeStep] = leftSteps_[iStepLeft].y() + stepYWidth_ / 2;
            z_[iTimeStep] = leftSteps_[iStepLeft].z();
        }
        else if (currentPhase == RSS)
        {
            xMin_[iTimeStep] = rightSteps_[iStepRight].x() - stepXWidth_ / 2;
            xMax_[iTimeStep] = rightSteps_[iStepRight].x() + stepXWidth_ / 2;
            yMin_[iTimeStep] = rightSteps_[iStepRight].y() - stepYWidth_ / 2;
            yMax_[iTimeStep] = rightSteps_[iStepRight].y() + stepYWidth_ / 2;
            z_[iTimeStep] = rightSteps_[iStepRight].z();
        }
        else if (currentPhase == DS)
        {
            double minX = rightSteps_[iStepRight].x();
            double maxX = leftSteps_[iStepLeft].x();
            if (minX > maxX)
            {
                double tmp = maxX;
                maxX = minX;
                minX = tmp;
            }
            xMin_[iTimeStep] = minX - stepXWidth_ / 2;
            xMax_[iTimeStep] = maxX + stepXWidth_ / 2;
            yMin_[iTimeStep] = rightSteps_[iStepRight].y() - stepYWidth_ / 2;
            yMax_[iTimeStep] = leftSteps_[iStepLeft].y() + stepYWidth_ / 2;
            z_[iTimeStep] = rightSteps_[iStepRight].z();
        }
        else
        {
            throw std::logic_error(
                "Phases other than LSS, RSS and DS are not handled. Is that a balistic phase?");
        }

        pos_[3 * iTimeStep] = (xMin_[iTimeStep] + xMax_[iTimeStep]) / 2;
        pos_[3 * iTimeStep + 1] = (yMin_[iTimeStep] + yMax_[iTimeStep]) / 2;
        pos_[3 * iTimeStep + 2] = z_[iTimeStep];
    }
    std::cout << "Plan Computed" << std::endl;
}
} /* wpg05 */

} /* mpc */
