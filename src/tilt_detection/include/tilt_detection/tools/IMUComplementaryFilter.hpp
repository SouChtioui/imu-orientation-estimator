/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

  @section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holder nor the names of its
        contributors may be used to endorse or promote products derived from
        this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

namespace tilt_detection::tools
{

class ComplementaryFilter
{
public:
    ComplementaryFilter() = default;
    ~ComplementaryFilter() = default;

    bool setGainAcc(double gain);
    double getGainAcc() const;

    bool setBiasAlpha(double bias_alpha);
    double getBiasAlpha() const;

    // When the filter is in the steady state, bias estimation will occur (if
    // the parameter is enabled).
    bool getSteadyState() const;

    void setDoBiasEstimation(bool do_bias_estimation);
    bool getDoBiasEstimation() const;

    void setDoAdaptiveGain(bool do_adaptive_gain);
    bool getDoAdaptiveGain() const;

    double getAngularVelocityBiasX() const;
    double getAngularVelocityBiasY() const;
    double getAngularVelocityBiasZ() const;

    // Set the orientation, as a Hamilton Quaternion, of the body frame wrt the
    // fixed frame.
    void setOrientation(double q0, double q1, double q2, double q3);

    // Get the orientation, as a Hamilton Quaternion, of the body frame wrt the
    // fixed frame.
    void getOrientation(double& q0, double& q1, double& q2, double& q3) const;

    // Update from accelerometer and gyroscope data.
    // [ax, ay, az]: Normalized gravity vector.
    // [wx, wy, wz]: Angular velocity, in rad / s.
    // dt: time delta, in seconds.
    void update(double ax, double ay, double az, double wx, double wy, double wz, double dt);

    // Reset the filter to the initial state.
    void reset();

private:
    void updateBiases(double ax, double ay, double az, double wx, double wy, double wz);

    bool checkState(double ax, double ay, double az, double wx, double wy, double wz) const;

    void getPrediction(double wx, double wy, double wz, double dt, double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred) const;

    static void getMeasurement(double ax, double ay, double az, double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas);

    static void getAccCorrection(double ax, double ay, double az, double p0, double p1, double p2, double p3, double& dq0, double& dq1, double& dq2, double& dq3);

    static double getAdaptiveGain(double alpha, double ax, double ay, double az);

    // Gain parameter for the complementary filter, belongs in [0, 1].
    double gain_acc_ = 0.01;

    // Bias estimation gain parameter, belongs in [0, 1].
    double bias_alpha_ = 0.01;

    // Parameter whether to do bias estimation or not.
    bool do_bias_estimation_ = true;

    // Parameter whether to do adaptive gain or not.
    bool do_adaptive_gain_ = true;

    bool initialized_ = false;
    bool steady_state_ = false;

    // The orientation as a Hamilton quaternion (q0 is the scalar). Represents
    // the orientation of the fixed frame wrt the body frame.
    double q0_ = 1.0;
    double q1_ = 0.0;
    double q2_ = 0.0;
    double q3_ = 0.0;

    // Bias in angular velocities;
    double wx_prev_ = 0.0;
    double wy_prev_ = 0.0;
    double wz_prev_ = 0.0;

    // Bias in angular velocities;
    double wx_bias_ = 0.0;
    double wy_bias_ = 0.0;
    double wz_bias_ = 0.0;

    static constexpr double KGravity = 9.81;
    static constexpr double Gamma = 0.01;
    // Bias estimation steady state thresholds
    static constexpr double KAngularVelocityThreshold = 0.2;
    static constexpr double KAccelerationThreshold = 0.1;
    static constexpr double KDeltaAngularVelocityThreshold = 0.01;
};

} // namespace tilt_detection::tools
