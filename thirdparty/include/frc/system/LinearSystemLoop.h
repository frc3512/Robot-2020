/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Eigen/Core>

#include "frc/controller/LinearQuadraticRegulator.h"
#include "frc/controller/PlantInversionFeedforward.h"
#include "frc/estimator/KalmanFilter.h"
#include "frc/system/LinearSystem.h"

namespace frc {

/**
 * Combines a plant, controller, and observer for controlling a mechanism with
 * full state feedback.
 *
 * For everything in this file, "inputs" and "outputs" are defined from the
 * perspective of the plant. This means U is an input and Y is an output
 * (because you give the plant U (powers) and it gives you back a Y (sensor
 * values). This is the opposite of what they mean from the perspective of the
 * controller (U is an output because that's what goes to the motors and Y is an
 * input because that's what comes back from the sensors).
 *
 * For more on the underlying math, read
 * https://file.tavsys.net/control/state-space-guide.pdf.
 */
template <int States, int Inputs, int Outputs>
class LinearSystemLoop {
 public:
  /**
   * Constructs a state-space loop with the given plant, controller, and
   * observer.
   *
   * @param plant      State-space plant.
   * @param controller State-space controller.
   * @param feedforward Plant-inversion feedforward.
   * @param observer   State-space observer.
   */
  LinearSystemLoop(LinearSystem<States, Inputs, Outputs>& plant,
                   LinearQuadraticRegulator<States, Inputs>& controller,
                   PlantInversionFeedforward<States, Inputs>& feedforward,
                   KalmanFilter<States, Inputs, Outputs>& observer)
      : m_plant(plant),
        m_controller(controller),
        m_feedforward(feedforward),
        m_observer(observer) {
    m_nextR.setZero();
    Reset(m_nextR);
  }

  virtual ~LinearSystemLoop() = default;

  LinearSystemLoop(LinearSystemLoop&&) = default;
  LinearSystemLoop& operator=(LinearSystemLoop&&) = default;

  /**
   * Enables the controller.
   */
  void Enable() {
    m_controller.Enable();
    m_feedforward.Enable();
  }

  /**
   * Disables the controller and zeros the control input.
   */
  void Disable() {
    m_controller.Disable();
    m_feedforward.Disable();
  }

  /**
   * Returns the observer's state estimate x-hat.
   */
  const Eigen::Matrix<double, States, 1>& Xhat() const {
    return m_observer.Xhat();
  }

  /**
   * Returns an element of the observer's state estimate x-hat.
   *
   * @param i Row of x-hat.
   */
  double Xhat(int i) const { return m_observer.Xhat(i); }

  /**
   * Returns the controller's next reference r.
   */
  const Eigen::Matrix<double, States, 1>& NextR() const { return m_nextR; }

  /**
   * Returns an element of the controller's next reference r.
   *
   * @param i Row of r.
   */
  double NextR(int i) const { return NextR()(i); }

  /**
   * Returns the controller's calculated control input u.
   */
  Eigen::Matrix<double, Inputs, 1> U() const {
    return m_plant.ClampInput(m_controller.U() + m_feedforward.Uff());
  }

  /**
   * Returns an element of the controller's calculated control input u.
   *
   * @param i Row of u.
   */
  double U(int i) const { return m_plant.ClampInput(U())(i); }

  /**
   * Set the initial state estimate x-hat.
   *
   * @param xHat The initial state estimate x-hat.
   */
  void SetXhat(const Eigen::Matrix<double, States, 1>& xHat) {
    m_observer.SetXhat(xHat);
  }

  /**
   * Set an element of the initial state estimate x-hat.
   *
   * @param i     Row of x-hat.
   * @param value Value for element of x-hat.
   */
  void SetXhat(int i, double value) { m_observer.SetXhat(i, value); }

  /**
   * Set the next reference r.
   *
   * @param nextR Next reference.
   */
  void SetNextR(const Eigen::Matrix<double, States, 1>& nextR) {
    m_nextR = nextR;
  }

  /**
   * Return the plant used internally.
   */
  const LinearSystem<States, Inputs, Outputs>& Plant() const { return m_plant; }

  /**
   * Return the controller used internally.
   */
  const LinearQuadraticRegulator<States, Inputs>& Controller() const {
    return m_controller;
  }

  /**
   * Return the feedforward used internally.
   *
   * @return the feedforward used internally.
   */
  const PlantInversionFeedforward<States, Inputs> Feedforward() const {
    return m_feedforward;
  }

  /**
   * Return the observer used internally.
   */
  const KalmanFilter<States, Inputs, Outputs>& Observer() const {
    return m_observer;
  }

  /**
   * Zeroes reference r, controller output u and plant output y.
   * The previous reference for PlantInversionFeedforward is set to the
   * initial reference.
   * @param initialReference The initial reference.
   */
  void Reset(Eigen::Matrix<double, States, 1> initialState) {
    m_plant.Reset();
    m_controller.Reset();
    m_feedforward.Reset(initialState);
    m_observer.Reset();
    m_nextR.setZero();
  }

  /**
   * Returns difference between reference r and x-hat.
   */
  const Eigen::Matrix<double, States, 1> Error() const {
    return m_controller.R() - m_observer.Xhat();
  }

  /**
   * Correct the state estimate x-hat using the measurements in y.
   *
   * @param y Measurement vector.
   */
  void Correct(const Eigen::Matrix<double, Outputs, 1>& y) {
    m_observer.Correct(U(), y);
  }

  /**
   * Sets new controller output, projects model forward, and runs observer
   * prediction.
   *
   * After calling this, the user should send the elements of u to the
   * actuators.
   *
   * @param dt Timestep for model update.
   */
  void Predict(units::second_t dt) {
    m_controller.Update(m_observer.Xhat(), m_nextR);
    m_feedforward.Calculate(m_nextR);
    m_observer.Predict(U(), dt);
  }

 protected:
  LinearSystem<States, Inputs, Outputs>& m_plant;
  LinearQuadraticRegulator<States, Inputs>& m_controller;
  PlantInversionFeedforward<States, Inputs>& m_feedforward;
  KalmanFilter<States, Inputs, Outputs>& m_observer;

  // Reference to go to in the next cycle (used by feedforward controller).
  Eigen::Matrix<double, States, 1> m_nextR;

  // These are accessible from non-templated subclasses.
  static constexpr int kStates = States;
  static constexpr int kInputs = Inputs;
  static constexpr int kOutputs = Outputs;
};

}  // namespace frc
