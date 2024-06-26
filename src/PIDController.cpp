// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PIDController.h"

#include <algorithm>
#include <cmath>

PIDController::PIDController(double Kp, double Ki, double Kd,
                             double period)
    : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd), m_period(period) {
  bool invalidGains = false;
  if (Kp < 0.0) {
    invalidGains = true;
  }
  if (Ki < 0.0) {
    invalidGains = true;
  }
  if (Kd < 0.0) {
    invalidGains = true;
  }
  if (invalidGains) {
    m_Kp = 0.0;
    m_Ki = 0.0;
    m_Kd = 0.0;
  }

  if (period <= 0) {
    m_period = 0.02;
  }
  static int instances = 0;
  instances++;
}

void PIDController::SetPID(double Kp, double Ki, double Kd) {
  m_Kp = Kp;
  m_Ki = Ki;
  m_Kd = Kd;
}

void PIDController::SetP(double Kp) {
  m_Kp = Kp;
}

void PIDController::SetI(double Ki) {
  m_Ki = Ki;
}

void PIDController::SetD(double Kd) {
  m_Kd = Kd;
}

void PIDController::SetIZone(double iZone) {
  if (iZone < 0) {
    iZone = 0;
  }
  m_iZone = iZone;
}

double PIDController::GetP() const {
  return m_Kp;
}

double PIDController::GetI() const {
  return m_Ki;
}

double PIDController::GetD() const {
  return m_Kd;
}

double PIDController::GetIZone() const {
  return m_iZone;
}

double PIDController::GetPeriod() const {
  return m_period;
}

double PIDController::GetPositionTolerance() const {
  return m_positionTolerance;
}

double PIDController::GetVelocityTolerance() const {
  return m_velocityTolerance;
}

void PIDController::SetSetpoint(double setpoint) {
  m_setpoint = setpoint;
  m_haveSetpoint = true;

  if (m_continuous) {
    double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
    m_positionError =
        InputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
  } else {
    m_positionError = m_setpoint - m_measurement;
  }

  m_velocityError = (m_positionError - m_prevError) / m_period;
}

double PIDController::GetSetpoint() const {
  return m_setpoint;
}

bool PIDController::AtSetpoint() const {
  return m_haveMeasurement && m_haveSetpoint &&
         std::abs(m_positionError) < m_positionTolerance &&
         std::abs(m_velocityError) < m_velocityTolerance;
}

void PIDController::EnableContinuousInput(double minimumInput,
                                          double maximumInput) {
  m_continuous = true;
  m_minimumInput = minimumInput;
  m_maximumInput = maximumInput;
}

void PIDController::DisableContinuousInput() {
  m_continuous = false;
}

bool PIDController::IsContinuousInputEnabled() const {
  return m_continuous;
}

void PIDController::SetIntegratorRange(double minimumIntegral,
                                       double maximumIntegral) {
  m_minimumIntegral = minimumIntegral;
  m_maximumIntegral = maximumIntegral;
}

void PIDController::SetTolerance(double positionTolerance,
                                 double velocityTolerance) {
  m_positionTolerance = positionTolerance;
  m_velocityTolerance = velocityTolerance;
}

double PIDController::GetPositionError() const {
  return m_positionError;
}

double PIDController::GetVelocityError() const {
  return m_velocityError;
}

double PIDController::Calculate(double measurement) {
  m_measurement = measurement;
  m_prevError = m_positionError;
  m_haveMeasurement = true;

  if (m_continuous) {
    double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
    m_positionError =
        InputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
  } else {
    m_positionError = m_setpoint - m_measurement;
  }

  m_velocityError = (m_positionError - m_prevError) / m_period;

  // If the absolute value of the position error is outside of IZone, reset the
  // total error
  if (std::abs(m_positionError) > m_iZone) {
    m_totalError = 0;
  } else if (m_Ki != 0) {
    m_totalError =
        clamp(m_totalError + m_positionError * m_period,
                   m_minimumIntegral / m_Ki, m_maximumIntegral / m_Ki);
  }

  return m_Kp * m_positionError + m_Ki * m_totalError + m_Kd * m_velocityError;
}

double PIDController::Calculate(double measurement, double setpoint) {
  m_setpoint = setpoint;
  m_haveSetpoint = true;
  return Calculate(measurement);
}

void PIDController::Reset() {
  m_positionError = 0;
  m_prevError = 0;
  m_totalError = 0;
  m_velocityError = 0;
  m_haveMeasurement = false;
}