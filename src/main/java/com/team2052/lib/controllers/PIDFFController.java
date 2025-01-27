package com.team2052.lib.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class PIDFFController {
  private PIDController pidController;
  private SimpleMotorFeedforward feedforward;

  public PIDFFController(double p, double i, double d) {
    pidController = new PIDController(p, i, d);
    feedforward = new SimpleMotorFeedforward(0, 0, 0);
  }

  public PIDFFController(double p, double i, double d, double s, double v, double a) {
    pidController = new PIDController(p, i, d);
    feedforward = new SimpleMotorFeedforward(s, v, a);
  }

  public void enableContinuousInput(double min, double max) {
    pidController.enableContinuousInput(min, max);
  }

  public void setTolerance(double posTol, double velTol) {
    pidController.setTolerance(posTol, velTol);
  }

  public void setSetpoint(double s) {
    pidController.setSetpoint(s);
  }

  public double calculate(double measurement) {
    return pidController.calculate(measurement)
        + feedforward.calculate(pidController.calculate(measurement));
  }

  public double calculate(double measurement, double setpoint) {
    return pidController.calculate(measurement)
        + feedforward.calculate(pidController.calculate(measurement, setpoint));
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }
}
