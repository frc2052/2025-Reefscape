package com.team2052.lib.planners;

import com.team2052.lib.controllers.PIDFFController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.OptionalDouble;

public class AutoAlignPlanner {
  private PIDFFController xController;
  private PIDFFController yController;
  private PIDFFController thetaController;
  boolean autoAlignComplete = false;

  private OptionalDouble startTime;

  public AutoAlignPlanner() {
    startTime = OptionalDouble.of(Timer.getFPGATimestamp());
    xController = new PIDFFController(2.5, 0.0, 0, 0.0, 0, 0.0, 0.0, 0.0);
    yController = new PIDFFController(3.5, 0.0, 0, 0.1, 0, 0.0, 0.0, 0.0);
    thetaController = new PIDFFController(1.0, 0.0, 0.1, 0.3, 0.0, 0.0, 3.0, 2.0);

    xController.setTolerance(0.08, 0.05);
    yController.setTolerance(0.02, 0.05);
    thetaController.setTolerance(0.03, 0.05);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public ChassisSpeeds calculate(Pose2d currentPose, Pose2d goalPose) {
    xController.setSetpoint(goalPose.getX());
    yController.setSetpoint(goalPose.getY());
    thetaController.setSetpoint(goalPose.getRotation().getRadians());

    double xOutput = xController.calculate(currentPose.getX());
    double yOutput = yController.calculate(currentPose.getY());
    double thetaOutput = thetaController.calculate(currentPose.getRotation().getRadians());

    ChassisSpeeds calculatedSpeeds;

    boolean xWithinTol = xController.atSetpoint();
    boolean yWithinTol = yController.atSetpoint();
    boolean thetaWithinTol = thetaController.atSetpoint();

    calculatedSpeeds =
        new ChassisSpeeds(
            xWithinTol ? 0.0 : xOutput, // TODO: un negate
            // 0.0,
            yWithinTol ? 0.0 : yOutput,
            thetaWithinTol ? 0.0 : thetaOutput);

    autoAlignComplete = xWithinTol && yWithinTol && thetaWithinTol;
    if (startTime.isPresent() && autoAlignComplete) {
      // System.out.println("Auto align took: " + (Timer.getFPGATimestamp() -
      // startTime.getAsDouble()));
      startTime = OptionalDouble.empty();
    }

    return calculatedSpeeds;
  }

  public boolean getAutoAlignComplete() {
    return autoAlignComplete;
  }

  public void resetPlanner() {
    autoAlignComplete = false;
  }
}
