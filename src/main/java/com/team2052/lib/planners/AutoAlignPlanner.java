package com.team2052.lib.planners;

import com.team2052.lib.PIDFFController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
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
    xController = new PIDFFController(1.5, 0.0, 0.25, 0.1, 0.0, 0.0);
    yController = new PIDFFController(1.5, 0.0, 0.25, 0.1, 0.0, 0.0);
    thetaController = new PIDFFController(1.5, 0.0, 0.1, 0.3, 0.0, 0.0);

    xController.setTolerance(0.08, 0.05);
    yController.setTolerance(0.02, 0.05);
    thetaController.setTolerance(0.03, 0.05);
  }

  public ChassisSpeeds calculate(
      Transform3d currentTransform, Transform2d goalTransform, Pose2d currentRobotPose) {
    xController.setSetpoint(goalTransform.getX());
    yController.setSetpoint(goalTransform.getY());
    thetaController.setSetpoint(goalTransform.getRotation().getRadians());
    

    double xOutput = xController.calculate(currentTransform.getX());
    double yOutput = yController.calculate(currentTransform.getY());
    double thetaOutput = thetaController.calculate(currentTransform.getRotation().getAngle());

    ChassisSpeeds calculatedSpeeds;

    boolean xWithinTol = xController.atSetpoint();
    boolean yWithinTol = yController.atSetpoint();
    boolean thetaWithinTol = thetaController.atSetpoint();

    calculatedSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            xWithinTol ? 0.0 : xOutput,
            yWithinTol ? 0.0 : yOutput,
            thetaWithinTol ? 0.0 : thetaOutput,
            currentRobotPose.getRotation());

    autoAlignComplete = xWithinTol && yWithinTol && thetaWithinTol;
    if (startTime.isPresent() && autoAlignComplete) {
      System.out.println(
          "Auto align took: " + (Timer.getFPGATimestamp() - startTime.getAsDouble()));
      startTime = OptionalDouble.empty();
    }

    return calculatedSpeeds;
  }

  public boolean getAutoAlignComplete() {
    return autoAlignComplete;
  }
}
