package com.team2052.lib.planners;

import com.team2052.lib.PIDFFController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    xController = new PIDFFController(2, 0.0, 0, 0.1, 0.0, 0.0);
    yController = new PIDFFController(2, 0.0, 0, 0.1, 0.0, 0.0);
    thetaController = new PIDFFController(1.5, 0.0, 0.1, 0.3, 0.0, 0.0);

    xController.setTolerance(0.08, 0.05);
    yController.setTolerance(0.02, 0.05);
    thetaController.setTolerance(0.03, 0.05);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public ChassisSpeeds calculate(
      Translation2d currentTranslation,
      Translation2d goalTranslationFromRobot,
      Rotation2d goalRotation,
      Pose2d currentRobotPose) {
    xController.setSetpoint(goalTranslationFromRobot.getX());
    yController.setSetpoint(goalTranslationFromRobot.getY());
    thetaController.setSetpoint(goalRotation.getRadians());

    double xOutput = xController.calculate(currentTranslation.getX());
    double yOutput = yController.calculate(currentTranslation.getY());
    double thetaOutput = thetaController.calculate(currentRobotPose.getRotation().getRadians());

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
