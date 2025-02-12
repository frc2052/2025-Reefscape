// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.team2052.lib.geometry.Pose2dPolar;
import com.team2052.lib.planners.AutoAlignPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotState;
import frc.robot.commands.drive.alignment.AlignWithSpecificReefCommand;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.AimingCalculator;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonPipelineResult;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateAroundReefCommand extends AlignWithSpecificReefCommand {
  private final RobotState robotState = RobotState.getInstance();
  private final VisionSubsystem vision = VisionSubsystem.getInstance();
  private final int tagID;
  private static Pose2dPolar polarPose;
  private static boolean isGoingRight = false;
  private static boolean hasTag = false;
  private static Supplier<Distance> radiusSupplier;
  private static AutoAlignPlanner planner;

  /** Creates a new RotateAroundReefCommand. */
  public RotateAroundReefCommand(
      Supplier<AlignLocation> scoringLocation, int tagID, Supplier<Distance> radiusSupplier) {
    super(
        scoringLocation,
        getXSupplier(),
        getYSupplier(),
        getRotationSupplier(),
        () -> hasTag,
        tagID);
    this.tagID = tagID;
    this.radiusSupplier = radiusSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  private static DoubleSupplier getXSupplier() {
    return new DoubleSupplier() {
      @Override
      public double getAsDouble() {
        return planner.calculate(
                    new Pose2d(0, 0, polarPose.getRotation()),
                    new Pose2d(
                        0,
                        0,
                        new Rotation2d(polarPose.getPolarPose().getTheta().in(Radians) + Math.PI)))
                .omegaRadiansPerSecond
            / 5;
      }
    };
  }

  private static DoubleSupplier getRotationSupplier() {
    return new DoubleSupplier() {
      @Override
      public double getAsDouble() {
        return planner.calculate(
                    new Pose2d(
                        polarPose.getPolarPose().getRadius().in(Meters), 0, new Rotation2d()),
                    new Pose2d(radiusSupplier.get().in(Meters), 0, new Rotation2d()))
                .vxMetersPerSecond
            / 5;
      }
    };
  }

  private static DoubleSupplier getYSupplier() {
    return new DoubleSupplier() {
      @Override
      public double getAsDouble() {
        if (isGoingRight) {
          return 0.5;
        } else {
          return -0.5;
        }
      }
    };
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    polarPose =
        AimingCalculator.getPositionFromReef(
            robotState.getFieldToRobot(), robotState.isRedAlliance());
    Optional<PhotonPipelineResult> tar = vision.getReefCamClosestTarget();
    if (tar.isPresent()) {
      if (tar.get().getBestTarget().getFiducialId() == tagID) {
        super.execute();
        hasTag = true;
      } else {
        hasTag = false;
        int closestTagID = tar.get().getBestTarget().getFiducialId();
        int tagDistance = tagID - closestTagID;
        if (robotState.isRedAlliance()) {
          if (tagDistance < 3 && tagDistance > 0) {
            isGoingRight = true;
          } else if (tagDistance > 3) {
            isGoingRight = false;
          } else if (tagDistance < 0 && tagDistance > -3) {
            isGoingRight = false;
          } else if (tagDistance < -3) {
            isGoingRight = true;
          } else if (tagDistance == 3 || tagDistance == -3) {
            isGoingRight = true;
          }
        } else {
          if (tagDistance < 3 && tagDistance > 0) {
            isGoingRight = false;
          } else if (tagDistance > 3) {
            isGoingRight = true;
          } else if (tagDistance < 0 && tagDistance > -3) {
            isGoingRight = true;
          } else if (tagDistance < -3) {
            isGoingRight = false;
          } else if (tagDistance == 3 || tagDistance == -3) {
            isGoingRight = false;
          }
        }
      }
    }
  }
}
