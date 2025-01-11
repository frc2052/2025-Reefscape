package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team2052.lib.vision.VisionUpdate;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants.TunerSwerveDrivetrain;

public class DrivetrainSubsystem extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier simNotifier = null;
  private double lastSimTime;

  /* Keep track if we've ever applied the driver perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

  private RobotState robotState = RobotState.getInstance();

  private static DrivetrainSubsystem INSTANCE;

  public static DrivetrainSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new DrivetrainSubsystem();
    }

    return INSTANCE;
  }

  private DrivetrainSubsystem() {
    super(
        DrivetrainConstants.TUNER_DRIVETRAIN_CONSTANTS.getDrivetrainConstants(),
        0,
        DrivetrainConstants.ODOMETRY_STDDEV,
        VisionConstants.VISION_STDDEV,
        DrivetrainConstants.TUNER_DRIVETRAIN_CONSTANTS.getModuleConstants());
    // configurePathPlanner();
    configureAutoBuilder();

    if (Robot.isSimulation()) {
      startSimThread();
    }
  }

  private void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  autoRequest
                      .withSpeeds(speeds)
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(10, 0, 0),
              // PID constants for rotation
              new PIDConstants(7, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  // public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
  //     return run(() -> this.setControl(requestSupplier.get()));
  // }

  public void stop() {
    setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()));
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return getKinematics().toChassisSpeeds(getState().ModuleStates);
  }

  public void addVisionMeasurement(VisionUpdate visionUpdate) {
    if (visionUpdate.getVisionMeasurementStdDevs() == null) {
      this.addVisionMeasurement(
          visionUpdate.estimatedPose.toPose2d(), visionUpdate.timestampSeconds);
    } else {
      this.addVisionMeasurement(
          visionUpdate.estimatedPose.toPose2d(),
          visionUpdate.timestampSeconds,
          visionUpdate.getVisionMeasurementStdDevs());
    }
  }

  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
    /*
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state
     */
    /*
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              (allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? Rotation2d.fromDegrees(180)
                        : Rotation2d.fromDegrees(0));
                hasAppliedOperatorPerspective = true;
              });
    }

    robotState.addDrivetrainState(super.getState());
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });

    simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
