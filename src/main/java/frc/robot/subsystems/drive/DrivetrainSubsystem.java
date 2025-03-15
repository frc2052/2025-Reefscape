package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team2052.lib.vision.MultiTagPoseEstimate;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.FieldLocation;
import frc.robot.controlboard.PositionSuperstructure;
import frc.robot.controlboard.PositionSuperstructure.TargetAction;
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

  public void addVisionUpdate(MultiTagPoseEstimate visionUpdate) {
    this.addVisionMeasurement(
        visionUpdate.estimatedPose.toPose2d(),
        Utils.fpgaToCurrentTime(visionUpdate.timestampSeconds),
        visionUpdate.getVisionMeasurementStdDevs());
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

    setFieldLocation();
  }

  private void setFieldLocation() {
    if (FieldLocation.HP.getArea().withInTheRegion(robotState.getFieldToRobot())) {
      robotState.setFieldLocation(FieldLocation.HP);

    } else if (FieldLocation.REEF.getArea().withInTheRegion(robotState.getFieldToRobot())) {
      robotState.setFieldLocation(FieldLocation.REEF);

    } else if (FieldLocation.BARGE.getArea().withInTheRegion(robotState.getFieldToRobot())) {
      robotState.setFieldLocation(FieldLocation.BARGE);

    } else if (FieldLocation.PROCESSOR.getArea().withInTheRegion(robotState.getFieldToRobot())) {
      robotState.setFieldLocation(FieldLocation.PROCESSOR);
    } else {
      robotState.setFieldLocation(FieldLocation.TRAVEL);
    }
  }

  private void setTargetAction() {
    PositionSuperstructure.getInstance().setTargetAction(TargetAction.TR);
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

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }
}
