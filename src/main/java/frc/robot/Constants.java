package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team2052.lib.vision.TagTracker.TagTrackerConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.drive.ctre.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.ctre.generated.TunerConstants;
import frc.robot.util.FieldConstants;
import frc.robot.util.io.Ports;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Constants {
  // spotless:off
  public static final class ElevatorConstants {
    public static final boolean ELEVATOR_MOTORS_INVERTED = false;

    // public static final double ROTATIONS_PER_INCH = (1.0 / 12.0) * 2.0; // wrong

    public static final double TICKS_DEADZONE = 0.5;

    public static final double MANUAL_MOTOR_SPEED = 0.2;
    public static final double HOMING_SPEED = -0.1;

    public static final Slot0Configs SLOT0_CONFIGS = 
        new Slot0Configs()
            .withKP(6.0)
            .withKI(0.0)
            .withKD(3.0)
            .withKS(0.5)
            .withKV(0.0)
            .withKA(0.0)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(3.75);

    public static final CurrentLimitsConfigs CURRENT_LIMIT_CONFIG =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(80))
            .withSupplyCurrentLowerLimit(Amps.of(40))
            .withSupplyCurrentLowerTime(Seconds.of(0.1));

    // set Motion Magic settings
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIG =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(70)
            .withMotionMagicExpo_kA(0.03)
            .withMotionMagicExpo_kV(0.01);
            // .withMotionMagicAcceleration(160) 
            // .withMotionMagicJerk(600);

    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG =
        new MotorOutputConfigs()
            .withInverted(
                ElevatorConstants.ELEVATOR_MOTORS_INVERTED
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake); 

    public static final SoftwareLimitSwitchConfigs SOFTWARE_LIMIT_SWITCH_CONFIG =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(66);

    public static final TalonFXConfiguration MOTOR_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(CURRENT_LIMIT_CONFIG)
            .withSoftwareLimitSwitch(SOFTWARE_LIMIT_SWITCH_CONFIG)
            .withMotorOutput(MOTOR_OUTPUT_CONFIG)
            .withMotionMagic(MOTION_MAGIC_CONFIG)
            .withSlot0(SLOT0_CONFIGS)
            .withAudio(new AudioConfigs().withBeepOnBoot(false));
  }

  public static class DriverConstants {
    public static final boolean DEV_CONTROLS = false;
    public static final boolean FORCE_GAMEPAD = false;
    public static final double STEER_DEADBAND = 0.075;
    public static final double JOYSTICK_DEADBAND = 0.075;
    public static final double GAMEPAD_DEADBAND = 0.025; // add deadband here if there is drift
  }

  public static class DrivetrainConstants {
    public static final CommandSwerveDrivetrain TUNER_DRIVETRAIN_CONSTANTS =
        TunerConstants.createDrivetrain();
    /*
     * If using the generator, the order in which modules are constructed is
     * Front Left, Front Right, Back Left, Back Right. This means if you need
     * the Back Left module, call {@code getModule(2);} to get the 3rd index
     * (0-indexed) module, corresponding to the Back Left module.
     */

    public static final LinearVelocity DRIVE_MAX_SPEED = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity DRIVE_MAX_ANGULAR_RATE =
        RadiansPerSecond.of(DRIVE_MAX_SPEED.in(MetersPerSecond) / 0.42);

    public static final Current DRIVE_CURRENT_LIMIT_AMPS = Amps.of(80.0);

    public static final Distance WHEEL_RADIUS =
        Meters.of(TUNER_DRIVETRAIN_CONSTANTS.getModuleConstants()[0].WheelRadius);
    // Left-to-right distance between drivetrain wheels
    public static final Distance DRIVETRAIN_TRACKWIDTH = Inches.of(23.5);
    // Front-to-back distance between drivetrain wheels
    public static final Distance DRIVETRAIN_WHEELBASE = Inches.of(23.5);

    public static final Mass DRIVETRAIN_MASS = Pounds.of(107.1);

    public static final Matrix<N3, N1> ODOMETRY_STDDEV = new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

    public static final Angle HEADING_TOLERANCE = Degrees.of(3);
  }

  public static class CoralArmConstants {
    public static final boolean ARM_MOTOR_INVERTED = true;
    public static final double DEG_TOL = 2.5;

    public static final Angle MIN_CORAL_ARM_ANGLE = Degrees.of(30);
    public static final Angle MAX_CORAL_ARM_ANGLE = Degrees.of(330);


    public static final CurrentLimitsConfigs CURRENT_LIMIT_CONFIG =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(70))
            .withSupplyCurrentLowerLimit(Amps.of(30))
            .withStatorCurrentLimit(Amps.of(100))
            .withSupplyCurrentLowerTime(Seconds.of(0.2));
            
    public static final Slot0Configs SLOT0_CONFIGS = 
        new Slot0Configs()
            .withKP(75.0)
            .withKI(0.2)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(11.7) //11.7
            .withKA(0.0);

    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG =
        new MotorOutputConfigs()
            .withInverted(
                CoralArmConstants.ARM_MOTOR_INVERTED
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake); 

    public static final FeedbackConfigs FEEDBACK_CONFIG =
        new FeedbackConfigs()
            .withFeedbackRemoteSensorID(Ports.ARM_CANCODER_ID)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withRotorToSensorRatio(99.556)
            .withSensorToMechanismRatio(1);
    
    public static final SoftwareLimitSwitchConfigs LIMIT_SWITCH_CONFIGS = 
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(Degrees.of(30)) // TODO: adjust as needed
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitThreshold(Degrees.of(330))
            .withReverseSoftLimitEnable(false);
    
    public static final TalonFXConfiguration MOTOR_CONFIG = 
        new TalonFXConfiguration()
            .withSlot0(SLOT0_CONFIGS)
            .withMotorOutput(MOTOR_OUTPUT_CONFIG)
            .withFeedback(FEEDBACK_CONFIG)
            .withAudio(new AudioConfigs().withBeepOnBoot(false))
            .withCurrentLimits(CURRENT_LIMIT_CONFIG);
  }

  public static class HandConstants {
    public static final boolean HAND_MOTOR_INVERTED = true;
    public static final double HAND_MOTOR_CURRENT_LIMIT = 40.0;
    public static final double IN_HAND_MOTOR_SPEED = 0.5;
    public static final double OUT_HAND_MOTOR_SPEED = 0.35;
  }

  public static class AlgaePivotConstants {
    public static final boolean MOTOR_INVERTED = true;
    public static final double TOLERANCE = 1.0;

    public static final CurrentLimitsConfigs CURRENT_LIMIT_CONFIG =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(false)
            .withSupplyCurrentLimit(Amps.of(70))
            .withSupplyCurrentLowerLimit(Amps.of(40))
            .withStatorCurrentLimit(Amps.of(120))
            .withSupplyCurrentLowerTime(Seconds.of(0.2));

    public static final FeedbackConfigs FEEDBACK_CONFIG =
    new FeedbackConfigs()
        .withFeedbackRemoteSensorID(Ports.ALGAE_ENCODER_ID)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withRotorToSensorRatio(100)
        .withSensorToMechanismRatio(1);

    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG =
        new MotorOutputConfigs()
            .withInverted(
                MOTOR_INVERTED
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);

    public static final SoftwareLimitSwitchConfigs LIMIT_SWITCH_CONFIGS = 
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(Degrees.of(260)) // TODO: adjust as needed
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(Degrees.of(120))
            .withReverseSoftLimitEnable(true);

    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
      .withMotorOutput(MOTOR_OUTPUT_CONFIG)
      .withSoftwareLimitSwitch(LIMIT_SWITCH_CONFIGS)
      .withFeedback(FEEDBACK_CONFIG)
      .withCurrentLimits(CURRENT_LIMIT_CONFIG);
  }

  public static class AlgaeShooterConstants {
      public static final double MOTOR_CURRENT_LIMIT = 60.0;
      public static final double INTAKE_SPEED = 1.0;
      public static final double SCORE_SPEED = -1.0;

      public static final boolean MOTOR_INVERTED = false;

      public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG =
          new MotorOutputConfigs()
              .withInverted(
                  MOTOR_INVERTED
                      ? InvertedValue.Clockwise_Positive
                      : InvertedValue.CounterClockwise_Positive)
              .withNeutralMode(NeutralModeValue.Brake); 

      public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotorOutput(MOTOR_OUTPUT_CONFIG);
  }
  
  public static final class ClimberConstants {
    public static final double baseSpeed = 1.0;
    public static final double fineSpeed = 0.25;

    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration().withMotorOutput(
      new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake)
    ).withCurrentLimits(
      new CurrentLimitsConfigs()
    );
  }

  public static class VisionConstants {
    public static final double DEFAULT_XY_STDDEV = 0.1;
    public static final double DEFAULT_HEADING_STDDEV = 0.1;
    public static final Matrix<N3, N1> VISION_STDDEV =  new Matrix<>(VecBuilder.fill(DEFAULT_XY_STDDEV, DEFAULT_XY_STDDEV, DEFAULT_HEADING_STDDEV));

    public static final double MAX_POSE_AMBIGUITY = 0.15;
    public static final Distance FIELD_BORDER_MARGIN = Meters.of(0.5);
    public static final Distance MAX_VISION_CORRECTION = Meters.of(1);

    /* CORAL Reef Camera */
    public static final class CoralReefCameraConstants {
      public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =  FieldConstants.CORAL_REEF_CAMERA_LAYOUT_TYPE.layout;
      public static final String CAMERA_NAME = "Arducam_OV2311_USB_Camera_001";

      public static final PoseStrategy STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

      public static final Distance X_OFFSET = Inches.of(9); // forward positive
      public static final Distance Y_OFFSET = Inches.of(-0.5); // left positive
      public static final Distance Z_OFFSET = Inches.of(7.875); // up positive

      public static final Angle THETA_X_OFFSET = Degrees.of(0); // roll
      public static final Angle THETA_Y_OFFSET = Degrees.of(-21.84); // pitch
      public static final Angle THETA_Z_OFFSET = Degrees.of(3); // yaw

      public static final Transform3d ROBOT_TO_CAMERA_METERS =
          new Transform3d(
              new Translation3d(X_OFFSET, Y_OFFSET, Z_OFFSET),
              new Rotation3d(THETA_X_OFFSET, THETA_Y_OFFSET, THETA_Z_OFFSET));

      public static TagTrackerConstants TagTrackerConstants() {
        return new TagTrackerConstants(
            CAMERA_NAME, ROBOT_TO_CAMERA_METERS, APRIL_TAG_FIELD_LAYOUT, STRATEGY);
      }
    }

    /* ALGAE Reef Camera */
    public static final class AlgaeReefCameraConstants {      
      public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = FieldConstants.ALGAE_REEF_CAMERA_LAYOUT_TYPE.layout;

      public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera_002";

      public static final PoseStrategy STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

      public static final Distance X_OFFSET = Inches.of(4.614); // forward
      public static final Distance Y_OFFSET = Inches.of(9.995); // left
      public static final Distance Z_OFFSET = Inches.of(8.418); // up

      public static final Angle THETA_X_OFFSET = Degrees.of(0); // roll
      public static final Angle THETA_Y_OFFSET = Degrees.of(-21.55352); // pitch
      public static final Angle THETA_Z_OFFSET = Degrees.of(90); // yaw

      public static final Transform3d ROBOT_TO_CAMERA_METERS =
          new Transform3d(
              new Translation3d(X_OFFSET, Y_OFFSET, Z_OFFSET),
              new Rotation3d(THETA_X_OFFSET, THETA_Y_OFFSET, THETA_Z_OFFSET));

      public static TagTrackerConstants TagTrackerConstants() {
        return new TagTrackerConstants(
            CAMERA_NAME, ROBOT_TO_CAMERA_METERS, APRIL_TAG_FIELD_LAYOUT, STRATEGY);
      }
    }
    /* Coral Station + Other Tags Camera */
    public static final class RearCameraConstants {      
      public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = FieldConstants.DEFAULT_APRIL_TAG_LAYOUT_TYPE.layout;

      public static final String CAMERA_NAME = "KrawlerCam_002";

      public static final PoseStrategy STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

      public static final Distance X_OFFSET = Inches.of(0.0);
      public static final Distance Y_OFFSET = Inches.of(4);
      public static final Distance Z_OFFSET = Inches.of(7);

      public static final Angle THETA_X_OFFSET = Degrees.of(0); // roll
      public static final Angle THETA_Y_OFFSET = Degrees.of(0); // pitch
      public static final Angle THETA_Z_OFFSET = Degrees.of(0); // yaw TODO: need to change back to 180 for actual stuff

      public static final Transform3d ROBOT_TO_CAMERA_METERS =
          new Transform3d(
              new Translation3d(X_OFFSET, Y_OFFSET, Z_OFFSET),
              new Rotation3d(THETA_X_OFFSET, THETA_Y_OFFSET, THETA_Z_OFFSET));

      public static TagTrackerConstants TagTrackerConstants() {
        return new TagTrackerConstants(
            CAMERA_NAME, ROBOT_TO_CAMERA_METERS, APRIL_TAG_FIELD_LAYOUT, STRATEGY);
      }
    }
  }

  
  public static final class DashboardConstants {
    public static final String DRIVE_MODE_KEY = "Drive Mode";
    public static final String AUTO_COMPILED_KEY = "Auto Compiled";
    public static final String AUTO_DESCRIPTION_KEY = "Auto Description";
    public static final String WAIT_SECONDS_SAVED_KEY = "Wait Seconds Saved";
    public static final String WAIT_SECONDS_DISPLAY_KEY = "Wait Seconds Display";
  }

  public static final class PathPlannerConstants {
    public static final double TRANSLATION_KP = 5.0;
    public static final double TRANSLATION_KI = 0;
    public static final double TRANSLATION_KD = 0;

    public static final double ROTATION_KP = 5.0;
    public static final double ROTATION_KI = 0;
    public static final double ROTATION_KD = 0;

    // Rough estimation (1/12) * mass * (length^2 + width^2)

    public static final ModuleConfig MODULE_CONFIG =
        new ModuleConfig(
            DrivetrainConstants.WHEEL_RADIUS,
            DrivetrainConstants.DRIVE_MAX_SPEED,
            1.1,
            DCMotor.getKrakenX60Foc(1),
            DrivetrainConstants.DRIVE_CURRENT_LIMIT_AMPS,
            1);

    public static final PPHolonomicDriveController PATH_FOLLOWING_CONTROLLER =
        new PPHolonomicDriveController(
            new PIDConstants(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD),
            new PIDConstants(ROTATION_KP, ROTATION_KI, ROTATION_KD));
  }

  // spotless:on
}
