package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

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
import com.team2052.lib.vision.photon.TagTracker.TagTrackerConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
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

    public static final double TICKS_DEADZONE = 0.25;

    public static final double MANUAL_MOTOR_SPEED = 0.2;
    public static final double HOMING_SPEED = -0.2;

    public static final Slot0Configs SLOT0_CONFIGS = 
        new Slot0Configs()
            .withKP(55.0)
            .withKI(0.0)
            .withKD(3.6)
            // .withKS(0.0)
            // .withKV(0.0)
            // .withKA(0.0)
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(8.5); //10.15

    public static final CurrentLimitsConfigs CURRENT_LIMIT_CONFIG =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(Amps.of(120))
            .withSupplyCurrentLimit(Amps.of(80))
            .withSupplyCurrentLowerLimit(Amps.of(40))
            .withSupplyCurrentLowerTime(Seconds.of(0.1));

    // // set Motion Magic settings
    // public static final MotionMagicConfigs MOTION_MAGIC_CONFIG =
    //     new MotionMagicConfigs()
    //         .withMotionMagicExpo_kA(0.1)
    //         .withMotionMagicExpo_kV(0.1);

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
            .withForwardSoftLimitThreshold(46.5);

    public static final TalonFXConfiguration MOTOR_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(CURRENT_LIMIT_CONFIG)
            .withSoftwareLimitSwitch(SOFTWARE_LIMIT_SWITCH_CONFIG)
            .withMotorOutput(MOTOR_OUTPUT_CONFIG)
            .withMotionMagic(new MotionMagicConfigs())
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

    public static final Matrix<N3, N1> ODOMETRY_STDDEV = new Matrix<>(VecBuilder.fill(0.01, 0.01, 0.01));

    public static final Angle HEADING_TOLERANCE = Degrees.of(3);
  }

  public static class ArmPivotConstants {
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
            .withKP(85.0)
            .withKI(0.15)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(11.7) //11.7
            .withKA(0.0);

    
    public static final MotionMagicConfigs MOTION_MAGIC_CONFIG =
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(8)
                .withMotionMagicAcceleration(8) 
                .withMotionMagicJerk(16);

    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG =
        new MotorOutputConfigs()
            .withInverted(
                ArmPivotConstants.ARM_MOTOR_INVERTED
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
            .withForwardSoftLimitThreshold(Degrees.of(30))
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

  public static class ArmRollerConstants {
    public static final boolean MOTOR_INVERTED = true;
    public static final double CORAL_IN_SPEED = -0.33;
    public static final double CORAL_L1_OUT_SPEED = 0.2052;
    public static final double CORAL_L4_OUT_SPEED = 0.30; // 35 --> 75
    public static final double CORAL_L2_L3_OUT_SPEED = 0.30; // 35 --> 75
    public static final double ALGAE_IN_SPEED = 0.5;
    public static final double ALGAE_OUT_SPEED = -1.00;
    // public static final CANrangeConfiguration CANRANGE_CONFIG = new CANrangeConfiguration()
    // .withToFParams(new ToFParamsConfigs().withUpdateMode(UpdateModeValue.ShortRange100Hz))
    // .withProximityParams(new ProximityParamsConfigs().withMinSignalStrengthForValidMeasurement(0.2).withProximityThreshold(0.38));

    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG =
        new MotorOutputConfigs()
            .withInverted(
              ArmRollerConstants.MOTOR_INVERTED
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake); 

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(40.0)
    .withSupplyCurrentLimitEnable(true);
    
    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
    .withMotorOutput(MOTOR_OUTPUT_CONFIG)
    .withCurrentLimits(CURRENT_LIMITS_CONFIG)
    .withAudio(new AudioConfigs().withBeepOnBoot(false));
  }

  public static final class IntakePivotConstants {
    public static final boolean MOTOR_INVERTED = true;
    public static final double DEG_TOL = 2.5;

    public static final double MOTOR_RANGE = 19.568359375; // rotations
    public static final double ENCODER_RANGE = 0.306; // rotations

    public static final Angle ENCODER_OFFSET = Rotations.of(-0.175); // .187

    public static final double MIN_INTAKE_ARM_ANGLE = 0;
    public static final double MAX_INTAKE_ARM_ANGLE = 20;

    public static final double kS = 0.0;
    public static final double kG = 0.3;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final double MAX_VELOCITY = 36 * Math.PI;
    public static final double MAX_ACCELERATION = 350;

    public static final CurrentLimitsConfigs CURRENT_LIMIT_CONFIG =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(Amps.of(70))
            .withSupplyCurrentLowerLimit(Amps.of(30))
            .withStatorCurrentLimit(Amps.of(100))
            .withSupplyCurrentLowerTime(Seconds.of(0.2));
            
    public static final Slot0Configs SLOT0_CONFIGS = 
        new Slot0Configs()
            .withKP(75.0) // 22.673
            .withKI(0.0)
            .withKD(7.0); // 14.89

    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG =
        new MotorOutputConfigs()
            .withInverted(
                IntakePivotConstants.MOTOR_INVERTED
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake); 

    // public static final FeedbackConfigs FEEDBACK_CONFIG =
    //     new FeedbackConfigs()
    //         .withFeedbackRemoteSensorID(Ports.INTAKE_ENCODER_ID)
    //         .withFeedbackSensorSource(FeedbackSensorSourceValue.SyncCANcoder)
    //         .withRotorToSensorRatio(58.333)
    //         .withSensorToMechanismRatio(1);
    
    public static final SoftwareLimitSwitchConfigs LIMIT_SWITCH_CONFIGS = 
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(MIN_INTAKE_ARM_ANGLE)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(MAX_INTAKE_ARM_ANGLE)
            .withReverseSoftLimitEnable(true);
    
    public static final TalonFXConfiguration MOTOR_CONFIG = 
        new TalonFXConfiguration()
            .withSlot0(SLOT0_CONFIGS)
            .withMotorOutput(MOTOR_OUTPUT_CONFIG)
            .withAudio(new AudioConfigs().withBeepOnBoot(false))
            .withCurrentLimits(CURRENT_LIMIT_CONFIG);
  }

  public static final class IntakeRollerConstants {
    public static final boolean MOTOR_INVERTED = false;
    public static final double INTAKE_SPEED = -1.00;
    public static final double OUTTAKE_SPEED = 1.00;
    public static final double MAX_RPS = 100;

    public static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG =
        new MotorOutputConfigs()
            .withInverted(
              IntakeRollerConstants.MOTOR_INVERTED
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake); 

    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs().withKS(2.5).withKP(5).withKI(0).withKD(0);

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(30.0)
    .withSupplyCurrentLimitEnable(true);
    
    public static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
    .withMotorOutput(MOTOR_OUTPUT_CONFIG)
    .withCurrentLimits(CURRENT_LIMITS_CONFIG)
    .withSlot0(SLOT0_CONFIGS)
    .withAudio(new AudioConfigs().withBeepOnBoot(false));
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

  public static final class SuperstructureConstants {
    public static final double MIN_SAFE_ROTATION = 8.25;
    public static final double MIN_MOVE_ROTATION = 35.0;
    public static final double RIGHT_LIMIT = 215;
    public static final double LEFT_LIMIT = 295;
  }

  public static class VisionConstants {
    public static final double DEFAULT_XY_STDDEV = 0.1;
    public static final double DEFAULT_HEADING_STDDEV = 0.1;
    public static final Matrix<N3, N1> VISION_STDDEV =  new Matrix<>(VecBuilder.fill(DEFAULT_XY_STDDEV, DEFAULT_XY_STDDEV, DEFAULT_HEADING_STDDEV));

    public static final double MAX_POSE_AMBIGUITY = 0.15;
    public static final Distance FIELD_BORDER_MARGIN = Meters.of(0.5);
    public static final Distance MAX_VISION_CORRECTION = Meters.of(1);

    public static final class LeftLimelightConstants {
      public static final String CAMERA_NAME = "limelight-left";
      public static final Distance X_OFFSET = Inches.of(7.685); // forward positive
      public static final Distance Y_OFFSET = Inches.of(-8.687); // right positive
      public static final Distance Z_OFFSET = Inches.of(12.294); // up positive
      public static final Angle THETA_X_OFFSET = Degrees.of(0); // roll
      public static final Angle THETA_Y_OFFSET = Degrees.of(0); // pitch
      public static final Angle THETA_Z_OFFSET = Degrees.of(-22.5); // yaw
    }

    public static final class RightLimelightConstants {
      public static final String CAMERA_NAME = "limelight-right";
      public static final Distance X_OFFSET = Inches.of(4.941); // forward positive
      public static final Distance Y_OFFSET = Inches.of(10.445); // right positive
      public static final Distance Z_OFFSET = Inches.of(10.599); // up positive
      public static final Angle THETA_X_OFFSET = Degrees.of(0); // roll
      public static final Angle THETA_Y_OFFSET = Degrees.of(0); // pitch
      public static final Angle THETA_Z_OFFSET = Degrees.of(22.5); // yaw
    }  

    public static final class CoralReefCameraConstants {
      public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =  FieldConstants.DEFAULT_APRIL_TAG_LAYOUT;
      public static final String CAMERA_NAME = "Arducam_OV2311_USB_Camera_001";

      public static final PoseStrategy STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

      public static final Distance X_OFFSET = Inches.of(9); // forward positive
      public static final Distance Y_OFFSET = Inches.of(-0.5); // left positive
      public static final Distance Z_OFFSET = Inches.of(7.875); // up positive

      public static final Angle THETA_X_OFFSET = Degrees.of(0); // roll
      public static final Angle THETA_Y_OFFSET = Degrees.of(-21.84); // pitch
      public static final Angle THETA_Z_OFFSET = Degrees.of(0); // yaw

      public static final Transform3d ROBOT_TO_CAMERA =
          new Transform3d(
              new Translation3d(X_OFFSET, Y_OFFSET, Z_OFFSET),
              new Rotation3d(THETA_X_OFFSET, THETA_Y_OFFSET, THETA_Z_OFFSET));

      public static TagTrackerConstants TagTrackerConstants() {
        return new TagTrackerConstants(
            CAMERA_NAME, ROBOT_TO_CAMERA, APRIL_TAG_FIELD_LAYOUT, STRATEGY);
      }
    }
  }

  
  public static final class DashboardConstants {
    public static final String DRIVE_MODE_KEY = "Drive Mode";
    public static final String AUTO_COMPILED_KEY = "Auto Compiled";
    public static final String CHOREO_AUTO_COMPILED = "Choreo Auto Compiled";
    public static final String AUTO_DESCRIPTION_KEY = "Auto Description";
    public static final String WAIT_SECONDS_SAVED_KEY = "Wait Seconds Saved";
    public static final String WAIT_SECONDS_DISPLAY_KEY = "Wait Seconds Display";

    public static final String ELVATOR_NUDGE_VALUE_DISPLAY_KEY = "Nudge Value";
    public static final String ELEVATOR_NUDGE_SAVED_KEY = "Elevator Nudge Value Saved";
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
