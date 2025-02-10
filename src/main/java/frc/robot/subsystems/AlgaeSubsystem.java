// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.team2052.lib.util.DelayedBoolean;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeSubsystemConstants;
import frc.robot.util.Ports;

public class AlgaeSubsystem extends SubsystemBase {
  private static AlgaeSubsystem INSTANCE;

  // private final TalonFX pivotMotor;
  // private final TalonFX scoringMotor;

  private final AnalogEncoder pivotEncoder;

  private AlgaePosition position;

  private PIDController pivotController;

  private boolean isIntaking = false;
  private DelayedBoolean intakingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.05);

  public static AlgaeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new AlgaeSubsystem();
    }
    return INSTANCE;
  }

  public AlgaeSubsystem() {
    // pivotMotor = new TalonFX(Ports.ALGAE_PIVOT_ID);
    // scoringMotor = new TalonFX(Ports.ALGAE_SCORING_ID);
    pivotEncoder = new AnalogEncoder(Ports.ALGAE_ENCODER_ID);

    // pivotMotor.getConfigurator().apply(AlgaeSubsystemConstants.Motors.PIVOT_CONFIG);
    // scoringMotor.getConfigurator().apply(AlgaeSubsystemConstants.Motors.SCORING_CONFIG);

    position = AlgaePosition.STOWED;

    pivotController =
        new PIDController(
            AlgaeSubsystemConstants.PIDs.PIVOT_KP,
            AlgaeSubsystemConstants.PIDs.PIVOT_KI,
            AlgaeSubsystemConstants.PIDs.PIVOT_KD);
  }

  private void goToPivotAngle(Angle angle) {
    // pivotMotor.set(pivotController.calculate(pivotEncoder.get(), angle.in(Rotations)));
  }

  @Override
  public void periodic() {
    goToPivotAngle(position.getAngle());
    if (isIntaking) {
      if (intakingDelay.update(
          Timer.getFPGATimestamp(), false
          // MathHelpers.epsilonEquals(pivotMotor.getVelocity().getValueAsDouble(), 0.0, 0.01)
          )) {
        stopScoringMotor();
      }
    }
  }

  public void setGoalPosition(AlgaePosition position) {
    this.position = position;
  }

  public AlgaePosition getCurrentPosition() {
    return position;
  }

  public void intake() {
    // scoringMotor.set(AlgaeSubsystemConstants.Motors.SCORING_INTAKE_SPEED);
    isIntaking = true;
  }

  public void score() {
    // scoringMotor.set(AlgaeSubsystemConstants.Motors.SCORING_SCORE_SPEED);
    isIntaking = false;
  }

  public void stopScoringMotor() {
    // scoringMotor.stopMotor();
    isIntaking = false;
  }

  public enum AlgaePosition {
    NET(Degrees.of(135)),
    DESCORE(Degrees.of(90)),
    STOWED(Degrees.of(0));

    private final Angle angle;

    AlgaePosition(Angle angle) {
      this.angle = angle;
    }

    public Angle getAngle() {
      return angle;
    }
  }
}
