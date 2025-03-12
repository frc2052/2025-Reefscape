// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team2052.lib.util.DelayedBoolean;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeShooterConstants;
import frc.robot.util.io.Ports;

public class AlgaeShooterSubsystem extends SubsystemBase {
    private static AlgaeShooterSubsystem INSTANCE;

    private final TalonFX scoringMotor;

    private boolean isIntaking = false;
    private boolean hasAlgae = false;
    private DelayedBoolean intakingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.05);

    public static AlgaeShooterSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AlgaeShooterSubsystem();
        }
        return INSTANCE;
    }

    private AlgaeShooterSubsystem() {
        scoringMotor = new TalonFX(Ports.ALGAE_SCORING_ID);

        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.SupplyCurrentLimit = AlgaeShooterConstants.MOTOR_CURRENT_LIMIT;
        limitConfigs.SupplyCurrentLowerTime = (0.75);
        limitConfigs.SupplyCurrentLowerLimit = (5.0);
        limitConfigs.SupplyCurrentLimitEnable = false;

        scoringMotor.getConfigurator().apply(AlgaeShooterConstants.MOTOR_CONFIG.withCurrentLimits(limitConfigs));
    }

    public void intake() {
        scoringMotor.set(AlgaeShooterConstants.INTAKE_SPEED);
        isIntaking = true;
    }

    public void outtake() {
        scoringMotor.set(AlgaeShooterConstants.SCORE_SPEED);
        isIntaking = false;
        hasAlgae = false;
    }

    public void stopScoringMotor() {
        scoringMotor.stopMotor();
        isIntaking = false;
    }

    @Override
    public void periodic() {}
}
