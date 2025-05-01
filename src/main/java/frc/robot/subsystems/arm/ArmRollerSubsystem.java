// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team2052.lib.util.DelayedBoolean;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmRollerConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.io.Ports;
import org.littletonrobotics.junction.Logger;

public class ArmRollerSubsystem extends SubsystemBase {
    private final TalonFX motor;
    // private final CANrange range;
    private final DigitalInput beamBreak;
    private static ArmRollerSubsystem INSTANCE;
    private boolean isIntaking = false;

    private DelayedBoolean hasCoralDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.1);
    private boolean hasCoral;

    public static ArmRollerSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new ArmRollerSubsystem();
        }
        return INSTANCE;
    }

    private ArmRollerSubsystem() {
        motor = new TalonFX(Ports.ARM_ROLLER_TALONFX_ID);
        beamBreak = new DigitalInput(Ports.CORAL_BEAM_BREAK_PIN);
        // range = new CANrange(Ports.ARM_ROLLER_CANRANGE_ID);

        // range.getConfigurator().apply(ArmRollerConstants.CANRANGE_CONFIG);

        motor.getConfigurator().apply(ArmRollerConstants.MOTOR_CONFIG);
    }

    private void setMotor(double speed) {
        motor.set(speed);
    }

    public void stopMotor() {
        isIntaking = false;
        motor.stopMotor();
    }

    public void coralOut() {
        if (SuperstructureSubsystem.getInstance().getCurrentAction().equals(TargetAction.L1H)) {
            setMotor(ArmRollerConstants.CORAL_L1_OUT_SPEED);
        } else if (SuperstructureSubsystem.getInstance().getCurrentAction().equals(TargetAction.L4)) {
            setMotor(ArmRollerConstants.CORAL_L4_OUT_SPEED);
        } else {
            setMotor(ArmRollerConstants.CORAL_L2_L3_OUT_SPEED);
        }
    }

    public void coralIn() {
        isIntaking = true;
        setMotor(ArmRollerConstants.CORAL_IN_SPEED);
    }

    public void algaeOut() {
        setMotor(ArmRollerConstants.ALGAE_OUT_SPEED);
    }

    public void algaeIn() {
        setMotor(ArmRollerConstants.ALGAE_IN_SPEED);
    }

    public double motorVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public boolean beamBreakHit() {
        return !beamBreak.get();
    }

    @Override
    public void periodic() {
        hasCoral = hasCoralDelay.update(Timer.getFPGATimestamp(), beamBreakHit());

        // This method will be called once per scheduler run
        Logger.recordOutput("Arm Rollers/Motor Velocity", motor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Arm Rollers/Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Arm Rollers/Beam Break Hit", beamBreakHit());
        Logger.recordOutput("Arm Rollers/Has Coral", hasCoral);
        RobotState.getInstance().setHasCoral(hasCoral);
        RobotState.getInstance().setIsIntaking(isIntaking);
    }
}
