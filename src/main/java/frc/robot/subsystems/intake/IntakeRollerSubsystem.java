// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.util.io.Ports;
import org.littletonrobotics.junction.Logger;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final DigitalInput beamBreak;
    private boolean holdCoral = false;
    private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(0);

    private static IntakeRollerSubsystem INSTANCE;

    public static IntakeRollerSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeRollerSubsystem();
        }
        return INSTANCE;
    }

    private IntakeRollerSubsystem() {
        beamBreak = new DigitalInput(Ports.INTAKE_BEAM_BREAK_ID);
        motor = new TalonFX(Ports.INTAKE_ROLLER_ID);
        motor.getConfigurator().apply(IntakeRollerConstants.MOTOR_CONFIG);
    }

    private void setMotorPct(double pct) {
        double desiredRPS = pct *= IntakeRollerConstants.MAX_RPS;
        motor.setControl(m_velocityTorque.withVelocity(desiredRPS));
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public void outtake() {
        setMotorPct(IntakeRollerConstants.OUTTAKE_SPEED);
    }

    public void intake() {
        if (!isHoldingCoral() && ArmPivotSubsystem.getInstance().atPosition(TargetAction.INTAKE)) {
            setMotorPct(IntakeRollerConstants.INTAKE_SPEED);
        } else {
            stopMotor();
        }
    }

    public void setHoldCoral(boolean holdCoral) {
        this.holdCoral = holdCoral;
    }

    public boolean isHoldingCoral() {
        return isBeamBreakHit() && holdCoral;
    }

    public boolean isBeamBreakHit() {
        return !beamBreak.get();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake Rollers/Beam Break Hit", isBeamBreakHit());
        Logger.recordOutput("Intake Rollers/Holding Coral", isHoldingCoral());
    }
}
