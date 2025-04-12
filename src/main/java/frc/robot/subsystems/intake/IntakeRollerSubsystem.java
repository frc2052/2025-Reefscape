// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.util.io.Ports;

public class IntakeRollerSubsystem extends SubsystemBase {
    private TalonFX motor;
    private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(0);

    private static IntakeRollerSubsystem INSTANCE;

    public static IntakeRollerSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new IntakeRollerSubsystem();
        }
        return INSTANCE;
    }

    private IntakeRollerSubsystem() {
        motor = new TalonFX(Ports.INTAKE_ROLLER_ID);
        motor.getConfigurator().apply(IntakeRollerConstants.MOTOR_CONFIG);
    }

    private void setMotorPct(double pct) {
        double desiredRPS = pct *= IntakeRollerConstants.MAX_RPS;
        motor.setControl(m_velocityTorque.withVelocity(desiredRPS));
        // motor.set(speed);
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    public void outtake() {
        setMotorPct(IntakeRollerConstants.OUTTAKE_SPEED);
    }

    public void intake() {
        setMotorPct(IntakeRollerConstants.INTAKE_SPEED);
    }

    public void intakeAlgae() {
        setMotorPct(-0.75);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
