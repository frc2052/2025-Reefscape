package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;

public class ArmCommandFactory {
    private static final ArmSubsystem arm = ArmSubsystem.getInstance();

    public static Command setArmPosition(ArmPosition position) {
        return Commands.runOnce(() -> arm.setArmPosition(position), arm);
    }
}
