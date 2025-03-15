package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmRollerSubsystem;

public class ArmRollerCommandFactory {
    private static final ArmRollerSubsystem hand = ArmRollerSubsystem.getInstance();

    public static Command coralIn() {
        return Commands.runEnd(() -> hand.coralIn(), () -> hand.stopMotor(), hand);
    }

    public static Command coralOut() {
        return Commands.runEnd(() -> hand.coralOut(), () -> hand.stopMotor(), hand);
    }

    public static Command algaeIn() {
        return Commands.runEnd(() -> hand.algaeIn(), () -> hand.stopMotor(), hand);
    }

    public static Command algaeOut() {
        return Commands.runEnd(() -> hand.algaeOut(), () -> hand.stopMotor(), hand);
    }
}
