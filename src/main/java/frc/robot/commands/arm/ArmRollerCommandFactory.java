package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.arm.ArmRollerSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.ActionType;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

public class ArmRollerCommandFactory {
    private static final ArmRollerSubsystem hand = ArmRollerSubsystem.getInstance();

    public static Command intake() {
        return new ConditionalCommand(
                algaeIn(),
                coralIn(),
                () -> SuperstructureSubsystem.getInstance().getCurrentAction().getType() == ActionType.ALGAE);
    }

    public static Command outtake() {
        return new ConditionalCommand(
                algaeOut(),
                coralIn().withTimeout(0.1).andThen(coralOut()),
                () -> SuperstructureSubsystem.getInstance().getCurrentAction().getType() == ActionType.ALGAE);
    }

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
