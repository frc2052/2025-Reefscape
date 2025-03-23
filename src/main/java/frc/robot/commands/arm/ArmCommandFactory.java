package frc.robot.commands.arm;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.arm.ArmRollerSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.ActionType;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

public class ArmCommandFactory {
    private static final ArmRollerSubsystem rollers = ArmRollerSubsystem.getInstance();
    private static final ArmPivotSubsystem pivot = ArmPivotSubsystem.getInstance();

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

    public static Command coralInTap() {
        return coralIn().withDeadline(new WaitCommand(0.75));
    }

    public static Command coralIn() {
        return Commands.runEnd(() -> rollers.coralIn(), () -> rollers.stopMotor(), rollers);
    }

    public static Command coralOut() {
        return Commands.runEnd(() -> rollers.coralOut(), () -> rollers.stopMotor(), rollers);
    }

    public static Command algaeIn() {
        return Commands.runEnd(() -> rollers.algaeIn(), () -> rollers.stopMotor(), rollers);
    }

    public static Command algaeOut() {
        return Commands.runEnd(() -> rollers.algaeOut(), () -> rollers.stopMotor(), rollers);
    }

    public static Command setCoast() {
        return Commands.runOnce(() -> pivot.setNeutralMode(NeutralModeValue.Coast), pivot);
    }

    public static Command setBrake() {
        return Commands.runOnce(() -> pivot.setNeutralMode(NeutralModeValue.Brake), pivot);
    }
}
