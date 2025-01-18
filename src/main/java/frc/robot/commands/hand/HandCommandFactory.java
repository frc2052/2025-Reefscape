package frc.robot.commands.hand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.HandSubsystem;

public class HandCommandFactory {
    private static final HandSubsystem hand = HandSubsystem.getInstance();

    public static Command motorIn() {
        return Commands.runOnce(() -> hand.motorIn(), hand);
    }
    
    public static Command motorOut() {
        return Commands.runOnce(() -> hand.motorOut(), hand);
    }
}
