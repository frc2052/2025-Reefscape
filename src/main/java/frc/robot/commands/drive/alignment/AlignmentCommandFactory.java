package frc.robot.commands.drive.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotState;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;
import frc.robot.util.io.Dashboard;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AlignmentCommandFactory {
    private static final RobotState robotState = RobotState.getInstance();
    private static final ControlBoard controlBoard = ControlBoard.getInstance();

    public static Command getReefAlignmentCommand(Supplier<AlignOffset> offset) {
        if (offset.get() != AlignOffset.LEFT_BRANCH
                && offset.get() != AlignOffset.MIDDLE_REEF
                && offset.get() != AlignOffset.RIGHT_BRANCH) {
            invalidCombination(DesiredElement.REEF, offset.get());
        }
        Supplier<Pose2d> targetSupplier = () -> robotState.getAlignPose();
        BooleanSupplier shouldAlign = () -> robotState.shouldAlign();

        return new InstantCommand(() -> robotState.setAlignOffset(offset.get()))
                .andThen(Commands.either(
                        new DriveToPose(
                                targetSupplier,
                                robotState::getFieldToRobot,
                                controlBoard::getThrottle,
                                controlBoard::getStrafe,
                                controlBoard::getRotation),
                        getDefaultDriveCommand(),
                        shouldAlign));
    }

    public static Command getSpecificReefAlignmentCommand(Supplier<AlignOffset> offset, FieldElementFace reefFace) {
        if (offset.get() != AlignOffset.LEFT_BRANCH
                && offset.get() != AlignOffset.MIDDLE_REEF
                && offset.get() != AlignOffset.RIGHT_BRANCH) {
            invalidCombination(DesiredElement.REEF, offset.get());
        }

        Supplier<Pose2d> targetSupplier = robotState::getAlignPose;
        BooleanSupplier seesDesiredFace = robotState::desiredReefFaceIsSeen;

        return new InstantCommand(() -> robotState.setDesiredReefFace(reefFace))
                .andThen(new InstantCommand(() -> robotState.setAlignOffset(offset.get())))
                .andThen(getDefaultDriveCommand()
                        .withDeadline(Commands.waitUntil(seesDesiredFace))
                        .andThen(new DriveToPose(
                                targetSupplier,
                                robotState::getFieldToRobot,
                                controlBoard::getThrottle,
                                controlBoard::getStrafe,
                                controlBoard::getRotation)));
    }

    private static Command getDefaultDriveCommand() {
        return new DefaultDriveCommand(
                controlBoard::getThrottle,
                controlBoard::getStrafe,
                controlBoard::getRotation,
                Dashboard.getInstance()::isFieldCentric);
    }

    public static FieldElementFace idToReefFace(int id) {
        switch (id) {
            case 18:
                return FieldElementFace.AB;
            case 7:
                return FieldElementFace.AB;
            case 17:
                return FieldElementFace.CD;
            case 8:
                return FieldElementFace.CD;
            case 22:
                return FieldElementFace.EF;
            case 9:
                return FieldElementFace.EF;
            case 21:
                return FieldElementFace.GH;
            case 10:
                return FieldElementFace.GH;
            case 20:
                return FieldElementFace.IJ;
            case 11:
                return FieldElementFace.IJ;
            case 19:
                return FieldElementFace.KL;
            case 6:
                return FieldElementFace.KL;
            default:
                return null;
        }
    }

    private static Command invalidCombination(DesiredElement desiredElement, AlignOffset offset) {
        return new PrintCommand("The desired element "
                + desiredElement.toString()
                + " and the field location "
                + offset.toString()
                + " do not match!");
    }

    public static enum DesiredElement {
        PROCESSOR,
        REEF,
        CORALSTATION,
        SPECIFIC_REEF_FACE;
    }
}
