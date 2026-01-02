package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.intake.IntakeCommandFactory;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.arm.ArmRollerSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;
import java.util.Set;

// auto logic is here as methods instead of separate files

public class AutoFactory2 {
    private final DriverStation.Alliance alliance;
    private final RobotContainer robotContainer;
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();

    // choreo paths are defined for blue alliance
    // all alliance switching logic is stored here

    public AutoFactory2(DriverStation.Alliance alliance, RobotContainer robotContainer) {

        this.alliance = alliance;
        this.robotContainer = robotContainer;
    }

    // -------------------------------- FACTORY METHODS -------------------------------- //

    Pair<Pose2d, Command> createNoAuto() {
        return Pair.of(new Pose2d(), Commands.none());
    }

    Pair<Pose2d, Command> createDriveForwardAuto() {
        return Pair.of(
                getChoreoPath("DRIVE FORWARD").getStartingHolonomicPose().get(),
                Commands.sequence(followPathCommand(getChoreoPath("DRIVE FORWARD"))));
    }

    Pair<Pose2d, Command> createLeftLoliLeftFirstAuto() {
        return Pair.of(
                getChoreoPath("SL A").getStartingHolonomicPose().get(),
                Commands.sequence(
                        Commands.runOnce(() -> RobotState.getInstance().setDesiredReefFace(FieldElementFace.AB)),

                        // home, raise L3 on the way to B
                        Commands.parallel(
                                Commands.run(() -> followPathCommand(getChoreoPath("SL A"))),
                                Commands.sequence(
                                        toPosition(TargetAction.HOME),
                                        Commands.waitUntil(() ->
                                                !ElevatorSubsystem.getInstance().isHoming()),
                                        Commands.waitSeconds(0.3),
                                        toPosition(TargetAction.L3))),

                        // align & score preload
                        Commands.parallel(
                                AlignmentCommandFactory.getSpecificReefAlignmentCommand(
                                                () -> AlignOffset.LEFT_BRANCH, FieldElementFace.AB)
                                        .withTimeout(2.25),
                                Commands.waitSeconds(0.3),
                                toPosition(TargetAction.L4),
                                score(TargetAction.L4)),

                        // 1st pickup
                        toPosition(TargetAction.INTAKE),
                        Commands.deadline(
                                Commands.sequence(
                                        Commands.waitSeconds(0.3), followPathCommand(getChoreoPath("AB LOLIPOP-C"))),
                                Commands.parallel(IntakeCommandFactory.intake(), ArmCommandFactory.coralIn()))));
    }

    // start = SL A
    // 2nd = AB LOLIPOP-C
    // 3rd = AB LOLIPOP-L
    // 4th = AB LOLIPOP-R

    // score 1st pickup
    // addCommands(new ConditionalCommand(
    //         Commands.sequence(
    //                 Commands.parallel(
    //                         AlignmentCommandFactory.getSpecificReefAlignmentCommand(
    //                                         () -> AlignOffset.RIGHT_BRANCH, FieldElementFace.AB)
    //                                 .withTimeout(2.25),
    //                         toPosition(TargetAction.L4)).beforeStarting(new WaitCommand(0.3)),
    //                 score(TargetAction.L4)),
    //         new PrintCommand("DIDN'T GET CENTER").andThen(new InstantCommand(() -> setAScored(false))),
    //         haveCoral()));

    //     // 2nd pickup
    //     addCommands(
    //             toPosition(TargetAction.INTAKE),
    //             ((followPathCommand(loadLeft.getChoreoPath()).beforeStarting(new WaitCommand(0.3)))
    //                             .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.coralIn())))
    //     );

    //     // score 2nd pickup - if have coral and L4 not scored, score L4
    //     addCommands(
    //         new ConditionalCommand(
    //             new ConditionalCommand(
    //                 Commands.sequence(
    //                     Commands.parallel(
    //                         AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH,
    // FieldElementFace.AB).withTimeout(2.25),
    //                         toPosition(TargetAction.L2)),
    //                     score(TargetAction.L2)),
    //                 Commands.sequence(
    //                     Commands.parallel(
    //                         AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH,
    // FieldElementFace.AB).withTimeout(2.25),
    //                         toPosition(TargetAction.L4)),
    //                     score(TargetAction.L4),
    //                     new InstantCommand(() -> setAScored(true))),
    //                 () -> aScored),
    //         new PrintCommand("DIDN'T GET 1st LOLLIPOP"),
    //         haveCoral()));

    //     // 3rd pickup
    //     addCommands(
    //             toPosition(TargetAction.INTAKE),
    //             ((followPathCommand(loadRight.getChoreoPath()).beforeStarting(new WaitCommand(0.3)))
    //                             .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.intake())))
    //     );

    //     // score 3rd pickup - if L4 still hasn't been scored, do it
    //     addCommands(
    //         new ConditionalCommand(
    //             new ConditionalCommand(
    //                 Commands.sequence(
    //                     Commands.parallel(
    //                         AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.RIGHT_BRANCH,
    // FieldElementFace.AB).withTimeout(2.25),
    //                         toPosition(TargetAction.L2)),
    //                     score(TargetAction.L2)),
    //                 Commands.sequence(
    //                     Commands.parallel(
    //                         AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH,
    // FieldElementFace.AB).withTimeout(2.25),
    //                         toPosition(TargetAction.L4)),
    //                     score(TargetAction.L4),
    //                     new InstantCommand(() -> setAScored(true))),
    //                 () -> aScored),
    //         new PrintCommand("DIDN'T GET 2nd LOLLIPOP"),
    //         haveCoral()));

    // -------------------------------- COMMON FUNCTIONS -------------------------------- //

    Command manualZero() {
        return new InstantCommand(() -> drivetrain.seedFieldCentric());
    }

    public PathPlannerPath getChoreoPath(String chorPathName) {
        try {
            return PathPlannerPath.fromChoreoTrajectory(chorPathName);
        } catch (Exception e) {
            DriverStation.reportError(
                    "FAILED TO GET CHOREO PATH FROM PATHFILE " + chorPathName + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    Command followPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    // Command getBumpCommand(){
    //     return new ConditionalCommand(
    //         new DefaultDriveCommand(() -> 0.7, () -> 0, () -> 0, () -> true).withDeadline(new WaitCommand(0.4)),
    //         new InstantCommand(),
    //         () -> AutoChooser.getBumpNeeded());
    // }

    // Command delaySelectedTime(){
    //     return new WaitCommand(AutoChooser.getWaitSeconds());
    // }

    Command elevatorToPos(TargetAction position) {
        return new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position));
    }

    Command score(TargetAction position) {
        return Commands.parallel(
                        IntakeCommandFactory.outtake().withTimeout(0.3),
                        ArmCommandFactory.coralOut().withTimeout(0.5))
                .andThen(() -> RobotState.getInstance().setAlignOffset(AlignOffset.MIDDLE_REEF));
    }

    // TODO: replace this w/ DeferredCommand for auto logic
    // protected BooleanSupplier haveCoral() {
    //     new PrintCommand("STOWED: " + (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L3));
    //     new PrintCommand("HAVE CORAL: " + (RobotState.getInstance().getHasCoral()));
    //     return () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L3
    //             || RobotState.getInstance().getHasCoral());
    // }

    Command toPosAndScore(TargetAction position) {
        return Commands.sequence(
                Commands.runOnce(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position)),
                Commands.runOnce(() -> ArmRollerSubsystem.getInstance().stopMotor()),
                Commands.waitUntil(() -> ElevatorSubsystem.getInstance().atPosition(2.0, position)
                        && ArmPivotSubsystem.getInstance().isAtDesiredPosition()),
                ArmCommandFactory.coralIn().withTimeout(0.2),
                Commands.runOnce(() -> ArmRollerSubsystem.getInstance().stopMotor()),
                ArmCommandFactory.coralOut().withTimeout(0.55));
    }

    Command toPosition(TargetAction position) {
        return Commands.runOnce(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position));
    }

    // TODO: deffered command?
    Command scoreNet() {
        return Commands.sequence(
                ArmCommandFactory.algaeIn().until(() -> ArmPivotSubsystem.getInstance()
                        .isAtPosition(2.0, TargetAction.ALGAE_NET.getArmPivotAngle())),
                ArmCommandFactory.algaeOut().withTimeout(0.5));
    }

    Command pickUp(String chorPathName) {
        // TODO: path needs to run through the coral & wait until pickup

        return Commands.sequence(
                Commands.deadline(
                        Commands.deadline(
                                followPathCommand(getChoreoPath(chorPathName)),
                                Commands.parallel(IntakeCommandFactory.intake(), ArmCommandFactory.coralIn())),
                        Commands.sequence(
                                Commands.waitSeconds(.1), Commands.run(() -> SuperstructureSubsystem.getInstance()
                                        .setCurrentAction(TargetAction.INTAKE)))),
                Commands.defer(
                        () -> {
                            boolean isL3 = SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L3;
                            boolean hasCoral = RobotState.getInstance().getHasCoral();

                            if (!(isL3 || hasCoral)) {
                                return new WaitCommand(0.5)
                                        .deadlineFor(IntakeCommandFactory.intake())
                                        .until(() -> RobotState.getInstance().getHasCoral());
                            } else {
                                // Return a do-nothing command
                                return Commands.none();
                            }
                        },
                        Set.of()));
    }
}
