package frc.robot.auto;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DashboardConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.auto.common.AutoBase;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.intake.IntakeCommandFactory;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.arm.ArmRollerSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.io.Dashboard;

// auto logic is here as methods instead of separate files

public class AutoFactory2 {
    private final DriverStation.Alliance alliance;
    private final RobotContainer robotContainer;
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final SuperstructureSubsystem superstructure = SuperstructureSubsystem.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    // choreo paths are defined for blue alliance
    // all alliance switching logic is stored here

    public AutoFactory2(
        DriverStation.Alliance alliance, 
        RobotContainer robotContainer){

            this.alliance = alliance;
            this.robotContainer = robotContainer;
    }

    // -------------------------------- FACTORY METHODS -------------------------------- //
    

    Pair<Pose2d, Command> createNoAuto(){
        return Pair.of(new Pose2d(), Commands.none());
    }

    Pair<Pose2d, Command> createDriveForwardAuto(){
        return Pair.of(
            getChoreoPath("DRIVE FORWARD").getStartingHolonomicPose().get(), 
            Commands.sequence(
                followPathCommand(getChoreoPath("DRIVE FORWARD"))
            ));
    }

   // -------------------------------- FACTORY METHODS -------------------------------- // 

    Command manualZero() {
        return new InstantCommand(() -> drivetrain.seedFieldCentric());
    }

    public PathPlannerPath getChoreoPath(String chorPathName) {
        try {
            return PathPlannerPath.fromChoreoTrajectory(chorPathName);
        } catch (Exception e) {
            DriverStation.reportError(
                    "FAILED TO GET CHOREO PATH FROM PATHFILE " + chorPathName + e.getMessage(),
                    e.getStackTrace());
            return null;
        }
    }

    Command followPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    Command getBumpCommand(){
        return new ConditionalCommand(
            new DefaultDriveCommand(() -> 0.7, () -> 0, () -> 0, () -> true).withDeadline(new WaitCommand(0.4)), 
            new InstantCommand(), 
            () -> AutoChooser.getBumpNeeded());
    }

    Command delaySelectedTime(){
        return new WaitCommand(AutoChooser.getWaitSeconds());
    }

    Command elevatorToPos(TargetAction position){
        return new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position));
    }

    Command score(TargetAction position){
        return Commands.parallel(
            IntakeCommandFactory.outtake().withTimeout(0.3),
            ArmCommandFactory.coralOut().withTimeout(0.5)
        )
        .andThen(() -> RobotState.getInstance().setAlignOffset(AlignOffset.MIDDLE_REEF));
    }

    // TODO: replace this w/ DeferredCommand for auto logic
    // protected BooleanSupplier haveCoral() {
    //     new PrintCommand("STOWED: " + (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L3));
    //     new PrintCommand("HAVE CORAL: " + (RobotState.getInstance().getHasCoral()));
    //     return () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L3
    //             || RobotState.getInstance().getHasCoral());
    // }

    Command toPosAndScore(TargetAction position){
        return Commands.sequence(
            Commands.runOnce(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position)),
            Commands.runOnce(() -> ArmRollerSubsystem.getInstance().stopMotor()),
            Commands.waitUntil(
                () -> ElevatorSubsystem.getInstance().atPosition(2.0, position)
                && ArmPivotSubsystem.getInstance().isAtDesiredPosition()),
            ArmCommandFactory.coralIn().withTimeout(0.2),
            Commands.runOnce(() -> ArmRollerSubsystem.getInstance().stopMotor()),
            ArmCommandFactory.coralOut().withTimeout(0.55)
        );
    }

    Command toPosition(TargetAction position){
        return Commands.runOnce(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position));
    }

    // TODO: deffered command?
    Command scoreNet(){
        return Commands.sequence(
            ArmCommandFactory.algaeIn()
                .until(() -> ArmPivotSubsystem.getInstance().isAtPosition(2.0, TargetAction.ALGAE_NET.getArmPivotAngle())),
            ArmCommandFactory.algaeOut().withTimeout(0.5)
        );
    }

    Command pickUp(String chorPathName){
        // TODO: path needs to run through the coral & wait until pickup

        return Commands.sequence(
 
            Commands.deadline( 
                Commands.deadline(
                    followPathCommand(getChoreoPath(chorPathName)),
                    Commands.parallel(
                        IntakeCommandFactory.intake(),
                        ArmCommandFactory.coralIn()
                    )
                ),
                Commands.sequence(
                    Commands.waitSeconds(.1),
                    Commands.run(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                )),
            Commands.defer(() -> {

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
            }, Set.of())
        );        
    }
}
