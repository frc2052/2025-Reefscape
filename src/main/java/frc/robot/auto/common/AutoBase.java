// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.intake.IntakeCommandFactory;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.arm.ArmRollerSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public abstract class AutoBase extends SequentialCommandGroup {
    protected final SuperstructureSubsystem superstructure = SuperstructureSubsystem.getInstance();
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final VisionSubsystem vision = VisionSubsystem.getInstance();
    private final AutoFactory autoFactory = AutoFactory.getInstance();
    private Pose2d startPose;

    protected AutoBase(Optional<Pose2d> pathStartPose) {
        if (pathStartPose.isEmpty()) {
            startPose = new Pose2d();
        } else {
            startPose = pathStartPose.get();
        }

        if (RobotState.getInstance().isRedAlliance()) {
            startPose = FlippingUtil.flipFieldPose(startPose);
        }

        setStartPose(startPose);
        RobotState.getInstance().setAutoStartPose(startPose);
    }

    public abstract void init(); // defined in each Auto class

    private void setStartPose(Pose2d pathStartPose) {
        addCommands(new InstantCommand(() -> drivetrain.resetPose(pathStartPose)));
    }

    protected Command manualZero() {
        return new InstantCommand(() -> drivetrain.seedFieldCentric());
    }

    protected Command followPathCommand(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    protected Command startHP() {
        return new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.STOW));
    }

    protected static PathPlannerPath getPathFromFile(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return path;
        } catch (Exception e) {
            DriverStation.reportError(
                    "FAILED TO GET PATH FROM PATHFILE" + pathName + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    protected static PathPlannerPath getChoreoTraj(String name) {
        try {
            PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory(name);
            return choreoPath;
        } catch (Exception e) {
            DriverStation.reportError("FAILED TO GET CHOREO PATH: " + name + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    protected PathPlannerPath getChoreoTraj(String name, int index) {
        try {
            PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory(name, index);
            return choreoPath;
        } catch (Exception e) {
            DriverStation.reportError("FAILED TO GET CHOREO PATH: " + name + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    public static Optional<Pose2d> getStartPoseFromAutoFile(String autoName) {
        try {
            List<PathPlannerPath> pathList = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
            return (pathList.get(0).getStartingHolonomicPose());
        } catch (Exception e) {
            DriverStation.reportError(
                    "Couldn't get starting pose from auto file: " + autoName + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    protected Command getBumpCommand() {
        return new ConditionalCommand(
                new DefaultDriveCommand(() -> 0.7, () -> 0, () -> 0, () -> true).withDeadline(new WaitCommand(0.4)),
                new InstantCommand(),
                () -> autoFactory.getBumpNeeded());
    }

    protected Command delaySelectedTime() {
        return new WaitCommand(autoFactory.getSavedWaitSeconds());
    }

    protected Command elevatorToPos(TargetAction position) {
        return new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position));
    }

    protected Command score(TargetAction position) {
        return new ParallelCommandGroup(
                IntakeCommandFactory.outtake().withTimeout(0.3),
                ArmCommandFactory.coralOut().withTimeout(0.5));
    }

    protected BooleanSupplier haveCoral() {
        return () -> (SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.STOW
                || RobotState.getInstance().getHasCoral());
    }

    protected Command toPosAndScore(TargetAction position) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position)),
                new InstantCommand(() -> ArmRollerSubsystem.getInstance().stopMotor()),
                Commands.waitUntil(() -> ElevatorSubsystem.getInstance().atPosition(2.0, position)
                                && ArmPivotSubsystem.getInstance().isAtDesiredPosition())
                        .andThen(ArmCommandFactory.coralIn().withTimeout(0.2))
                        .andThen(new InstantCommand(
                                () -> ArmRollerSubsystem.getInstance().stopMotor()))
                        .andThen(ArmCommandFactory.coralOut().withTimeout(0.55)));
    }

    protected Command pickup(Path path) {
        return (new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                        .beforeStarting(new WaitCommand(0.1)))
                .alongWith(((followPathCommand(path.getPathPlannerPath()))
                        .deadlineFor(IntakeCommandFactory.intake().alongWith(ArmCommandFactory.coralIn()))))
                // interrupted (have coral)? continue
                // path went all the way through? pause + intake
                .andThen(
                        !(SuperstructureSubsystem.getInstance().getCurrentAction() == TargetAction.L3
                                        || RobotState.getInstance().getHasCoral())
                                ? new WaitCommand(0.5)
                                        .deadlineFor(IntakeCommandFactory.intake())
                                        .until(() -> RobotState.getInstance().getHasCoral())
                                : new InstantCommand());
    }

    protected Command scoreNet() {
        return Commands.sequence(
                (Commands.waitUntil(() -> ArmPivotSubsystem.getInstance()
                                        .isAtPosition(2.0, TargetAction.ALGAE_NET.getArmPivotAngle()))
                                .andThen(new WaitCommand(0.3)))
                        .deadlineFor(ArmCommandFactory.algaeIn()),
                ArmCommandFactory.algaeOut().withTimeout(0.5));
    }

    public static class Path { // combines access to pathplanner and choreo
        private String pathPlannerPathName, chorPathName;

        public Path(String PPName, String chorName) {
            pathPlannerPathName = PPName;
            chorPathName = chorName;
        }

        public String getTrajName() {
            return chorPathName;
        }

        public PathPlannerPath getChoreoPath() {
            try {
                return AutoBase.getChoreoTraj(chorPathName); // return choreo
            } catch (Exception e) {
                DriverStation.reportError(
                        "FAILED TO GET CHOREO PATH FROM PATHFILE " + pathPlannerPathName + e.getMessage(),
                        e.getStackTrace());
                return null;
            }
        }

        public PathPlannerPath getPathPlannerPath() {
            try {
                return getPathFromFile(pathPlannerPathName);

            } catch (Exception e) {
                DriverStation.reportError(
                        "FAILED TO GET PATH FROM PATHFILE " + pathPlannerPathName + e.getMessage(), e.getStackTrace());
                return null;
            }
        }
    }

    public static final class PathsBase {

        public static final Path LEFT_ALIGN_REPOS = new Path(null, "LEFT ALIGNMENT REPOSITION");
        public static final Path RIGHT_ALIGN_REPOS = new Path(null, "RIGHT ALIGNMENT REPOSITION");

        public static final Path BLUE_NET_FINAL = new Path(null, "BLUE NET FORWARD");

        public static final Path BLUE_LL_RETRY_STRAIGHT = new Path("SLOW BLUE LL RETRY", "BLUE LL RETRY");
        public static final Path BLUE_RL_RETRY_STRAIGHT = new Path("SLOW BLUE RL RETRY", "BLUE RL RETRY");

        public static final Path B_SC_GH_L1 = new Path("SC GH", "BLUE SC GH L1");

        public static final Path BLUE_LL_LOLIPOP = new Path("LL LEFT LOLIPOP", null);
        public static final Path BLUE_RL_LOLIPOP = new Path("RL RIGHT LOLIPOP", null);

        // left side L1 backups
        // TODO: validate pathplanner paths (not going to use but prevent error)
        public static final Path B_SL_IJ = new Path("SL IJ", "BLUE SL IJ");
        public static final Path B_IJ_LL = new Path("IJ LL", "BLUE IJ LL");

        public static final Path B_LL_KL = new Path("LL KL", "BLUE LL KL");
        public static final Path B_KL_LL = new Path("KL LL", "BLUE KL LL");

        // TODO: right side L1 backups

        // EXTENDED STARTS
        public static final Path EXTENDED_J_LL = new Path("EXTENDED J LL", "EXTENDED J LL");
        public static final Path EXTENDED_E_RL = new Path("EXTENDED E RL", "EXTENDED E RL");

        public static final Path B_SL_J = new Path("SL J", "BLUE SL J");
        public static final Path B_J_LL = new Path("J LL", "BLUE J LL");
        public static final Path B_LL_K = new Path("LL K", "BLUE LL K");
        public static final Path B_K_LL = new Path("K LL", "BLUE K LL");
        public static final Path B_LL_L = new Path("LL L", "BLUE LL L");

        // E/F4D4C4
        public static final Path B_SR_E = new Path("SR E", "BLUE SR E");
        public static final Path B_SR_F = new Path("SR F", "BLUE SR F");
        public static final Path B_E_RL = new Path("E RL", "BLUE E RL");
        public static final Path B_F_RL = new Path("F RL", "BLUE F RL");
        public static final Path B_RL_D = new Path("RL D", "BLUE RL D");
        public static final Path B_D_RL = new Path("D RL", "BLUE D RL");
        public static final Path B_RL_C = new Path("RL C", "BLUE RL C");

        // 5.5 y val for blue net score
        // 5.5 y val for

        // GH
        public static final Path B_SC_G = new Path("SC G", "BLUE SC G"); //
        public static final Path B_SC_H = new Path("SC H", "BLUE SC H"); //

        // GH / NET
        public static final Path B_SC_GH = new Path("SC GH", "BLUE SC GH"); //
        public static final Path B_GH_NET = new Path("GH Net", "BLUE GH NET"); //
        public static final Path B_NET_GH = new Path("NET GH", "BLUE NET GH"); //

        // KL / NET
        public static final Path B_KL_NET = new Path("KL Net", "BLUE KL NET"); //
        public static final Path B_NET_KL = new Path("KL Net", "BLUE NET KL"); //

        // IJ / NET
        public static final Path B_IJ_NET = new Path("IJ Net", "BLUE IJ NET"); //
        public static final Path B_NET_IJ = new Path("Net IJ", "BLUE NET IJ"); //

        public static final Path B_NET_EF = new Path(null, "BLUE NET EF");
        public static final Path B_EF_NET = new Path(null, "BLUE EF NET");

        public static final Path B_GH_REPOSITION_IN = new Path("GH REPOSITION IN", "BLUE GH REP");

        public static final Path B_KL_REPOSITION = new Path("KL Descore Algae", "BLUE KL Reposition"); //

        public static final Path B_IJ_REPOSITION = new Path("IJ Descore Algae", "BLUE IJ Reposition"); //
    }
}
