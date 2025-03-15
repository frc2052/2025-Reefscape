// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.algae.AlgaeCommandFactory;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.SnapToLocationAngleCommand;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.hand.HandCommandFactory;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public abstract class AutoBase extends SequentialCommandGroup {
    private final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    private final VisionSubsystem vision = VisionSubsystem.getInstance();
    private final AutoFactory autoFactory = AutoFactory.getInstance();
    private Pose2d startPose;

    // left side blue l4 // good enough, L4 mmissing
    // left side red l4
    // left side blue l1 // adjusting
    // left side red l1

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

    protected Command followPathSnap(PathPlannerPath path, FieldElementFace snapLoc) {
        return new SequentialCommandGroup(followPathCommand(path), snapToReefAngle(snapLoc));
    }

    protected Command startHP() {
        return new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.HP));
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
        if (autoFactory.getBumpNeeded()) {
            return new DefaultDriveCommand(() -> 0.6, () -> 0, () -> 0, () -> true).withDeadline(new WaitCommand(1.5));
        } else {
            return new InstantCommand();
        }
    }

    protected Command delaySelectedTime() {
        return new WaitCommand(autoFactory.getSavedWaitSeconds());
    }

    protected Command snapToReefAngle(FieldElementFace snapLocation) {
        return new SnapToLocationAngleCommand(snapLocation, () -> 0, () -> 0, () -> 0, () -> true);
    }

    protected Command safeReefAlignment(Path startPath, AlignOffset branchside, FieldElementFace fieldLoc) {

        double distance;

        // start paths start alignment from closer to drive path longer
        if (startPath.equals(PathsBase.B_SL_J2) || startPath.equals(PathsBase.R_SL_J2)) {
            distance = 2.0;
        } else {
            distance = 2.5;
        }

        return new ParallelCommandGroup(
                new InstantCommand(() -> HandSubsystem.getInstance().motorIn())
                        .withTimeout(
                                (startPath.equals(PathsBase.B_SL_J2) || startPath.equals(PathsBase.R_SL_J2))
                                        ? 0.0
                                        : 0.35),
                new InstantCommand(() -> RobotState.getInstance().setDesiredReefFace(fieldLoc)),
                followPathCommand(startPath.getChoreoPath()) // TODO: default to choreo path
                        .until(() -> RobotState.getInstance().shouldAlignAutonomous(distance))
                        .andThen(AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> branchside, fieldLoc)));
    }

    protected Command safeStationAlignment(Path altAlignPath) {
        return new SequentialCommandGroup(followPathCommand(altAlignPath.getChoreoPath())
                .alongWith(new InstantCommand(
                        () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.HP)))
                .alongWith(new InstantCommand(() -> HandSubsystem.getInstance().motorIn())
                        .beforeStarting(new WaitCommand(1.0))));
    }

    protected Command combinedReefChassisElevatorAlign(
            PathPlannerPath backupPath, AlignOffset branchSide, FieldElementFace snapSide, TargetAction level) {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(followPathCommand(backupPath), snapToReefAngle(snapSide))
                        .andThen(AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> branchSide, snapSide)),
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(level)));
    }

    // game piece interactions

    protected Command HPIntake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> HandSubsystem.getInstance().motorIn()), new WaitCommand(2.0));
    }

    protected Command elevatorToPos(TargetAction position) {
        return new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position));
    }

    protected Command score(TargetAction position) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position)),
                Commands.waitUntil(() -> ElevatorSubsystem.getInstance().atPosition(2.0, position)
                                && CoralArmSubsystem.getInstance().isAtDesiredPosition(4.0))
                        .andThen(HandCommandFactory.motorOut().withTimeout(0.30)));
        // .andThen( // do in path to station
        //     new InstantCommand(
        //         () ->
        //             SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.HP))));
    }

    protected Command toPosAndScore(TargetAction position) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(position)),
                new InstantCommand(() -> HandSubsystem.getInstance().stopMotor()),
                Commands.waitUntil(() -> ElevatorSubsystem.getInstance().atPosition(2.0, position)
                                && CoralArmSubsystem.getInstance().isAtDesiredPosition())
                        .andThen(HandCommandFactory.motorIn().withTimeout(0.2))
                        .andThen(new InstantCommand(
                                () -> HandSubsystem.getInstance().stopMotor()))
                        .andThen(HandCommandFactory.motorOut().withTimeout(0.55)));
    }

    protected Command scoreL1(TargetAction position) {
        return new SequentialCommandGroup(
                Commands.waitUntil(() -> ElevatorSubsystem.getInstance().atPosition(2.0, position)
                                && CoralArmSubsystem.getInstance().isAtDesiredPosition())
                        .andThen(HandCommandFactory.motorOut().withTimeout(1.0)));
    }

    protected Command prepareForScoreWhenReady(TargetAction action) {
        final double maxSpeed;
        final double maxDist;
        TargetAction prepAction = TargetAction.TR;
        if (action == null) {
            maxSpeed = 0;
            maxDist = 0.1;
        } else
            switch (action) {
                case L1H -> {
                    maxSpeed = 2.5;
                    maxDist = 1.5;
                    prepAction = action;
                }
                case L2, L3 -> {
                    maxSpeed = 1.5;
                    maxDist = 1.0;
                    prepAction = TargetAction.L2;
                }
                case L4 -> {
                    maxSpeed = 0.5;
                    maxDist = 0.5; // TODO: test farther distances to raise elevator
                    prepAction = TargetAction.L4;
                }
                default -> {
                    maxSpeed = 0;
                    maxDist = 0.1;
                }
            }

        BooleanSupplier speedCheck =
                () -> (MathHelpers.chassisSpeedsNorm(RobotState.getInstance().getChassisSpeeds()) < maxSpeed);
        BooleanSupplier distanceCheck = () -> RobotState.getInstance().distanceToAlignPose() < maxDist;
        return elevatorToPos(prepAction)
                .beforeStarting(Commands.waitUntil(distanceCheck))
                .andThen(new PrintCommand("CLOSE ENOUGH**************"));
    }

    // protected Command descoreAlgae(PathPlannerPath toPosition,TargetAction algaeLevel){
    //   return new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //       new InstantCommand(() ->
    // SuperstructureSubsystem.getInstance().setCurrentAction(algaeLevel)).withTimeout(1.0),
    //       followPathCommand(toPosition)
    //     ),
    //     new InstantCommand(() -> AlgaeShooterSubsystem.getInstance().)
    //   );
    // }

    protected Command descoreScoreNetAlgae(
            PathPlannerPath toPositionPath, TargetAction algaeLevel, PathPlannerPath score) {
        return new SequentialCommandGroup(
                // descore
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(algaeLevel)),
                followPathCommand(toPositionPath),

                // score
                new ParallelCommandGroup(
                                followPathCommand(score),
                                new SequentialCommandGroup(
                                        new WaitCommand(1.0),
                                        new InstantCommand(() -> SuperstructureSubsystem.getInstance()
                                                .setCurrentAction(TargetAction.L4))),
                                AlgaeCommandFactory.intake().withTimeout(1.5))
                        .andThen(AlgaeCommandFactory.outtake().withTimeout(1.0)));
    }

    public static class Path { // combines access to pathplanner and choreo
        private String pathPlannerPathName, chorPathName;

        public Path(String PPName, String chorName) {
            pathPlannerPathName = PPName;
            chorPathName = chorName;
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
        // public PathPlannerPath getChoreoPathAtIndex(){

        // }

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
        // USING W/ CHOREO PATHS, CREATED
        public static final Path B_SL_J2 = new Path("SL J", "BLUE SL J");
        public static final Path B_J2_LL = new Path("J LL", "BLUE J LL");
        public static final Path B_LL_K4 = new Path("LL K", "BLUE LL K");
        public static final Path B_K4_LL = new Path("K LL", "BLUE K LL");

        public static final Path R_SL_J2 = new Path("SL J", "RED SL J");
        public static final Path R_J2_LL = new Path("J LL", "RED J LL");
        public static final Path R_LL_K4 = new Path("LL K", "RED LL K");
        public static final Path R_K4_LL = new Path("K LL", "RED K LL");
        //

        // public static final Path LL_STOP = new Path("LL STOP", "LL STOP");
        // public static final Path LL_AB = new Path("LL AB", "LL AB");
        // public static final Path RL_C4 = new Path("RL C", "RL C");
        // public static final Path C4_RL = new Path("C RL", "C RL");
        // public static final Path RL_C3 = RL_C4;
        // public static final Path RL_D4 = new Path("RL D", "RL D");
        // public static final Path D4_RL = new Path("D RL", "D RL");
        // public static final Path SR_D4 = new Path("SR D", "SR D");
        // public static final Path RL_D3 = RL_D4;
        // public static final Path D3_RL = new Path("D RL", "D RL");
        // public static final Path RL_CD_L1 = new Path("RL CD L1", "RL CD L1");
        // public static final Path CD_RL = new Path("CD RL", "CD RL");

        // // ef
        // public static final Path E2_RL = new Path("E RL", "E RL");
        // public static final Path SR_E2 = new Path("SR E", "SR E");
        // public static final Path SR_F = new Path("SR F", "SR F");
        // public static final Path SR_EF = new Path("SR EF", "SR EF");
        // public static final Path SR_EF_L1 = new Path("SR EF L1", "SR EF L1");
        // public static final Path EF_RL = new Path("EF RL", "EF RL");
        // public static final Path RL_EF = new Path("RL EF", "RL EF");

        // // gh
        // public static final Path SC_H4 = new Path("SC H", "SC H");
        // public static final Path H4_PROCESS = new Path("H Processor", "H Processor");
        // public static final Path H_ALGAE_PREP = new Path("H Algae Prep", "H ALGAE PREP");
        // public static final Path SC_GH = new Path("SC G", "SC G");
        // public static final Path G_AlGAE_PREP = new Path("G Algae Prep", "G Algae Prep");

        // // ij
        // public static final Path SL_IJ = new Path("SL IJ", "SL IJ");
        // public static final Path LL_IJ = new Path("LL IJ", "LL IJ");
        // public static final Path SL_IJ_L1 = new Path("SL IJ L1", "SL IJ L1");

        // // kl
        // public static final Path K3_LL = new Path("K LL", "K LL");
        // public static final Path LL_K3 = new Path("LL K", "LL K");
        // public static final Path L4_LL = new Path("L LL", "L LL");
        // public static final Path LL_L3 = new Path("LL L", "LL L");
        // public static final Path LL_L4 = LL_L3;
        // public static final Path SL_K4 = new Path("SL K", "SL K");
        // public static final Path LL_KL = new Path("LL KL", "LL KL");
        // public static final Path KL_LL = new Path("KL LL", "KL LL");
        // public static final Path LL_KL_L1 = new Path("LL KL L1", "LL KL L1");

        //  // algae score and descore
        //  public static final Path KL_NET = new Path("KL Net", "KL Net");
        //  public static final Path NET_KL = new Path("Net KL", "Net KL");

        //  public static final Path CD_NET = new Path("CD Net", "CD Net");
        //  public static final Path NET_CD = new Path("Net CD", "Net CD");

        //  public static final Path GH_NET = new Path("GH Net", "GH Net");
        //  public static final Path NET_GH = new Path("Net GH", "Net GH");

        //  public static final Path NET_IJ = new Path("Net IJ", "Net IJ");
        //  public static final Path IJ_NET = new Path("IJ Net", "IJ Net");

        //  public static final Path NET_EF = new Path("Net EF", "Net EF");
        //  public static final Path EF_NET = new Path("EF Net", "EF Net");

        //  public static final PathPlannerPath NET_SCORE_LEFT_STATION = getPathFromFile("Net Left Station");
        //  public static final PathPlannerPath NET_SCORE_RIGHT_STATION = getPathFromFile("Net Right Station");

        //  public static final PathPlannerPath KL_SCORE_TO_DESCORE = getPathFromFile("KL Descore Algae");
        //  public static final PathPlannerPath CD_SCORE_TO_DESCORE = getPathFromFile("CD Descore Algae");
        //  public static final PathPlannerPath GH_SCORE_TO_DESCORE = getPathFromFile("GH Descore Algae");
    }

    public static final class Paths {
        // SL = Start Left
        // SR = Start Right
        // LL = Left (Barge Side) Coral Station
        // RL = Right (Processor Side) Coral Station
        // DA = Descore (Remove) Algae From Reef
        // Letter + Number = Reef Scoring Position

        public static final PathPlannerPath LL_STOP = getPathFromFile("LL STOP");
        public static final PathPlannerPath LL_AB = getPathFromFile("LL AB");

        // CD
        public static final PathPlannerPath RL_C4 = getPathFromFile("RL C");
        public static final PathPlannerPath C4_RL = getPathFromFile("C RL");
        public static final PathPlannerPath RL_C3 = getPathFromFile("RL C");
        public static final PathPlannerPath RL_D4 = getPathFromFile("RL D");
        public static final PathPlannerPath D4_RL = getPathFromFile("D RL");
        public static final PathPlannerPath SR_D4 = getPathFromFile("SR D");
        public static final PathPlannerPath RL_D3 = getPathFromFile("RL D");
        public static final PathPlannerPath D3_RL = getPathFromFile("D RL");
        public static final PathPlannerPath RL_CD_L1 = getPathFromFile("RL CD L1");
        public static final PathPlannerPath CD_RL = getPathFromFile("CD RL");

        // ef
        public static final PathPlannerPath E2_RL = getPathFromFile("E RL");
        public static final PathPlannerPath SR_E2 = getPathFromFile("SR E");
        public static final PathPlannerPath SR_F = getPathFromFile("SR F");
        public static final PathPlannerPath SR_EF = getPathFromFile("SR EF");
        public static final PathPlannerPath SR_EF_L1 = getPathFromFile("SR EF L1");
        public static final PathPlannerPath EF_RL = getPathFromFile("EF RL");
        public static final PathPlannerPath RL_EF = getPathFromFile("RL EF");

        // gh
        public static final PathPlannerPath SC_H4 = getPathFromFile("SC H");
        public static final PathPlannerPath H4_PROCESS = getPathFromFile("H Processor");
        public static final PathPlannerPath H_ALGAE_PREP = getPathFromFile("H Algae Prep");
        public static final PathPlannerPath SC_GH = getPathFromFile("SC G");
        public static final PathPlannerPath G_AlGAE_PREP = getPathFromFile("G Algae Prep");

        // ij
        public static final PathPlannerPath J2_LL = getPathFromFile("J LL");
        public static final PathPlannerPath SL_J2 = getPathFromFile("SL J");
        public static final PathPlannerPath SL_IJ = getPathFromFile("SL IJ");
        public static final PathPlannerPath LL_IJ = getPathFromFile("LL IJ");
        public static final PathPlannerPath SL_IJ_L1 = getPathFromFile("SL IJ L1");

        // kl
        public static final PathPlannerPath K3_LL = getPathFromFile("K LL");
        public static final PathPlannerPath K4_LL = getPathFromFile("K LL");
        public static final PathPlannerPath L4_LL = getPathFromFile("L LL");
        public static final PathPlannerPath LL_K3 = getPathFromFile("LL K");
        public static final PathPlannerPath LL_K4 = getPathFromFile("LL K");
        public static final PathPlannerPath LL_L3 = getPathFromFile("LL L");
        public static final PathPlannerPath LL_L4 = getPathFromFile("LL L");
        public static final PathPlannerPath SL_K4 = getPathFromFile("SL K");
        public static final PathPlannerPath LL_KL = getPathFromFile("LL KL");
        public static final PathPlannerPath KL_LL = getPathFromFile("KL LL");
        public static final PathPlannerPath LL_KL_L1 = getPathFromFile("LL KL L1");

        // algae score and descore
        public static final PathPlannerPath KL_NET = getPathFromFile("KL Net");
        public static final PathPlannerPath NET_KL = getPathFromFile("Net KL");

        public static final PathPlannerPath CD_NET = getPathFromFile("CD Net");
        public static final PathPlannerPath NET_CD = getPathFromFile("Net CD");

        public static final PathPlannerPath GH_NET = getPathFromFile("GH Net");
        public static final PathPlannerPath NET_GH = getPathFromFile("Net GH");

        public static final PathPlannerPath NET_IJ = getPathFromFile("Net IJ");
        public static final PathPlannerPath IJ_NET = getPathFromFile("IJ Net");

        public static final PathPlannerPath NET_EF = getPathFromFile("Net EF");
        public static final PathPlannerPath EF_NET = getPathFromFile("EF Net");

        public static final PathPlannerPath NET_SCORE_LEFT_STATION = getPathFromFile("Net Left Station");
        public static final PathPlannerPath NET_SCORE_RIGHT_STATION = getPathFromFile("Net Right Station");

        public static final PathPlannerPath KL_SCORE_TO_DESCORE = getPathFromFile("KL Descore Algae");
        public static final PathPlannerPath CD_SCORE_TO_DESCORE = getPathFromFile("CD Descore Algae");
        public static final PathPlannerPath GH_SCORE_TO_DESCORE = getPathFromFile("GH Descore Algae");
    }
}
