// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.common;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.team2052.lib.helpers.MathHelpers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.auto.common.AutoBase.PathsBase;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;
import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class Autos {

    private final AutoFactory autoFactory;

    //     private final BooleanSupplier flip = () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    private static Autos INSTANCE;

    public static Autos getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Autos();
        }
        return INSTANCE;
    }

    public Autos() {
        autoFactory = new AutoFactory(
                () -> DrivetrainSubsystem.getInstance().getState().Pose,
                DrivetrainSubsystem.getInstance()::resetPose,
                // follow trajectory
                DrivetrainSubsystem.getInstance()::followTrajectory,
                false,
                DrivetrainSubsystem.getInstance());
    }

    public Command getBumpCommand() {
        if (frc.robot.auto.common.AutoFactory.getInstance().getBumpNeeded()) {
            return new DefaultDriveCommand(() -> 0.6, () -> 0.0, () -> 0.0, () -> true)
                    .withDeadline(new WaitCommand(1.5));
        } else {
            return new InstantCommand();
        }
    }

    public Command delaySelectedTime() {
        return new WaitCommand(frc.robot.auto.common.AutoFactory.getInstance().getSavedWaitSeconds());
    }

    // ---------------------- BASE AUTOS -------------------- //

    public Command test() { // validate chooser and if choreo goes to start pose first!
        AutoRoutine TESTPATH = autoFactory.newRoutine("TEST PATH");

        AutoTrajectory traj1 = TESTPATH.trajectory("TEST TRAJECTORY 1");
        AutoTrajectory traj2 = TESTPATH.trajectory("TEST TRAJECTORY 2");

        TESTPATH.active()
                .onTrue(Commands.sequence(
                        traj1.resetOdometry(),
                        traj1.cmd().andThen(new PrintCommand("DONE TRAJ 1")).andThen(traj2.cmd())));

        return TESTPATH.cmd();
    }

    // ---------- AUTO 1: J4K4L4 -------- //
    public Command J4K4L4() {
        AutoRoutine J4K4L4 = autoFactory.newRoutine("J4K4L4");

        // load trajectories
        AutoTrajectory startPath = J4K4L4.trajectory(AutoBase.PathsBase.R_SL_J2.getTrajName());
        AutoTrajectory load1 = J4K4L4.trajectory(AutoBase.PathsBase.R_J2_LL.getTrajName());
        AutoTrajectory score2 = J4K4L4.trajectory(AutoBase.PathsBase.R_LL_K4.getTrajName());
        AutoTrajectory load2 = J4K4L4.trajectory(AutoBase.PathsBase.R_K4_LL.getTrajName());
        AutoTrajectory score3 = J4K4L4.trajectory(AutoBase.PathsBase.R_LL_L.getTrajName());

        // retry trajectories
        AutoTrajectory BLUE_LL_RETRY = J4K4L4.trajectory(AutoBase.PathsBase.BLUE_LL_RETRY.getTrajName());
        AutoTrajectory RED_LL_RETRY = J4K4L4.trajectory(AutoBase.PathsBase.RED_LL_RETRY.getTrajName());

        // if @ the end of the path we don't have a coral, it could just be stuck in
        // checks to make sure we don't snap the intake

        J4K4L4.active()
                .onTrue(Commands.sequence(
                        startPath.resetOdometry(),
                        Commands.either(
                                new InstantCommand(() -> RobotState.getInstance()
                                        .setAutoStartPose(
                                                startPath.getInitialPose().get())),
                                new InstantCommand(),
                                () -> startPath.getInitialPose().isPresent()),
                        getBumpCommand(),
                        delaySelectedTime(),

                        // score preload
                        reefAlignment(startPath, AlignOffset.RIGHT_BRANCH, FieldElementFace.IJ)
                                .alongWith(prepareForScore(TargetAction.L4))
                                .andThen(new PrintCommand("AUTO ALIGNMENT DONE")
                                        .andThen(ArmCommandFactory.coralIn().withTimeout(0.05))
                                        .andThen(score(TargetAction.L4)))

                                // pickup 2nd coral
                                .andThen(loadWithPath(load1, RED_LL_RETRY, true))

                                // score 2nd coral
                                .andThen(reefAlignment(score2, AlignOffset.LEFT_BRANCH, FieldElementFace.KL)
                                        .alongWith(prepareForScore(TargetAction.L4))
                                        .andThen(new PrintCommand("AUTO ALIGNMENT DONE")
                                                .andThen(ArmCommandFactory.coralIn()
                                                        .withTimeout(0.05))
                                                .andThen(score(TargetAction.L4))))
                                .andThen(RobotState.getInstance().setAlignOffsetCommand(AlignOffset.MIDDLE_REEF))

                                // pickup 3rd coral
                                .andThen(loadWithPath(load2, RED_LL_RETRY, true))

                                // score 3rd coral
                                .andThen(reefAlignment(score3, AlignOffset.RIGHT_BRANCH, FieldElementFace.KL)
                                        .alongWith(prepareForScore(TargetAction.L4))
                                        .andThen(new PrintCommand("AUTO ALIGNMENT DONE")
                                                .andThen(ArmCommandFactory.coralIn()
                                                        .withTimeout(0.05))
                                                .andThen(score(TargetAction.L4))))));

        return J4K4L4.cmd();
    }

    // ---------- AUTO 2: E4D4C4 -------- //
    public Command E4D4C4() {
        AutoRoutine E4D4C4 = autoFactory.newRoutine("E4D4C4");

        // load trajectories
        AutoTrajectory startPath = E4D4C4.trajectory(AutoBase.PathsBase.B_SR_E.getTrajName());
        AutoTrajectory load1 = E4D4C4.trajectory(AutoBase.PathsBase.B_E_RL.getTrajName());
        AutoTrajectory score2 = E4D4C4.trajectory(AutoBase.PathsBase.B_RL_D.getTrajName());
        AutoTrajectory load2 = E4D4C4.trajectory(AutoBase.PathsBase.B_D_RL.getTrajName());
        AutoTrajectory score3 = E4D4C4.trajectory(AutoBase.PathsBase.B_RL_C.getTrajName());

        AutoTrajectory BLUE_RL_RETRY = E4D4C4.trajectory(AutoBase.PathsBase.BLUE_RL_RETRY.getTrajName());
        AutoTrajectory RED_RL_RETRY = E4D4C4.trajectory(AutoBase.PathsBase.RED_RL_RETRY.getTrajName());

        E4D4C4.active()
                .onTrue(Commands.sequence(
                        startPath.resetOdometry(),
                        getBumpCommand(),
                        delaySelectedTime(),

                        // score preload
                        reefAlignment(startPath, AlignOffset.RIGHT_BRANCH, FieldElementFace.IJ)
                                .alongWith(prepareForScore(TargetAction.L4))
                                .andThen(new PrintCommand("AUTO ALIGNMENT DONE")
                                        .andThen(ArmCommandFactory.coralIn().withTimeout(0.05))
                                        .andThen(score(TargetAction.L4)))

                                // pickup 2nd coral
                                .andThen(loadWithPath(load1, BLUE_RL_RETRY, true))

                                // score 2nd coral
                                .andThen(reefAlignment(score2, AlignOffset.LEFT_BRANCH, FieldElementFace.KL)
                                        .alongWith(prepareForScore(TargetAction.L4))
                                        .andThen(new PrintCommand("AUTO ALIGNMENT DONE")
                                                .andThen(ArmCommandFactory.coralIn()
                                                        .withTimeout(0.05))
                                                .andThen(score(TargetAction.L4))))

                                // pickup 3rd coral
                                .andThen(loadWithPath(load2, BLUE_RL_RETRY, true))

                                // score 3rd coral
                                .andThen(reefAlignment(score3, AlignOffset.RIGHT_BRANCH, FieldElementFace.KL)
                                        .alongWith(prepareForScore(TargetAction.L4))
                                        .andThen(new PrintCommand("AUTO ALIGNMENT DONE")
                                                .andThen(ArmCommandFactory.coralIn()
                                                        .withTimeout(0.05))
                                                .andThen(score(TargetAction.L4))))));

        return E4D4C4.cmd();
    }

    // ---------- AUTO 3: CENTER BACKUP L1 -------- //
    public Command CENTERL1() {
        AutoRoutine CENTERL1 = autoFactory.newRoutine("IJ1KL1");

        // load trajectories
        AutoTrajectory startPath = CENTERL1.trajectory(AutoBase.PathsBase.B_SC_GH_L1.getTrajName());

        CENTERL1.active()
                .onTrue(Commands.sequence(
                        startPath.resetOdometry(),
                        startPath.cmd().andThen(ArmCommandFactory.coralOut().withTimeout(1.0))));

        return CENTERL1.cmd();
    }

    // ---------------------- ALGAE AUTOS -------------------- //
    public Command G4_CLEAN_LEFT_ALGAE() {
        AutoRoutine G4_DESCORE_SCORE = autoFactory.newRoutine("G4_CLEAN_LEFT_ALGAE");

        AutoTrajectory startPath = G4_DESCORE_SCORE.trajectory(AutoBase.PathsBase.B_SC_G.getTrajName());
        AutoTrajectory repositionDescore =
                G4_DESCORE_SCORE.trajectory(AutoBase.PathsBase.B_GH_REPOSITION.getTrajName());
        AutoTrajectory score1 = G4_DESCORE_SCORE.trajectory(AutoBase.PathsBase.B_GH_NET.getTrajName());
        AutoTrajectory descore2 = G4_DESCORE_SCORE.trajectory(AutoBase.PathsBase.B_NET_IJ.getTrajName());
        AutoTrajectory score2 = G4_DESCORE_SCORE.trajectory(AutoBase.PathsBase.B_IJ_NET.getTrajName());
        AutoTrajectory descore3 = G4_DESCORE_SCORE.trajectory(AutoBase.PathsBase.B_NET_KL.getTrajName());
        AutoTrajectory score3 = G4_DESCORE_SCORE.trajectory(AutoBase.PathsBase.B_KL_NET.getTrajName());

        // start intake + reposition after safe zone
        repositionDescore
                .atTime("intake")
                .onTrue(new InstantCommand(
                                () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.LA))
                        .andThen(ArmCommandFactory.algaeIn()));

        // set up to position / stop intake for score paths
        score1.atTime("raise")
                .onTrue(Commands.sequence(
                        new InstantCommand(
                                () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.AS)),
                        ArmCommandFactory.algaeIn().withTimeout(0.01)));
        score2.atTime("raise")
                .onTrue(Commands.sequence(
                        new InstantCommand(
                                () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.AS)),
                        ArmCommandFactory.algaeIn().withTimeout(0.01)));
        score3.atTime("raise")
                .onTrue(Commands.sequence(
                        new InstantCommand(
                                () -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.AS)),
                        ArmCommandFactory.algaeIn().withTimeout(0.01)));

        G4_DESCORE_SCORE
                .active()
                .onTrue(Commands.sequence(
                        getBumpCommand(),
                        delaySelectedTime(),

                        // score L4
                        reefAlignment(startPath, AlignOffset.LEFT_BRANCH, FieldElementFace.GH)
                                .alongWith(prepareForScore(TargetAction.L4))
                                .andThen(new PrintCommand("AUTO ALIGNMENT DONE")
                                        .andThen(ArmCommandFactory.coralIn().withTimeout(0.05))
                                        .andThen(score(TargetAction.L4))),

                        // use event marker to intake
                        repositionDescore.cmd(),

                        // score first algae
                        scoreNet(score3),

                        // descore 2nd
                        descoreAlgae(descore2, TargetAction.UA),

                        // score 2nd
                        scoreNet(score3),

                        // descore 3rd
                        descoreAlgae(descore3, TargetAction.LA),

                        // score 3rd
                        scoreNet(score3)));

        return G4_DESCORE_SCORE.cmd();
    }

    // ---------------------- HELPER METHODS -------------------- //

    public Command reefAlignment(AutoTrajectory startPath, AlignOffset offset, FieldElementFace fieldLoc) {
        double distance;

        if (startPath.toString().equals(PathsBase.B_SL_J.getTrajName())
                || startPath.toString().equals(PathsBase.B_SR_E.getTrajName())
                || startPath.toString().equals(PathsBase.B_SR_F.getTrajName())) { // start paths start align closer
            distance = 2.0;
        } else {
            distance = 2.5;
        }

        return new ParallelCommandGroup(
                ArmCommandFactory.coralIn()
                        .withTimeout(2.0)
                        .onlyWhile(() -> !RobotState.getInstance().getHasCoral())
                        .alongWith(new InstantCommand(
                                () -> RobotState.getInstance().setDesiredReefFace(fieldLoc))),
                startPath
                        .cmd()
                        .until(() -> RobotState.getInstance().shouldAlignAutonomous(distance))
                        .andThen(AlignmentCommandFactory.getSpecificReefAlignmentCommand(() -> offset, fieldLoc)));
    }

    public Command loadWithPath(AutoTrajectory path, AutoTrajectory retry, boolean redo) {
        BooleanSupplier hasCoralSupplier = () -> RobotState.getInstance().getHasCoral();
        return new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.INTAKE))
                .andThen(ArmCommandFactory.coralIn())
                .andThen(new ParallelRaceGroup(
                        path.cmd().andThen(new WaitCommand(1.0)),
                        Commands.waitUntil(() -> hasCoralSupplier.getAsBoolean())))
                // first fail: try again from human player
                .andThen(hasCoralSupplier.getAsBoolean() && redo ? new InstantCommand() : retry.cmd())
                // 2nd fail: try to get coral from nearest lolipop
                .andThen(hasCoralSupplier.getAsBoolean() && redo ? new InstantCommand() : retry.cmd());
    }

    protected Command descoreAlgae(AutoTrajectory toPosition, TargetAction algaeLevel) {
        return Commands.sequence(
                new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(algaeLevel)),
                ArmCommandFactory.algaeIn(),
                toPosition.cmd(),
                new WaitCommand(0.5) // pause to grab
                );
    }

    protected Command scoreNet(AutoTrajectory score) {
        return Commands.sequence(
                score.cmd(),
                Commands.waitUntil(() -> SuperstructureSubsystem.getInstance().isAtTargetState())
                        .andThen(ArmCommandFactory.algaeIn())
                        .withTimeout(0.2)
                        .andThen(ArmCommandFactory.algaeOut())
                        .withTimeout(1.0)
                        .andThen(() -> SuperstructureSubsystem.getInstance().setCurrentAction(TargetAction.STOW)));
    }

    public Command prepareForScore(TargetAction action) {
        final double maxSpeed;
        final double maxDist;
        TargetAction prepAction = TargetAction.TR;

        if (action == null) {
            maxSpeed = 0;
            maxDist = 0.1;
        } else {
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
        }

        BooleanSupplier speedOkay =
                () -> (MathHelpers.chassisSpeedsNorm(RobotState.getInstance().getChassisSpeeds()) < maxSpeed);
        BooleanSupplier distanceCheck = () -> RobotState.getInstance().distanceToAlignPose() < maxDist;

        if (true) {}
        return elevatorToPos(prepAction)
                .beforeStarting(Commands.waitUntil(distanceCheck))
                .andThen(new PrintCommand("CLOSE ENOUGH ***************"));
    }

    public Command elevatorToPos(TargetAction pos) {
        return new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(pos));
    }

    public Command score(TargetAction pos) {
        return new InstantCommand(() -> SuperstructureSubsystem.getInstance().setCurrentAction(pos))
                .andThen(Commands.waitUntil(
                                () -> (ElevatorSubsystem.getInstance().atPosition(2.0, pos)
                                        && ArmPivotSubsystem.getInstance().isAtDesiredPosition(4.0)))
                        .andThen(ArmCommandFactory.coralOut().withTimeout(0.3)));
    }
}
