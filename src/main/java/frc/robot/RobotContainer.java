// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.auto.common.AutoFactory;
import frc.robot.commands.arm.ArmRollerCommandFactory;
import frc.robot.commands.climber.ClimberCommandFactory;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.drive.auto.AutoSnapToLocationAngleCommand;
import frc.robot.commands.intake.IntakeRollerCommandFactory;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.ActionType;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
import frc.robot.util.AlignmentCalculator.FieldElementFace;
import frc.robot.util.Telemetry;
import frc.robot.util.io.Dashboard;

public class RobotContainer {
    private final ControlBoard controlBoard = ControlBoard.getInstance();
    public final Dashboard dashboard = Dashboard.getInstance();

    public final RobotState robotState = RobotState.getInstance();
    public final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.getInstance();
    public final VisionSubsystem vision = VisionSubsystem.getInstance();
    public final AdvantageScopeSubsystem advantageScope = AdvantageScopeSubsystem.getInstance();
    public final AutoFactory autoFactory = AutoFactory.getInstance();
    public final SuperstructureSubsystem superstructure = SuperstructureSubsystem.getInstance();
    public final IntakePivotSubsystem intakePivot = IntakePivotSubsystem.getInstance();
    public final IntakeRollerSubsystem intakeRollers = IntakeRollerSubsystem.getInstance();
    public final LedSubsystem leds = LedSubsystem.getInstance();

    public static boolean deadReckoning = false;

    private final Telemetry logger = new Telemetry(DrivetrainConstants.DRIVE_MAX_SPEED.in(MetersPerSecond));

    public RobotContainer() {
        drivetrain.setDefaultCommand(new DefaultDriveCommand(
                controlBoard::getThrottle,
                // Sideways velocity supplier.
                controlBoard::getStrafe,
                // Rotation velocity supplier.
                controlBoard::getRotation,
                dashboard::isFieldCentric));

        configureNamedCommands();

        configureBindings();
    }

    private void configureNamedCommands() {
        // snap to angle in gui
        NamedCommands.registerCommand(
                "Snap Left Coral Station", new AutoSnapToLocationAngleCommand(FieldElementFace.LCS));
        NamedCommands.registerCommand(
                "Snap Right Coral Station", new AutoSnapToLocationAngleCommand(FieldElementFace.RCS));

        NamedCommands.registerCommand("Snap to AB", new AutoSnapToLocationAngleCommand(FieldElementFace.AB));
        NamedCommands.registerCommand("Snap to CD", new AutoSnapToLocationAngleCommand(FieldElementFace.CD));
        NamedCommands.registerCommand("Snap to EF", new AutoSnapToLocationAngleCommand(FieldElementFace.EF));
        NamedCommands.registerCommand("Snap to GH", new AutoSnapToLocationAngleCommand(FieldElementFace.GH));
        NamedCommands.registerCommand("Snap to IJ", new AutoSnapToLocationAngleCommand(FieldElementFace.IJ));
        NamedCommands.registerCommand("Snap to KL", new AutoSnapToLocationAngleCommand(FieldElementFace.KL));

        // // score in gui
        // NamedCommands.registerCommand("Score L1", ToLevel.L1.getCommand());
        // NamedCommands.registerCommand("Score L2", ToLevel.L2.getCommand());
        // NamedCommands.registerCommand("Score L3", ToLevel.L3.getCommand());
        // NamedCommands.registerCommand("Score L4", ToLevel.L4.getCommand());
        // NamedCommands.registerCommand("Score Processor", null); // TODO: auto processor score
        // NamedCommands.registerCommand("DeScore Algae", null); // TODO: auto descore algae
        // NamedCommands.registerCommand("Coral Station Intake", null); // TODO: auto coral station
        // intake
    }

    private void configureBindings() {
        /* Primary Driver */
        configurePOVBindings();

        drivetrain.registerTelemetry(logger::telemeterize);

        controlBoard.resetGyro().onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));

        controlBoard
                .intake()
                .onTrue(new ConditionalCommand(
                        new InstantCommand(),
                        superstructure.set(TargetAction.INTAKE, true),
                        () -> superstructure.getCurrentAction().getType() == ActionType.ALGAE))
                .whileTrue(ArmRollerCommandFactory.intake())
                .whileTrue(IntakeRollerCommandFactory.intake());
        controlBoard
                .outtake()
                .whileTrue(ArmRollerCommandFactory.outtake())
                .whileTrue(IntakeRollerCommandFactory.outtake())
                .onFalse(superstructure.set(TargetAction.STOW, true));

        controlBoard.rollerTap().whileTrue(ArmRollerCommandFactory.coralIn());
        // controlBoard.shootAlgae().whileTrue(ArmRollerCommandFactory.algaeOut());

        controlBoard
                .alignWithReefLeft()
                .whileTrue(AlignmentCommandFactory.getReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH));

        controlBoard
                .alignWithReefRight()
                .whileTrue(AlignmentCommandFactory.getReefAlignmentCommand(() -> AlignOffset.RIGHT_BRANCH));

        /* Secondary Driver */
        controlBoard.actTrigger().onTrue(superstructure.confirm());

        controlBoard.setGoalCL().onTrue(superstructure.set(TargetAction.INTAKE, true));
        controlBoard
                .setGoalL1H()
                .onTrue(robotState.setAlignOffsetCommand(AlignOffset.MIDDLE_REEF))
                .onTrue(superstructure.set(TargetAction.L1H, false));
        controlBoard.setGoalL2().onTrue(superstructure.set(TargetAction.L2, false));
        controlBoard.setGoalL3().onTrue(superstructure.set(TargetAction.L3, false));
        controlBoard.setGoalL4().onTrue(superstructure.set(TargetAction.L4, false));
        controlBoard
                .setGoalLowerAlgae()
                .onTrue(robotState.setAlignOffsetCommand(AlignOffset.MIDDLE_REEF))
                .onTrue(superstructure.set(TargetAction.LA, false));
        controlBoard
                .setGoalUpperAlgae()
                .onTrue(robotState.setAlignOffsetCommand(AlignOffset.MIDDLE_REEF))
                .onTrue(superstructure.set(TargetAction.UA, false));
        controlBoard.setGoalCoralStation().onTrue(superstructure.set(TargetAction.STOW, false));
        controlBoard.homeElevator().onTrue(superstructure.set(TargetAction.HM, false));

        controlBoard.setSubReefLeft().onTrue(robotState.setAlignOffsetCommand(AlignOffset.LEFT_BRANCH));
        controlBoard.setSubReefRight().onTrue(robotState.setAlignOffsetCommand(AlignOffset.RIGHT_BRANCH));

        controlBoard.climbUp().whileTrue(ClimberCommandFactory.climberUp());
        controlBoard.climbDown().whileTrue(ClimberCommandFactory.climberDown());

        controlBoard.algaeScoreAngle().onTrue(superstructure.set(TargetAction.AS, false));
        controlBoard.algaeLowAngle().onTrue(superstructure.set(TargetAction.AP, false));

        // controlBoard.sysIDQuasiForward().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // controlBoard.sysIDQuasiReverse().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // controlBoard.sysIDDynamicForward().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // controlBoard.sysIDDynamicReverse().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // controlBoard
        //     .shoot()
        //     .onTrue(Commands.runOnce(SignalLogger::start))
        //     .onFalse(Commands.runOnce(SignalLogger::stop));

        // controlBoard
        //     .manualUp()
        //     .onTrue(ElevatorSubsystem.getInstance().manualUp())
        //     .onFalse(ElevatorSubsystem.getInstance().stopElevator());

        // controlBoard
        //     .manualDown()
        //     .onTrue(ElevatorSubsystem.getInstance().manualDown())
        //     .onFalse(ElevatorSubsystem.getInstance().stopElevator());
    }

    private void configurePOVBindings() {
        ControlBoard controlBoard = ControlBoard.getInstance();

        controlBoard.povUp().whileTrue(new DefaultDriveCommand(() -> 0.2, () -> 0.0, () -> 0.0, () -> false));
        controlBoard.povUpRight().whileTrue(new DefaultDriveCommand(() -> 0.2, () -> -0.2, () -> 0.0, () -> false));

        controlBoard.povRight().whileTrue(new DefaultDriveCommand(() -> 0.0, () -> -0.2, () -> 0.0, () -> false));

        controlBoard.povDownRight().whileTrue(new DefaultDriveCommand(() -> -0.2, () -> -0.2, () -> 0.0, () -> false));

        controlBoard.povDown().whileTrue(new DefaultDriveCommand(() -> -0.2, () -> 0.0, () -> 0.0, () -> false));

        controlBoard.povDownLeft().whileTrue(new DefaultDriveCommand(() -> -0.2, () -> 0.2, () -> 0.0, () -> false));

        controlBoard.povLeft().whileTrue(new DefaultDriveCommand(() -> 0.0, () -> 0.2, () -> 0.0, () -> false));

        controlBoard.povUpLeft().whileTrue(new DefaultDriveCommand(() -> 0.2, () -> 0.2, () -> 0.0, () -> false));

        System.out.println("POV Bindings Configured");
    }

    public void forceRecompile() {
        autoFactory.recompile();
    }

    public void precompileAuto() {
        if (autoFactory.recompileNeeded()) {
            autoFactory.recompile();
        }
    }

    public Command getAutonomousCommand() {
        return autoFactory.getCompiledAuto();
    }

    public static boolean getDeadReckoning() {
        return deadReckoning;
    }

    public static void setDeadReckoning(boolean isDeadReckoning) {
        deadReckoning = isDeadReckoning;
    }
}
