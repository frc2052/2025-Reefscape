// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.auto.common.AutoFactory;
import frc.robot.commands.arm.ArmCommandFactory;
import frc.robot.commands.climber.ClimberCommandFactory;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.drive.alignment.AlignmentCommandFactory;
import frc.robot.commands.intake.IntakeCommandFactory;
import frc.robot.commands.superstructure.SuperstructureCommandFactory;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.AdvantageScopeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.arm.ArmPivotSubsystem;
import frc.robot.subsystems.arm.ArmRollerSubsystem;
import frc.robot.subsystems.drive.DrivetrainSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.superstructure.SuperstructurePosition.ActionType;
import frc.robot.subsystems.superstructure.SuperstructurePosition.TargetAction;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.AlignmentCalculator.AlignOffset;
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
    public final ArmPivotSubsystem armPivot = ArmPivotSubsystem.getInstance();
    public final ArmRollerSubsystem armRollers = ArmRollerSubsystem.getInstance();
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

        configureBindings();
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
                        new InstantCommand(() -> superstructure.setCurrentAction(TargetAction.INTAKE)),
                        () -> superstructure.getCurrentAction().getType() == ActionType.ALGAE))
                .whileTrue(ArmCommandFactory.intake())
                .whileTrue(new ConditionalCommand(
                        new InstantCommand(),
                        IntakeCommandFactory.intake(),
                        () -> superstructure.getCurrentAction().getType() == ActionType.ALGAE));
        controlBoard
                .outtake()
                .whileTrue(ArmCommandFactory.outtake())
                .onFalse(new InstantCommand(() -> superstructure.stow()));

        controlBoard.rollerTap().whileTrue(ArmCommandFactory.coralIn());

        controlBoard.groundOuttake().whileTrue(IntakeCommandFactory.outtake());

        controlBoard
                .alignWithReefLeft()
                .whileTrue(AlignmentCommandFactory.getReefAlignmentCommand(() -> AlignOffset.LEFT_BRANCH));

        controlBoard
                .alignWithReefRight()
                .whileTrue(AlignmentCommandFactory.getReefAlignmentCommand(() -> AlignOffset.RIGHT_BRANCH));

        /* Secondary Driver */
        controlBoard.actTrigger().onTrue(superstructure.confirm());

        controlBoard.setGoalCL().onTrue(superstructure.set(TargetAction.CL, true));
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

        controlBoard.climbUp().whileTrue(ClimberCommandFactory.climberUp());
        controlBoard.climbDown().whileTrue(ClimberCommandFactory.climberDown());

        controlBoard.algaeScoreAngle().onTrue(superstructure.set(TargetAction.AS, false));
        controlBoard.algaeLowAngle().onTrue(superstructure.set(TargetAction.AP, false));

        /* SysID */
        controlBoard.sysIDDynamicForward().onTrue(SuperstructureCommandFactory.setCoast());
        controlBoard.sysIDDynamicReverse().onTrue(SuperstructureCommandFactory.setBrake());
        // controlBoard.sysIDQuasiForward().whileTrue(intakePivot.sysIdQuasistatic(Direction.kForward));
        // controlBoard.sysIDQuasiReverse().whileTrue(intakePivot.sysIdQuasistatic(Direction.kReverse));
        // controlBoard.sysIDDynamicForward().whileTrue(intakePivot.sysIdDynamic(Direction.kForward));
        // controlBoard.sysIDDynamicReverse().whileTrue(intakePivot.sysIdDynamic(Direction.kReverse));
        // controlBoard
        //         .outtake()
        //         .onTrue(Commands.runOnce(SignalLogger::start))
        //         .onFalse(Commands.runOnce(SignalLogger::stop));
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
        if (autoFactory.recompileNeeded() || autoFactory.choreoRecompileNeeded()) {
            autoFactory.recompile();
        }
    }

    public Command getAutonomousCommand() {
        // return autoFactory.getCompiledChoreoAuto(); // test
        return autoFactory.getCompiledAuto();
    }

    public static boolean getDeadReckoning() {
        return deadReckoning;
    }

    public static void setDeadReckoning(boolean isDeadReckoning) {
        deadReckoning = isDeadReckoning;
    }
}
