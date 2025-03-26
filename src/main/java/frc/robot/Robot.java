// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.common.AutoFactory;
import frc.robot.auto.common.Autos;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    // replace Auto Factory
    private Command currentChoreoAuto;
    private Command compiledChoreoAuto;

    private final AutoChooser autoChooser;

    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
        if (isReal()) {
            // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            setUseTiming(false); // Run as fast as possible
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        }

        Logger.start(); // Start logging

        autoChooser = new AutoChooser();
        autoChooser.addCmd("NO AUTO", this::emptyCommandSupplier);
        autoChooser.addCmd("J4K4L4", this::getJ4K4L4);
        autoChooser.addCmd("E4D4C4", this::getE4D4C4);
        autoChooser.addCmd("CENTER 1", this::center1);
        autoChooser.addCmd("Test", this::test);

        SmartDashboard.putData("CHOREO AUTO CHOOSER V1", autoChooser);
        Pose2d loadPose = FieldConstants.blueLeftBranchL1.get(0);
        if (loadPose != null) {
            System.out.println("Loaded Field Constants");
        }
    }

    public void recompile() {
        // Wipe out all the precompiled autos so they must be recreated to add new bump and wait
        j4k4l4 = null;
        e4d4c4 = null;
        center1 = null;
        test = null;
        AutoFactory.getInstance().setChoreoAutoCompiled(false);
        currentChoreoAuto = autoChooser.selectedCommand();
        compiledChoreoAuto = currentChoreoAuto;
        if (compiledChoreoAuto == null) {
            compiledChoreoAuto = random();
        }
        AutoFactory.getInstance().setChoreoAutoCompiled(true);
    }

    // make auto Commands

    public Command random() {
        return new SequentialCommandGroup();
    }

    private Command j4k4l4 = null;

    public Command getJ4K4L4() {
        if (j4k4l4 == null) {
            j4k4l4 = Autos.getInstance().J4K4L4();
        }

        return j4k4l4;
    }

    private Command e4d4c4 = null;

    public Command getE4D4C4() {
        if (e4d4c4 == null) {
            e4d4c4 = Autos.getInstance().E4D4C4();
        }
        return e4d4c4;
    }

    private Command center1 = null;

    public Command center1() {
        if (center1 == null) {
            center1 = Autos.getInstance().CENTERL1();
        }
        return center1;
    }

    private Command test = null;

    public Command test() {
        if (test == null) {
            test = Autos.getInstance().test();
        }
        return center1;
    }

    public Command emptyCommandSupplier() {
        return new InstantCommand();
    }

    @Override
    public void robotPeriodic() {
        RobotState.getInstance().output();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    public void forceRecompile() {
        AutoFactory.getInstance().recompile();
    }

    public void precompileAuto() {
        if (autoChooser.selectedCommand() != null
                && (AutoFactory.getInstance().recompileNeeded()
                        || (!autoChooser.selectedCommand().toString().equals(compiledChoreoAuto.toString())))) {
            AutoFactory.getInstance().recompile();
            recompile();
        }
    }

    @Override
    public void disabledPeriodic() {
        // if (compiledChoreoAuto != null) {
        //     System.out.println(compiledChoreoAuto.getName());
        // }
        m_robotContainer.precompileAuto();
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // m_autonomousCommand = autoChooser.selectedCommand();
        // m_autonomousCommand = compiledChoreoAuto;

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
