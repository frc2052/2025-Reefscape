// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto.common;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DashboardConstants;
import frc.robot.RobotState;
import frc.robot.auto.modes.BackupMiddleL1;
import frc.robot.auto.modes.DeadReckoning;
import frc.robot.auto.modes.H4AlgaeGHEFIJ;
import frc.robot.auto.modes.Left3CoralJKL;
import frc.robot.auto.modes.LeftLolipop;
import frc.robot.auto.modes.MiddleH4;
import frc.robot.auto.modes.Right3CoralEDC;
import frc.robot.auto.modes.RightLolipop;
import frc.robot.util.io.Dashboard;

public class AutoFactory {
    private final Supplier<Auto> autoSupplier = () -> Dashboard.getInstance().getAuto();
    private final Supplier<Double> waitSecondsEntrySupplier =
            () -> Dashboard.getInstance().getWaitSeconds();
    private final Supplier<Boolean> bumpNeededSupplier =
            () -> Dashboard.getInstance().getBumpNeeded();
    private final Supplier<Boolean> lollipopOrder =
            () -> Dashboard.getInstance().getLeftLollipopFirst();

    private Auto currentAuto;
    private AutoBase compiledAuto;

    private Command compiledChoreoAuto;

    private double selectedWaitSeconds;
    private double savedWaitSeconds;

    private boolean savedLollipopOrder;
    private boolean savedBumpNeeded;

    private boolean isRedAlliance = RobotState.getInstance().isRedAlliance();

    private static LoggedNetworkBoolean autoCompiled =
            new LoggedNetworkBoolean(DashboardConstants.AUTO_COMPILED_KEY, false);

    private static LoggedNetworkBoolean choreoAutoCompiled =
            new LoggedNetworkBoolean(DashboardConstants.CHOREO_AUTO_COMPILED, false);

    private static LoggedNetworkString autoDescription =
            new LoggedNetworkString(DashboardConstants.AUTO_DESCRIPTION_KEY, "No Description");

    private static LoggedNetworkBoolean waitSecondsSavedKey =
            new LoggedNetworkBoolean(DashboardConstants.WAIT_SECONDS_SAVED_KEY, false);

    private static LoggedNetworkString waitSecondsDisplay =
            new LoggedNetworkString(DashboardConstants.WAIT_SECONDS_DISPLAY_KEY, "DEFAULT - 0.0");

    private static AutoFactory INSTANCE;

    public static AutoFactory getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AutoFactory();
        }
        return INSTANCE;
    }

    private AutoFactory() {}
    ;

    public boolean recompileNeeded() {
        return autoSupplier.get() != currentAuto
                || waitSecondsEntrySupplier.get() != savedWaitSeconds
                || isRedAlliance == !RobotState.getInstance().isRedAlliance()
                || savedBumpNeeded != bumpNeededSupplier.get()
                || savedLollipopOrder != lollipopOrder.get();
    }

    public boolean choreoRecompileNeeded() {
        return waitSecondsEntrySupplier.get() != savedWaitSeconds
                || isRedAlliance == !RobotState.getInstance().isRedAlliance();
    }

    public void setChoreoAutoCompiled(boolean compiled) {
        choreoAutoCompiled.set(compiled);
    }

    public void recompile() {
        isRedAlliance = RobotState.getInstance().isRedAlliance();
        // update wait seconds
        waitSecondsSavedKey.set(false);
        selectedWaitSeconds = waitSecondsEntrySupplier.get().doubleValue();
        savedWaitSeconds = selectedWaitSeconds;
        waitSecondsDisplay.set("Chosen Wait Seconds: " + savedWaitSeconds);
        waitSecondsSavedKey.set(true);

        // update auto
        autoCompiled.set(false);
        currentAuto = autoSupplier.get();
        if (currentAuto == null) {
            currentAuto = Auto.NO_AUTO;
        }
        compiledAuto = currentAuto.getInstance();
        if (compiledAuto == null) {
            autoDescription.set("No Auto Selected");
        } else {
            compiledAuto.init(); // starts?
        }
        autoCompiled.set(true);

        // update bump needed
        savedBumpNeeded = bumpNeededSupplier.get();

        // update lollipop auto order
        savedLollipopOrder = lollipopOrder.get();

        System.out.println("BUMP NEEDED?: " + savedBumpNeeded);
    }

    public AutoBase getCompiledAuto() {
        return compiledAuto;
    }

    public Command getCompiledChoreoAuto() {
        return compiledChoreoAuto;
    }

    public double getSavedWaitSeconds() {
        return savedWaitSeconds;
    }

    public boolean getBumpNeeded() {
        return savedBumpNeeded;
    }

    public boolean getLeftLollipopFirst() {
        return savedLollipopOrder;
    }

    public Command getJ4K4L4() {
        return new InstantCommand();
    }

    public static enum Auto {
        LEFT_3_CORAL_JKL(Left3CoralJKL.class),
        RIGHT_3_CORAL_EDC(Right3CoralEDC.class),

        LEFT_LOLIPOP(LeftLolipop.class),
        RIGHT_LOLIPOP(RightLolipop.class),
        MIDDLE_L4(MiddleH4.class),
        BACKUP_MIDDLE_L1(BackupMiddleL1.class),
        DRIVE_FORWARD(DeadReckoning.class),
        MIDDLE_ALGAE_GH_EF_IJ(H4AlgaeGHEFIJ.class),
        NO_AUTO(null);

        private final Class<? extends AutoBase> autoClass;

        private Auto(Class<? extends AutoBase> autoClass) {
            this.autoClass = autoClass;
        }

        public AutoBase getInstance() {
            if (autoClass != null) {
                try {

                    // AutoDescription autoDescription =
                    // autoClass.getClass().getAnnotation(AutoDescription.class);
                    // if (autoClass.isAnnotationPresent(AutoDescription.class)) {
                    autoDescription.set(
                            autoClass.isAnnotationPresent(AutoDescription.class)
                                    ? autoClass
                                            .getAnnotation(AutoDescription.class)
                                            .description()
                                    : "No Description");
                    // } else {
                    //     autoDescription.set("No description");
                    // }

                    return autoClass.getConstructor().newInstance();

                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            return null;
        }
    }
}
