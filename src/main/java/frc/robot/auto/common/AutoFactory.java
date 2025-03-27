// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto.common;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DashboardConstants;
import frc.robot.RobotState;
import frc.robot.auto.modes.StartLeft.AutoJ1K1L1;
import frc.robot.auto.modes.StartLeft.AutoJ1K4L4;
import frc.robot.auto.modes.StartLeft.AutoJ4K4L4;
import frc.robot.auto.modes.StartLeft.AutoK4L4DAK3L3;
import frc.robot.auto.modes.StartRight.AutoE4D4C4;
import frc.robot.auto.modes.choreoRemake.V2J4K4L4;
import frc.robot.auto.modes.choreoRemake.leftSide.BlueJ4K4L4;
import frc.robot.auto.modes.choreoRemake.leftSide.RedJ4K4L4;
import frc.robot.auto.modes.choreoRemake.rightSide.BlueE4D4C4;
import frc.robot.auto.modes.choreoRemake.rightSide.BlueF4D4C4;
import frc.robot.auto.modes.choreoRemake.rightSide.RedE4D4C4;
import frc.robot.auto.modes.choreoRemake.rightSide.RedF4D4C4;
import frc.robot.auto.modes.safety.AutoG4AlgaePrep;
import frc.robot.auto.modes.safety.DeadReckoning;
import frc.robot.auto.modes.startCenter.AutoG4LeftAlgaeRemoval;
import frc.robot.auto.modes.startCenter.AutoH4RightAlgaeRemoval;
import frc.robot.util.io.Dashboard;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class AutoFactory {
    private final Supplier<Auto> autoSupplier = () -> Dashboard.getInstance().getAuto();
    // private final Supplier<ChoreoAuto> choreoAutoSupplier =
    //         () -> Dashboard.getInstance().getChoreoAuto();
    private final Supplier<Double> waitSecondsEntrySupplier =
            () -> Dashboard.getInstance().getWaitSeconds();
    private final Supplier<Boolean> bumpNeededSupplier =
            () -> Dashboard.getInstance().getBumpNeeded();

    private Auto currentAuto;
    private AutoBase compiledAuto;

    // private ChoreoAuto currentChoreoAuto;
    private Command compiledChoreoAuto;

    private double selectedWaitSeconds;
    private double savedWaitSeconds;

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
                || isRedAlliance == !RobotState.getInstance().isRedAlliance();
    }

    public boolean choreoRecompileNeeded() {
        return
        // choreoAutoSupplier.get() != currentChoreoAuto||
        waitSecondsEntrySupplier.get() != savedWaitSeconds
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

        // update choreo Auto
        // choreoAutoCompiled.set(false);
        // currentChoreoAuto = choreoAutoSupplier.get();
        // if (currentChoreoAuto == null) {
        //     currentChoreoAuto = ChoreoAuto.NO_AUTO;
        // }
        // compiledChoreoAuto = currentChoreoAuto.getInstance();
        // choreoAutoCompiled.set(true);
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
        return bumpNeededSupplier.get();
    }

    public Command getJ4K4L4() {
        return Autos.getInstance().J4K4L4();
    }

    // public static enum ChoreoAuto {
    //     // NO_AUTO(null),
    //     // TEST_AUTO(Autos.getInstance().testPath()),
    //     // DEAD_RECKONING(new DefaultDriveCommand(() -> 0.5, () -> 0.0, () -> 0.0, () -> false).withTimeout(2.0)),
    //     J4_K4_L4(this::getJ4K4L4);
    //     // E4_D4_C4(Autos.getInstance().E4D4C4()),
    //     // CENTER_L1(Autos.getInstance().CENTERL1());
    //     // G4_CLEAN_LEFT_ALGAE(Autos.getInstance().G4_CLEAN_LEFT_ALGAE());

    //     private final Command autoCommand;

    //     private ChoreoAuto(Command autoCommand) {
    //         this.autoCommand = autoCommand;
    //     }

    //     public Command getInstance() {
    //         if (autoCommand != null) {
    //             try {
    //                 return autoCommand;
    //             } catch (Exception e) {
    //                 e.printStackTrace();
    //             }
    //         }

    //         return null;
    //     }
    // }

    public static enum Auto {
        V2_J4K4L4(V2J4K4L4.class),
        DEAD_RECKONING(DeadReckoning.class),
        NO_AUTO(null),
        // LL_K4_VISION_TEST(AutoLLToK4.class),
        // BACKUP_AUTO_H4_DA_NET(AutoG4AlgaePrep.class),

        // choreo
        C_Blue_J4K4L4(BlueJ4K4L4.class),
        C_Blue_E4D4C4(BlueE4D4C4.class),
        C_Blue_F4D4C4(BlueF4D4C4.class),
        C_Red_J4KL4(RedJ4K4L4.class),
        C_RED_E4D4C4(RedE4D4C4.class),
        C_RED_F4D4C4(RedF4D4C4.class),

        // start center
        AUTO_G4_ALGAE_PREP(AutoG4AlgaePrep.class),
        AUTO_H4_LEFT_ALGAE_REMOVAL(AutoG4LeftAlgaeRemoval.class),
        AUTO_H4_RIGHT_ALGAE_REMOVAL(AutoH4RightAlgaeRemoval.class),

        // start left
        LEFT_J1_K1_L1(AutoJ1K1L1.class),
        // LEFT_J2_K4_L4(AutoJ2K4L4.class),
        LEFT_J4_K4_L4(AutoJ4K4L4.class),
        LEFT_K4_L4_DA_K3_L3(AutoK4L4DAK3L3.class),
        LEFT_J1_K4_L4(AutoJ1K4L4.class),

        // start right
        // RIGHT_D4_C4_DA_D3_C3(AutoD4C4DAD3C3.class),
        // RIGHT_E1_D1_C1(AutoE1D1C1.class),
        RIGHT_E4_D4_C4(AutoE4D4C4.class);
        // RIGHT_E2_D4_C4(AutoE2D4C4.class),
        // RIGHT_F4_D4_C4(AutoF4D4C4.class),
        // RIGHT_E1_D4_C4(AutoE1D4C4.class);

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
