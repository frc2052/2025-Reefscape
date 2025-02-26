// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto.common;

import frc.robot.Constants.DashboardConstants;
import frc.robot.RobotState;
import frc.robot.auto.modes.AutoLLToK4;
import frc.robot.auto.modes.StartLeft.AutoJ1K1L1;
import frc.robot.auto.modes.StartLeft.AutoJ2K4L4;
import frc.robot.auto.modes.StartLeft.AutoJ4K4L4;
import frc.robot.auto.modes.StartLeft.AutoK4L4DAK3L3;
import frc.robot.auto.modes.StartRight.AutoD4C4DAD3C3;
import frc.robot.auto.modes.StartRight.AutoE1D1C1;
import frc.robot.auto.modes.StartRight.AutoE2D4C4;
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
  private final Supplier<Double> waitSecondsEntrySupplier =
      () -> Dashboard.getInstance().getWaitSeconds();
  private final Supplier<Boolean> bumpNeededSupplier =
      () -> Dashboard.getInstance().getBumpNeeded();

  private Auto currentAuto;
  private AutoBase compiledAuto;

  private double selectedWaitSeconds;
  private double savedWaitSeconds;

  private boolean isRedAlliance = RobotState.getInstance().isRedAlliance();

  private static LoggedNetworkBoolean autoCompiled =
      new LoggedNetworkBoolean(DashboardConstants.AUTO_COMPILED_KEY, false);

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
      compiledAuto.init();
    }
    autoCompiled.set(true);
  }

  public AutoBase getCompiledAuto() {
    return compiledAuto;
  }

  public double getSavedWaitSeconds() {
    return savedWaitSeconds;
  }

  public boolean getBumpNeeded() {
    return bumpNeededSupplier.get();
  }

  public static enum Auto {
    DEAD_RECKONING(DeadReckoning.class),
    NO_AUTO(null),
    LL_K4_VISION_TEST(AutoLLToK4.class),
    BACKUP_AUTO_H4_DA_NET(AutoG4AlgaePrep.class),

    // start center
    AUTO_G4_ALGAE_PREP(AutoG4AlgaePrep.class),
    AUTO_H4_LEFT_ALGAE_REMOVAL(AutoG4LeftAlgaeRemoval.class),
    AUTO_H4_RIGHT_ALGAE_REMOVAL(AutoH4RightAlgaeRemoval.class),

    // start left
    AUTO_J1_K1_L1(AutoJ1K1L1.class),
    AUTO_J2_K4_L4(AutoJ2K4L4.class),
    AUTO_J4_K4_L4(AutoJ4K4L4.class),
    AUTO_K4_L4_DA_K3_L3(AutoK4L4DAK3L3.class),

    // start right
    AUTO_D4_C4_DA_D3_C3(AutoD4C4DAD3C3.class),
    AUTO_E1_D1_C1(AutoE1D1C1.class),
    AUTO_E2_D4_C4(AutoE2D4C4.class);

    private final Class<? extends AutoBase> autoClass;

    private Auto(Class<? extends AutoBase> autoClass) {
      this.autoClass = autoClass;
    }

    public AutoBase getInstance() {
      if (autoClass != null) {
        try {
          // TODO: does this fix the problem

          // AutoDescription autoDescription =
          // autoClass.getClass().getAnnotation(AutoDescription.class);
          // if (autoClass.isAnnotationPresent(AutoDescription.class)) {
          autoDescription.set(
              autoClass.isAnnotationPresent(AutoDescription.class)
                  ? autoClass.getAnnotation(AutoDescription.class).description()
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
