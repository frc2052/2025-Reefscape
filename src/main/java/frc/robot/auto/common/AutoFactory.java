// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.auto.common;

import frc.robot.Constants.DashboardConstants;
import frc.robot.util.io.Dashboard;
// select, compile, recompile autos before start of a match
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class AutoFactory {
  private final Supplier<Auto> autoSupplier = () -> Dashboard.getInstance().getAuto();
  private Auto currentAuto;
  private AutoBase compiledAuto;

  private static LoggedNetworkBoolean autoCompiled =
      new LoggedNetworkBoolean(DashboardConstants.AUTO_COMPILED_KEY, false);

  private static LoggedNetworkString autoDescription =
      new LoggedNetworkString(DashboardConstants.AUTO_DESCRIPTION_KEY, "No Description");

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
    return autoSupplier.get() != currentAuto;
  }

  public void recompile() {
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

  public static enum Auto {
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
          autoDescription.set(autoClass.getAnnotation(AutoDescription.class).description());
          // } else {
          //     loggedAutoDescription.set("No description");
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
