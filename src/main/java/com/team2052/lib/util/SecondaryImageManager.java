package com.team2052.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.*;
import java.nio.file.Paths;

public class SecondaryImageManager {
  public static void setCurrentImage(SecondaryImage image) {
    SmartDashboard.getEntry("Secondary Image").setString(image.getAbsolutePath());
  }

  public enum SecondaryImage {
    L1(Paths.get("src/main/java/com/team2052/lib/util/L1.png").toAbsolutePath().toString()),
    L2(Paths.get("src/main/java/com/team2052/lib/util/L2.png").toAbsolutePath().toString()),
    L3(Paths.get("src/main/java/com/team2052/lib/util/L3.png").toAbsolutePath().toString()),
    L4(Paths.get("src/main/java/com/team2052/lib/util/L4.png").toAbsolutePath().toString());

    private final String absolutePath;

    private SecondaryImage(String absolutePath) {
      this.absolutePath = absolutePath;
    }

    public String getAbsolutePath() {
      return absolutePath;
    }
  }
}
