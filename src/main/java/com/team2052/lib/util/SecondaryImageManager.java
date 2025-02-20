package com.team2052.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SecondaryImageManager {
  public static void setCurrentImage(SecondaryImage image) {
    SmartDashboard.getEntry("Secondary Image").setString(image.getAbsolutePath());
  }

  public enum SecondaryImage {
    L1("C:\\Users\\Merlin\\Documents\\Github\\2025-Reefscape\\src\\main\\deploy\\images\\L1.png"),
    L2("C:\\Users\\Merlin\\Documents\\Github\\2025-Reefscape\\src\\main\\deploy\\images\\L2.png"),
    L3("C:\\Users\\Merlin\\Documents\\Github\\2025-Reefscape\\src\\main\\deploy\\images\\L3.png"),
    L4("C:\\Users\\Merlin\\Documents\\Github\\2025-Reefscape\\src\\main\\deploy\\images\\L4.png"),
    A1("C:\\Users\\Merlin\\Documents\\Github\\2025-Reefscape\\src\\main\\deploy\\images\\A1.png"),
    A2("C:\\Users\\Merlin\\Documents\\Github\\2025-Reefscape\\src\\main\\deploy\\images\\A2.png"),
    NONE(
        "C:\\Users\\Merlin\\Documents\\Github\\2025-Reefscape\\src\\main\\deploy\\images\\None.png");

    private final String absolutePath;

    private SecondaryImage(String absolutePath) {
      this.absolutePath = absolutePath;
    }

    public String getAbsolutePath() {
      return absolutePath;
    }
  }
}
