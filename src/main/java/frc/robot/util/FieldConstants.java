package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.RobotState;
import java.io.IOException;
import java.nio.file.Path;

public class FieldConstants {
  public static final FieldType FIELD_TYPE = FieldType.WELDED;

  public static final Distance FIELD_LENGTH =
      Meters.of(AprilTagLayoutType.OFFICIAL.layout.getFieldLength());
  public static final Distance FIELD_WIDTH =
      Meters.of(AprilTagLayoutType.OFFICIAL.layout.getFieldWidth());

  public static final Translation2d BLUE_REEF_CENTER =
      new Translation2d(Inches.of(177.06927), Inches.of(158.5));
  public static final Translation2d RED_REEF_CENTER =
      new Translation2d(Inches.of(FIELD_LENGTH.in(Inches) - 177.06927), Inches.of(158.5));

  public static final AprilTagLayoutType DEFAULT_APRIL_TAG_LAYOUT_TYPE =
      AprilTagLayoutType.NO_BARGE;
  public static final AprilTagLayoutType CORAL_REEF_CAMERA_LAYOUT_TYPE =
      RobotState.getInstance().isRedAlliance()
          ? AprilTagLayoutType.RED_REEF
          : AprilTagLayoutType.BLUE_REEF;
  public static final AprilTagLayoutType ALGAE_REEF_CAMERA_LAYOUT_TYPE =
      AprilTagLayoutType.NO_BARGE;

  public enum AprilTagLayoutType {
    OFFICIAL("2025-official"),
    NO_BARGE("2025-no-barge"),
    BLUE_REEF("2025-blue-reef"),
    RED_REEF("2025-red-reef");

    AprilTagLayoutType(String name) {
      try {
        layout =
            new AprilTagFieldLayout(
                Path.of(
                    Filesystem.getDeployDirectory().getPath(),
                    "apriltags",
                    FIELD_TYPE.jsonFolder,
                    name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
      if (layout == null) {
        layoutString = "";
      } else {
        try {
          layoutString = new ObjectMapper().writeValueAsString(layout);
        } catch (JsonProcessingException e) {
          throw new RuntimeException("Failed to serialize AprilTag layout JSON " + toString());
        }
      }
    }

    public final AprilTagFieldLayout layout;
    public final String layoutString;
  }

  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    private final String jsonFolder;

    private FieldType(String string) {
      this.jsonFolder = string;
    }
  }
}
