package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import java.util.List;

public class FieldConstants {
    // public static final FieldType FIELD_TYPE = FieldType.WELDED;

    public static final AprilTagFieldLayout DEFAULT_APRIL_TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    public static final Distance FIELD_LENGTH = Meters.of(DEFAULT_APRIL_TAG_LAYOUT.getFieldLength());
    public static final Distance FIELD_WIDTH = Meters.of(DEFAULT_APRIL_TAG_LAYOUT.getFieldWidth());

    public static final Translation2d BLUE_REEF_CENTER = new Translation2d(Inches.of(176.495), Inches.of(158.751));
    public static final Translation2d RED_REEF_CENTER = new Translation2d(Inches.of(513.880), Inches.of(158.749));
    // public static final AprilTagLayoutType CORAL_REEF_CAMERA_LAYOUT_TYPE =
    //         RobotState.getInstance().isRedAlliance() ? AprilTagLayoutType.RED_REEF : AprilTagLayoutType.BLUE_REEF;
    // public static final AprilTagLayoutType ALGAE_REEF_CAMERA_LAYOUT_TYPE = AprilTagLayoutType.NO_BARGE;

    // public enum AprilTagLayoutType {
    //     OFFICIAL("2025-official"),
    //     NO_BARGE("2025-no-barge"),
    //     BLUE_REEF("2025-blue-reef"),
    //     RED_REEF("2025-red-reef");

    //     AprilTagLayoutType(String name) {
    //         try {
    //             layout = new AprilTagFieldLayout(Path.of(
    //                     Filesystem.getDeployDirectory().getPath(), "apriltags", FIELD_TYPE.jsonFolder, name +
    // ".json"));
    //         } catch (IOException e) {
    //             throw new RuntimeException(e);
    //         }
    //         if (layout == null) {
    //             layoutString = "";
    //         } else {
    //             try {
    //                 layoutString = new ObjectMapper().writeValueAsString(layout);
    //             } catch (JsonProcessingException e) {
    //                 throw new RuntimeException("Failed to serialize AprilTag layout JSON " + toString());
    //             }
    //         }
    //     }

    //     public final AprilTagFieldLayout layout;
    //     public final String layoutString;
    // }

    // public enum FieldType {
    //     ANDYMARK("andymark"),
    //     WELDED("welded");

    //     private final String jsonFolder;

    //     private FieldType(String string) {
    //         this.jsonFolder = string;
    //     }
    // }

    /* All positions in blue coordinate system
     *
     * Lists go in this order:
     * AB
     * CD
     * EF
     * GH
     * IJ
     * KL
     */
    public static final List<Pose2d> blueLeftBranches = List.of(
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
            new Pose2d(5.35, 4.98, new Rotation2d(Math.toRadians(240))),
            new Pose2d(4.01, 5.22, new Rotation2d(Math.toRadians(300)))); // 4.01, 5.24
    public static final List<Pose2d> blueRightBranches = List.of(
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
            new Pose2d(5.03, 5.23, new Rotation2d(Math.toRadians(240))),
            new Pose2d(3.71, 5.08, new Rotation2d(Math.toRadians(300)))); // 3.71, 5.08
    public static final List<Pose2d> blueLeftBranchL1 = List.of(
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(240))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(300))));
    public static final List<Pose2d> blueRightBranchL1 = List.of(
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(240))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(300))));

    public static final List<Pose2d> redLeftBranches = List.of(
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(240))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(300))));
    public static final List<Pose2d> redRightBranches = List.of(
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(240))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(300))));
    public static final List<Pose2d> redLeftBranchL1 = List.of(
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(240))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(300))));
    public static final List<Pose2d> redRightBranchL1 = List.of(
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(240))),
            new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(300))));
}
