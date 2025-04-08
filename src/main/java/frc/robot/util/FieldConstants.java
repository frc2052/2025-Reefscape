package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
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
            new Pose2d(3.09, 4.27, new Rotation2d(Math.toRadians(0))), //
            new Pose2d(3.58, 2.94, new Rotation2d(Math.toRadians(60))), //
            new Pose2d(4.97, 2.69, new Rotation2d(Math.toRadians(120))), //
            new Pose2d(5.9, 3.8, new Rotation2d(Math.toRadians(180))), //
            new Pose2d(5.4, 5.12, new Rotation2d(Math.toRadians(240))), //
            new Pose2d(3.98, 5.36, new Rotation2d(Math.toRadians(300)))); //
    public static final List<Pose2d> blueRightBranches = List.of(
            new Pose2d(3.07, 3.89, new Rotation2d(Math.toRadians(0))), //
            new Pose2d(3.92, 2.72, new Rotation2d(Math.toRadians(60))), //
            new Pose2d(5.32, 2.86, new Rotation2d(Math.toRadians(120))), //
            new Pose2d(5.91, 4.18, new Rotation2d(Math.toRadians(180))), //
            new Pose2d(5.07, 5.33, new Rotation2d(Math.toRadians(240))), //
            new Pose2d(3.65, 5.18, new Rotation2d(Math.toRadians(300)))); //
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
            new Pose2d(14.43, 3.82, new Rotation2d(Math.toRadians(180))), //
            new Pose2d(13.95, 5.14, new Rotation2d(Math.toRadians(240))), //
            new Pose2d(12.54, 5.35, new Rotation2d(Math.toRadians(300))), //
            new Pose2d(11.65, 4.23, new Rotation2d(Math.toRadians(0))), //
            new Pose2d(12.17, 2.91, new Rotation2d(Math.toRadians(60))), //
            new Pose2d(13.59, 2.71, new Rotation2d(Math.toRadians(120)))); //
    public static final List<Pose2d> redRightBranches = List.of(
            new Pose2d(14.47, 4.21, new Rotation2d(Math.toRadians(180))), //
            new Pose2d(13.61, 5.33, new Rotation2d(Math.toRadians(240))), //
            new Pose2d(12.2, 5.16, new Rotation2d(Math.toRadians(300))), //
            new Pose2d(11.65, 3.88, new Rotation2d(Math.toRadians(0))), //
            new Pose2d(12.5, 2.72, new Rotation2d(Math.toRadians(60))), //
            new Pose2d(13.91, 2.89, new Rotation2d(Math.toRadians(120)))); //
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

    //     public static final List<Pose2d> blueLeftBranches = List.of(
    //         new Pose2d(3.09, 4.24, new Rotation2d(Math.toRadians(0))),
    //         new Pose2d(3.60, 2.93, new Rotation2d(Math.toRadians(60))),
    //         new Pose2d(4.98, 2.70, new Rotation2d(Math.toRadians(120))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
    //         new Pose2d(5.37, 5.12, new Rotation2d(Math.toRadians(240))),
    //         new Pose2d(3.95, 5.34, new Rotation2d(Math.toRadians(300))));
    // public static final List<Pose2d> blueRightBranches = List.of(
    //         new Pose2d(3.08, 3.88, new Rotation2d(Math.toRadians(0))),
    //         new Pose2d(3.91, 2.73, new Rotation2d(Math.toRadians(60))),
    //         new Pose2d(5.29, 2.86, new Rotation2d(Math.toRadians(120))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
    //         new Pose2d(5.07, 5.32, new Rotation2d(Math.toRadians(240))),
    //         new Pose2d(3.62, 5.16, new Rotation2d(Math.toRadians(300))));
    // public static final List<Pose2d> blueLeftBranchL1 = List.of(
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(240))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(300))));
    // public static final List<Pose2d> blueRightBranchL1 = List.of(
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(240))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(300))));

    // public static final List<Pose2d> redLeftBranches = List.of(
    //         new Pose2d(14.46, 3.82, new Rotation2d(Math.toRadians(180))),
    //         new Pose2d(13.94, 5.13, new Rotation2d(Math.toRadians(240))),
    //         new Pose2d(12.55, 5.35, new Rotation2d(Math.toRadians(300))),
    //         new Pose2d(11.65, 4.20, new Rotation2d(Math.toRadians(0))),
    //         new Pose2d(12.18, 2.92, new Rotation2d(Math.toRadians(60))),
    //         new Pose2d(13.59, 2.71, new Rotation2d(Math.toRadians(120))));
    // public static final List<Pose2d> redRightBranches = List.of(
    //         new Pose2d(14.47, 4.16, new Rotation2d(Math.toRadians(180))),
    //         new Pose2d(13.63, 5.32, new Rotation2d(Math.toRadians(240))),
    //         new Pose2d(12.23, 5.16, new Rotation2d(Math.toRadians(300))),
    //         new Pose2d(11.66, 3.85, new Rotation2d(Math.toRadians(0))),
    //         new Pose2d(12.51, 2.73, new Rotation2d(Math.toRadians(60))),
    //         new Pose2d(13.91, 2.90, new Rotation2d(Math.toRadians(120))));
    // public static final List<Pose2d> redLeftBranchL1 = List.of(
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(240))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(300))));
    // public static final List<Pose2d> redRightBranchL1 = List.of(
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(0))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(60))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(120))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(180))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(240))),
    //         new Pose2d(0.0, 0, new Rotation2d(Math.toRadians(300))));
}
