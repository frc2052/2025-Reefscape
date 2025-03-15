package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class Area2D {
    private Translation2d P1;
    private double rad;
    private List<Translation2d> points;
    private List<Translation2d> deadZonesPoints;
    ;
    private boolean deadZones;
    private boolean circle;

    public Area2D(Translation2d P1, double rad) {
        this.P1 = P1;
        this.rad = rad;
        circle = true;
    }

    public Area2D(Translation2d P1, double rad, List<Translation2d> deadPoints) {
        this.P1 = P1;
        this.rad = rad;
        circle = true;
        if (verification(deadPoints)) {
            deadZones = true;
            deadZonesPoints = deadPoints;
        } else {
            System.out.print(
                    "the values you entered for dead zones are outside of the shape, pls enter points that are inside the shape");
        }
    }

    public Area2D(List<Translation2d> areaPoints) {
        points = areaPoints;
    }

    public Area2D(List<Translation2d> areaPoints, List<Translation2d> deadPoints) {
        points = areaPoints;
        if (verification(deadPoints)) {
            deadZones = true;
            for (Translation2d i : deadPoints) {
                deadZonesPoints.add(i);
            }
        } else {
            System.out.print(
                    "the values you entered for dead zones are outside of the shape, pls enter points that are inside the shape");
        }
    }

    private boolean verification(List<Translation2d> verificationPoints) {
        for (Translation2d point : verificationPoints) {
            if (multiPointCalc(new Pose2d(point.getX(), point.getY(), new Rotation2d(0)), points)) {
                return true;
            }
        }
        return false;
    }

    private boolean singlePointCalc(Pose2d pose) {
        Translation2d robotPose = pose.getTranslation();
        return robotPose.getDistance(P1) <= rad;
    }

    private boolean multiPointCalc(Pose2d pose, List<Translation2d> points) {
        double x = pose.getX();
        double y = pose.getY();
        int num = points.size();
        boolean inside = false;

        for (int i = 0, j = num - 1; i < num; j = i++) {
            double xi = points.get(i).getX();
            double yi = points.get(i).getY();
            double xj = points.get(j).getX();
            double yj = points.get(j).getY();
            if ((yi > y) != (yj > y) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
                inside = !inside;
            }
        }
        return inside;
    }

    public boolean withInTheRegion(Pose2d pose) {
        if (circle) {
            if (deadZones) {
                if (multiPointCalc(pose, deadZonesPoints)) {
                    return false;
                } else {
                    return singlePointCalc(pose);
                }
            }
        } else {
            if (deadZones) {
                if (multiPointCalc(pose, deadZonesPoints)) {
                    System.out.println("out of zone");
                    return false;
                } else {
                    System.out.println("zone in");
                    return multiPointCalc(pose, points);
                }
            }
        }
        return false;
    }
}
