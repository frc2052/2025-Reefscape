package frc.robot;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Area2D {
    private Translation2d P1;
    private Translation2d P2;
    private Translation2d P3;
    private Translation2d P4;
    private double rad;
    private List<Translation2d> points;
    private List<Translation2d> deadZonesPoints;
    private Translation2d DP1;
    private double DRad;
    private boolean deadZones;
    private boolean circle;

    public Area2D(Translation2d P1, double rad) {
        this.P1 = P1;
        this.rad = rad;
        circle = true;
    }

    public Area2D(Translation2d P1, double rad, List<Translation2d> deadZonesPoints) {
        this.P1 = P1;
        this.rad = rad;
        circle = true;
        deadZones = true;
        if (verification(deadZonesPoints)){
        this.deadZonesPoints = deadZonesPoints;
        }else{
            System.out.print("the values you entered for dead zones are outside of the shape, pls enter points that are inside the shape");
        }
    }

    public Area2D(List<Translation2d> points) {
        this.points = points;
    }

    public Area2D(List<Translation2d> points,List<Translation2d> deadZonesPoints){
        this.points = points;
        deadZones = true;
        if (verification(deadZonesPoints)){
          for (Translation2d i : deadZonesPoints){
            points.add(i);
          }
            }else{
                System.out.print("the values you entered for dead zones are outside of the shape, pls enter points that are inside the shape");
            }

    }


    private boolean verification(List<Translation2d> verificationPoints) {
        for (Translation2d point : verificationPoints) {
            if (!multiPointCalc(new Pose2d(point.getX(), point.getY(), new Rotation2d(0)), points)) {
                return false;
            }
        }
        return true;
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
            if(deadZones){
            if (multiPointCalc(pose, deadZonesPoints)){
                return false;
            }else{
                return singlePointCalc(pose);
            }
            }
        } else {
            return multiPointCalc(pose, points);
        }

        return false;
    }
}