package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;

public class Area2D {
    private Translation2d P1;
    private double rad;
    private List<Translation2d> points;
    ;
    private boolean circle;

    public Area2D(Translation2d P1, double rad) {
        this.P1 = P1;
        this.rad = rad;
        circle = true;
    }


    public Area2D(List<Translation2d> areaPoints) {
        points = areaPoints;
    }


    private boolean singlePointCalc(Pose2d pose) {
        Translation2d robotPose = pose.getTranslation();
        return robotPose.getDistance(P1) <= rad;
    }

    private boolean multiPointCalc(Pose2d pose, List<Translation2d> shapepoints) {
        double x = pose.getX();
        double y = pose.getY();
        int num = shapepoints.size();
        boolean inside = false;

        if (num != 0) {
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
        } else {
            return false;
        }
    }

    public boolean withInTheRegion(Pose2d pose) {
        if(circle){
            return singlePointCalc(pose);
        }else{
            return multiPointCalc(pose,points);

        }
    
}
}
