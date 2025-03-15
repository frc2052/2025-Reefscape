package com.team2052.lib.planners;

import com.team2052.lib.controllers.PIDFFController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.OptionalDouble;

public class AutoAlignPlanner {
    private PIDFFController xController;
    private PIDFFController yController;
    private PIDFFController thetaController;
    private boolean autoAlignComplete = false;

    private OptionalDouble startTime;

    public AutoAlignPlanner() {
        startTime = OptionalDouble.of(Timer.getFPGATimestamp()); // x 3 y 4 r 3
        xController = new PIDFFController(0.2, 0.0, 0, 0.0, 0, 0.0, 1, 0.25);
        yController = new PIDFFController(0.1, 0.0, 0, 0.0, 0, 0.0, 1, 0.25);
        thetaController = new PIDFFController(3.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.25, 0.25);

        xController.setTolerance(0.08, 0.05);
        yController.setTolerance(0.02, 0.05);
        thetaController.setTolerance(0.03, 0.05);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d goalPose) {
        double xOutput = xController.calculate(currentPose.getX() - goalPose.getX(), 0);
        double yOutput = yController.calculate(currentPose.getY() - goalPose.getY(), 0);
        double thetaOutput = thetaController.calculate(
                MathUtil.angleModulus(goalPose.getRotation().getRadians()
                        - currentPose.getRotation().getRadians()),
                MathUtil.angleModulus(0));

        ChassisSpeeds calculatedSpeeds;

        boolean xWithinTol = xController.atSetpoint();
        boolean yWithinTol = true; // yController.atSetpoint();
        boolean thetaWithinTol = true;

        calculatedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xWithinTol ? 0.0 : xOutput,
                yWithinTol ? 0.0 : yOutput,
                thetaWithinTol ? 0.0 : thetaOutput,
                currentPose.getRotation());

        autoAlignComplete = xWithinTol && yWithinTol && thetaWithinTol;
        if (startTime.isPresent() && autoAlignComplete) {
            // System.out.println("Auto align took: " + (Timer.getFPGATimestamp() -
            // startTime.getAsDouble()));
            startTime = OptionalDouble.empty();
        }

        return calculatedSpeeds;
    }

    public ChassisSpeeds calculateBangBang(Pose2d currentPose, Pose2d goalPose) {
        double xOutput =
                (goalPose.getX() - currentPose.getX() > 1) ? 0.5 : ((goalPose.getX() - currentPose.getX()) / 20);

        double yOutput =
                (goalPose.getX() - currentPose.getX() > 1) ? 0.5 : ((goalPose.getX() - currentPose.getX()) / 20);
        double thetaOutput = thetaController.calculate(
                MathUtil.angleModulus(goalPose.getRotation().getRadians()
                        - currentPose.getRotation().getRadians()),
                MathUtil.angleModulus(0));

        ChassisSpeeds calculatedSpeeds;

        boolean xWithinTol = xController.atSetpoint();
        boolean yWithinTol = yController.atSetpoint();
        boolean thetaWithinTol = thetaController.atSetpoint();

        System.out.println("X: " + xOutput + "   Y:  " + yOutput + "   Theta:  " + thetaOutput);

        calculatedSpeeds = new ChassisSpeeds(
                xWithinTol ? 0.0 : xOutput, yWithinTol ? 0.0 : yOutput, thetaWithinTol ? 0.0 : thetaOutput);

        autoAlignComplete = xWithinTol && yWithinTol && thetaWithinTol;
        if (startTime.isPresent() && autoAlignComplete) {
            // System.out.println("Auto align took: " + (Timer.getFPGATimestamp() -
            // startTime.getAsDouble()));
            startTime = OptionalDouble.empty();
        }

        return calculatedSpeeds;
    }

    public boolean getAutoAlignComplete() {
        return autoAlignComplete;
    }

    public void resetPlanner() {
        autoAlignComplete = false;
        xController = new PIDFFController(3, 0.0, 0, 0.0, 0, 0.0, 1.0, 0.5);
        yController = new PIDFFController(4, 0.0, 0, 0.0, 0, 0.0, 1.0, 0.5);
        thetaController = new PIDFFController(3.0, 0.0, 0.5, 0.0, 0.0, 0.0, 2.0, 2.0);
    }
}
