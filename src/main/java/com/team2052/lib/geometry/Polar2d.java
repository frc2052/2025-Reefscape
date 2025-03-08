package com.team2052.lib.geometry;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Polar2d {
    private final Distance radius;
    private final Angle theta;

    public Polar2d(Distance radius, Angle theta) {
        this.radius = radius;
        this.theta = theta;
    }

    public Polar2d(Translation2d position) {
        radius = Meters.of(position.getNorm());
        theta = Radians.of(Math.atan2(position.getX(), position.getY()));
    }

    public Polar2d(Distance xPosition, Distance yPosition) {
        radius = Meters.of(Math.hypot(xPosition.in(Meters), yPosition.in(Meters)));
        theta = Radians.of(Math.atan2(xPosition.in(Meters), yPosition.in(Meters)));
    }

    public Distance getRadius() {
        return radius;
    }

    public Angle getTheta() {
        return theta;
    }

    public Translation2d toTranslation2d() {
        return new Translation2d(radius.in(Meters), new Rotation2d(theta.in(Radians)));
    }
}
