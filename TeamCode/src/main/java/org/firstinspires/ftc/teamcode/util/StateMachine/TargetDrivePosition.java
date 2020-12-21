package org.firstinspires.ftc.teamcode.util.StateMachine;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class TargetDrivePosition {
    private double x, y, heading; // main position variables - x and y are in inches - r is in radians
    private boolean usingSpline; // a boolean flag that determines if the drive should traverse there using a spline path
    private double splineHeading; // the heading that the spline generator uses as a target approach heading, in radians

    public TargetDrivePosition(){ // default constructor
        this.x = 0;
        this.y = 0;
        this.heading = 0;

        this.usingSpline = false; // if no spline heading passed in, assume not using spline
        this.splineHeading = 0;
    }
    public TargetDrivePosition(double x, double y, double heading){ // no spline heading constructor
        this.x = x;
        this.y = y;
        this.heading = heading;

        usingSpline = false; // if no spline heading passed in, assume not using spline
        splineHeading = 0;
    }
    public TargetDrivePosition(double x, double y, double heading, double splineHeading){ // spline heading constructor
        this.x = x;
        this.y = y;
        this.heading = heading;

        usingSpline = true; // if spline heading passed in, assume using spline
        this.splineHeading = splineHeading;
    }

    public double getX(){ return x; }
    public double getY(){ return y; }
    public double getHeading(){ return heading; }
    public Pose2d getPose2d(){ // returns a pose2d of the target location
        return new Pose2d(x, y, heading);
    }
    public double getSplineHeading(){ return splineHeading; }
    public boolean isUsingSpline(){ return usingSpline; }

    public void setX(double newX){ x = newX; }
    public void setY(double newY){ y = newY; }
    public void setHeading(double newHeading){
        heading = newHeading;
        usingSpline = true; // if passed a new spline heading, assume using a spline path
    }
    public void setUsingSpline(boolean usingSpline){
        this.usingSpline = usingSpline;
    }

    public double getDistanceFrom(TargetDrivePosition other){
        return Math.sqrt( Math.pow(this.x - other.x, 2) + Math.pow(this.y - other.y, 2) );
    }
    public double getDistanceFrom(Pose2d other){
        return Math.sqrt( Math.pow(this.x - other.getX(), 2) + Math.pow(this.y - other.getY(), 2) );
    }
}
