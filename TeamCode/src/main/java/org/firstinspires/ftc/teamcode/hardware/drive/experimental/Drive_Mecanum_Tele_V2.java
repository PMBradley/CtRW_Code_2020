package org.firstinspires.ftc.teamcode.hardware.drive.experimental;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Math.PIDController;
import org.firstinspires.ftc.teamcode.util.Math.Range2d;


public class Drive_Mecanum_Tele_V2 {
    ElapsedTime localRuntime;


    // X Translational PID Variables
    public static PIDCoefficients X_COEFFICIENTS = new PIDCoefficients(3, 0, 20);
    public static Range2d X_I_RANGE = new Range2d(0.1, 1.5);
    private PIDController xPID;

    // Y Translational PID Variables
    public static PIDCoefficients Y_COEFFICIENTS = new PIDCoefficients(3, 0, 20);
    public static Range2d Y_I_RANGE = new Range2d(0.1, 1.5);
    private PIDController yPID;

    // Heading PID Variables
    public static PIDCoefficients HEADING_COEFFICIENTS = new PIDCoefficients(3, 0, 20);
    public static Range2d HEADING_I_RANGE = new Range2d(Math.toRadians(0.1), Math.toRadians(0.8));
    private PIDController headingPID;


    //Motor variables
    private DcMotor driveFL, driveFR, driveBL, driveBR; // motors that are being used for mecanum driving
    private double veloFL, veloFR, veloBL, veloBR; // powers that are being passed to those motors



    // Default constructor
    public Drive_Mecanum_Tele_V2(DcMotor driveMotorFL, DcMotor driveMotorFR, DcMotor driveMotorBL, DcMotor driveMotorBR){ // passing of individual motors in a constructor as an alternative to needing a robot class passed with the proper motor names
        // setup motors from passed motors
        driveFL = driveMotorFL;
        driveFR = driveMotorFR;
        driveBL = driveMotorBL;
        driveBR = driveMotorBR;

        // setup runtime
        localRuntime = new ElapsedTime();

        // setup PID
        xPID = new PIDController(X_COEFFICIENTS, X_I_RANGE);
        yPID = new PIDController(Y_COEFFICIENTS, Y_I_RANGE);
        headingPID = new PIDController(HEADING_COEFFICIENTS, HEADING_I_RANGE);
    }


    // Drive functions
    public void driveFieldRelative(double x, double y, double r, double currentHeading) { // use with controller only - this drives relative to field
        // if using controller inputs, ensure you reverse y in the arguments because down on the stick is positive and up is negative, and we need that to be the opposite way

        // adjust the raw heading (between -180 and 180) to be reversed and between 0 and 359.9999... to make the math easier later
        double heading = currentHeading * -1;
        if (heading >= 0) { // if the degree value is positive, subtract 360 degrees (in radians) from it until it is between 0 and 359.999....
            while (heading >= 2*Math.PI) {
                heading -= 2*Math.PI;
            }
        }
        else { // else it must be negative, so then add 360 to it until it is greater than or equal to 0 (and therefore must be between 0 and 359.999...
            while (heading < 0) {
                heading += 2*Math.PI;
            }
        }

        // Set up heading factor for relative to field (convert the heading to radians, then get the sine and cosine of that radian heading
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);

        // do math to adjust to make the input drive vector relative to field (rather than relative to robot)
        double field_x = (y * cos) - (x * sin);
        double field_y = (y * sin) + (x * cos);


        // do math to get powers relative to field in addition to the cartesian mecanum formula
        veloFL = (field_y + (r) + field_x);
        veloFR = (field_y - (r) - field_x);
        veloBL = (field_y + (r) - field_x);
        veloBR = (field_y - (r) + field_x);


        // Normalize the powers before we pass them into the motors (so that no power is outside of the range when passed in, preserving the intended slope)
        double largest_power = Math.max( Math.max( Math.abs(veloFL), Math.abs(veloFR)), Math.max(Math.abs(veloBL), Math.abs(veloBR)) ); // first find the largest of all the powers (get the max of the first two, max of the second two, then get the max of the two maxes)
        if(largest_power > 1.0){ // if the largest power value is greater than 1
            veloFL /= largest_power; // divide each power by the largest one
            veloFR /= largest_power; // resulting in the largest power being 1 (x/x = 1)
            veloBL /= largest_power; // and the rest scaled appropriately
            veloBR /= largest_power;
        }

        // set the motor powers based off of the math done previously - Make that robot go VROOOOM
        driveFL.setPower(veloFL);
        driveFR.setPower(veloFR);
        driveBL.setPower(veloBL);
        driveBR.setPower(veloBR);
    }
    public void driveRobotRelative(double x, double y, double r) { // this drives relative to the robot, you can pass controller inputs right on in
        driveFieldRelative(x, y, r, 0); // pass values into the drive field relative function, but passing a heading of 0 (meaning it will end up acting robot relative, as a rotation of 0 will not alter translation)
    }
    public void driveToPose(Pose2d currentPose, Pose2d targetPose){ // drives the robot to a target position when called in a loop
        double xVelo = xPID.getOutput(currentPose.getX(), targetPose.getX()); // get the directions we need to move to reach target and how fast to get to those positions properly
        double yVelo = yPID.getOutput(currentPose.getY(), targetPose.getY());
        double headingVelo = headingPID.getOutput(currentPose.getHeading(), targetPose.getHeading());

        driveFieldRelative(xVelo, yVelo, headingVelo, currentPose.getHeading()); // then drive field relative at those velocities
    }
}

