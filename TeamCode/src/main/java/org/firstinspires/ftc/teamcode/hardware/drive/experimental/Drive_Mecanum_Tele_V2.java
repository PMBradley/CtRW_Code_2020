package org.firstinspires.ftc.teamcode.hardware.drive.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Math.PIDController;
import org.firstinspires.ftc.teamcode.util.Math.Range2d;


@Config
public class Drive_Mecanum_Tele_V2 {
    ElapsedTime localRuntime;

    // X Translational PID Variables
    public static PIDCoefficients X_COEFFICIENTS = new PIDCoefficients(0.8, 0.0, 0);
    public static Range2d X_I_RANGE = new Range2d(0.1, 4);
    private PIDController xPID;

    // Y Translational PID Variables
    public static PIDCoefficients Y_COEFFICIENTS = new PIDCoefficients(0.8, 0.0, 0);
    public static Range2d Y_I_RANGE = new Range2d(0.1, 4);
    private PIDController yPID;

    // Heading PID Variables
    public static PIDCoefficients HEADING_COEFFICIENTS = new PIDCoefficients(4, 0.0, 0);
    public static Range2d HEADING_I_RANGE = new Range2d(Math.toRadians(0.05), Math.toRadians(0.8));
    private PIDController headingPID;

    // Speed Limiter Variables
    public static double MAX_TRANSLATE_SPEED_PERCENT = 1.00; // allow the bot to translate at 100% speed
    public static double MAX_TRANSLATE_ACCELERATION = 1.00; // allow the bot to accelerate/decelerate by 100% of max translate speed per second (this number is equal to 1/(seconds to accelerate fully))
    public static double MAX_ROTATE_SPEED_PERCENT = 1.00; // allow the bot rotate at 100% speed
    public static double MAX_ROTATE_ACCELERATION = 2.00; // allow the bot to accelerate/decelerate by 75% of max rotate speed per second (this number is equal to 1/(seconds to accelerate fully))
    private double lastXSpeed = 0; // robot relative recorded speed inputs into the drive method, used to regulate how fast we can accelerate
    private double lastYSpeed = 0;
    private double lastRSpeed = 0;
    private double lastRunTime = 0;


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
    public void driveFieldRelative(double x, double y, double r, double currentHeading, boolean limitingSpeed) { // this drives relative to field (+x is forward, +y is left, heading is in radians)
        // if using controller inputs, ensure you reverse the y on the stick input before passing into this method because down on the stick is positive and up is negative, and we need that to be the opposite way

        // Set up heading factor for relative to field (convert the heading to radians, then get the sine and cosine of that radian heading
        //double sin = Math.sin(currentHeading);
        //double cos = Math.cos(currentHeading);

        // do math to adjust to make the input drive vector relative to field (rather than relative to robot)
        //double field_x = (y * cos) - (x * -sin);
       // double field_y = (y * -sin) + (x * cos);


        // limiting speed before made field relative, as it is more useful to
        if( limitingSpeed ){
            double timeFactor = (localRuntime.milliseconds() - lastRunTime) / 1000; // gets how long it has been since the last run, in seconds so we can use it the "acceleration/second" math
            double allowedXAccel = MAX_TRANSLATE_ACCELERATION * timeFactor * MAX_TRANSLATE_SPEED_PERCENT; // how much the new speeds are allowed to be above the old speeds
            double allowedYAccel = MAX_TRANSLATE_ACCELERATION * timeFactor * MAX_TRANSLATE_SPEED_PERCENT;
            double allowedRAccel = MAX_ROTATE_ACCELERATION * timeFactor * MAX_ROTATE_SPEED_PERCENT;

            // limit X acceleration/deceleration
            if(x > lastXSpeed + allowedXAccel){ // if x is accelerating too fast
                x = lastXSpeed + allowedXAccel; // set x to the fastest speed it is allowed to accelerate
            }
            else if(x < lastXSpeed - allowedXAccel){ // or if decelerating too fast
                x = lastXSpeed - allowedXAccel; // set x to the fastest speed it is allowed to accelerate
            }
            // then do the same for the other 2 axises of movement
            if(y > lastYSpeed + allowedYAccel){ // if y is accelerating too fast
                y = lastYSpeed + allowedYAccel; // set y to the fastest speed it is allowed to accelerate
            }
            else if(y < lastYSpeed - allowedYAccel){ // or if decelerating too fast
                y = lastYSpeed - allowedYAccel; // set y to the fastest speed it is allowed to accelerate
            }

            if(r > lastRSpeed + allowedRAccel){ // if r is accelerating too fast
                r = lastRSpeed + allowedRAccel; // set r to the fastest speed it is allowed to accelerate
            }
            else if(r < lastRSpeed - allowedRAccel){ // or if decelerating too fast
                r = lastRSpeed - allowedRAccel; // set r to the fastest speed it is allowed to accelerate
            }

            if(Math.abs(x) > MAX_TRANSLATE_SPEED_PERCENT){ // if X is above the speed limit
                x = MAX_TRANSLATE_SPEED_PERCENT * (Math.abs(x) / x); // set X to the speed limit (multiplied by 1 if x is positive, negative 1 if x is negative)
            }
            if(Math.abs(y) > MAX_TRANSLATE_SPEED_PERCENT){ // if Y is above the speed limit
                y = MAX_TRANSLATE_SPEED_PERCENT * (Math.abs(y) / y); // set Y to the speed limit (multiplied by 1 if y is positive, negative 1 if y is negative)
            }
            if(Math.abs(r) > MAX_ROTATE_SPEED_PERCENT){ // if R is above the speed limit
                r = MAX_ROTATE_SPEED_PERCENT * (Math.abs(r) / r); // set R to the speed limit (multiplied by 1 if r is positive, negative 1 if r is negative)
            }
        }
        lastXSpeed = x; // speed target is limited before being recorded, to ensure proper acceleration behavior
        lastYSpeed = y;
        lastRSpeed = r;
        lastRunTime = localRuntime.milliseconds();


        double heading = Math.toDegrees(currentHeading) * -1;
        if (heading >= 0) { // if the degree value is positive, subtract 360 from it until it is between 0 and 359.999....
            while (heading >= 360) {
                heading -= 360;
            }
        }
        else { // else it must be negative, so then add 360 to it until it is greater than or equal to 0 (and therefore must be between 0 and 359.999...
            while (heading < 0) {
                heading += 360;
            }
        }

        // Set up heading factor for relative to field (convert the heading to radians, then get the sine and cosine of that radian heading
        double sin = Math.sin(Math.toRadians(heading));
        double cos = Math.cos(Math.toRadians(heading));

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
    public void driveFieldRelative(double x, double y, double r, double currentHeading){
        driveFieldRelative(x, y, r, currentHeading, false);
    }

    public void driveRobotRelative(double x, double y, double r) { // this drives relative to the robot, you can pass controller inputs right on in
        driveFieldRelative(x, y, r, 0, false); // pass values into the drive field relative function, but passing a heading of 0 (meaning it will end up acting robot relative, as a rotation of 0 will not alter translation)
    }
    public void driveRobotRelative(double x, double y, double r, boolean limitingSpeed) { // this drives relative to the robot, you can pass controller inputs right on in
        driveFieldRelative(x, y, r, 0, limitingSpeed); // pass values into the drive field relative function, but passing a heading of 0 (meaning it will end up acting robot relative, as a rotation of 0 will not alter translation)
    }

    public void driveToPose(Pose2d currentPose, Pose2d targetPose){ // drives the robot to a target position when called in a loop
        driveToPose(currentPose, targetPose, false);
    }
    public void driveToPose(Pose2d currentPose, Pose2d targetPose, boolean limitingSpeed){ // drives the robot to a target position when called in a loop
        double xVelo = xPID.getOutput(currentPose.getX(), targetPose.getX()); // get the directions we need to move to reach target and how fast to get to those positions properly
        double yVelo = -yPID.getOutput(currentPose.getY(), targetPose.getY()); // this one made negative to accommodate for the drive field relative method y negation

        double currentHeading = currentPose.getHeading(); // adjust the current heading if needed, to make sure the most efficient path is being taken from current to target
        double targetHeading = targetPose.getHeading(); // needed because if at 0 and wanting to get to 270, you don't want to go all the way through 90 and 180, you can just go in the negative direction
        if(currentHeading < targetHeading - 180){ // if current heading is 180 (or more) degrees below the target position, subtract 360 from the current heading so we travel the most efficient route towards the target
            targetHeading -= 360;
        }
        else if(currentHeading > targetHeading + 180){ // else if current heading is 180 (or more) degrees above the target position, add 360 to the target heading so we travel the most efficient route towards the target
            targetHeading += 360;
        }

        double headingVelo = -headingPID.getOutput(currentHeading, targetHeading);

        driveFieldRelative(xVelo, yVelo, headingVelo, currentPose.getHeading(), limitingSpeed); // then drive field relative at those velocities
    }
}

