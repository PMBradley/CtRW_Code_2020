package org.firstinspires.ftc.teamcode.hardware.drive.experimental;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.Math.AngleMath;
import org.firstinspires.ftc.teamcode.util.Math.PIDController;
import org.firstinspires.ftc.teamcode.util.Math.Range2d;


@Config
public class Drive_Mecanum_Tele_V2 {

/*     --------------------------  EDITABLE CONSTANTS  --------------------------
        These constants are set to default values but can
        (or should) be edited to improve robot performance
*/
    // X Translational PID Variables
    public static PIDCoefficients X_COEFFICIENTS = new PIDCoefficients(0.8, 0.0, 0.4);
    public static Range2d X_I_RANGE = new Range2d(0.1, 4);

    // Y Translational PID Variables
    public static PIDCoefficients Y_COEFFICIENTS = new PIDCoefficients(0.8, 0.0, 0.4);
    public static Range2d Y_I_RANGE = new Range2d(0.1, 4);

    // Heading PID Variables
    public static PIDCoefficients HEADING_COEFFICIENTS = new PIDCoefficients(6, 0.0, 0.9);
    public static Range2d HEADING_I_RANGE = new Range2d(Math.toRadians(0.05), Math.toRadians(0.8));

    // Speed Limiter Variables
    public static double MAX_TRANSLATE_SPEED = 1.00; // allow the bot to translate at 100% speed
    public static double TRANSLATE_ACCEL_TIME = 1.0; // allow the robot to accelerate to max translational speed in 1 second
    public static double MAX_ROTATE_SPEED = 1.00; // allow the bot rotate at 100% speed
    public static double ROTATE_ACCEL_TIME = 0.5; // allow the robot to accelerate to max rotational speed in 0.5 seconds

    // Rotational Center Correction Variables
    public static Vector2d ROTATIONAL_CENTER = new Vector2d(0,0); // the x and y of where the rotational center of the robot is to the geometric center (Ex: (1, -2) would be 1 inch up and 2 inches right of the geometric center, with up being towards the front of the bot
    /* TO DETERMINE ROTATIONAL CENTER:
        (instructions to be written)
    */


//     --------------------------  CLASS START  --------------------------
    // Hardware variables
    private DcMotor driveFL, driveFR, driveBL, driveBR; // motors that are being used for mecanum driving
    private double veloFL, veloFR, veloBL, veloBR; // powers that are being passed to those motors
    private PIDController xPID; // PID Controllers used for translation and rotational control when driving autonomously
    private PIDController yPID;
    private PIDController headingPID;

    // Time and tracking variables
    ElapsedTime localRuntime;
    private double lastXSpeed = 0; // robot relative recorded speed inputs into the drive method, used to regulate how fast we can accelerate
    private double lastYSpeed = 0;
    private double lastRSpeed = 0;
    private double lastRunTime = 0;

    // Calculated Constants (constants calculated based on editable constants or other conditions)
    private static double TRANSLATE_ACCEL_PER_SEC =  MAX_TRANSLATE_SPEED / TRANSLATE_ACCEL_TIME; // allow the bot to accelerate/decelerate by X% of max translate speed per second (this number is equal to 1/(seconds to accelerate fully))
    private static double ROTATE_ACCEL_PER_SEC =  MAX_ROTATE_SPEED / ROTATE_ACCEL_TIME; // allow the bot to accelerate/decelerate by Y% of max rotate speed per second (this number is equal to 1/(seconds to accelerate fully))
    private static double ROTATIONAL_CENTER_MAGNITUDE = 1; // the distance of the rotational center from the geometric center (also serves as the magnitudional component of the polar coordinate position of the rotational center)
    private static double ROTATIONAL_CENTER_THETA = 0; // the rotation component of the polar coordinate of the rotational center

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

        // set calculated constants to their appropriate starting values (or ensure they are set to those values)
        updateTranslateAccelPerSec(); // calculate the values for the allowed acceleration per second variables
        updateRotateAccelPerSec();
        setRotationalCenter(ROTATIONAL_CENTER); // update the rotational theta and magnitude variables based on the current rotaitonal centere
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


        // limiting speed before made field relative, as it is more useful to limit speed relative to robot (physics cares more about the robot's momentum relative to itself than to the field)
        if( limitingSpeed ){
            x = getLimitedSpeed(x, 'X'); // limit the speeds
            y = getLimitedSpeed(y, 'Y');
            r = getLimitedSpeed(r, 'R');
        }
        lastXSpeed = x; // speed target is limited before being recorded, to ensure proper acceleration behavior
        lastYSpeed = y;
        lastRSpeed = r;
        lastRunTime = localRuntime.milliseconds();


        double heading =  Math.toDegrees(currentHeading) * -1;
        heading = AngleMath.clipAngle(heading, AngleUnit.DEGREES);

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
        //currentPose = getRotationalCenterRelativePose(currentPose); // account for rotational center differences by navigating around the center of rotation
        //targetPose = getRotationalCenterRelativePose(targetPose);

        double xVelo = xPID.getOutput(currentPose.getX(), targetPose.getX()); // get the directions we need to move to reach target and how fast to get to those positions properly
        double yVelo = -yPID.getOutput(currentPose.getY(), targetPose.getY()); // the y one made negative to accommodate for the drive field relative method y negation

        double currentHeading = currentPose.getHeading(); // adjust the current heading if needed, to make sure the most efficient path is being taken from current to target
        double targetHeading = targetPose.getHeading(); // needed because if at 0 and wanting to get to 270, you don't want to go all the way through 90 and 180, you can just go in the negative direction
        if(currentHeading + Math.PI < targetHeading){ // if current heading is 180 (or more) degrees below the target position, subtract 360 from the current heading so we travel the most efficient route towards the target
            targetHeading -= 2*Math.PI;
        }
        else if(currentHeading - Math.PI > targetHeading){ // else if current heading is 180 (or more) degrees above the target position, add 360 to the target heading so we travel the most efficient route towards the target
            targetHeading += 2*Math.PI;
        }

        double headingVelo = -headingPID.getOutput(currentHeading, targetHeading);

        driveFieldRelative(xVelo, yVelo, headingVelo, currentPose.getHeading(), limitingSpeed); // then drive field relative at those velocities
    }


    public double getLimitedSpeed(double inputSpeed, char speedAxis){ // takes an input speed and the axis of movement for that speed, then range clip the speed to not accelerate or decelerate too quickly
        double outputSpeed = inputSpeed;
        double timeFactor = (localRuntime.milliseconds() - lastRunTime) / 1000; // gets how long it has been since the last run, in seconds so we can use it the "acceleration/second" math

        // first limit acceleration
        if( speedAxis == 'X' ){ // if the axis of this speed is X, limit accordingly
            double allowedXAccel = TRANSLATE_ACCEL_PER_SEC * timeFactor * MAX_TRANSLATE_SPEED; // how much the new speeds are allowed to be above/below the old speed

            // limit X acceleration/deceleration
            if(inputSpeed > lastXSpeed + allowedXAccel){ // if x is accelerating too fast
                outputSpeed = lastXSpeed + allowedXAccel; // set x to the fastest speed it is allowed to accelerate
            }
            else if(inputSpeed < lastXSpeed - allowedXAccel){ // or if decelerating too fast
                outputSpeed = lastXSpeed - allowedXAccel; // set x to the fastest speed it is allowed to accelerate
            }
        }
        else if( speedAxis == 'Y' ){ // if the axis of this speed is Y, limit accordingly
            double allowedYAccel = TRANSLATE_ACCEL_PER_SEC * timeFactor * MAX_TRANSLATE_SPEED; // how much the new speeds are allowed to be above/below the old speed

            if(inputSpeed > lastYSpeed + allowedYAccel){ // if y is accelerating too fast
                outputSpeed = lastYSpeed + allowedYAccel; // set y to the fastest speed it is allowed to accelerate
            }
            else if(inputSpeed < lastYSpeed - allowedYAccel){ // or if decelerating too fast
                outputSpeed = lastYSpeed - allowedYAccel; // set y to the fastest speed it is allowed to accelerate
            }
        }
        else { // then the axis for this speed is Y
            double allowedRAccel = ROTATE_ACCEL_PER_SEC * timeFactor * MAX_ROTATE_SPEED; // how much the new speeds are allowed to be above/below the old speed

            if(inputSpeed > lastRSpeed + allowedRAccel){ // if r is accelerating too fast
                outputSpeed = lastRSpeed + allowedRAccel; // set r to the fastest speed it is allowed to accelerate
            }
            else if(inputSpeed < lastRSpeed - allowedRAccel){ // or if decelerating too fast
                outputSpeed = lastRSpeed - allowedRAccel; // set r to the fastest speed it is allowed to accelerate
            }
        }

        // then ensure it is below or at the max speed
        if (speedAxis == 'R' && Math.abs(outputSpeed) > MAX_ROTATE_SPEED){ // if a rotational speed and
            outputSpeed = MAX_ROTATE_SPEED * (Math.abs(outputSpeed) / outputSpeed); // set R to the speed limit (multiplied by 1 if x is positive, negative 1 if x is negative)
        }
        else if ( (speedAxis == 'X' || speedAxis == 'Y') && Math.abs(outputSpeed) > MAX_TRANSLATE_SPEED){
            outputSpeed = MAX_TRANSLATE_SPEED * (Math.abs(outputSpeed) / outputSpeed); // set the output speed to the speed limit (multiplied by 1 if x is positive, negative 1 if x is negative)
        }

        return outputSpeed;
    }


    public void  setMaxTranslateSpeed(double maxTranslateSpeed){ // sets the max translational speed and updates the accel per second
        MAX_TRANSLATE_SPEED = maxTranslateSpeed;
        updateTranslateAccelPerSec();
    }
    public void setTranslationalAccelTime(double accelSeconds){ // sets time to accel to max speed and updates the accel per second
        TRANSLATE_ACCEL_TIME = accelSeconds;
        updateTranslateAccelPerSec();
    }
    public double updateTranslateAccelPerSec(){ // updates the acceleration per second to be proper according to the max speed and minimum time to reach that max speed
        TRANSLATE_ACCEL_PER_SEC = MAX_TRANSLATE_SPEED / TRANSLATE_ACCEL_TIME;
        return TRANSLATE_ACCEL_PER_SEC;
    }

    public void setMaxRotateSpeed(double maxRotateSpeed){ // sets the max translational speed and updates the accel per second
        MAX_ROTATE_SPEED = maxRotateSpeed;
        updateRotateAccelPerSec();
    }
    public void setRotationalAccelTime(double accelSeconds){ // sets time to accel to max speed and updates the accel per second
        ROTATE_ACCEL_TIME = accelSeconds;
        updateRotateAccelPerSec();
    }
    public double updateRotateAccelPerSec(){ // updates the acceleration per second to be proper according to the max speed and minimum time to reach that max speed
        ROTATE_ACCEL_PER_SEC = MAX_ROTATE_SPEED / ROTATE_ACCEL_TIME;
        return ROTATE_ACCEL_PER_SEC;
    }

    public void setRotationalCenter(Vector2d position){ // sets the rotational center based on an input, plus updates the rotational center magnitude
        ROTATIONAL_CENTER = position;
        ROTATIONAL_CENTER_MAGNITUDE = AngleMath.getVectorMagnitude(ROTATIONAL_CENTER); // get hypotenuse of the right triangle that has the sides X and Y (gets the straight line distance of the rotational center from the geometric center)
        ROTATIONAL_CENTER_THETA = AngleMath.getVectorAngle(ROTATIONAL_CENTER);
    }
    public static Pose2d getRotationalCenterRelativePose(Pose2d geometricalCenterRelativePose){
        double geoX = geometricalCenterRelativePose.getX(); // get the values out into variables to make things neater and perform fewer method calls
        double geoY = geometricalCenterRelativePose.getY();
        double geoHeading = geometricalCenterRelativePose.getHeading();

        double rotatedTheta = ROTATIONAL_CENTER_THETA + geoHeading; // rotate the rotational center's position by the robot's heading (so it lines up with reality)
        if( rotatedTheta > Math.PI*2 ) // if the new theta is now above 360, subtract 360 to keep it within a 0 to 360 range (geo heading must always be within 0 to 360, so can only increase)
            rotatedTheta -= Math.PI*2;

        // then convert the new rotated rotaional center back to rectangular coordinates, to be added to the position
        double rotationalCenterXOffset = ROTATIONAL_CENTER_MAGNITUDE * Math.cos(rotatedTheta);
        double rotationalCenterYOffset = ROTATIONAL_CENTER_MAGNITUDE * Math.sin(rotatedTheta);
        if( rotatedTheta > Math.PI/2 && rotatedTheta < 3*Math.PI/2 ) // if the theta of this point is in the 2nd or 3rd quadrants, x is negative
            rotationalCenterXOffset *= -1;
        if( rotatedTheta > Math.PI ) // If the theta of this point is in the 3rd or 4th quadrants, the y is negative
            rotationalCenterYOffset *= -1;

        return new Pose2d(geoX + rotationalCenterXOffset, geoY + rotationalCenterYOffset, geoHeading);
    }

}

