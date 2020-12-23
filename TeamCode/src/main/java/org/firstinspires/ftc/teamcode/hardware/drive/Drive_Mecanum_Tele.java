package org.firstinspires.ftc.teamcode.hardware.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Drive_Mecanum_Tele {
    ElapsedTime localRuntime;

    // Create and initialize speed modifier variables - using default values (then can be set via a constructor)
    private double turnMultiplier = DEFAULT_TURN_MULTIPLIER; // what percentage of maximum turning speed should be used as a base turning speed (50% = 0.5, etc) - a multiplier
    private double translateMultiplier = DEFAULT_TRANSLATE_MULTIPLIER; // what percentage of maximum translational speed should be used as a base translational speed (50% = 0.5, etc) - a multiplier
    private double boostingMultiplier  = DEFAULT_BOOSTING_MULTIPLIER; // what percentage of maximum translational speed should be used as a boost translational speed (50% = 0.5, etc) - a multiplier

    // Default speed modifier values
    private static final double DEFAULT_TURN_MULTIPLIER = 0.6; // default values to use in the event no custom values are passed
    private static final double DEFAULT_TRANSLATE_MULTIPLIER = 0.6;
    private static final double DEFAULT_BOOSTING_MULTIPLIER = 1.0;

    // turn PID coeficients
    public static final double Kp = 3.0;
    public static final double Ki = 0.0;
    public static final double Kd = 20.0;

    private double lastError = 0.0;
    private double lastRuntime = 0.0;
    private double integral = 0.0;
    private double lastTargetHeading = 0.0;


    //Motor variables
    private DcMotor driveFL, driveFR, driveBL, driveBR; // motors that are being used for mecanum driving
    private double powerFL, powerFR, powerBL, powerBR; // powers that are being passed to those motors


    // Default constructor
    public Drive_Mecanum_Tele(DcMotor driveMotorFL, DcMotor driveMotorFR, DcMotor driveMotorBL, DcMotor driveMotorBR){ // passing of individual motors in a constructor as an alternative to needing a robot class passed with the proper motor names
        localRuntime = new ElapsedTime();

        // setup motors from passed motors
        driveFL = driveMotorFL;
        driveFR = driveMotorFR;
        driveBL = driveMotorBL;
        driveBR = driveMotorBR;
    }

    // Secondary constructor that can be used to pass different speed divisor values
    public Drive_Mecanum_Tele(DcMotor driveMotorFL, DcMotor driveMotorFR, DcMotor driveMotorBL, DcMotor driveMotorBR, double turnSpeed, double translateSpeed, double boostingSpeed){ // passing of individual motors in a constructor as an alternative to needing a robot class passed with the proper motor names
        localRuntime = new ElapsedTime();

        // setup motors from passed motors
        driveFL = driveMotorFL;
        driveFR = driveMotorFR;
        driveBL = driveMotorBL;
        driveBR = driveMotorBR;

        // set custom values to the speed divisors
        turnMultiplier = turnSpeed;
        translateMultiplier = translateSpeed;
        boostingMultiplier = boostingSpeed;
    }


    // Drive functions

    public void drive_field_relative(double x, double y, double r, double rawHeading, boolean isBoosting) { // use with controller only - this drives relative to field
        // if using controller inputs, ensure you reverse y in the arguments because down on the stick is positive and up is negative, and we need that to be the opposite way

        // adjust the raw heading (between -180 and 180) to be reversed and between 0 and 359.9999... to make the math easier later
        double heading = rawHeading * -1;
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
        double field_x = (x * cos) - (y * sin);
        double field_y = (x * sin) + (y * cos);


        // if boosting is true, the robot will use the boostingMultiplier instead of translateMultiplier for speed setting
        if(isBoosting){ // if boosting, use the boosting speed
            field_x *= boostingMultiplier; // multiply the speeds by the boostingDivisor (what percentage of max speed you want to be at while boosting)
            field_y *= boostingMultiplier;
        }
        else{ // if moving regularly, use the regular translate speed
            field_x *= translateMultiplier; // multiply the speeds by the translateDivisor (what percentage of max speed you want to be at while moving normally)
            field_y *= translateMultiplier;
        }
        r *= turnMultiplier;  // also multiply the r value by the turn speed modifier, but do it outside of the if statement because it happens the same either way


        // do math to get powers relative to field in addition to the cartesian mecanum formula
        powerFL = (field_y + (r * turnMultiplier) + field_x);
        powerFR = (field_y - (r * turnMultiplier) - field_x);
        powerBL = (field_y + (r * turnMultiplier) - field_x);
        powerBR = (field_y - (r * turnMultiplier) + field_x);


        // Normalize the powers before we pass them into the motors (so that no power is outside of the range when passed in, preserving the intended slope)
        double largest_power = Math.max( Math.max( Math.abs(powerFL), Math.abs(powerFR)), Math.max(Math.abs(powerBL), Math.abs(powerBR)) ); // first find the largest of all the powers (get the max of the first two, max of the second two, then get the max of the two maxes)

        if(largest_power > 1.0){ // if the largest power value is greater than 1
            powerFL /= largest_power; // divide each power by the largest one
            powerFR /= largest_power; // resulting in the largest power being 1 (x/x = 1)
            powerBL /= largest_power; // and the rest scaled appropriately
            powerBR /= largest_power;
        }


        // set the motor powers based off of the math done previously - Make that robot go VROOOOM
        driveFL.setPower(powerFL);
        driveFR.setPower(powerFR);
        driveBL.setPower(powerBL);
        driveBR.setPower(powerBR);
    }

    public void drive_robot_relative(double x, double y, double r, boolean isBoosting) { // use with controller only - this drives relative to the robot
        drive_field_relative(x, y, r, 0, isBoosting); // pass values into the drive field relative function, but passing a heading of 0 (meaning it will end up acting robot relative)
    }

    public double calcTurnPIDPower(double targetHeading, double currentHeading){ // ACCEPTS RADIANS

        double error = targetHeading - currentHeading; // the error is the difference between where we want to be and where we are right now
        double timeDifference = localRuntime.milliseconds() - lastRuntime; // timeDifference is the time since the last runtime

        integral += error * timeDifference; // the integral is the sum of all error over time, and is used to push past unexpected resistance (as if the arm stays in a single position away from the set position for too long, it builds up over time and pushes past the resistance)
        // multiplied by the timeDifference to prevent wild variation in how much it is increase if cycle time increases/decreases for some reason
        double dError = ((error - lastError) / timeDifference); // the rate of change of the current error, this component creates a smooth approach to the set point

        double rotationPower = (Kp * error) + (Ki * integral) + (Kd * dError); // multiply each term by its coefficient, then add together to get the final power


        lastError = error; // update the last error to be the current error
        lastRuntime = localRuntime.milliseconds(); // update the last runtime to be the current runtime
        lastTargetHeading = targetHeading; //update the last target head to be the current target heading

        return -rotationPower;
    }
}

