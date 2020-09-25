package org.firstinspires.ftc.teamcode.hardware.drive;

import com.qualcomm.robotcore.hardware.DcMotor;


public class Drive_Mecanum_Tele {

    // Create and initialize speed modifier variables - using default values (then can be set via a constructor)
    private double turnMultiplier = DEFAULT_TURN_MULTIPLIER; // what percentage of maximum turning speed should be used as a base turning speed (50% = 0.5, etc) - a multiplier
    private double translateMultiplier = DEFAULT_TRANSLATE_MULTIPLIER; // what percentage of maximum translational speed should be used as a base translational speed (50% = 0.5, etc) - a multiplier
    private double boostingMultiplier  = DEFAULT_BOOSTING_MULTIPLIER; // what percentage of maximum translational speed should be used as a boost translational speed (50% = 0.5, etc) - a multiplier

    // Default speed modifier values
    private static final double DEFAULT_TURN_MULTIPLIER = 0.6; // default values to use in the event no custom values are passed
    private static final double DEFAULT_TRANSLATE_MULTIPLIER = 0.6;
    private static final double DEFAULT_BOOSTING_MULTIPLIER = 1.0;


    //Motor variables
    private DcMotor driveFL, driveFR, driveBL, driveBR; // motors that are being used for mecanum driving
    private double powerFL, powerFR, powerBL, powerBR; // powers that are being passed to those motors


    // Default constructor
    public Drive_Mecanum_Tele(DcMotor driveMotorFL, DcMotor driveMotorFR, DcMotor driveMotorBL, DcMotor driveMotorBR){ // passing of individual motors in a constructor as an alternative to needing a robot class passed with the proper motor names
        // setup motors from passed motors
        driveFL = driveMotorFL;
        driveFR = driveMotorFR;
        driveBL = driveMotorBL;
        driveBR = driveMotorBR;
    }

    // Secondary constructor that can be used to pass different speed divisor values
    public Drive_Mecanum_Tele(DcMotor driveMotorFL, DcMotor driveMotorFR, DcMotor driveMotorBL, DcMotor driveMotorBR, double turnSpeed, double translateSpeed, double boostingSpeed){ // passing of individual motors in a constructor as an alternative to needing a robot class passed with the proper motor names
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

        // Unit Vector Normalization - Normalizes the translational inputs (ensure that all motor values are between -1 and 1, while maintaining the ratio between inputs)
        double magnitude = Math.abs(x) + Math.abs(y); // get the total magnitude of the inputs by adding their absolute values

        if(magnitude > 1.0){  // if the magnitude is over the max motor power, divide all numbers by the largest number (meaning the largest number will become 1 and the rest scaled appropriately)
            x /= magnitude;
            y /= magnitude;
        }
        

        // Set up heading factor for relative to field (convert the heading to radians, then get the sine and cosine of that radian heading
        double sin = Math.sin(Math.toRadians(heading));
        double cos = Math.cos(Math.toRadians(heading));

        // do math to adjust to make the input drive vector relative to field (rather than relative to robot)
        double field_x = (x * cos) - (y * sin);
        double field_y = (x * sin) + (y * cos);


        // if boosting is true, the robot will use the boostingMultiplier instead of translateMultiplier for speed setting
        if(isBoosting){ // if boosting, use the boosting speed
            field_x = field_x * boostingMultiplier; // multiply the speeds by the boostingDivisor (what percentage of max speed you want to be at while boosting)
            field_y = field_y * boostingMultiplier;
        }
        else{ // if moving regularly, use the regular translate speed
            field_x = field_x * translateMultiplier; // multiply the speeds by the translateDivisor (what percentage of max speed you want to be at while moving normally)
            field_y = field_y * translateMultiplier;
        }


        // do math to get powers relative to field in addition to the cartesian mecanum formula
        powerFL = (field_y + (r * turnMultiplier) + field_x);
        powerFR = (field_y - (r * turnMultiplier) - field_x);
        powerBL = (field_y + (r * turnMultiplier) - field_x);
        powerBR = (field_y - (r * turnMultiplier) + field_x);


        // set the motor powers based off of the math done previously
        driveFL.setPower(powerFL);
        driveFR.setPower(powerFR);
        driveBL.setPower(powerBL);
        driveBR.setPower(powerBR);
    }

    public void drive_robot_relative(double x, double y, double r, boolean isBoosting) { // use with controller only - this drives relative to the robot
        drive_field_relative(x, y, r, 0, isBoosting); // pass values into the drive field relative function, but passing a heading of 0 (meaning it will end up acting robot relative)
    }

}

