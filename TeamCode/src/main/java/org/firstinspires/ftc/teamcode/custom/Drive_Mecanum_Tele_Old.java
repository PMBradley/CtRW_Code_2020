package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.robotcore.hardware.DcMotor;


public class Drive_Mecanum_Tele_Old {

    // Speed modifier variables
    private double turnDivisor = 0.5;
    private double speedDivisor = 0.40;


    public Drive_Mecanum_Tele_Old(DcMotor driveMotorFL, DcMotor driveMotorFR, DcMotor driveMotorBL, DcMotor driveMotorBR){ // passing of individual motors in a constructor as an alternative to needing a robot class passed with the proper motor names
        driveFL = driveMotorFL;
        driveFR = driveMotorFR;
        driveBL = driveMotorBL;
        driveBR = driveMotorBR;
    }

    //Variables
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private double rotateSpeedFactor = 10;
    private double correctionPower = 0.5;

    boolean isInitilized = false;

    private DcMotor driveFL = null;
    private DcMotor driveFR = null;
    private DcMotor driveBL = null;
    private DcMotor driveBR = null;

    public double powerFL = 0;
    public double powerFR = 0;
    public double powerBL = 0;
    public double powerBR = 0;


    public void init_motors(){ // a function to init motor encoders and can also be used to hard reset the motor encoders
        isInitilized = true;

        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void drive_field_relative(double x, double y, double r, double rawHeading, boolean limiter, double boostFactor) { // use with controller only - this drives relative to field
        y = -y; // reverse y because down on the stick is positive and up is negative, and we need that to be opposite

        if (boostFactor < .5 && limiter == true) { // if the value is not high enough for full speed and the limiter is enabled, reduce the x and y speeds
            x = x * speedDivisor;
            y = y * speedDivisor;
        }

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

        // do math to get powers relative to field in addition to the mecanum formula
        double sin = Math.sin(heading * 0.0174533);
        double cos = Math.cos(heading * 0.0174533);

        double forward = (x * sin) + (y * cos);
        double right = (x * cos) - (y * sin);

        powerFL = (forward + (r * turnDivisor) + right);
        powerFR = -(forward - (r * turnDivisor) - right);
        powerBL = (forward + (r * turnDivisor) - right);
        powerBR = -(forward - (r * turnDivisor) + right);

        // set the motor powers based off of the math done previously
        driveFL.setPower(powerFL);
        driveFR.setPower(powerFR);
        driveBL.setPower(powerBL);
        driveBR.setPower(powerBR);
    }

    public void drive_robot_relative(double x, double y, double r, boolean limiter, double boostFactor) { // use with controller only - this drives relative to the robot
        y = -y; // reverse y because down on the stick is positive and up is negative, and we need that to be opposite

        if (boostFactor < .5 && limiter == true) { // if the value is not high enough for full speed and the limiter is enabled, reduce the x and y speeds
            x = x * speedDivisor;
            y = y * speedDivisor;
        }

        // do math to get powers set according to the mecanum formula
        double forward = y;
        double right = x;

        powerFL = (forward + (r * turnDivisor) + right);
        powerFR = -(forward - (r * turnDivisor) - right);
        powerBL = (forward + (r * turnDivisor) - right);
        powerBR = -(forward - (r * turnDivisor) + right);

        // set the motor powers based off of the math done previously
        driveFL.setPower(powerFL);
        driveFR.setPower(powerFR);
        driveBL.setPower(powerBL);
        driveBR.setPower(powerBR);
    }


    public boolean gyroTurn(double targetHeading, double currentHeading, double turnTolerance) { // turns the robot to a certain heading, within a certain tolerance. Returns true when within tolerance of the target.
        //Gyro turn code
        //double targetHeading  - 0 to 360
        //double currentHeading - 0 to 360
        //double turnDirection  - -1 to 1
        boolean turnComplete = false;

        if(Math.abs(currentHeading - targetHeading) <= turnTolerance) {
            turnComplete = true;
        }

        return turnComplete;
    }

}
