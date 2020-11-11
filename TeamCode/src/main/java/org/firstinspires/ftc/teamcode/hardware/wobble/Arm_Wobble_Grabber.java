package org.firstinspires.ftc.teamcode.hardware.wobble;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Arm_Wobble_Grabber {
    private DcMotor armMotor;
    private Servo leftServo;
    private Servo rightServo;
    private ElapsedTime localRuntime;


    // constants for setting arm position
    private static final double IDLE_POSITION = 0.0; // target position for idling, in degrees (all degree values are relative to the starting position)
    private static final double UP_POSITION = 90.0;  // target position for going over the wall, in degrees
    private static final double GRAB_POSITION = 190.0;  // target position for grabbing, in degrees
    private static final double ENCODER_TICS_PER_REVOLUTION = 537.0; // the number of encoder tics measured per revolution of the motor used (see the manufacturer website for info)


    private static final double ENCODER_TICS_PER_DEGREE = 360 / ENCODER_TICS_PER_REVOLUTION;


    private static double Ki = 1;
    private static double Kp = 1;
    private static double Kd = 1;


    // constants for setting intake power
    private static final double INTAKE_POWER = 1.0; // the power to set to make the robot intake the wobble goal
    private static final double OUTTAKE_POWER = 0.0; // the power to set to the servo to make the robot outtake the wobble goal
    private static final double STOP_POWER = 0.5; // the power to set to the servo to make it stop moving

    private static final double MARGIN_OF_ERROR = 0.5; // margin of error for position setting


    private double lastError = 0.0;
    private double lastRuntime = 0.0;
    private double integral = 0.0;



    public Arm_Wobble_Grabber( DcMotor armMotor, Servo leftServo, Servo rightServo ){
        this.armMotor = armMotor;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.localRuntime = new ElapsedTime();
    }

    // sets the intake run direction for the grabber
    public void setIntakeDirection( int direction ){ // 0 for direction is no motion, 1 moves the intake wheels to intake, -1 moves the intake wheels to outake
            if(direction == 0){
                leftServo.setPosition(STOP_POWER);
                rightServo.setPosition(STOP_POWER);
            }
            if (direction == 1) {
                leftServo.setPosition(INTAKE_POWER);
                rightServo.setPosition(INTAKE_POWER);
            }
            if (direction == -1) {
                leftServo.setPosition(OUTTAKE_POWER);
                rightServo.setPosition(OUTTAKE_POWER);
            }
    }

    public boolean setArmPosition( double targetPosition ){
        double position = encoderTicsToDegrees( armMotor.getCurrentPosition() ); // set our current position to be the encoder position in tics converted to degrees

        double error = targetPosition - position; // the error is the difference between where we want to be and where we are right now
        double timeDifference = localRuntime.milliseconds() - lastRuntime; // timeDifference is the time since the last runtime

        integral += error * timeDifference; // the integral is the sum of all error over time, and is used to push past unexpected resistance (as if the arm stays in a single position away from the set position for too long, it builds up over time and pushes past the resistance)
                                            // multiplied by the timeDifference to prevent wild variation in how much it is increase if cycle time increases/decreases for some reason
        double dError = ((error - lastError) / timeDifference); // the rate of change of the current error, this component creates a smooth approach to the set point

        double motorPower = (Kp * error) + (Ki * integral) + (Kd * dError); // multiply each term by its coefficient, then add together to get the final power
        armMotor.setPower(motorPower); // the actually set the power


        lastError = error; // update the last error to be the current error
        lastRuntime = localRuntime.milliseconds(); // update the last runtime to be the current runtime

        return withinMarginOfError( targetPosition, position ); // returns true if close enough to the target to be within the margin of error
    }
    public boolean goToIdlePos(){ // sets the arm to go to the idle position, returns true when there (within the margin of error)
        return setArmPosition(IDLE_POSITION);
    }
    public boolean goToUpPos(){ // sets the arm to go to the up position, returns true when there (within the margin of error)
        return setArmPosition(UP_POSITION);
    }
    public boolean goToGrabPos(){ // sets the arm to go to the grab position, returns true when there (within the margin of error)
        return setArmPosition(GRAB_POSITION);
    }


    public double getArmPosition(){ // returns the current arm position in degrees
        return encoderTicsToDegrees( armMotor.getCurrentPosition() ); // get the current encoder tics position, then convert that to degrees
    }
    public double getArmError(){ // returns the last known error to the target position
        return lastError;
    }


    private static double encoderTicsToDegrees( double tics ){ // converts tics of the encoder to degrees of arm rotation (used for getting position from the arm)
        return tics * ENCODER_TICS_PER_DEGREE;// aka tics * (360 / encoder tics per revolution)
    }
    private static boolean withinMarginOfError( double targetPosition, double position ) { // returns true if the current position is within the margin of error of the target position
        return (Math.abs(targetPosition - position) < MARGIN_OF_ERROR);
    }


}