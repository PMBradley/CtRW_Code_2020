package org.firstinspires.ftc.teamcode.hardware.wobble;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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


    private double lastError = 0;
    private double lastRuntime = 0;
    private double integral = 0;



    Arm_Wobble_Grabber( DcMotor armMotor, Servo leftServo, Servo rightServo ){
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

        double position = 0;
        position = encoderTicsToDegrees( armMotor.getCurrentPosition() );

        double error = targetPosition - position;
        double timeDifference = localRuntime.milliseconds() - lastRuntime;
        integral += error * timeDifference;

        double motorPower = (Kp * error) + (Ki * integral) + (Kd * ((error - lastError)/timeDifference) );
        armMotor.setPower(motorPower);


        lastError = error;
        lastRuntime = localRuntime.milliseconds();


        return withinMarginOfError( position, targetPosition );
    }
    public boolean goToIdlePos(){ // sets the arm to go to the idle position, returns true when there (within the margin of error
        return setArmPosition(IDLE_POSITION);
    }
    public boolean goToUpPos(){ // sets the arm to go to the up position, returns true when there (within the margin of error
        return setArmPosition(UP_POSITION);
    }
    public boolean goToGrabPos(){ // sets the arm to go to the grab position, returns true when there (within the margin of error
        return setArmPosition(GRAB_POSITION);
    }

    private static double encoderTicsToDegrees(double tics){ // converts tics of the encoder to degrees of arm rotation (used for getting position from the arm)
        return tics * ENCODER_TICS_PER_DEGREE;// aka tics * 360 / encoder tics per revolution
    }

    private static boolean withinMarginOfError(double position, double targetPosition) { // returns true if the current position is within the margin of error of the target position
        return (Math.abs(targetPosition - position) < MARGIN_OF_ERROR);
    }



}
