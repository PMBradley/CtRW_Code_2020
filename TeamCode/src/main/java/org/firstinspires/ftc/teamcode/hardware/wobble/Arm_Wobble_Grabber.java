package org.firstinspires.ftc.teamcode.hardware.wobble;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class Arm_Wobble_Grabber {
    private DcMotor armMotor;
    private Servo leftServo;
    private Servo rightServo;
    private ElapsedTime localRuntime;


    // constants for setting arm position
    private static final double IDLE_POSITION = 0.0; // target position for idling, in degrees (all degree values are relative to the starting position)
    private static final double UP_POSITION = 20.0;  // target position for going over the wall, in degrees
    private static final double GRAB_POSITION = 94.0;  // target position for grabbing, in degrees
    private static final double ENCODER_TICS_PER_REVOLUTION = 696.5; // the number of encoder tics measured per revolution of the motor used (see the manufacturer website for info)


    private static final double ENCODER_TICS_PER_DEGREE = 360 / ENCODER_TICS_PER_REVOLUTION;


    public static final double Kp = 0.007;
    public static final double Ki = 0.000001;
    public static final double Kd = 0.00001;


    // constants for setting intake power
    public static final double INTAKE_POWER = 1.0; // the power to set to make the robot intake the wobble goal
    public static final double OUTTAKE_POWER = 0.0; // the power to set to the servo to make the robot outtake the wobble goal
    public static final double STOP_POWER = 0.5; // the power to set to the servo to make it stop moving

    private static final double MARGIN_OF_ERROR = 0.5; // margin of error for position setting

    private double gearRatio = 1.0;

    private double lastError = 0.0;
    private double lastRuntime = 0.0;
    private double integral = 0.0;
    private double lastTargetPosition = 0.0;


    public Arm_Wobble_Grabber(DcMotor armMotor, Servo leftServo, Servo rightServo, double gearRatio){
        this.armMotor = armMotor;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
        this.localRuntime = new ElapsedTime();
        this.gearRatio = gearRatio;
    }
    public Arm_Wobble_Grabber(DcMotor armMotor, Servo leftServo, Servo rightServo){
        this(armMotor, leftServo, rightServo, 1);
    }

    // sets the intake run direction for the grabber
    public void setIntakeDirection( int direction ){ // 0 for direction is no motion, 1 moves the intake wheels to intake, -1 moves the intake wheels to outake
            if(direction == 0){
                leftServo.setPosition(STOP_POWER);
                rightServo.setPosition(STOP_POWER);
               // rightServo.setPosition(leftToRight(STOP_POWER));
            }
            if (direction == 1) {
                leftServo.setPosition(INTAKE_POWER);
                rightServo.setPosition(INTAKE_POWER);
                //rightServo.setPosition(leftToRight(INTAKE_POWER));
            }
            if (direction == -1) {
                leftServo.setPosition(OUTTAKE_POWER);
                rightServo.setPosition(OUTTAKE_POWER);
                //rightServo.setPosition(leftToRight(OUTTAKE_POWER));
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
        lastTargetPosition = targetPosition; //update the last target position to be the current target position

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


    public double getArmTargetPosition(){
        return lastTargetPosition;
    }
    public double getArmPosition(){ // returns the current arm position in degrees
        return encoderTicsToDegrees( armMotor.getCurrentPosition() ); // get the current encoder tics position, then convert that to degrees
    }
    public double getArmError(){ // returns the last known error to the target position
        return lastError;
    }


    private double encoderTicsToDegrees( double tics ){ // converts tics of the encoder to degrees of arm rotation (used for getting position from the arm)
        return tics * ENCODER_TICS_PER_DEGREE * gearRatio;// aka tics * (360 / encoder tics per revolution) * gear ratio
    }
    private static boolean withinMarginOfError( double targetPosition, double position ) { // returns true if the current position is within the margin of error of the target position
        return (Math.abs(targetPosition - position) < MARGIN_OF_ERROR);
    }


    private double leftToRight(double input)
    {
        return ((-1*(input-0.5)) + 0.5);
    }

}
