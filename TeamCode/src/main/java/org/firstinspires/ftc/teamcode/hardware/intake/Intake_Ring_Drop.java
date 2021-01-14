package org.firstinspires.ftc.teamcode.hardware.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



public class Intake_Ring_Drop {
    private DcMotor intakeMotor;
    private Servo lockServo;

    public static final double DEFAULT_INTAKE_RUN_SPEED = 1.0;
    private static final double UNLOCK_POSITION = 1.0;

    private double intakeRunSpeed = DEFAULT_INTAKE_RUN_SPEED;
    private double currentIntakePower = 0.0;

    public Intake_Ring_Drop( DcMotor intakeMotor, Servo lockServo){
        this.intakeMotor = intakeMotor;
        this.lockServo = lockServo;
    }


    public void spinUp(){ // sets the motor to run at the spinning speed (will continue to run at that speed until set otherwise
        intakeMotor.setPower(intakeRunSpeed);
        currentIntakePower = intakeRunSpeed;
    }
    public void spinDown(){
        intakeMotor.setPower(0.0);
        currentIntakePower = 0.0;
    }

    public void setRunning( boolean isRunning ){
        if (isRunning == true) {
            spinUp();
        }
        else {
            spinDown();
        }
    }

    public void setIntakeRunSpeed(double newSpeed){
        intakeRunSpeed = newSpeed;
    }

    public double getCurrentIntakePower(){return currentIntakePower;}

}
