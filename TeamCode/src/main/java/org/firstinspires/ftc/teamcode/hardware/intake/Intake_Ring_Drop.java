package org.firstinspires.ftc.teamcode.hardware.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



public class Intake_Ring_Drop {
    private DcMotor intakeMotor;
    private Servo gateServo;


    public static final double DEFAULT_INTAKE_RUN_SPEED = 1.0;

    private double intakeRunSpeed = DEFAULT_INTAKE_RUN_SPEED;
    private double currentIntakePower = 0.0;


    private static double GATE_UP_POSITION   = degToServoPos(70.0);
    private static double GATE_PREP_POSITION = degToServoPos(120.0);
    private static double GATE_DOWN_POSITION = degToServoPos(135.0);
    private String gatePosition = "UP";


    public Intake_Ring_Drop( DcMotor intakeMotor, Servo gateServo){
        this.intakeMotor = intakeMotor;
        this.gateServo = gateServo;
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


    public void raiseGate(){
        gateServo.setPosition(GATE_UP_POSITION);
        gatePosition = "UP";
    }
    public void prepGate(){
        gateServo.setPosition(GATE_PREP_POSITION);
        gatePosition = "PREP";
    }
    public void lowerGate(){
        gateServo.setPosition(GATE_DOWN_POSITION);
        gatePosition = "DOWN";
    }
    public String getGatePosition(){
        return gatePosition;
    }


    private static double servoPosToDeg(double servoPos){
        return servoPos * 180;
    }
    private static double degToServoPos(double degrees){
        return degrees / 180;
    }
}
