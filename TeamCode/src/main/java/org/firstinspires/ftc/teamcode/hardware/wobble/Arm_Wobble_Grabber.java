package org.firstinspires.ftc.teamcode.hardware.wobble;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



public class Arm_Wobble_Grabber {
    DcMotor armMotor;
    Servo leftServo;
    Servo rightServo;

    private static final double INTAKE_POWER = 1.0;
    private static final double OUTTAKE_POWER = 0.0;
    private static final double STOP_POWER = 0.5;


    Arm_Wobble_Grabber( DcMotor armMotor, Servo leftServo, Servo rightServo ){
        this.armMotor = armMotor;
        this.leftServo = leftServo;
        this.rightServo = rightServo;
    }

    // sets the intake run direction for the grabber
    public void setIntakeDirection( int direction ){ // 0 for direction is no motion, positive moves the intake wheels to intake, negative moves the intake wheels to outake
            if(direction==0){
                leftServo.setPosition(STOP_POWER);
                rightServo.setPosition(STOP_POWER);
            }
            if (direction==1) {
                leftServo.setPosition(INTAKE_POWER);
                rightServo.setPosition(INTAKE_POWER);
            }
            if (direction==-1) {
                leftServo.setPosition(OUTTAKE_POWER);
                rightServo.setPosition(OUTTAKE_POWER);
            }


    }

    public boolean setArmPosition( double position ){
        boolean atPosition = false;



        return atPosition;
    }

    // rohit, write the functions here that pass the arguments etc into the set arm position, i lazyyyy


}
