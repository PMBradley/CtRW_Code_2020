package org.firstinspires.ftc.teamcode.hardware.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



public class Intake_Ring_Drop {
    DcMotor intakeMotor;
    Servo lockServo;

    Intake_Ring_Drop( DcMotor intakeMotor, Servo lockServo){
        this.intakeMotor = intakeMotor;
        this.lockServo = lockServo;
    }


    public void spinUp(){
        intakeMotor.setPower(1);
    }

    public void spinDown(){
        intakeMotor.setPower(0);
    }

    public void setRunning( boolean isRunning ){
        if (isRunning == true) {
            spinUp();
        } else  {
            spinDown();
        }
    }

    public void dropIntake(){
        lockServo.setPosition(1);
    }


}
