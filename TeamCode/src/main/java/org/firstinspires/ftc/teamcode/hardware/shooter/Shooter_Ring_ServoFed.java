package org.firstinspires.ftc.teamcode.hardware.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



public class Shooter_Ring_ServoFed {
    DcMotor shooterMotor;
    Servo feederServo;

    Shooter_Ring_ServoFed( DcMotor shooterMotor, Servo feederServo){
        this.shooterMotor = shooterMotor;
        this.feederServo = feederServo;
    }


    public void spinUp(){

    }
    public void spinDown(){

    }
    public void setFlywheelMode( boolean isRunning ){

    }

    public void dropIntake(){

    }

}
