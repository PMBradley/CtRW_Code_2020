package org.firstinspires.ftc.teamcode.hardware.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Shooter_Ring_ServoFed {
    private DcMotor shooterMotor;
    private Servo feederServo;
    private ElapsedTime localRuntime;

    private static final double SHOOTER_RUN_POWER = 1.0;
    private static final double SPIN_UP_TIME = 1000; // in milliseconds

    private boolean firstSpinUp = false;
    private boolean spunUp = false;
    private double spinUpEndTime = 0;


    private static final double FEEDER_EXTENDED_POSITION = 0.74;
    private static final double FEEDER_RETRACTED_POSITION = 0.55;
    private static final double FEEDER_EXTENSION_TIME = 650; // in milliseconds

    private boolean isFiring = false;
    private int firingState = 0;
    private double moveStartTime = 0;


    public Shooter_Ring_ServoFed( DcMotor shooterMotor, Servo feederServo){
        this.shooterMotor = shooterMotor;
        this.feederServo = feederServo;
        localRuntime = new ElapsedTime();
    }


    public boolean spinUp(){
        shooterMotor.setPower( SHOOTER_RUN_POWER );

        if (firstSpinUp){
            spinUpEndTime = localRuntime.milliseconds() + SPIN_UP_TIME;
            firstSpinUp = false;
        }
        if(localRuntime.milliseconds() >= spinUpEndTime){
            spunUp = true;
        }

        return spunUp;
    }
    public void spinDown(){
        shooterMotor.setPower( 0.0 );

        spunUp = false;
        firstSpinUp = true;
    }
    public void setFlywheelMode( boolean isRunning ){
        if(!isFiring){
            if( isRunning ){
                spinUp();
            }
            else {
                spinDown();
            }
        }
    }


    public void instructFire(){
        isFiring = true;
    }

    public void updateFeeder(){
        if( isFiring ){

            switch (firingState){
                case 0:
                    spinUp();
                    if(isSpunUp()){
                        moveStartTime = localRuntime.milliseconds();
                        firingState = 1;
                    }

                    break;
                case 1:
                    feederServo.setPosition( FEEDER_EXTENDED_POSITION );

                    if(localRuntime.milliseconds() >= moveStartTime + FEEDER_EXTENSION_TIME){
                        firingState = 2;
                    }
                    break;
                case 2:
                    feederServo.setPosition( FEEDER_RETRACTED_POSITION );

                    if(localRuntime.milliseconds() >= moveStartTime + (2*FEEDER_EXTENSION_TIME)){
                        firingState = 0;
                        isFiring = false;
                    }
                    break;
            } // end of case switch
        } // end of if
    } // end of funtion

    public int getFiringState(){
        return firingState;
    }
    public boolean isSpunUp(){
        return spunUp;
    }
}
