package org.firstinspires.ftc.teamcode.hardware.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class J_Shooter_Ring_ServoFed {
    private DcMotor shooterMotorFront;
    private DcMotor shooterMotorBack;
    private Servo feederServo;
    private Servo indexerServo;
    private Servo anglerServo;
    private ElapsedTime localRuntime;

    private static final double SHOOTER_RUN_POWER = 1.0;
    private static final double SPIN_UP_TIME = 1000; // in milliseconds

    private boolean firstSpinUp = true;
    private boolean spunUp = false;
    private double spinUpEndTime = 0;

    private boolean firstIndex = true;
    private boolean indexUp = false;
    private double indexEndTime = 0;

    private double anglerPos = 0;

    private static final double FEEDER_RETRACTED_POSITION = degToServoPos(99.0);// the feeder servo extened position
    private static final double FEEDER_EXTENDED_POSITION = degToServoPos(136.8);
    private static final double FEEDER_EXTENSION_TIME = 650; // in milliseconds

    private static final double INDEXER_DOWN_POSITION = degToServoPos(0.0); // the little ring lifter down position
    private static final double INDEXER_UP_POSITION = degToServoPos(125.0);
    private static final double INDEXER_MOVE_TIME = 300; // in milliseconds

    private static final double ANGLER_DOWN_POSITION = degToServoPos(90.0); // the trajectory angler down position
    private static final double ANGLER_UP_POSITION = degToServoPos(170.0);

    private boolean isFiring = false;
    private int firingState = 0;
    private double moveStartTime = 0;


    public J_Shooter_Ring_ServoFed(DcMotor shooterMotorFront, DcMotor shooterMotorBack, Servo feederServo, Servo indexerServo, Servo anglerServo){
        //shooterMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // bad programming practice, do not do this, should set running using encoder mode in provider class

        this.shooterMotorFront = shooterMotorFront;
        this.shooterMotorBack = shooterMotorBack;
        this.feederServo = feederServo;
        this.indexerServo = indexerServo;
        this.anglerServo = anglerServo;
        localRuntime = new ElapsedTime();
    }


    public boolean spinUp(){
        shooterMotorFront.setPower( SHOOTER_RUN_POWER );
        //shooterMotorBack.setPower( SHOOTER_RUN_POWER );
        shooterMotorBack.setPower(shooterMotorFront.getPower());

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
        shooterMotorFront.setPower( 0.0 );
        shooterMotorBack.setPower( 0.0 );

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

    public boolean indexerUp(){
        indexerServo.setPosition( INDEXER_UP_POSITION );

        if (firstIndex){
            indexEndTime = localRuntime.milliseconds() + INDEXER_MOVE_TIME;
            firstIndex = false;
        }
        if(localRuntime.milliseconds() >= indexEndTime){
            indexUp = true;
        }

        return indexUp;
    }
    public void indexerDown(){
        indexerServo.setPosition( INDEXER_DOWN_POSITION );

        indexUp = false;
        firstIndex = true;
    }
    public void setIndexerMode( boolean isRunning ){
        if(!isFiring){
            if( isRunning ){
                indexerUp();
            }
            else {
                indexerDown();
            }
        }
    }

    public void angleUp(){
        anglerPos = ANGLER_UP_POSITION;
        anglerServo.setPosition( anglerPos );
    }
    public void angleDown(){
        anglerPos = ANGLER_DOWN_POSITION;
        anglerServo.setPosition( anglerPos );
    }
    public void setAnglerServoDegrees(double servoDegrees){
        anglerPos = degToServoPos( servoDegrees );
        anglerServo.setPosition( anglerPos );
    }

    public void instructFire(){
        isFiring = true;
    }

    public void updateFeeder(){
        if( isFiring ){

            switch (firingState){
                case 0:
                    spinUp(); // spin up shooter
                    indexerUp(); // move indexer up

                    if(isSpunUp() && isIndexerUp()){
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
    public boolean isIndexerUp(){return indexUp;}
    public boolean isFiring(){return isFiring;}
    public double getAnglerPos(){
        return servoPosToDeg(anglerPos);
    }

    private static double servoPosToDeg(double servoPos){
        return servoPos * 180;
    }
    private static double degToServoPos(double degrees){
        return degrees / 180;
    }
}



