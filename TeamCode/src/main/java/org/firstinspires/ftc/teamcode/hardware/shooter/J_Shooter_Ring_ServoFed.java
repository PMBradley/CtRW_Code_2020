package org.firstinspires.ftc.teamcode.hardware.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;


public class J_Shooter_Ring_ServoFed {
    private DcMotor shooterMotorFront;
    private DcMotor shooterMotorBack;
    private Servo feederServo;
    private Servo indexerServo;
    private Servo anglerServo;
    private ElapsedTime localRuntime;
    private Encoder shooterEncoder;

    private static final double SPIN_UP_TIME = 1200; // in milliseconds

    private static final boolean USING_PID = true;
    private static final double Kp = 2.5;
    private static final double Ki = 0.00;
    private static final double Kd = 0.00;
    private double lastRuntime;
    private double integral;
    private double lastError;
    private double lastTargetSpeed;

    private double shooterRunSpeed; // the variable that holds the current set shooter speed
    private double shooterShootSpeed; // the variable

    public static final double SHOOTER_SPEED     = .65; // the power the shooter uses as a default for no PID mode
    public static final double SHOOTER_PID_HIGHGOAL_SPEED = 0.75; // the power the shooter uses as a default for PID mode
    public static final double SHOOTER_PID_POWERSHOT_SPEED = 0.65; // the power the shooter uses as a default for PID mode
    public static final double SHOOTER_PID_LONGGOAL_SPEED = 1.0; // the power the shooter uses as a default for PID mode
    private boolean firstSpinUp = true;
    private boolean spunUp = false;
    private double spinUpEndTime = 0;

    private boolean firstIndex = true;
    private boolean indexUp = false;
    private double indexEndTime = 0;

    private double anglerPos = 0;

    private static final double FEEDER_RETRACTED_POSITION = degToServoPos(86.0);// the feeder servo extened position
    private static final double FEEDER_EXTENDED_POSITION = degToServoPos(135.0);
    private static final double FEEDER_EXTENSION_TIME = 130; // in milliseconds

    private static final double INDEXER_DOWN_POSITION = degToServoPos(134.0); // the little ring lifter down position
    private static final double INDEXER_UP_POSITION = degToServoPos(0.0);
    private static final double INDEXER_MOVE_TIME = 300; // in milliseconds

    private static final double ANGLER_POWERSHOT_POSITION = degToServoPos(114.0); // the trajectory angler down position
    private static final double ANGLER_HIGHGOAL_POSITION = degToServoPos(110.0);
    private static final double ANGLER_LONGGOAL_POSITION = degToServoPos( 110.0);

    private static final double VELOCITY_TICS_PER_MOTOR_POWER = 2598.4;

    private boolean isFiring = false;
    private int firingState = 0;
    private double moveStartTime = 0;


    public J_Shooter_Ring_ServoFed(DcMotor shooterMotorFront, DcMotor shooterMotorBack, Servo feederServo, Servo indexerServo, Servo anglerServo){

        if(USING_PID){ // if using PID
            shooterEncoder = new Encoder((DcMotorEx)shooterMotorBack); // setup the encoder
            shooterRunSpeed = SHOOTER_PID_HIGHGOAL_SPEED;  // and set the base runspeed to the shooter PID speed
            shooterShootSpeed = SHOOTER_PID_HIGHGOAL_SPEED;
        }
        else {
            shooterRunSpeed = SHOOTER_SPEED;
        }

        this.shooterMotorFront = shooterMotorFront;
        this.shooterMotorBack = shooterMotorBack;
        this.feederServo = feederServo;
        this.indexerServo = indexerServo;
        this.anglerServo = anglerServo;
        localRuntime = new ElapsedTime();
    }


    public boolean spinUp(){
        if(USING_PID){
            double motorPower = getPIDPower( shooterRunSpeed ); // calling this only once to mess with timing things less
            shooterMotorFront.setPower(motorPower);
            shooterMotorBack.setPower(motorPower);
        }
        else{
            shooterMotorFront.setPower( shooterRunSpeed );
            shooterMotorBack.setPower( shooterRunSpeed );
        }


        if (firstSpinUp){
            spinUpEndTime = localRuntime.milliseconds() + SPIN_UP_TIME;
            firstSpinUp = false;
        }

        if(USING_PID && Math.abs(encoderVeloToMotorSpeed(getFlywheelVelo()) - shooterRunSpeed) < 0.025){ // say the motor is spun up if within 0.05 of the target speed
            spunUp = true;
        }
        else if(localRuntime.milliseconds() >= spinUpEndTime){
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
    public void setTargetShooterSpeed(double targetShooterPower){
        shooterRunSpeed = targetShooterPower;
    }
    public double getTargetShooterShootingSpeed() {
        return shooterShootSpeed;
    }
    public double getTargetShootingSpeed() {
        if(USING_PID){
            return shooterRunSpeed * 0.8; // because shooter power is scaled for the PID
        }
        else {
            return shooterRunSpeed;
        }
    }
    public void optimizeForHighgoal(){
        angleUp();
        shooterShootSpeed = SHOOTER_PID_HIGHGOAL_SPEED;
    }
    public void optimizeForPowershots(){
        angleDown();
        shooterShootSpeed = SHOOTER_PID_POWERSHOT_SPEED;
    }
    public void optimizeForLonggoal(){
        setAnglerServoDegrees(ANGLER_LONGGOAL_POSITION);
        shooterShootSpeed = SHOOTER_PID_LONGGOAL_SPEED;
    }



    public double getFlywheelVelo(){
        return shooterEncoder.getCorrectedVelocity();
    }
    private double getPIDPower(double targetSpeed){ // gets the power needed to reach the target velocity based on our current velocity
        double speed = encoderVeloToMotorSpeed( shooterEncoder.getCorrectedVelocity() ); // convert from encoder tics velocity to a -1 to 1 scale

        double error = targetSpeed - speed; // the error is the difference between where we want to be and where we are right now
        double timeDifference = localRuntime.milliseconds() - lastRuntime; // timeDifference is the time since the last runtime

        integral += error * timeDifference; // the integral is the sum of all error over time, and is used to push past unexpected resistance (as if the arm stays in a single position away from the set position for too long, it builds up over time and pushes past the resistance)
        // multiplied by the timeDifference to prevent wild variation in how much it is increase if cycle time increases/decreases for some reason
        double dError = ((error - lastError) / timeDifference); // the rate of change of the current error, this component creates a smooth approach to the set point

        double speedChange = (Kp * error) + (Ki * integral) + (Kd * dError); // multiply each term by its coefficient, then add together to get the final power


        lastError = error; // update the last error to be the current error
        lastRuntime = localRuntime.milliseconds(); // update the last runtime to be the current runtime
        lastTargetSpeed = targetSpeed; //update the last target speed to be the current target position

        return speed + speedChange; // we return the speed change (PID output) plus the current speed because the PID is outputting a rate of change for speed, to reach target speed (just as you would have a rate of change of position to reach a target position)
    }
    public static double encoderVeloToMotorSpeed(double encoderVelo){
        return encoderVelo / VELOCITY_TICS_PER_MOTOR_POWER; // correct this with some conversion rate multiplier
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
        anglerPos = ANGLER_HIGHGOAL_POSITION;
        anglerServo.setPosition( anglerPos );
    }
    public void angleDown(){
        anglerPos = ANGLER_POWERSHOT_POSITION;
        anglerServo.setPosition( anglerPos );
    }
    public void setAnglerServoDegrees(double servoDegrees){
        anglerPos = servoDegrees;
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



