package org.firstinspires.ftc.teamcode.hardware.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;


public class Shooter_Ring_ServoFed {
    private DcMotor shooterMotor;
    private Servo feederServo;
    private ElapsedTime localRuntime;
    private Encoder shooterEncoder;

    private static final double SPIN_UP_TIME = 1500; // in milliseconds

    private double shooterRunPower = 1.00;
    private boolean firstSpinUp = false;
    private boolean spunUp = false;
    private double spinUpEndTime = 0;


    private static final boolean USING_PID = true;
    private static final double Kp = 2.5;
    private static final double Ki = 0.00;
    private static final double Kd = 0.00;
    private double lastRuntime;
    private double integral;
    private double lastError;
    private double lastTargetSpeed;

    private static final double FEEDER_EXTENDED_POSITION = 0.52;
    private static final double FEEDER_RETRACTED_POSITION = 0.38;
    private static final double FEEDER_EXTENSION_TIME = 200; // in milliseconds

    private static final double VELOCITY_TICS_PER_MOTOR_POWER = 2350;


    private boolean isFiring = false;
    private int firingState = 0;
    private double moveStartTime = 0;


    public Shooter_Ring_ServoFed( DcMotor shooterMotor, Servo feederServo){
        this.shooterMotor = shooterMotor;
        this.feederServo = feederServo;
        localRuntime = new ElapsedTime();

        shooterEncoder = new Encoder((DcMotorEx)shooterMotor); // setup the encoder, even if we aren't using it directly
    }

    public boolean spinUp(){
        if(USING_PID){
            lastTargetSpeed = getPIDPower( shooterRunPower );
        }
        else {
            lastTargetSpeed = shooterRunPower;
        }
        shooterMotor.setPower( lastTargetSpeed );


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

    public void setTargetShooterPower(double targetShooterPower){
        shooterRunPower = targetShooterPower;
    }
    public double getLastTargetSpeed() {return lastTargetSpeed;}
    public double getFlywheelVelo(){
        return shooterEncoder.getCorrectedVelocity();
    }
    private double getPIDPower( double targetSpeed ){ // gets the power needed to reach the target velocity based on our current velocity
        double speed = encoderVeloToMotorSpeed( shooterEncoder.getCorrectedVelocity() ); // convert from encoder tics velocity to a -1 to 1 scale
        targetSpeed *= 0.8; // scale the target speed so that we have headroom

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
    public boolean isFiring() {return isFiring;}
}
