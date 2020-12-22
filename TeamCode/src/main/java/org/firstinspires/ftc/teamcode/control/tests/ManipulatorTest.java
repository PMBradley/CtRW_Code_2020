package org.firstinspires.ftc.teamcode.control.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.templates.Provider20XX;
import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Tele;
import org.firstinspires.ftc.teamcode.hardware.intake.Intake_Ring_Drop;
import org.firstinspires.ftc.teamcode.hardware.shooter.Shooter_Ring_ServoFed;
import org.firstinspires.ftc.teamcode.hardware.wobble.Arm_Wobble_Grabber;
import org.firstinspires.ftc.teamcode.util.Encoder;


/*
    Control scheme:
        Shooter fire - Gamepad 2 x
        Shooter spin up toggle -

 */



@TeleOp(name = "Manipulator Test (shooter/intake/wobble)", group = "@@T")


public class ManipulatorTest extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "Molly";

    // Robot Speed variables
    double turnSpeed = 0.5; // Speed multiplier for turning (1 being 100% of power going in)
    double translateSpeed = 0.4; // Speed multiplier for translation (1 being 100% of power going in)
    double boostSpeed = 1; // Speed multiplier for BOOSTING (1 being 100% of power going in)
    double stopSpeed = 0;
    double testSpeed = 1.0;
    private static final double SHOOTER_HIGH_SPEED = 0.99;
    private static final double SHOOTER_LOW_SPEED = 0.80;

    boolean firstSpinUpToggle = true;
    boolean isSpinningUp = false;
    boolean firstAngleToggle = true;
    boolean shooterAngledUp = true;

    // Robot Classes
  //  private Provider2020 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime; // internal clock
   // Drive_Mecanum_Tele mecanum_drive; // the main mecanum drive class
    private Intake_Ring_Drop intake;
    private Shooter_Ring_ServoFed shooter;
    private Arm_Wobble_Grabber wobble;

    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        runtime = new ElapsedTime();

       // intake = new Intake_Ring_Drop(hardwareMap.get(DcMotor.class, "intakeMotor"), hardwareMap.get(Servo.class, "intakeLockServo"));
        DcMotor shooterMoter = hardwareMap.get(DcMotor.class, "shooterMotor");
        Servo feederServo = hardwareMap.get(Servo.class, "feederServo");
        Encoder shooterEncoder = new Encoder((DcMotorEx)shooterMoter);

        shooterMoter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter = new Shooter_Ring_ServoFed(shooterMoter, feederServo);

        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();

        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            // Variables
            boolean instructFire = false;

            // Logic (figuring out what the robot should do)

            if(gamepad2.x){
                instructFire = true;
            }

            if( gamepad2.right_bumper && firstSpinUpToggle ){
                isSpinningUp = !isSpinningUp;

                firstSpinUpToggle = false;
            }
            else if (gamepad2.right_trigger < 0.5){
                firstSpinUpToggle = true;
            }

            if( gamepad2.y && firstAngleToggle ){ // code to toggle if the shooter is spinning up
                shooterAngledUp = !shooterAngledUp;

                firstAngleToggle = false;
            }
            else if (!gamepad2.y){
                firstAngleToggle = true;
            }


            // Hardware instruction (telling the hardware what to do)
            if(instructFire){
                shooter.instructFire();
            }

            shooter.setFlywheelMode(isSpinningUp);

            shooter.updateFeeder();

            if(shooterAngledUp){
                shooter.setTargetShooterPower(SHOOTER_HIGH_SPEED);
            }
            else {
                shooter.setTargetShooterPower(SHOOTER_LOW_SPEED);
            }

            //telemetry
            telemetry.addData("Shooter is spun up?", shooter.isSpunUp());
            telemetry.addData("Firing state", shooter.getFiringState());
            telemetry.addData("Running in high power mode", shooterAngledUp);
            telemetry.addData("Motor velocity", shooterEncoder.getRawVelocity());


            telemetry.update(); // send the queued telemetry to the output
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}

