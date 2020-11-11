package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.templates.Provider20XX;
import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Tele;
import org.firstinspires.ftc.teamcode.hardware.intake.Intake_Ring_Drop;
import org.firstinspires.ftc.teamcode.hardware.shooter.Shooter_Ring_ServoFed;
import org.firstinspires.ftc.teamcode.hardware.wobble.Arm_Wobble_Grabber;


/*
    Welcome to the template TeleOp class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named TeleOp2020 and all code within would have to reflect that change)

    Happy coding!
 */



@TeleOp(name = "Manipulator Test (shooter/intake/wobble)", group = "@@T")


public class Manipulator_Test extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "Molly";

    // Robot Speed variables
    double turnSpeed = 0.5; // Speed multiplier for turning (1 being 100% of power going in)
    double translateSpeed = 0.4; // Speed multiplier for translation (1 being 100% of power going in)
    double boostSpeed = 1; // Speed multiplier for BOOSTING (1 being 100% of power going in)
    double stopSpeed = 0;
    double testSpeed = 1.0;

    boolean firstSpinUpToggle = true;
    boolean isSpinningUp = false;

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
        shooter = new Shooter_Ring_ServoFed(hardwareMap.get(DcMotor.class, "shooterMotor"), hardwareMap.get(Servo.class, "feederServo"));

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

            if( gamepad2.right_trigger >= 0.5 && firstSpinUpToggle ){
                isSpinningUp = !isSpinningUp;

                firstSpinUpToggle = false;
            }
            else if (gamepad2.right_trigger < 0.5){
                firstSpinUpToggle = true;
            }




            // Hardware instruction (telling the hardware what to do)
            if(instructFire){
                shooter.instructFire();
            }

            shooter.setFlywheelMode(isSpinningUp);

            //telemetry
            telemetry.addData("Shooter is spun up?", shooter.isSpunUp());
            telemetry.addData("Firing state", shooter.getFiringState());

            telemetry.update(); // send the queued telemetry to the output
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}

