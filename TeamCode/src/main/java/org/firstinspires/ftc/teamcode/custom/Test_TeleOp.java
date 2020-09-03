package org.firstinspires.ftc.teamcode.custom;


import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Test_TeleOp")

public class Test_TeleOp extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "Funky Bot";

    // Robot Speed variables
    double turnSpeed = 0.6; // Speed multiplier for turning (1 being 100% of power going in)
    double translateSpeed = 0.6; // Speed multiplier for translation (1 being 100% of power going in)
    double boostSpeed = 1; // Speed multiplier for BOOSTING (1 being 100% of power going in)

    // Robot Classes
    private Provider20XX robot = new Provider20XX(hardwareMap); // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime  = new ElapsedTime(); // internal clock
    // Drive_Mecanum_Tele mecanum_drive = new Drive_Mecanum_Tele(robot.driveFL, robot.driveFR, robot.driveBL, robot.driveBR, turnSpeed, translateSpeed, boostSpeed); // the main mecanum drive class
    Drive_Mecanum_Tele_Old mecanum_drive = new Drive_Mecanum_Tele_Old(robot.driveFL, robot.driveFR, robot.driveBL, robot.driveBR);
    MediaPlayer musicPlayer; // the music player, because we have fun here :)


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */


        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();

        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate
        //musicPlayer.start(); // start da TUNES


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            // Variables

            boolean isBoosting = false; // If true, the robot will go at the boost speed, otherwise it will go at the base speed (just impacts translation)


            // Logic (figuring out what the robot should do)

            if(gamepad1.right_bumper == true){ // Figure out if the robot should be boosting
                isBoosting = true;
            }


            // Hardware instruction (telling the hardware what to do)

            //mecanum_drive.drive_field_relative(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, robot.getHeading(), isBoosting);
            mecanum_drive.drive_field_relative(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, robot.getHeading(), isBoosting, gamepad1.right_trigger);
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}
