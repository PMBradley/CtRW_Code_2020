package org.firstinspires.ftc.teamcode.control.tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Provider2020;
import org.firstinspires.ftc.teamcode.control.templates.Provider20XX;
import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Tele;


/*
    Welcome to the template TeleOp class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named TeleOp2020 and all code within would have to reflect that change)

    Happy coding!
 */



@TeleOp(name = "DriveWheelTesting", group = "@@T")
@Disabled

public class DriveWheelTesting extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "INSERT_ROBOT_NAME_HERE";

    // Robot Speed variables
    double turnSpeed = 0.5; // Speed multiplier for turning (1 being 100% of power going in)
    double translateSpeed = 0.4; // Speed multiplier for translation (1 being 100% of power going in)
    double boostSpeed = 1; // Speed multiplier for BOOSTING (1 being 100% of power going in)
    double stopSpeed = 0;

    double testSpeed = 1.0;

    // Robot Classes
    private Provider2020 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime; // internal clock
    Drive_Mecanum_Tele mecanum_drive; // the main mecanum drive class


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new Provider2020(hardwareMap);
        runtime = new ElapsedTime();
        mecanum_drive = new Drive_Mecanum_Tele(robot.driveFL, robot.driveFR, robot.driveBL, robot.driveBR, turnSpeed, translateSpeed, boostSpeed);


        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();

        robot.setEncoderActive(false);

        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            // Variables

            boolean isBoosting = false; // If true, the robot will go at the boost speed, otherwise it will go at the base speed (just impacts translation)


            // Logic (figuring out what the robot should do)
            if(gamepad1.dpad_down == true){ // if the dpad down is pressed, set the testSpeed to be negative, otherwise keep it positive
                testSpeed *= -1;
            }


            if(gamepad1.right_bumper == true){ // Figure out if the robot should be boosting
                isBoosting = true;
            }

            if(gamepad1.a == true){ // if a is pressed, move FL at testSpeed
                robot.driveFL.setPower(testSpeed);
            }
            else{
                robot.driveFL.setPower(stopSpeed);
            }
            if(gamepad1.b == true){ // if a is pressed, move FR at testSpeed
                robot.driveFR.setPower(testSpeed);
            }
            else{
                robot.driveFR.setPower(stopSpeed);
            }
            if(gamepad1.x == true){ // if a is pressed, move BL at testSpeed
                robot.driveBL.setPower(testSpeed);
            }
            else{
                robot.driveBL.setPower(stopSpeed);
            }
            if(gamepad1.y == true){ // if a is pressed, move BR at testSpeed
                robot.driveBR.setPower(testSpeed);
            }
            else{
                robot.driveBR.setPower(stopSpeed);
            }


            // Hardware instruction (telling the hardware what to do)

            //mecanum_drive.drive_field_relative(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, robot.getHeading(), isBoosting);

            //telemetry
            telemetry.addData("Drive FL: ", robot.driveFL.getCurrentPosition()); // add telemetry data for motor encoders
            telemetry.addData("Drive FR: ", robot.driveFR.getCurrentPosition());
            telemetry.addData("Drive BL: ", robot.driveBL.getCurrentPosition());
            telemetry.addData("Drive BR: ", robot.driveBR.getCurrentPosition());

            telemetry.update(); // send the queued telemetry to the output
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}
