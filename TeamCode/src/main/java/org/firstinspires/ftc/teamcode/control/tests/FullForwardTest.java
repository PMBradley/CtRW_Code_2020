package org.firstinspires.ftc.teamcode.control.tests;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.templates.Provider20XX;
import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Tele;


/*
    Welcome to the template TeleOp class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named TeleOp2020 and all code within would have to reflect that change)

    Happy coding!
 */



@TeleOp(name = "FullForwardTest", group = "@@T")
//@Disabled

public class FullForwardTest extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "INSERT_ROBOT_NAME_HERE";

    // Robot Speed variables
    double turnSpeed = 0.5; // Speed multiplier for turning (1 being 100% of power going in)
    double translateSpeed = 0.4; // Speed multiplier for translation (1 being 100% of power going in)
    double boostSpeed = 1; // Speed multiplier for BOOSTING (1 being 100% of power going in)
    double stopSpeed = 0;

    // Robot Classes
    private ElapsedTime runtime; // internal clock



    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        runtime = new ElapsedTime();
        double testSpeed = 1.0;


                telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        DcMotor driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        DcMotor driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        DcMotor driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        DcMotor driveBR = hardwareMap.get(DcMotor.class, "driveBR");


        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            // Variables

            boolean isRunning = false; // If true, the robot will go at the boost speed, otherwise it will go at the base speed (just impacts translation)


            // Logic (figuring out what the robot should do)


            if(gamepad1.right_bumper == true){ // Figure out if the robot should be boosting
                isRunning = true;
            }


            // Hardware instruction (telling the hardware what to do)

            //mecanum_drive.drive_field_relative(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, robot.getHeading(), isBoosting);

            //telemetry
            if(isRunning){
                driveFL.setPower(testSpeed);
                driveFR.setPower(testSpeed);
                driveBL.setPower(testSpeed);
                driveBR.setPower(testSpeed);



                telemetry.addLine("Running motors");
            }
            else{
                telemetry.addLine("Press the right bumper on Gamepad1 to run motors.");
            }

            telemetry.update(); // send the queued telemetry to the output
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}
