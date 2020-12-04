package org.firstinspires.ftc.teamcode.control.tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.templates.Provider20XX;
import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Tele;
import org.firstinspires.ftc.teamcode.hardware.intake.Intake_Ring_Drop;
import org.firstinspires.ftc.teamcode.hardware.shooter.Shooter_Ring_ServoFed;


/*
    Welcome to the template TeleOp class!

    Happy coding!
 */


@TeleOp(name = "TeleOpActivityName", group = "zActivity")
@Disabled


public class TeleOpActivity extends LinearOpMode {
    // TeleOp Variables


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {



        waitForStart(); // Wait for the start button to be pressed before continuing further


        wait( 1000 ) ;// the regular wait function, will wait for the input number of milliseconds, in this case 1 seconds worth of milliseconds


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {


        }
    }


    /* PUT ALL CLASS METHOD DECLARATIONS HERE */

}
