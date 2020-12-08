package org.firstinspires.ftc.teamcode.control.tests;


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


/*
    Welcome to the template TeleOp class!

    Happy coding!
 */


@TeleOp(name = "TeleOpActivityName", group = "zActivity")
@Disabled


public class TeleOpActivity extends LinearOpMode {
    // TeleOp Variables
    private DcMotor shooterMotor;
    private Servo feederServo;
    private DcMotor intakeMotor;
    private Servo lockServo;

    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        Intake_Ring_Drop intake = new Intake_Ring_Drop(intakeMotor, lockServo);
        Shooter_Ring_ServoFed shooter = new Shooter_Ring_ServoFed(shooterMotor, feederServo);


        waitForStart(); // Wait for the start button to be pressed before continuing further


        wait( 1000 ) ;// the regular wait function, will wait for the input number of milliseconds, in this case 1 seconds worth of milliseconds


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {

            if (gamepad1.dpad_down == true) {
                intake.setRunning(true);
                shooter.spinUp();
            } else {
                intake.setRunning(false);
                shooter.spinDown();
            }

            if (gamepad1.left_bumper == true) {
                shooter.instructFire();

            }

            shooter.updateFeeder();





        }
    }


    /* PUT ALL CLASS METHOD DECLARATIONS HERE */

}
