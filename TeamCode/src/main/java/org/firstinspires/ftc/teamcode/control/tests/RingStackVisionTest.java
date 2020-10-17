package org.firstinspires.ftc.teamcode.control.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.vision.OpenCV.RingStackHeightPipeline;
import org.firstinspires.ftc.teamcode.hardware.vision.OpenCV.Vision_OpenCV_ExternalCam;




@TeleOp(name = "Ring Stack Vision Test", group = "@@T")

public class RingStackVisionTest extends LinearOpMode {
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "VisionTestBot";


    // Constants
    static final double DEAD_ZONE_RADIUS = 0.05; // the minimum value that can be passed into the drive function

    // Robot Classes
    private ElapsedTime runtime; // internal clock
    private Vision_OpenCV_ExternalCam vision; // the vision class

    // Flags
    private boolean reversed = false; // default
    private boolean firstIncreaseSpeed = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstDecreaseSpeed = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstReverseToggle = true;



    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        runtime = new ElapsedTime();

        vision = new Vision_OpenCV_ExternalCam(hardwareMap, "Webcam 1", new RingStackHeightPipeline());

        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();

        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate



        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {

            // Telemetry
            telemetry.addData("System Analysis", vision.getOutput());

            telemetry.update();

            
            sleep(100);
        }
    }


    /* PUT ALL FUNCTIONS HERE */

}