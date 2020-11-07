package org.firstinspires.ftc.teamcode.control.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
    Welcome to the 2020-2021 TeleOp class!

    Just kidding! This is test class. You can't trust any of the comments here, it is made in a hurry to do a job, usually lots of copy paste
 */


@TeleOp(name = "Single Motor/Servo Run Test", group = "@@T")

public class MotorAndServoRunTest extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "TestBot";

    // Constants
    static final double DEAD_ZONE_RADIUS = 0.05; // the minimum value that can be passed into the drive function
    private static final double MOTOR_STOP_SPEED = 0.0; // the motor speed for stopping the robot
    private static final double MOTOR_CHANGE_AMOUNT = 0.1;
    private static final double SERVO_FORWARD_POS = 1.0;
    private static final double SERVO_STOP_POS = 0.75;

    // Robot Speed variables
    private double motorSpeed = 1.00;
    private double servoSpeed = SERVO_STOP_POS;



    // Robot Classes
    private ElapsedTime runtime; // internal clock

    // Flags
    private boolean motorReversed = false; // default
    private boolean firstIncreaseSpeed = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstDecreaseSpeed = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstReverseToggle = true;
    private boolean firstRunToggle = true;
    private boolean firstServoX = true;
    private boolean firstServoY;

    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        runtime = new ElapsedTime();

        DcMotor mainMotor = hardwareMap.get(DcMotor.class, "driveFR");
        Servo mainServo = hardwareMap.get(Servo.class, "servoTest0");
        
        mainMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mainServo.getController().pwmEnable(); // set the servo to limited range mode


        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate

        boolean isMotorRunning = false;


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            // Variables


            // Logic (figuring out what the robot should do)

            if(Math.abs(gamepad1.left_stick_x) > 0.5){ // if stick be pushed far enough, go go power rangers
                isMotorRunning = true;
            }
            if(gamepad1.right_bumper && firstRunToggle){ // toggle driving realtive to field if dpad up is pressed
                isMotorRunning = !isMotorRunning;

                firstIncreaseSpeed = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.right_bumper){ // wait to set the flag back to true until the button is released
                firstIncreaseSpeed = true; // until the button is released
            }

            if(gamepad1.dpad_up && firstIncreaseSpeed){ // toggle driving realtive to field if dpad up is pressed
                motorSpeed += MOTOR_CHANGE_AMOUNT;

                motorSpeed = Math.min(motorSpeed, 1);

                firstIncreaseSpeed = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_up){ // wait to set the flag back to true until the button is released
                firstIncreaseSpeed = true; // until the button is released
            }

            if(gamepad1.dpad_down && firstDecreaseSpeed){ // toggle driving using encoders on the press of dpad down
                motorSpeed -= MOTOR_CHANGE_AMOUNT;

                motorSpeed = Math.max(motorSpeed, 0);

                firstDecreaseSpeed = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_down){ // wait to set the flag back to true until the button is released
                firstDecreaseSpeed = true; // until the button is released
            }


            if(gamepad1.dpad_left && firstReverseToggle){ // toggle driving using encoders on the press of dpad down
                motorReversed = !motorReversed;

                firstReverseToggle = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_left){ // wait to set the flag back to true until the button is released
                firstReverseToggle = true; // until the button is released
            }


            if(gamepad1.x){
                servoSpeed = SERVO_FORWARD_POS;
            }
            if(gamepad1.y){
                servoSpeed = negateServoPower(SERVO_FORWARD_POS);
            }
            else {
                servoSpeed = SERVO_STOP_POS;
            }


            // Telemetry
            if(isMotorRunning){ // add telemetry relating to robot drive mode
                telemetry.addLine("Running motors at " + motorSpeed * 100.0 + "% of max speed");
                telemetry.addLine("Press D-Pad Up to increase speed by " + (MOTOR_CHANGE_AMOUNT * 100) + "%");
                telemetry.addLine("Press D-Pad Down to decrease speed by " +( -MOTOR_CHANGE_AMOUNT * 100) + "%");
                telemetry.addLine("Is Reversed: " + motorReversed + " (press D-Pad left to toggle direction)");

                if(motorReversed){
                    mainMotor.setPower(-motorSpeed);
                }
                else {
                    mainMotor.setPower(motorSpeed);
                }
            }
            else{
                telemetry.addLine("Motor not running, push up on the left stick (on gamepad1) to run the motor.");

                mainMotor.setPower(MOTOR_STOP_SPEED);
            }


            mainServo.setPosition(servoSpeed);

            if(servoSpeed < SERVO_STOP_POS){ // if the servo be runnin one way
                telemetry.addLine("The servo be movin one direction!");
            }
            else if (servoSpeed > SERVO_STOP_POS){
                telemetry.addLine("The servo be movin the other direction!");
            }
            else {
                telemetry.addLine("Press X or Y to move the servo in different directions");
            }


            telemetry.update();
        }
    }


    /* PUT ALL FUNCTIONS HERE */
    double negateServoPower(double inputPower){
        return (  ( -1*(inputPower - 0.5) ) + 0.5);
    }
}
