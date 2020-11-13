package org.firstinspires.ftc.teamcode.control;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Tele;
import org.firstinspires.ftc.teamcode.hardware.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.intake.Intake_Ring_Drop;
import org.firstinspires.ftc.teamcode.hardware.shooter.Shooter_Ring_ServoFed;
import org.firstinspires.ftc.teamcode.hardware.wobble.Arm_Wobble_Grabber;


/*
    Welcome to the 2020-2021 TeleOp class!

    Robot control scheme:
        Main Drive:
        - Controller 1 Left Stick = translation
        - Controller 1 Right Stick (x axis only) = rotation
        - Controller 1 Right Bumper = boost button
        - Controller 1 D-Pad Up = toggle drive relative to field

        Ring Shooter:
        - Controller 2 X button = start a firing sequence (spins up if not spun up, then shoots a ring. can be held to fire rapidly)
        - Controller 2 Right Bumper = toggle if the shooter motor is spun up

        Ring Intake:
        - Controller 2 Left Bumper = toggle if the ring intake is active

        Wobble Intake/Arm:
        - Controller 2 Right Stick (y axis) = up moves the intake wheels to outtake the wobble goal, down moves the intake wheels to intake the wobble goal
        - Controller 2 D-Pad Up = Move the wobble arm to the lifted position (for going over the wall)
        - Controller 2 D-Pad Down = Move the wobble arm to the grab position (for grabbing the wobble goal)
        - Controller 2 D-Pad Right = Move the wobble arm to the folded position (NOT recommended while holding the wobble goal)
 */



@TeleOp(name = "TeleOp2020", group = "@@@")

public class TeleOp2020 extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "Robot 2020";

    // Robot Speed variables
    double turnSpeed = 0.8; // Speed multiplier for turning (1 being 100% of power going in)
    double translateSpeed = 0.4; // Speed multiplier for translation (1 being 100% of power going in)
    double boostSpeed = 1.0; // Speed multiplier for BOOSTING (1 being 100% of power going in)
    double stopSpeed = 0.0; // the motor speed for stopping the robot

    // Constants
    static final double DEAD_ZONE_RADIUS = 0.05; // the minimum value that can be passed into the drive function

    // Robot Classes
    private Provider2020 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private ElapsedTime runtime; // internal clock
    private Drive_Mecanum_Tele mecanum_drive; // the main mecanum drive class
    private StandardTrackingWheelLocalizer localizer; // the odometry based localizer - uses dead wheels to determine (x, y, r) position on the field
    private Intake_Ring_Drop intake; // the intake class instance
    private Shooter_Ring_ServoFed shooter; // the shooter class instance
    private Arm_Wobble_Grabber wobble; // the wobble intake/arm class instance

    private Arm_Wobble_Grabber wobbleClamp; // wobble intake, but clamp

    // Flags
    private boolean firstToggleDriveRelative = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstSpinUpToggle = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstIntakeRunToggle = true; // used to ensure proper toggling behavior (see usage under logic section)

    private boolean driveFieldRelative = true; // default is driving relative to field
    private boolean isSpinningUp = false;
    private int wobbleArmPosition = 0; // 0 = folded pos, 1 = up pos, 2 = grab position
    private int wobbleIntakeDirection = 0; // 0 = stopped, 1 = intaking, -1 = outtaking
    private boolean intakeIsRunning = false; // holds if the intake should be running or not

    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new Provider2020(hardwareMap);
        runtime = new ElapsedTime();
        mecanum_drive = new Drive_Mecanum_Tele(robot.driveFL, robot.driveFR, robot.driveBL, robot.driveBR, turnSpeed, translateSpeed, boostSpeed); // pass in the drive motors and the speed variables to setup properly
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        intake = new Intake_Ring_Drop(robot.intakeMotor, robot.intakeLockServo);
        shooter = new Shooter_Ring_ServoFed(robot.shooterMotor, robot.shooterFeederServo);
        wobble = new Arm_Wobble_Grabber(robot.wobbleArmMotor, robot.wobbleLeftWheelServo, robot.wobbleRightWheelServo);
        wobbleClamp = new Arm_Wobble_Grabber(robot.wobbleArmMotor2, robot.wobbleLeftClampServo, robot.wobbleRightClampServo);

        robot.setEncoderActive(false); // start the game without running encoders

        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            if(localizer != null){ // if the localier exists
                localizer.update(); // update our current position
            }

            // Variables
            boolean isBoosting = gamepad1.right_bumper;  // If true, the robot will go at the boost speed, otherwise it will go at the base speed (just impacts translation)
            double xTranslatePower = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x); // set the robot translation/rotation speed variables based off of controller input (set later in hardware manipluation section)
            double yTranslatePower = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y); // specifically the y stick is negated because up is negative on the stick, but we want up to move the robot forward
            double rotatePower = gamepad1.right_stick_x;
            boolean instructFire = gamepad2.x; // if pressing the second gamepad x, instruct a fire event

            // Logic (figuring out what the robot should do)
            if(gamepad1.dpad_up && firstToggleDriveRelative){ // toggle driving relative to field if dpad up is pressed
                driveFieldRelative = !driveFieldRelative; // toggle the value

                firstToggleDriveRelative = false; // set the variable false so that it cannot toggle again
            }
            else if (!gamepad1.dpad_up){ // wait to set the flag back to true until the button is released
                firstToggleDriveRelative = true; // until the button is released
            }

            if( gamepad2.right_bumper && firstSpinUpToggle ){ // code to toggle if the shooter is spinning up
                isSpinningUp = !isSpinningUp;

                firstSpinUpToggle = false;
            }
            else if (!gamepad2.right_bumper){
                firstSpinUpToggle = true;
            }

            if( gamepad2.left_bumper && firstIntakeRunToggle ){ // code to toggle if the intake is running
                intakeIsRunning = !intakeIsRunning;

                firstSpinUpToggle = false;
            }
            else if (gamepad2.left_bumper){
                firstSpinUpToggle = true;
            }

            if(gamepad2.dpad_up){ // if pressing up
                wobbleArmPosition = 1; // tell it to go to up position
            }
            else if(gamepad2.dpad_down){ // if pressing down
                wobbleArmPosition = 2; // tell it to go to down position
            }
            else if(gamepad2.dpad_right){ // if pressing right
                wobbleArmPosition = 0; // tell it to go to the idle position
            }

            if(gamepad2.right_stick_y > DEAD_ZONE_RADIUS){ // if pulling down on the stick enough, intake
                wobbleIntakeDirection = 1;
            }
            else if(gamepad2.right_stick_y < -DEAD_ZONE_RADIUS){ // if pusing up on the stick enough, outtake
                wobbleIntakeDirection = -1;
            }
            else { // default state is 0
                wobbleIntakeDirection = 0;
            }


            //setup a dead zone for the controllers
            if(Math.abs(xTranslatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                xTranslatePower = stopSpeed;
            }
            if(Math.abs(yTranslatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                yTranslatePower = stopSpeed;
            }
            if(Math.abs(rotatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
               rotatePower = stopSpeed;
            }



            // Hardware instruction (telling the hardware what to do)
            if(driveFieldRelative) {
                mecanum_drive.drive_field_relative(xTranslatePower, yTranslatePower, rotatePower, robot.getHeading(), isBoosting); // call the drive field relative method
            }
            else {
                mecanum_drive.drive_robot_relative(xTranslatePower, yTranslatePower, rotatePower, isBoosting); // call the drive robot relative method
            }

            if(instructFire) {
                shooter.instructFire(); // tell the shooter it should fire (only ever queues a single fire)
            }
            shooter.setFlywheelMode(isSpinningUp); // make sure the shooting mode it set properly
            shooter.updateFeeder(); // update the shooter feeder position based off of where it is in the cycle

            intake.setRunning(intakeIsRunning); // make sure the intake intakin is set to the proper intake mode

            wobble.setIntakeDirection(wobbleIntakeDirection); // make sure it is intaking properly
            wobbleClamp.setIntakeDirection(wobbleIntakeDirection);
            if(wobbleArmPosition == 1) { // set the wobble arm position
                wobble.goToUpPos();
                wobbleClamp.goToUpPos();
            }
            else if(wobbleArmPosition == 2) {
                wobble.goToGrabPos();
                wobbleClamp.goToGrabPos();
            }
            else{
                wobble.goToIdlePos();
                wobbleClamp.goToIdlePos();
            }


            // Telemetry
            if(driveFieldRelative){ // add telemetry relating to robot drive mode
                telemetry.addLine("Driving field relative");
            }
            else{
                telemetry.addLine("Driving robot relative");
            }

            if(robot.driveUsingEncoders){
                telemetry.addLine("Driving using encoders");

                telemetry.addData("Drive FL Encoder: ", robot.driveFL.getCurrentPosition()); // add telemetry data for motor encoders
                telemetry.addData("Drive FR Encoder: ", robot.driveFR.getCurrentPosition());
                telemetry.addData("Drive BL Encoder: ", robot.driveBL.getCurrentPosition());
                telemetry.addData("Drive BR Encoder: ", robot.driveBR.getCurrentPosition());
            }
            else{
                telemetry.addLine("Driving without drive encoders");
            }

            telemetry.addData("Robot is Boosting?", isBoosting);

            if(localizer != null){ // if we have a localizer that exists, get the position estimate from it
                telemetry.addData("Field Position", localizer.getPoseEstimate());
            }

            telemetry.addData("Shooter is spun up?", shooter.isSpunUp());
            telemetry.addData("Firing state", shooter.getFiringState());

            telemetry.update();
        }  // end of running while loop
    }


    /* PUT ALL FUNCTIONS HERE */

}
