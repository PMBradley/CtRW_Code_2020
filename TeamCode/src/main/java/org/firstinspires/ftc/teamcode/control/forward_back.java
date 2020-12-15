package org.firstinspires.ftc.teamcode.control;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Tele;
import org.firstinspires.ftc.teamcode.hardware.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.intake.Intake_Ring_Drop;
import org.firstinspires.ftc.teamcode.hardware.shooter.J_Shooter_Ring_ServoFed;
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


    @Autonomous(name = "forward_back", group = "@@@")

    public class forward_back extends LinearOpMode{
        // TeleOp Variables

        // Robot Name - Feel free to set it to whatever suits your creative fancy :)
        String robotName = "Robot 2020";

        // Robot Speed variables
        double turnSpeed = 0.80; // Speed multiplier for turning (1 being 100% of power going in)
        double translateSpeed = 0.40; // Speed multiplier for translation (1 being 100% of power going in)
        double boostSpeed = 1.00; // Speed multiplier for BOOSTING (1 being 100% of power going in)
        double stopSpeed = 0.00; // the motor speed for stopping the robot

        // Constants
        static final double DEAD_ZONE_RADIUS = 0.05; // the minimum value that can be passed into the drive function

        // Robot Classes
        private Provider2020_exp robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
        private ElapsedTime runtime; // internal clock
        private Drive_Mecanum_Tele mecanum_drive; // the main mecanum drive class
        private StandardTrackingWheelLocalizer localizer; // the odometry based localizer - uses dead wheels to determine (x, y, r) position on the field
        private Intake_Ring_Drop intake; // the intake class instance
        private J_Shooter_Ring_ServoFed shooter; // the shooter class instance
        private Arm_Wobble_Grabber wobble; // the wobble intake/arm class instance
        //private Arm_Wobble_Grabber wobbleClamp; // wobble intake, but clamp

        // Flags
        private boolean firstToggleDriveRelative = true; // used to ensure proper toggling behavior (see usage under logic section)
        private boolean firstSpinUpToggle = true; // used to ensure proper toggling behavior (see usage under logic section)
        private boolean firstIntakeRunToggle = true; // used to ensure proper toggling behavior (see usage under logic section)

        private boolean driveFieldRelative = true; // default is driving relative to field
        private boolean isSpinningUp = false;
        private int wobbleArmPosition = 0; // 0 = folded pos, 1 = up pos, 2 = grab position
        private int wobbleIntakeDirection = 0; // 0 = stopped, 1 = intaking, -1 = outtaking
        private boolean intakeIsRunning = false; // holds if the intake should be running or not
        private int robostatus = 0;
        double xTranslatePower = 0;
        double yTranslatePower = -.85;

        // The "Main" for TeleOp (the place where the main code is run)
        @Override
        public void runOpMode() throws InterruptedException {
            /* INCLUDE ANY ROBOT SETUP CODE HERE */
            // Call class constructors here (so that nothing major happens before init)
            robot = new Provider2020_exp(hardwareMap);
            runtime = new ElapsedTime();
            mecanum_drive = new Drive_Mecanum_Tele(robot.driveFL, robot.driveFR, robot.driveBL, robot.driveBR, turnSpeed, translateSpeed, boostSpeed); // pass in the drive motors and the speed variables to setup properly
            localizer = new StandardTrackingWheelLocalizer(hardwareMap);
            intake = new Intake_Ring_Drop(robot.intakeMotor, robot.intakeLockServo);
            //shooter = new Shooter_Ring_ServoFed(robot.shooterMotor, robot.shooterFeederServo);
            shooter = new J_Shooter_Ring_ServoFed(robot.JShootFront, robot.JShootBack, robot.shooterFeederServo, robot.shooterIndexerServo, robot.shooterAnglerServo);
            wobble = new Arm_Wobble_Grabber(robot.wobbleArmMotor, robot.wobbleLeftWheelServo, robot.wobbleRightWheelServo);
            //wobbleClamp = new Arm_Wobble_Grabber(robot.wobbleArmMotor2, robot.wobbleClampServo, robot.wobbleClampServo, 1.0/6.0);

            robot.setEncoderActive(false); // start the game without running encoders

            telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
            telemetry.update();

            waitForStart(); // Wait for the start button to be pressed before continuing further


            runtime.reset(); // reset the clock once start has been pressed so runtime is
            intake.setRunning(true);

            mecanum_drive.drive_robot_relative(0,.85,0,false);
            sleep(2000);
            mecanum_drive.drive_robot_relative(0,-.85,0,false);
            sleep(2000);
            mecanum_drive.drive_robot_relative(0,0,0,false);

        }


        /* PUT ALL FUNCTIONS HERE */

    }
