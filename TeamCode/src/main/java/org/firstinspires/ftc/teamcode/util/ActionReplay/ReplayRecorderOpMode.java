package org.firstinspires.ftc.teamcode.util.ActionReplay;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Provider2020;
import org.firstinspires.ftc.teamcode.hardware.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.drive.experimental.Drive_Mecanum_Tele_V2;
import org.firstinspires.ftc.teamcode.util.io.DashboardUtil;


/*
    Welcome to the 2020-2021 TeleOp class!

    Robot control scheme:
        Main Drive:
        - Controller 1 Left Stick = x and y translation
        - Controller 1 Right Stick (x axis only) = rotation
        - Controller 1 D-Pad Up = toggle drive relative to field


        Recording/Replaying:
        - Controllers 1&2 Y = Toggle recording
        - Controllers 1&2 B = Toggle replaying
 */


@TeleOp(name = "Recorder Opmode", group = "@@R")

@Config
public class ReplayRecorderOpMode extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    public static String robotName = "Lil' ring flinga";
    public static String REPLAY_FILE_NAME = "TestPath1.csv";

    // Constants
    static final double DEAD_ZONE_RADIUS = 0.005; // the minimum value that can be passed into the drive function
    static final int TELEMETRY_TRANSMISSION_INTERVAL = 7;
    public static int RECORD_INTERVAL = 25; // how many milliseconds between recording waypoints, lower number = more waypoints but more computer resource use from waypoints
    public static Pose2d startPose = new Pose2d(0, 0, 0);

    // Robot Classes
    private Provider2020 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private Drive_Mecanum_Tele_V2 mecanumDrive; // the main mecanum drive class
    private StandardTrackingWheelLocalizer localizer; // the odometry based localizer - uses dead wheels to determine (x, y, r) position on the field

    private ReplayManager replayManager;
    private FtcDashboard dashboard;
    private ElapsedTime runtime;
    private ElapsedTime timeSinceLastRecord;

    // Flags
    private boolean firstReplayToggle = true;
    private boolean firstRecordToggle = true;
    private boolean recordingPrimed = false;
    private boolean firstRelativeToFieldToggle = true;
    private boolean drivingFieldRelative = false;


    // Tracking variables
    private RobotState currentTargetState = new RobotState();
    private int failedLoadCount = 0;

    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new Provider2020(hardwareMap);
        mecanumDrive = new Drive_Mecanum_Tele_V2(robot.driveFL, robot.driveFR, robot.driveBL, robot.driveBR); // pass in the drive motors and the speed variables to setup properly

        replayManager = new ReplayManager(REPLAY_FILE_NAME);
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        runtime = new ElapsedTime();
        timeSinceLastRecord = new ElapsedTime();



        dashboard = FtcDashboard.getInstance(); // setup the dashboard
        dashboard.setTelemetryTransmissionInterval(TELEMETRY_TRANSMISSION_INTERVAL); // interval in milliseconds

        robot.setEncoderActive(false); // start the game without running encoders on drive encoders
        localizer.setPoseEstimate(startPose);


        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            if(localizer != null){ // if the localizer exists
                localizer.update(); // update our current position
            }

            // Variables
            double xTranslatePower = -gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y); // specifically the y stick is negated because up is negative on the stick, but we want up to move the robot forward
            double yTranslatePower = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x); // set the robot translation/rotation speed variables based off of controller input (set later in hardware manipluation section)
            double rotatePower = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);


            // Logic
            if( (gamepad1.y || gamepad2.y) && firstRecordToggle){ // toggle if recording is primed
                if(replayManager.isRecording()){
                    if(!replayManager.stopRecording())
                        failedLoadCount++;
                }
                else {
                    localizer.setPoseEstimate(startPose);

                    if(!replayManager.startRecording())
                        failedLoadCount++;
                }

                firstRecordToggle = false;
            }
            else if( !(gamepad1.y || gamepad2.y) ){
                firstRecordToggle = true;
            }

          //  if(recordingPrimed && (Math.abs(xTranslatePower) > 0.2 || Math.abs(yTranslatePower) > 0.2 || Math.abs(rotatePower) > 0.2)){ // if recording is primed and the driver is moving the stick, start recording
            //}


            if( (gamepad1.b || gamepad2.b) && firstReplayToggle && !replayManager.isRecording()){ // toggle if we are replaying
                if(replayManager.isReplaying()){
                    replayManager.stopStateReplay();
                }
                else {
                    localizer.setPoseEstimate(startPose);

                    replayManager.startStateReplay();
                }

                recordingPrimed = false;
                firstReplayToggle = false;
            }
            else if( !(gamepad1.b || gamepad2.b) ){
                firstReplayToggle = true;
            }


            if(gamepad1.dpad_up && firstRelativeToFieldToggle){ // toggle relative to field drive
                drivingFieldRelative = !drivingFieldRelative;

                firstRelativeToFieldToggle = false;
            }
            else if( !(gamepad1.b || gamepad2.b) ){
                firstRelativeToFieldToggle = true;
            }


            //setup a dead zone for the controllers
            if(Math.abs(xTranslatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                xTranslatePower = 0;
            }
            if(Math.abs(yTranslatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                yTranslatePower = 0;
            }
            if(Math.abs(rotatePower) <= DEAD_ZONE_RADIUS){ // if the value is less than the maximum deadzone value, set to zero (to stop the motor)
                rotatePower = 0;
            }


            // Hardware instruction
            if(replayManager.isRecording() && timeSinceLastRecord.milliseconds() > RECORD_INTERVAL){
                replayManager.recordRobotState(new RobotState(replayManager.getTimerMsec(), localizer.getPoseEstimate(), new GamepadState(gamepad1), new GamepadState(gamepad2))); // save the robot state

                timeSinceLastRecord.reset();
            }

            if(replayManager.isReplaying()){
                currentTargetState = replayManager.getCurrentTargetState();

                mecanumDrive.driveToPose(localizer.getPoseEstimate(), currentTargetState.getPosition());
            }
            else if (drivingFieldRelative) { // if not replaying, allow the user to drive normally, either field relative or not
                mecanumDrive.driveFieldRelative(xTranslatePower, yTranslatePower, rotatePower, localizer.getPoseEstimate().getHeading());
            }
            else {
                mecanumDrive.driveRobotRelative(xTranslatePower, yTranslatePower, rotatePower);
            }



            if(replayManager.isRecording()){
                telemetry.addLine("Currently Recording a Path. Press Y again to stop recording.");
            }
            else {
                telemetry.addLine("Not Recording a Path. Press the Y to start recording.");
            }
            if(replayManager.isReplaying()){
                telemetry.addLine("Currently Replaying a Path. Press B again to stop replaying.");
                telemetry.addData("Target Position:", currentTargetState.getPosition());
                telemetry.addData("Gamepad 1 Recorded Left Stick", new Vector2d(currentTargetState.getGamepad1State().left_stick_x(), currentTargetState.getGamepad1State().left_stick_y()));
            }
            else {
                telemetry.addLine("Not Replaying a Path. Press the B to start replaying.");
            }
            telemetry.addData("Current Position:", localizer.getPoseEstimate());
            telemetry.addData("Failed Load Count:", failedLoadCount);

            telemetry.update();


            updateDashboard();
        }  // end of running while loop
    }


    /* PUT ALL FUNCTIONS HERE */
    public void updateDashboard(){
        TelemetryPacket packet = new TelemetryPacket();
        Pose2d currentPose = localizer.getPoseEstimate();


        packet.put("Recording", replayManager.isRecording()); // get the shooter velocity and add that
        packet.put("Replaying", replayManager.isReplaying()); // get the shooter velocity and add that

        packet.put("X", currentPose.getX()); // get the shooter velocity and add that
        packet.put("Target X", currentTargetState.getPosition().getX()); // get the shooter velocity and add that
        packet.put("Y", currentPose.getY()); // get the shooter velocity and add that
        packet.put("Target Y", currentTargetState.getPosition().getY()); // get the shooter velocity and add that
        packet.put("Heading", currentPose.getHeading()); // get the shooter velocity and add that
        packet.put("Target Heading", currentTargetState.getPosition().getHeading()); // get the shooter velocity and add that


        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#3F51B5"); // set the current draw color to blue

        DashboardUtil.drawRobot(fieldOverlay, currentPose);

        if(replayManager.isRecording()){
            DashboardUtil.drawPoseHistory(fieldOverlay, replayManager.getRecordedPositionsHistory());
        }
        else if(replayManager.isReplaying()){
            fieldOverlay.setStroke("4CAF50"); // set the field draw color for this bit to black
            DashboardUtil.drawPoseHistory(fieldOverlay, replayManager.getReplayPositions());
            DashboardUtil.drawRobot(fieldOverlay, currentTargetState.getPosition());
        }


        if(dashboard != null){ // only send to the dashboard if it is properly setup
            dashboard.sendTelemetryPacket(packet);
        }
    }

}
