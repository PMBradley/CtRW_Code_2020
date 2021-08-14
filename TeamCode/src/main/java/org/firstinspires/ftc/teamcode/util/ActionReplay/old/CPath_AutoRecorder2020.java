package org.firstinspires.ftc.teamcode.util.ActionReplay.old;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.Provider2020;
import org.firstinspires.ftc.teamcode.hardware.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.hardware.intake.Intake_Ring_Drop;
import org.firstinspires.ftc.teamcode.hardware.shooter.J_Shooter_Ring_ServoFed;
import org.firstinspires.ftc.teamcode.hardware.wobble.Arm_Wobble_Grabber;
import org.firstinspires.ftc.teamcode.util.io.DashboardUtil;



/*
    Welcome to the 2020-2021 TeleOp class!

    Robot control scheme:
        Main Drive:
        - Controller 1 Left Stick = translation
        - Controller 1 Right Stick (x axis only) = rotation
        - Controller 1 Right Bumper = boost button
        - Controller 1 D-Pad Up = toggle drive relative to field
        - Controller 1 D-Pad Left/Right = preform the powershot shooting subroutine

        Ring Shooter:
        - Controller 2 X button = start a firing sequence (spins up if not spun up, then shoots a ring. can be held to fire rapidly)
        - Controller 2 Right Bumper = toggle if the shooter motor is spun up
        - Constroller 2 Y Button = toggle powershot/highgoal mode

        Ring Intake:
        - Controller 2 Left Bumper = toggle if the ring intake is active
        - Controller 2 Left Trigger = toggle ring blocking arm

        Wobble Intake/Arm:
        - Controller 2 Right Stick (y axis) = up moves the intake wheels to outtake the wobble goal, down moves the intake wheels to intake the wobble goal
        - Controller 2 D-Pad Up = Move the wobble arm to the lifted position (for going over the wall)
        - Controller 2 D-Pad Down = Move the wobble arm to the grab position (for grabbing the wobble goal)
        - Controller 2 D-Pad Right = Move the wobble arm to the folded position (NOT recommended while holding the wobble goal)
 */


@TeleOp(name = "C Path AutoRecorder2020", group = "@@A")

@Config
public class CPath_AutoRecorder2020 extends LinearOpMode{
    // TeleOp Variables

    // Robot Name - Feel free to set it to whatever suits your creative fancy :)
    String robotName = "Lil' ring flinga";
    public static String REPLAY_FILE_NAME = "AutoPathC.bin";

    // Robot Speed variables
    public static double SLOW_MODE_MULT = 0.50; // Speed multiplier for SLOW MODE (1 being 100% of power going in)
    public static double stopSpeed = 0.00; // the motor speed for stopping the robot

    // Constants
    static final double DEAD_ZONE_RADIUS = 0.005; // the minimum value that can be passed into the drive function
    static final int TELEMETRY_TRANSMISSION_INTERVAL = 25;
    static final int ENDGAME_START_TIME = 120000;
    public static Pose2d powershotAStartPos = new Pose2d(-65.3, -37.36, Math.toRadians(0.0));
    public static Pose2d powershotBStartPos = new Pose2d(0, 0, Math.toRadians(0.0));
    public static int RECORD_INTERVAL = 75; // how many milliseconds between recording waypoints, lower number = more waypoints but more computer resource use from waypoints
    public static Pose2d START_POSE = new Pose2d(-65.3, 7.39, Math.toRadians(0));

    // Robot Classes
    private Provider2020 robot; // Main robot data class (ALWAYS CREATE AN INSTANCE OF THIS CLASS FIRST - HARDWARE MAP SETUP IS DONE WITHIN)
    private Drive_Mecanum_Tele_RepRec mecanumDrive; // the main mecanum drive class
    private StandardTrackingWheelLocalizer localizer; // the odometry based localizer - uses dead wheels to determine (x, y, r) position on the field
    private Intake_Ring_Drop intake; // the intake class instance
    private J_Shooter_Ring_ServoFed shooter; // the shooter class instance
    private Arm_Wobble_Grabber wobble; // the wobble intake/arm class instance

    private FtcDashboard dashboard;
    private ElapsedTime runtime;
    private ElapsedTime timeSinceLastRecord;

    private ReplayManager replayManager;
    private RobotState currentTargetState = new RobotState();
    private GamepadState gp1 = new GamepadState();
    private GamepadState gp2 = new GamepadState();


    // Flags
    private boolean firstToggleDriveRelative = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstSpinUpToggle = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstIntakeRunToggle = true; // used to ensure proper toggling behavior (see usage under logic section)
    private boolean firstAngleToggle = true;

    private boolean driveFieldRelative = false; // default to driving robot relative
    private boolean isSpinningUp = false;
    private boolean shooterAngledUp = true;
    private int wobbleArmPosition = 0; // 0 = folded pos, 1 = up pos, 2 = grab position
    private int wobbleIntakeDirection = 0; // 0 = stopped, 1 = intaking, -1 = outtaking
    private boolean intakeIsRunning = false; // holds if the intake should be running or not
    private boolean firstReplayToggle = true;
    private boolean firstRecordToggle = true;
    private boolean firstGateMoveToggle = true;
    private boolean regulatingSpeed = true;
    private boolean firstRegulatingSpeedToggle = true;


    // The "Main" for TeleOp (the place where the main code is run)
    @Override
    public void runOpMode() throws InterruptedException {
        /* INCLUDE ANY ROBOT SETUP CODE HERE */
        // Call class constructors here (so that nothing major happens before init)
        robot = new Provider2020(hardwareMap);
        runtime = new ElapsedTime();
        mecanumDrive = new Drive_Mecanum_Tele_RepRec(robot.driveFL, robot.driveFR, robot.driveBL, robot.driveBR); // pass in the drive motors and the speed variables to setup properly
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        intake = new Intake_Ring_Drop(robot.intakeMotor, robot.ringGateServo);
        shooter = new J_Shooter_Ring_ServoFed(robot.JShootFront, robot.JShootBack, robot.shooterFeederServo, robot.shooterIndexerServo, robot.shooterAnglerServo);
        wobble = new Arm_Wobble_Grabber(robot.wobbleArmMotor, robot.wobbleLeftWheelServo, robot.wobbleRightWheelServo, 1.0/5.0);

        replayManager = new ReplayManager(REPLAY_FILE_NAME, telemetry);
        localizer = new StandardTrackingWheelLocalizer(hardwareMap);
        runtime = new ElapsedTime();
        timeSinceLastRecord = new ElapsedTime();



        dashboard = FtcDashboard.getInstance(); // setup the dashboard
        dashboard.setTelemetryTransmissionInterval(TELEMETRY_TRANSMISSION_INTERVAL); // interval in milliseconds
        dashboard = FtcDashboard.getInstance(); // setup the dashboard
        dashboard.setTelemetryTransmissionInterval(TELEMETRY_TRANSMISSION_INTERVAL); // interval in milliseconds


        robot.setEncoderActive(false); // start the game without running encoders on drive encoders
        localizer.setPoseEstimate(START_POSE);


        telemetry.addData(robotName + "'s setup completed ", ")"); // Tell the user that robot setup has completed :)
        telemetry.update();


        waitForStart(); // Wait for the start button to be pressed before continuing further


        runtime.reset(); // reset the clock once start has been pressed so runtime is accurate
        intake.raiseGate();


        // The main run loop - write the main robot run code here
        while (opModeIsActive()) {
            if(localizer != null){ // if the localizer exists
                localizer.update(); // update our current position
            }


            // Replay Logic
            if( (gamepad1.y) && firstRecordToggle){ // toggle if recording
                if(replayManager.isRecording()){
                    replayManager.stopRecording();
                }
                else {
                    localizer.setPoseEstimate(START_POSE);

                    replayManager.startRecording();
                }

                firstRecordToggle = false;
            }
            else if( !(gamepad1.y) ){
                firstRecordToggle = true;
            }

            if( (gamepad1.x ) && firstReplayToggle && !replayManager.isRecording()){ // toggle if we are replaying
                if(replayManager.isReplaying()){
                    replayManager.stopStateReplay();
                }
                else {
                    localizer.setPoseEstimate(START_POSE);

                    replayManager.setReplayFile(REPLAY_FILE_NAME);
                    replayManager.startStateReplay();
                }

                firstReplayToggle = false;
            }
            else if( !(gamepad1.x ) ){
                firstReplayToggle = true;
            }


            if(replayManager.isReplaying()) {
                currentTargetState = replayManager.getCurrentTargetState();
                gp1 = currentTargetState.getGamepad1State();
                gp2 = currentTargetState.getGamepad2State();
            }
            else {
                gp1 = new GamepadState(gamepad1);
                gp2 = new GamepadState(gamepad2);
            }


            // Variables
            boolean isSlowMode = gp1.right_bumper;  // If true, the robot will go at the boost speed, otherwise it will go at the base speed (just impacts translation)
            double xTranslatePower = -gp1.left_stick_y * Math.abs(gp1.left_stick_y); // specifically the y stick is negated because up is negative on the stick, but we want up to move the robot forward
            double yTranslatePower = gp1.left_stick_x * Math.abs(gp1.left_stick_x); // set the robot translation/rotation speed variables based off of controller input (set later in hardware manipluation section)
            double rotatePower = gp1.right_stick_x * Math.abs(gp1.right_stick_x);
            boolean instructFire = gp2.x; // if pressing the second gamepad x, instruct a fire event


            // Logic (figuring out what the robot should do)
            if(gp1.dpad_down && firstRegulatingSpeedToggle){ // toggle drive speed limiting
                regulatingSpeed = !regulatingSpeed;

                firstRegulatingSpeedToggle = false;
            }
            else if( !gp1.dpad_down ){
                firstRegulatingSpeedToggle = true;
            }

            if(gp1.dpad_up && firstToggleDriveRelative){ // toggle driving relative to field if dpad up is pressed
                driveFieldRelative = !driveFieldRelative; // toggle the value

                if(driveFieldRelative){ // if toggling back to driving field relative
                    robot.reset_imu(); // reset the robot's imu
                    localizer.setPoseEstimate(START_POSE);
                }

                firstToggleDriveRelative = false; // set the variable false so that it cannot toggle again
            }
            else if (!gp1.dpad_up){ // wait to set the flag back to true until the button is released
                firstToggleDriveRelative = true; // until the button is released
            }


            if( gp2.right_bumper && firstSpinUpToggle ){ // code to toggle if the shooter is spinning up
                isSpinningUp = !isSpinningUp;

                firstSpinUpToggle = false;
            }
            else if (!gp2.right_bumper){
                firstSpinUpToggle = true;
            }
            if( gp2.y && firstAngleToggle ){ // code to toggle if the shooter is spinning up
                shooterAngledUp = !shooterAngledUp;

                firstAngleToggle = false;
            }
            else if (!gp2.y){
                firstAngleToggle = true;
            }

            if( gp2.left_bumper == true && firstIntakeRunToggle ){ // code to toggle if the intake is running
                intakeIsRunning = !intakeIsRunning;

                firstIntakeRunToggle = false;
            }
            else if (!gp2.left_bumper){
                firstIntakeRunToggle = true;
            }

            if(gp2.dpad_up){ // if pressing up
                wobbleArmPosition = 1; // tell it to go to up position
            }
            else if(gp2.dpad_down){ // if pressing down
                wobbleArmPosition = 2; // tell it to go to down position
            }
            else if(gp2.dpad_right){ // if pressing right
                wobbleArmPosition = 0; // tell it to go to the idle position
            }

            if(gp2.right_stick_y > DEAD_ZONE_RADIUS){ // if pulling down on the stick enough, intake
                wobbleIntakeDirection = -1;
            }
            else if(gp2.right_stick_y < -DEAD_ZONE_RADIUS){ // if pushing up on the stick enough, outtake
                wobbleIntakeDirection = 1;
            }
            else { // default state is 0
                wobbleIntakeDirection = 0;
            }

            if(gp2.left_trigger >= 0.3 && firstGateMoveToggle){
                String gatePos = intake.getGatePosition();// get what position the gate thinks it is in

                if(gatePos.equals("UP") || gatePos.equals("PREP")){ // if in one of the two up positions, we wanna toggle down
                    intake.lowerGate();
                }
                else if(runtime.milliseconds() < ENDGAME_START_TIME){ // if not up, we are down. If we are down and not in endgame, we don't wanna go all the way up
                    intake.prepGate();
                }
                else {
                    intake.raiseGate();
                }

                firstGateMoveToggle = false;
            }
            else if(gp2.left_trigger < 0.3){
                firstGateMoveToggle = true;
            }

            if(wobbleIntakeDirection != 0 || shooter.isFiring()){
                isSlowMode = true; // if intaking/outtaking with the wobble or the shooter is spun up, slow down the robot to allow for finer control
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





            // Hardware instruction (telling the hardware what to do now that main logic is complete)
            if(replayManager.isRecording() && timeSinceLastRecord.milliseconds() > RECORD_INTERVAL){
                replayManager.recordRobotState(new RobotState(replayManager.getTimerMsec(), localizer.getPoseEstimate(), gp1, gp2)); // save the robot state

                timeSinceLastRecord.reset();
            }

            if(replayManager.isReplaying()){ // move the robot
                mecanumDrive.setReplayBaseMovement(xTranslatePower, yTranslatePower, rotatePower, localizer.getPoseEstimate().getHeading(), regulatingSpeed, driveFieldRelative);
                mecanumDrive.driveToReplayPose(localizer.getPoseEstimate(), currentTargetState.getPosition());
            }
            else if (driveFieldRelative) { // if not replaying, allow the user to drive normally, either field relative or not
                mecanumDrive.driveFieldRelative(xTranslatePower, yTranslatePower, rotatePower, localizer.getPoseEstimate().getHeading(), regulatingSpeed);
            }
            else {
                mecanumDrive.driveRobotRelative(xTranslatePower, yTranslatePower, rotatePower, regulatingSpeed);
            }


            if (instructFire) {
                shooter.instructFire(); // tell the shooter it should fire (only ever queues a single fire)
            } else {
                shooter.resetShotCount();
            }

            if (shooter.isFiring()) { // if the shooter is firing, make sure the be updating the feeder
                shooter.setTargetShooterSpeed(shooter.getTargetShooterShootingSpeed());
                shooter.spinUp();
                shooter.updateFeeder(); // update the shooter feeder position based off of where it is in the cycle
                intake.spinDown();

                if(shooter.getFiringState() > 0){ // lower the gate once the shooter has actually started moving rings out of the shooter, meaning we won't get penalized for controlling more than 3 rings because it will have shot at least 1 by the time it gets all the way down
                    intake.lowerGate();
                }
            } else if (gamepad2.right_trigger > 0.5) { // then next in the priority list, if the shooter isn't firing check if the intake should be ejecting
                shooter.indexerDown(); // move the indexer to the intaking position
                shooter.setFlywheelMode(isSpinningUp); // set the shooter mode based on the toggle

                intake.setIntakeRunSpeed(-intake.DEFAULT_INTAKE_RUN_SPEED);
                intake.spinUp(); // and run the intake
            } else if (intakeIsRunning) { // if the intake is set to be running by the user and the shooter isn't firing
                shooter.indexerDown(); // move the indexer to the intaking position

                if (isSpinningUp) { // if the shooter should be spinning up
                    shooter.setTargetShooterSpeed(shooter.getTargetShooterShootingSpeed()); // set the shooter to its full speed
                } else { // if not supposed to be spinnig up shooter, spin it up to a low speed to help with intaking
                    shooter.setTargetShooterSpeed(0.2);
                }
                shooter.spinUp();


                intake.setIntakeRunSpeed(intake.DEFAULT_INTAKE_RUN_SPEED);
                intake.spinUp(); // and run the intake
            } else { // otherwise set the shooter to the proper mode
                shooter.setFlywheelMode(isSpinningUp); // make sure the shooting mode it set properly
                shooter.indexerUp();

                intake.spinDown(); // and ensure that the intake is spun down
            }

            if (gamepad2.b) {
                //shooter.optimizeForLonggoal();
            } else if (shooterAngledUp) {
                shooter.optimizeForHighgoal();
            } else {
                shooter.optimizeForPowershots();
            }


            wobble.setIntakeDirection(wobbleIntakeDirection); // make sure it is intaking properly
            //wobbleClamp.setIntakeDirection(wobbleIntakeDirection);
            if(wobbleArmPosition == 1) { // set the wobble arm position
                wobble.goToUpPos();
                //wobbleClamp.goToUpPos();
            }
            else if(wobbleArmPosition == 2) {
                wobble.goToGrabPos();
                //wobbleClamp.goToGrabPos();
            }
            else{
                wobble.goToIdlePos();
                //wobbleClamp.goToIdlePos();
            }



            // Telemetry



            if(replayManager.isRecording()){
                telemetry.addLine("Currently Recording a Path. (Y again to stop recording)");
            }
            else {
                telemetry.addLine("Not Recording a Path. (GP1 Y button to start recording)");
            }
            if(replayManager.isReplaying()){
                telemetry.addLine("Currently Replaying a Path. (X again to stop replaying)\n");
                telemetry.addData("Target Position:", currentTargetState.getPosition());
                //telemetry.addData("Gamepad 1 Recorded Left Stick", new Vector2d(currentTargetState.getGamepad1State().left_stick_x(), currentTargetState.getGamepad1State().left_stick_y()));
            }
            else {
                telemetry.addLine("Not Replaying a Path. (GP1 X button to start replaying)\n");
            }
            telemetry.addData("Current Position:", localizer.getPoseEstimate());
            telemetry.addData("Drive acceleration limiting? (GP1 D-Pad up to toggle)", regulatingSpeed);
            telemetry.addData("Running in Slowmode? (GP1 Right Bumper hold)", isSlowMode);
            if(driveFieldRelative){ // add telemetry relating to robot drive mode
                telemetry.addLine("Driving field relative (GP1 D-Pad down to toggle)");
            }
            else{
                telemetry.addLine("Driving robot relative (GP1 D-Pad down to toggle)");
            }
            if(shooterAngledUp){
                telemetry.addLine("Shooter Optimized for: Highgoal (GP2 Y button to toggle)");
            }
            else {
                telemetry.addLine("Shooter Optimized for: Powershots (GP2 Y button to toggle)");
            }

            telemetry.addData("Wheel arm position", wobble.getArmPosition());
            telemetry.addData("Arm target position", wobble.getArmTargetPosition());

            telemetry.addData("\nShooter is spun up?", shooter.isSpunUp());
            telemetry.addData("Shot Count: ", shooter.getShotCount());
            telemetry.addData("Firing state", shooter.getFiringState());
            telemetry.addData("Corrected Flywheel Velocity: ", shooter.encoderVeloToMotorSpeed(shooter.getFlywheelVelo()));
            telemetry.addData("Shooter Integral: ", shooter.getIntegral());
            telemetry.addData("Target Flywheel Velocity: ", shooter.getTargetShootingSpeed());
            telemetry.addData("\nGP1 State", gp1.toCSVLine());
            telemetry.addData("\nGP2 State", gp2.toCSVLine());



            /*if(localizer != null){ // if we have a localizer that exists, get the position estimate from it
                telemetry.addData("Field Position", localizer.getPoseEstimate());
                telemetry.addData("Wheel Positions", localizer.getWheelPositions());
            }*/



            // telemetry.addData("Wheel arm encoder position", robot.wobbleArmMotor.getCurrentPosition());
            //telemetry.addData("Claw arm position", wobbleClamp.getArmPosition());
            //telemetry.addData("Claw arm encoder position", robot.wobbleArmMotor2.getCurrentPosition())


            telemetry.update();


            updateDashboard();
        }  // end of running while loop
    }


    /* PUT ALL FUNCTIONS HERE */
    //public static double FIRST_POWERSHOT_BACK_DISTANCE = -21.0;


    public void updateDashboard(){
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("raw_shooter_velo", shooter.getFlywheelVelo()); // get the shooter velocity and add that
        packet.put("shooter_velo", J_Shooter_Ring_ServoFed.encoderVeloToMotorSpeed(shooter.getFlywheelVelo())); // get the shooter velocity and convert it to motor speed for readability
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
            fieldOverlay.setStroke("#4CAF50"); // set the field draw color for this bit to black
            DashboardUtil.drawPoseHistory(fieldOverlay, replayManager.getReplayPositions());
            DashboardUtil.drawRobot(fieldOverlay, currentTargetState.getPosition());
        }


        if(dashboard != null){
            dashboard.sendTelemetryPacket(packet);
        }
    }

}
