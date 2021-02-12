 package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Auto;
import org.firstinspires.ftc.teamcode.hardware.intake.Intake_Ring_Drop;
import org.firstinspires.ftc.teamcode.hardware.shooter.J_Shooter_Ring_ServoFed;
import org.firstinspires.ftc.teamcode.hardware.vision.OpenCV.RingStackHeightPipeline;
import org.firstinspires.ftc.teamcode.hardware.vision.OpenCV.Vision_OpenCV_ExternalCam;
import org.firstinspires.ftc.teamcode.hardware.wobble.Arm_Wobble_Grabber;
import org.firstinspires.ftc.teamcode.util.StateMachine.AutoTask;
import org.firstinspires.ftc.teamcode.util.StateMachine.AutoTaskManager;
import org.firstinspires.ftc.teamcode.util.StateMachine.DriveFollowerTask;
import org.firstinspires.ftc.teamcode.util.StateMachine.TargetDrivePosition;

import java.util.ArrayList;

/*
 * Welcome to the Autonomous OpMode! This opmode coordinates the robot's manipulators and drive without user input to complete challenges
 */
@Autonomous(name = "AutoOp2020", group = "@@@") // the name and group for the opmode
@Config // allow FTC dashboard to access public static member variables here

public class AutoOp2020 extends LinearOpMode {

    Provider2020 robot;
    Drive_Mecanum_Auto drive;
    J_Shooter_Ring_ServoFed shooter;
    Intake_Ring_Drop intake;
    Arm_Wobble_Grabber wobble;
    Vision_OpenCV_ExternalCam vision;

    AutoTaskManager taskManager;
    ElapsedTime runtime;
    ElapsedTime currentTaskTime;


    public static String ROBOT_NAME = "Lil' ring flinga";

    private static Pose2d startPose = new Pose2d(-65.3, -9.61, Math.toRadians(0)); // the starting position of the robot relative to the middle of the field

    public static TargetDrivePosition wobbleGoalPosA  = new TargetDrivePosition(18.0, -20.0, Math.toRadians(0.0)); // the positions that the robot needs to drive to
    public static TargetDrivePosition wobbleGoalPosB  = new TargetDrivePosition(14.0, -12.0, Math.toRadians(90.0));
    public static TargetDrivePosition wobbleGoalPosC  = new TargetDrivePosition(42, -30.0, Math.toRadians(70.0), Math.toRadians(-25));

    public static TargetDrivePosition wobblePickupPos = new TargetDrivePosition(-41.3, -16.8, Math.toRadians(-45.0));
    public static TargetDrivePosition shootPos        = new TargetDrivePosition(-51.1, -8, Math.toRadians(1.2));
    public static TargetDrivePosition ringPickupPos   = new TargetDrivePosition(-35.5, -12.5, Math.toRadians(0.0));
    public static TargetDrivePosition parkPos         = new TargetDrivePosition(2.0, -2.0, Math.toRadians(0.0));
    public static double RING_COLLECT_DISTANCE = 9.7; // how far the robot moves to
    public static double ARM_OFFSET_DEGREES = -300; // an offset for the wobble arm
    public static double ARM_COLLECT_DROP_DISTANCE = 18; // how far the robot is from collecting a wobble before it deploys the arm early
    public static double ARM_DROP_DROP_DISTANCE = 12; // how far the robot is from the drop position before it deploys the arm early

    private String wobbleDropPos = "A";
    private boolean scanningComplete = false;
    private boolean autonomousComplete = false;



    @Override
    public void runOpMode() throws InterruptedException {
        // Pre match start initialization
        runtime = new ElapsedTime();
        currentTaskTime = new ElapsedTime();

        robot = new Provider2020(hardwareMap); // setup the hardware classes with the hardware map info (device ports, device names, etc)
        drive = new Drive_Mecanum_Auto(hardwareMap);
        intake = new Intake_Ring_Drop(robot.intakeMotor, robot.intakeLockServo);
        shooter = new J_Shooter_Ring_ServoFed(robot.JShootFront, robot.JShootBack, robot.shooterFeederServo, robot.shooterIndexerServo, robot.shooterAnglerServo);
        wobble = new Arm_Wobble_Grabber(robot.wobbleArmMotor, robot.wobbleLeftWheelServo, robot.wobbleRightWheelServo);

        vision = new Vision_OpenCV_ExternalCam(hardwareMap, "Webcam 1", new RingStackHeightPipeline()); // setup the camera handling class, passing in the pipeline we want to use to process inputs


        drive.setPoseEstimate(startPose); // set the localizer's start position to our position on the field relative to the center

        ArrayList<AutoTask> autoTasks = setupAutoTasks(); // add the list of task objects to the task list
        taskManager = new AutoTaskManager(autoTasks); // then set the drive to use those tasks when required

        // set the first target position based on priorities and what is closest
        taskManager.updateCurrentTaskToClosest( drive.getPoseEstimate() ); // have the task manager set the current auto task to the next closest incomplete task

        drive.setTasks(taskManager.generateCurrentTaskDriveTaskList( drive.getPoseEstimate(), drive.getDriveConstraints() )); // then set the drive to go to that task's position and do any of that auto task's drive tasks

        wobble.setArmOffset(ARM_OFFSET_DEGREES); // set the wobble's starting offset, such that the starting position behaves like 0 (as in Auto the encoder likes to start in a weird position)


        telemetry.addLine(ROBOT_NAME + "'s setup completed.");
        telemetry.addData("Chosen first task", taskManager.getCurrentTask().getTaskName());
        telemetry.update();



        waitForStart(); // wait until play is pressed on the robot controller to progress

        // Post start setup (usually for anything that requires moving any parts on the robot)
        vision.startWebcamStreaming(); // start the streaming process

        wobble.setArmPosition(ARM_OFFSET_DEGREES);


        currentTaskTime.reset();


        while (opModeIsActive() && !isStopRequested() && !autonomousComplete){ // while it is ok to keep running in a loop, do so
            String currentTaskName = taskManager.getCurrentTask().getTaskName();

            boolean autoTaskComplete = drive.doTasksAsync(); // use the built in drive state machine that decides if/when each task is over, and follows them in order appropriately


            updateSensors(currentTaskName); // do any processing of sensor data

            updateManipulators(currentTaskName); // update any non-drive mechanical systems


            if( autoTaskComplete ){ // if the current task is complete
                taskManager.markCurrentTaskComplete(); // set the current task to complete

                // check the name of the current task against as list of prerequisite tasks
                if( currentTaskName.equals("Scan & Shoot Rings") ){  // if this task completed
                    wobbleDropPos = getCalculatedDropPosition(); // get which position was voted for by the camera system the most
                    setWobbleGoalPos(wobbleDropPos); // update the task positions based on sensor data

                    vision.webcam.closeCameraDevice(); // stop the camera to prevent slowing down the system
                    scanningComplete = true;

                    if( !wobbleDropPos.equals("A") ){ // if there are rings to collect (aka any target position other than the one that happens when no rings)
                        taskManager.markTaskWithNameIncomplete("Collect Rings"); // activate the dependant task
                    }
                }
                else if( currentTaskName.equals("Collect Rings") ){ // if this prerec task completed
                    taskManager.markTaskWithNameIncomplete("Shoot Rings"); // activate the dependant task
                }
                else if( currentTaskName.equals("Place Wobble 1") ){ // if this prerec task completed
                    taskManager.markTaskWithNameIncomplete("Collect Wobble"); // activate the dependant task
                }
                else if( currentTaskName.equals("Collect Wobble") ){ // if this prerec task completed
                    taskManager.markTaskWithNameIncomplete("Place Wobble 2"); // activate the dependant task
                }
                else if( currentTaskName.equals("END")){
                    autonomousComplete = true; // breaks free of the main loop
                }

                Pose2d currentPos = drive.getPoseEstimate(); // get the current position of the robot
                taskManager.updateCurrentTaskToClosest( currentPos ); // have the task manager set the current auto task to the next closest incomplete task
                drive.setTasks(taskManager.generateCurrentTaskDriveTaskList( currentPos, drive.getDriveConstraints() )); // then set the drive to go to that task's position and do any of that auto task's drive tasks

                shooter.resetShotCount();
                currentTaskTime.reset(); // reset the current task time after each completion of an AutoTask
            }


            // Print telemetry output to the phones for better understanding of what the robot is doing
            telemetry.addData("Current task name", taskManager.getCurrentTask().getTaskName() );
            telemetry.addData("Drive task index", drive.getTaskIndex()); // print out what the current drive task index is

            if(drive.currentTaskHasTrajectory()) { // if current task has a trajectory to follow
                telemetry.addLine("Currently following a trajectory. Elapsed time: " + changePrecision( mSecToSec(drive.getTaskElapsedTime()), 2) + " seconds"); // print the projected remaining time on that trajectory
                telemetry.addData("Target position", drive.getCurrentTask().getTraj().end()); // print the end position of the trajectory
            }
            else {
                telemetry.addLine("Currently waiting. Elapsed time: " + changePrecision(mSecToSec(drive.getCurrentTask().getNum() - drive.getRemainingWaitMSecs()), 2) + " seconds");
                telemetry.addLine("  Time remaining: " + changePrecision(mSecToSec(drive.getRemainingWaitMSecs()), 2) + " seconds");
            }
            telemetry.addData("Current position", drive.getLocalizer().getPoseEstimate()); // print the estimated current position of the robot

            telemetry.addLine("Shooter velocity: " + shooter.getFlywheelVelo());
            telemetry.addLine("Target Arm position: " + wobble.getArmTargetPosition() + " degrees");
            telemetry.addLine("Arm position: " + wobble.getArmPosition() + " degrees");
            telemetry.addLine("Wobble intake power: " + wobble.getIntakeDirection());
            telemetry.addLine("Ring intake power: " + intake.getCurrentIntakePower());

            if(scanningComplete) {
                telemetry.addLine("Stack scanning complete. Scan count: " + ringStackEstimates.size());
                telemetry.addLine("  Detected drop position: " + wobbleDropPos);
            }
            else {
                int[] counts = getEstimateOccurances();
                telemetry.addLine("Ring Count Votes - 0: " + counts[0] + ", 1: " + counts[1] + ", 4: " + counts[2]);
            }

            telemetry.update(); // push all of the telemetry to the phone

        }
    }


    void updateSensors(String currentTaskName){ // updates any processing related to sensor data, in this game that mostly means camera data

        if( currentTaskName.equals("Scan & Shoot Rings") && drive.getTaskIndex() >= 1 ){ // if current task is this task and we have completed driving to location
            String visionOutput = vision.getOutput(); // get the first character of the output string, it holds the single digit number of rings output
            char ringCount = visionOutput.charAt(0);

            if(ringCount == '0'){ // if no rings, add a vote for A position
                ringStackEstimates.add("A");
            }
            else if(ringCount == '1'){ // if one ring, add a vote for B position
                ringStackEstimates.add("B");
            }
            else { // if some other amount of rings (usually meaning 4), add a vote for C position
                ringStackEstimates.add("C");
            }
        }
    }
    void updateManipulators(String currentTaskName) { // updates any processing and processes related to non-drive mechanical systems on the robot
        if ( (currentTaskName.equals("Place Wobble 1") || currentTaskName.equals("Place Wobble 2") )) { // if current task is this task (or the other one)
            shooter.spinDown();

            if(!drive.firstTaskCompleted()){
                intake.spinUp();
                shooter.indexerDown();
            }
            else {
                intake.spinDown();
                shooter.indexerUp();
            }
            if(drive.getTaskIndex() == 1 || taskManager.getDistanceFromTaskLocation(drive.getPoseEstimate()) <= ARM_DROP_DROP_DISTANCE ){ // if at location and on the first subtask
                wobble.goToGrabPos();

                if(drive.getTaskIndex() == 1){
                    wobble.intakeSpinIn();
                }
            }
            else if(drive.getTaskIndex() == 2){ // if at location and on the second subtask
                wobble.stopIntake();
            }
            else {
                wobble.setArmPosition(ARM_OFFSET_DEGREES);
            }
        }
        else if ( currentTaskName.equals("Scan & Shoot Rings") || currentTaskName.equals("Shoot Rings") ) { // if current task is this task (or the other one)
            wobble.goToRecentPos();
            shooter.spinUp(); // spin up the shooter on the way to
            shooter.optimizeForLonggoal(); // set the shooter to optimize for shooting from across the field

            if(drive.firstTaskCompleted()){ // if at location, start shooting
                shooter.instructFire();


                if(currentTaskTime.milliseconds() > 2500 || shooter.getFiringState() > 0 || currentTaskName.equals("Scan & Shoot Rings")){
                    intake.spinDown();
                }
                else {
                    intake.spinUp();
                }

            }
            else { // if not at location
                if(currentTaskName.equals("Shoot Rings")){
                    intake.spinUp();
                    shooter.indexerDown();
                }
                else {
                    intake.spinDown();
                    shooter.indexerUp();
                }
            }
        }
        else if( currentTaskName.equals("Collect Rings")  ){ // if current task is this task
            intake.spinUp(); // spin up the intake for ring collection
            shooter.indexerDown();
            // specifically leaves the shooter in whatever state it was already in, but no need to maintain the PID by calling spinUp or spinDown as the motor will keep its last known speed and that is close enough
        }
        else if( currentTaskName.equals("Collect Wobble") ){ // if current task is this task
            shooter.spinDown();

            if( drive.getTaskIndex() == 1 || taskManager.getDistanceFromTaskLocation(drive.getPoseEstimate()) <= ARM_COLLECT_DROP_DISTANCE ) { // if close to location or on the first subtask
                wobble.intakeSpinOut();
                wobble.goToGrabPos();
            }
            else if (drive.firstTaskCompleted()){ // if at location and on the second subtask
                wobble.setArmPosition(ARM_OFFSET_DEGREES);
                wobble.stopIntake();
            }

        }
        else if (currentTaskName.equals("END")){
            shooter.spinDown();
            intake.spinDown();
            wobble.stopIntake();
            shooter.indexerUp();
            wobble.setArmPosition(ARM_OFFSET_DEGREES);
        }
        else { // if no tasks selected above, default states are as follows
            shooter.spinDown();
            intake.spinDown();
            wobble.intakeSpinIn();
            shooter.indexerUp();
            wobble.setArmPosition(ARM_OFFSET_DEGREES);
        }

        wobble.goToRecentPos();
        shooter.updateFeeder(); // always update the shooter feeder, so that it can start the feeding process as soon as possible
    }


    ArrayList<AutoTask> setupAutoTasks(){
        ArrayList<AutoTask> autoTasks = new ArrayList<AutoTask>(); // create an array list to hold the tasks that will be output by the method
        ArrayList<DriveFollowerTask> atLocationTasks = new ArrayList<DriveFollowerTask>(); // create an array list to hold the driveTasks for each location (reset between adding tasks unless their at location drive task list is the same)


        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(360)); // once at this location, wait msecs
        atLocationTasks.add(new DriveFollowerTask(10)); // then wait msecs
        autoTasks.add(new AutoTask("Place Wobble 1", 2, wobbleGoalPosA, atLocationTasks));
        autoTasks.add(new AutoTask("Place Wobble 2", 2, wobbleGoalPosA, atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(1000)); // once at this location, wait msecs
        autoTasks.add(new AutoTask("Scan & Shoot Rings", 1, shootPos, atLocationTasks));
        atLocationTasks.add(new DriveFollowerTask(450)); // only for the second one, give it more time just to be safe
        autoTasks.add(new AutoTask("Shoot Rings", 2, new TargetDrivePosition(shootPos.getX(), shootPos.getY(), shootPos.getHeading() - Math.toRadians(2.0)), atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)

       // autoTasks.add(new AutoTask("Shoot Rings", 2, new TargetDrivePosition(shootPos.getX(), shootPos.getY() + 5.0, shootPos.getHeading() - Math.toRadians(2.8)), atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(drive.trajectoryBuilder(ringPickupPos.getPose2d())
                .lineToLinearHeading(new Pose2d(ringPickupPos.getX() + RING_COLLECT_DISTANCE, ringPickupPos.getY(), ringPickupPos.getHeading()))
                .build()
        )); // once at this location, move forwards into the rings by the RING COLLECT DISTANCE
        /*atLocationTasks.add(new DriveFollowerTask(drive.trajectoryBuilder(atLocationTasks.get(0).getTraj().end())
                .lineToLinearHeading(new Pose2d(ringPickupPos.getX() + (2*RING_COLLECT_DISTANCE), ringPickupPos.getY(), ringPickupPos.getHeading()))
                .build()
        )); */// once at this location, move forwards into the rings by the RING COLLECT DISTANCE
        atLocationTasks.add(new DriveFollowerTask(260)); // once at this location, wait msecs
        autoTasks.add(new AutoTask("Collect Rings", 2, ringPickupPos, atLocationTasks).setCompleted(true)); // set completed so it won't try to collect rings before shooting rings for the first time

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(250)); // wait msecs once at location
        atLocationTasks.add(new DriveFollowerTask(70)); // then wait msecs
        autoTasks.add(new AutoTask("Collect Wobble", 2, wobblePickupPos, atLocationTasks).setCompleted(true)); // set completed so it won't try to collect rings before shooting rings for the first time

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(2000)); // wait msecs once at location
        autoTasks.add(new AutoTask("END", 4, parkPos, atLocationTasks));

        return autoTasks;
    }



    ArrayList<AutoTask> setupTestAutoTasks(){
        ArrayList<AutoTask> autoTasks = new ArrayList<AutoTask>();
        ArrayList<DriveFollowerTask> atLocationTasks = new ArrayList<DriveFollowerTask>();

        double matWidth = 22;

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(00)); // once at this location, wait 100 msecs
        autoTasks.add(new AutoTask("A", 1, new TargetDrivePosition(matWidth, matWidth, 0), atLocationTasks));

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(00)); // once at this location, wait 700 msecs
        autoTasks.add(new AutoTask("B1", 1, new TargetDrivePosition(0, matWidth, 0), atLocationTasks));
        autoTasks.add(new AutoTask("B2", 1, new TargetDrivePosition(-matWidth, -matWidth, 0), atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(00)); // once at this location, wait 700 msecs
        autoTasks.add(new AutoTask("C1", 1, new TargetDrivePosition(matWidth, -matWidth, 0), atLocationTasks));
        autoTasks.add(new AutoTask("C2", 1, new TargetDrivePosition(-matWidth, matWidth, 0), atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(00)); // once at this location, wait 700 msecs
        autoTasks.add(new AutoTask("D1", 1, new TargetDrivePosition(matWidth, 0, 0), atLocationTasks));
        autoTasks.add(new AutoTask("D2", 1, new TargetDrivePosition(-matWidth, 0, 0), atLocationTasks).setCompleted(true));
        autoTasks.add(new AutoTask("D3", 1, new TargetDrivePosition(0, -matWidth, 0), atLocationTasks).setCompleted(true));

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(00)); // once at this location, wait 700 msecs
        autoTasks.add(new AutoTask("END", 10, new TargetDrivePosition(startPose.getX(), startPose.getY(), startPose.getHeading()), atLocationTasks));


        autoTasks.add(new AutoTask("Apple", 1, new TargetDrivePosition(40, -25, Math.toRadians(90)), atLocationTasks));


        return autoTasks;
    }

    private ArrayList<String> ringStackEstimates = new ArrayList<String>(); // holds all outputs given by the vision system during the capturing period
    public String getCalculatedDropPosition(){ // sees which position gets the most "votes" by the system and return it

        int[] counts = getEstimateOccurances();

        if(counts[2] > counts[0] && counts[2] > counts[1]){ // if there are the most votes for c
            return "C";
        }
        else if(counts[1] > counts[0] && counts[1] > counts[2]){ // if there are the most votes for b
            return "B";
        }
        else { // if nothing else, must be A
            return "A";
        }
    }
    private int[] getEstimateOccurances(){
        int[] occuranceCounts = new int[3];

        for (String estimate : ringStackEstimates){
            if (estimate.equals("A")){
                occuranceCounts[0]++;
            }
            else  if (estimate.equals("B")){
                occuranceCounts[1]++;
            }
            else { // if not A or B, then C must have just gotten a vote
                occuranceCounts[2]++;
            }
        }

        return occuranceCounts;
    }
    public void setWobbleGoalPos(String positionLabel){ // based on which position label is passed in, change the target position for dropping the wobble goal
        TargetDrivePosition wobbleGoalPosition;

        if(positionLabel.equals("A")){
            wobbleGoalPosition = wobbleGoalPosA;
            taskManager.setTaskWithNameLocation("Place Wobble 1", wobbleGoalPosition); // note: it is ok that if it is "A" the setting is redundant, the resources required to set are low and in FTC readability is favored over efficiency
            taskManager.setTaskWithNameLocation("Place Wobble 2", new TargetDrivePosition(wobbleGoalPosition.getX()-17.5, wobbleGoalPosition.getY(), wobbleGoalPosition.getHeading() + Math.toRadians(30))); // once the position has been found, set the tasks to their new positions
            
            ArrayList<DriveFollowerTask> waitTasks = new ArrayList<DriveFollowerTask>();
            waitTasks.add(new DriveFollowerTask(1000));
            
            taskManager.setTaskWithNameLocationTasks("Place Wobble 1", waitTasks);
            taskManager.setTaskWithNameLocationTasks("Place Wobble 2", waitTasks);
        }
        else if (positionLabel.equals("B")){
            wobbleGoalPosition = wobbleGoalPosB;
            taskManager.setTaskWithNameLocationTasks("Collect Rings", new ArrayList<DriveFollowerTask>()); // if only one ring there, remove the drive movement forward once at location to pick up the third ring (by resetting the at location tasks list for collecting rings)

            taskManager.setTaskWithNameLocation("Place Wobble 1", wobbleGoalPosition); // once the position has been found, set the tasks to their new positions
            taskManager.setTaskWithNameLocation("Place Wobble 2", new TargetDrivePosition(wobbleGoalPosition.getX(), wobbleGoalPosition.getY()+9.5, wobbleGoalPosition.getHeading())); // note: it is ok that if it is "A" the setting is redundant, the resources required to set are low and in FTC readability is favored over efficiency
        }
        else {
            wobbleGoalPosition = wobbleGoalPosC;

            taskManager.setTaskWithNameLocation("Place Wobble 1", new TargetDrivePosition(wobbleGoalPosition.getX()+5.3, wobbleGoalPosition.getY()+5.3, wobbleGoalPosition.getHeading() )); // note: it is ok that if it is "A" the setting is redundant, the resources required to set are low and in FTC readability is favored over efficiency
            taskManager.setTaskWithNameLocation("Place Wobble 2", wobbleGoalPosition); // once the position has been found, set the tasks to their new positions
            taskManager.setTaskWithNameLocation("Collect Wobble", new TargetDrivePosition(wobblePickupPos.getX(), wobblePickupPos.getY(), wobblePickupPos.getHeading() + Math.toRadians(-5) )); // note: it is ok that if it is "A" the setting is redundant, the resources required to set are low and in FTC readability is favored over efficiency
        }


    }

    public static double mSecToSec(double mSec) { return (mSec/1000); }
    public static double changePrecision(double input, int precision){
        return (double)((int)(input * Math.pow(10, precision))) / Math.pow(10, precision);
    }
}
