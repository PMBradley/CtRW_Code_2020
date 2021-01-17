package org.firstinspires.ftc.teamcode.control.experimental;

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
@Autonomous(name = "AutoOp2020_EXP", group = "@@E") // the name and group for the opmode
@Config // allow FTC dashboard to access public static member variables here

public class AutoOp2020_exp extends LinearOpMode {

    Provider2020_exp robot;
    Drive_Mecanum_Auto drive;
    J_Shooter_Ring_ServoFed shooter;
    Intake_Ring_Drop intake;
    Arm_Wobble_Grabber wobble;
    Vision_OpenCV_ExternalCam vision;

    AutoTaskManager taskManager;
    ElapsedTime runtime;


    public static String ROBOT_NAME = "Robot 2021";

    private static Pose2d startPose = new Pose2d(-60.0, 0.0, Math.toRadians(0)); // the starting position of the robot relative to the middle of the field

    public static TargetDrivePosition wobbleGoalPosA  = new TargetDrivePosition(-6.0, -26.0, Math.toRadians(70.0)); // the positions that the robot needs to drive to
    public static TargetDrivePosition wobbleGoalPosB  = new TargetDrivePosition(20.0, -18.0, Math.toRadians(70.0));
    public static TargetDrivePosition wobbleGoalPosC  = new TargetDrivePosition(44.0, -26.0, Math.toRadians(70.0));
    public static TargetDrivePosition wobblePickupPos = new TargetDrivePosition(-38.0, -24.0, Math.toRadians(300.0));
    public static TargetDrivePosition shootPos        = new TargetDrivePosition(-42.0, -12.0, Math.toRadians(0.0));
    public static TargetDrivePosition ringPickupPos   = new TargetDrivePosition(-28.0, -15.0, Math.toRadians(0.0));
    public static TargetDrivePosition parkPos         = new TargetDrivePosition(12.0, 12.0, Math.toRadians(0.0), Math.toRadians(90.0));

    public static double ARM_OFFSET_DEGREES = -340;

    //public static double RING_COLLECT_DISTANCE = 0;

    private String wobbleDropPos = "A";
    private boolean scanningComplete = false;
    private boolean autonomousComplete = false;



    @Override
    public void runOpMode() throws InterruptedException {
        // Pre match start initialization
        runtime = new ElapsedTime();

        robot = new Provider2020_exp(hardwareMap); // setup the hardware classes with the hardware map info (device ports, device names, etc)
        drive = new Drive_Mecanum_Auto(hardwareMap);
        intake = new Intake_Ring_Drop(robot.intakeMotor, robot.intakeLockServo);
        shooter = new J_Shooter_Ring_ServoFed(robot.JShootFront, robot.JShootBack, robot.shooterFeederServo, robot.shooterIndexerServo, robot.shooterAnglerServo);
        wobble = new Arm_Wobble_Grabber(robot.wobbleArmMotor, robot.wobbleLeftWheelServo, robot.wobbleRightWheelServo);

        vision = new Vision_OpenCV_ExternalCam(hardwareMap, new RingStackHeightPipeline()); // setup the camera handling class, passing in the pipeline we want to use to process inputs


        drive.setPoseEstimate(startPose); // set the localizer's start position to our position on the field relative to the center
        vision.stopWebcamStreaming(); // prevent wasting resources by stopping the webcam streaming process until it is needed

        ArrayList<AutoTask> autoTasks = setupAutoTasks(); // add the list of task objects to the task list
        taskManager = new AutoTaskManager(autoTasks); // then set the drive to use those tasks when required

        // set the first target position based on priorities and what is closest
        taskManager.updateCurrentTaskToClosest( drive.getPoseEstimate() ); // have the task manager set the current auto task to the next closest incomplete task

        drive.setTasks(taskManager.generateCurrentTaskDriveTaskList( drive.getPoseEstimate(), drive.getDriveConstraints() )); // then set the drive to go to that task's position and do any of that auto task's drive tasks

        wobble.setArmStartingOffset(ARM_OFFSET_DEGREES); // set the wobble's starting offset, such that the starting position behaves like 0 (as in Auto the encoder likes to start in a weird position)


        telemetry.addLine(ROBOT_NAME + "'s setup completed.");
        telemetry.addData("Chosen first task", taskManager.getCurrentTask().getTaskName());
        telemetry.update();



        waitForStart(); // wait until play is pressed on the robot controller to progress

        // Post start setup (usually for anything that requires moving any parts on the robot)
        wobble.goToUpPos();


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

                    taskManager.markTaskWithNameIncomplete("Collect Rings"); // activate the dependant task
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

            telemetry.update(); // push all of the telemetry to the phone

        }
    }


    void updateSensors(String currentTaskName){ // updates any processing related to sensor data, in this game that mostly means camera data

        if( currentTaskName.equals("Scan & Shoot Rings") && drive.getTaskIndex() > 1 ){ // if current task is this task and we have completed driving to location
            if( !vision.isCameraStreaming() ){ // if the vision streaming process isn't started,
                vision.startWebcamStreaming(); // start the streaming process
            }

            char ringCount = vision.getOutput().charAt(0); // get the first character of the output string, it holds the single digit number of rings output

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
            intake.spinDown();

            if(drive.getTaskIndex() == 1){ // if at location and on the first subtask
                wobble.intakeSpinIn();
                wobble.goToGrabPos();
            }
            else if(drive.getTaskIndex() == 2){ // if at location and on the second subtask
                wobble.stopIntake();
                wobble.goToUpPos();
            }
            else {
                wobble.goToGrabPos();
            }
        }
        else if ( currentTaskName.equals("Scan & Shoot Rings") || currentTaskName.equals("Shoot Rings") ) { // if current task is this task (or the other one)
            wobble.goToRecentPos();
            shooter.spinUp(); // spin up the shooter on the way to
            shooter.optimizeForLonggoal(); // set the shooter to optimize for shooting from across the field

            if(drive.firstTaskCompleted()){ // if at location, start shooting
                intake.spinDown();

                shooter.instructFire();
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
            intake.spinDown();


            if(drive.getTaskIndex() <= 1){ // if at location and on the first subtask
                wobble.intakeSpinOut();
                wobble.goToGrabPos();
            }
            else if(drive.getTaskIndex() == 2){ // if at location and on the second subtask
                wobble.goToUpPos();
                wobble.stopIntake();
            }
        }
        else { // if no tasks selected above, default states are as follows
            shooter.spinDown();
            intake.spinDown();
            wobble.stopIntake();
            shooter.indexerUp();
            wobble.goToRecentPos();
        }


        shooter.updateFeeder(); // always update the shooter feeder, so that it can start the feeding process as soon as possible
    }


    ArrayList<AutoTask> setupAutoTasks(){
        ArrayList<AutoTask> autoTasks = new ArrayList<AutoTask>(); // create an array list to hold the tasks that will be output by the method
        ArrayList<DriveFollowerTask> atLocationTasks = new ArrayList<DriveFollowerTask>(); // create an array list to hold the driveTasks for each location (reset between adding tasks unless their at location drive task list is the same)


        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(1500)); // once at this location, wait msecs
        atLocationTasks.add(new DriveFollowerTask(50)); // then wait msecs
        autoTasks.add(new AutoTask("Place Wobble 1", 2, wobbleGoalPosA, atLocationTasks));
        autoTasks.add(new AutoTask("Place Wobble 2", 2, wobbleGoalPosA, atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(1050)); // once at this location, wait msecs
        atLocationTasks.add(new DriveFollowerTask(50)); // once at this location, wait msecs
        autoTasks.add(new AutoTask("Scan & Shoot Rings", 1, shootPos, atLocationTasks));
        autoTasks.add(new AutoTask("Shoot Rings", 2, shootPos, atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)

        atLocationTasks = new ArrayList<DriveFollowerTask>();
      /*  atLocationTasks.add(new DriveFollowerTask(drive.trajectoryBuilder(ringPickupPos.getPose2d())
                .forward(RING_COLLECT_DISTANCE)
                .build()
        )); */// once at this location, move forwards into the rings
        atLocationTasks.add(new DriveFollowerTask(700)); // once at this location, wait msecs
        autoTasks.add(new AutoTask("Collect Rings", 2, ringPickupPos, atLocationTasks).setCompleted(true)); // set completed so it won't try to collect rings before shooting rings for the first time

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(1800)); // wait msecs once at location
        atLocationTasks.add(new DriveFollowerTask(50)); // then wait msecs
        autoTasks.add(new AutoTask("Collect Wobble", 2, wobblePickupPos, atLocationTasks).setCompleted(true)); // set completed so it won't try to collect rings before shooting rings for the first time

        atLocationTasks = new ArrayList<DriveFollowerTask>();
        atLocationTasks.add(new DriveFollowerTask(50000)); // wait msecs once at location
        autoTasks.add(new AutoTask("END", 3, parkPos, atLocationTasks));

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
        int aCount = 0, bCount = 0, cCount = 0;

        for(String estimate : ringStackEstimates){ // count how many of each letter there is
            if(estimate.equals("A")){
                aCount++;
            }
            else if(estimate.equals("B")){
                bCount++;
            }
            else if(estimate.equals("C")){
                cCount++;
            }
        }

        if(cCount > aCount && cCount > bCount){ // if there are the most votes for c
            return "C";
        }
        else if(bCount > aCount && bCount > cCount){ // if there are the most votes for b
            return "B";
        }
        else { // if nothing else, must be A
            return "A";
        }
    }
    public void setWobbleGoalPos(String posLabel){
        // TODO
    }

    public static double mSecToSec(double mSec) { return (mSec/1000); }
    public static double changePrecision(double input, int precision){
        return (double)((int)(input * Math.pow(10, precision))) / Math.pow(10, precision);
    }
}
