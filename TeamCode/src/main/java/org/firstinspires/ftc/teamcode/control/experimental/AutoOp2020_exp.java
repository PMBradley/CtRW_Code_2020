package org.firstinspires.ftc.teamcode.control.experimental;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Auto;
import org.firstinspires.ftc.teamcode.util.StateMachine.AutoTask;
import org.firstinspires.ftc.teamcode.util.StateMachine.AutoTaskManager;
import org.firstinspires.ftc.teamcode.util.StateMachine.DriveFollowerTask;
import org.firstinspires.ftc.teamcode.util.StateMachine.TargetDrivePosition;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "AutoOp2020_EXP", group = "@@E")

public class AutoOp2020_exp extends LinearOpMode {

    Drive_Mecanum_Auto drive;
    AutoTaskManager taskManager;
    ElapsedTime runtime;

    Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));


     // private Pose2d startPose = new Pose2d(-50.0, -50.0, Math.toRadians(0));
    private Pose2d startPose = new Pose2d(0.0, 0.0, Math.toRadians(0)); // TODO: change back to actual start position once done testing

    private TargetDrivePosition ringScanPos     = new TargetDrivePosition(-30.0, -50.0, Math.toRadians(0.0));
    private TargetDrivePosition wobbleGoalPosA  = new TargetDrivePosition(-7.0, -55.0, Math.toRadians(0.0));
    private TargetDrivePosition wobbleGoalPosB  = new TargetDrivePosition(-7.0, -55.0, Math.toRadians(0.0));
    private TargetDrivePosition wobbleGoalPosC  = new TargetDrivePosition(-7.0, -55.0, Math.toRadians(0.0));
    private TargetDrivePosition wobblePickupPos = new TargetDrivePosition(-44.0, -30.0, Math.toRadians(-90.0));
    private TargetDrivePosition shootPos        = new TargetDrivePosition(-7.0, -24.0, Math.toRadians(0.0));
    private TargetDrivePosition ringPickupPos   = new TargetDrivePosition(-7.0, -24.0, Math.toRadians(0.0));


    private boolean autonomousComplete = false;


    @Override
    public void runOpMode() throws InterruptedException {
        runtime = new ElapsedTime();

        drive = new Drive_Mecanum_Auto(hardwareMap);

        drive.setPoseEstimate(startPos); // set the localizer's start position

        telemetry.addLine("Test Point -2");
        telemetry.update();

        ArrayList<AutoTask> autoTasks = setupTestAutoTasks(); // add the list of task objects to the task list TODO: make sure to update to non test version once testing complete
        taskManager = new AutoTaskManager(autoTasks); // then set the drive to use those tasks when required



        // set the first target position based on priorities and what is closest
        taskManager.updateCurrentTaskToClosest( drive.getPoseEstimate() ); // have the task manager set the current auto task to the next closest incomplete task


        drive.setTasks(taskManager.generateCurrentTaskDriveTaskList( drive.getPoseEstimate(), drive.getDriveConstraints() )); // then set the drive to go to that task's position and do any of that auto task's drive tasks

        telemetry.addLine("Robot setup completed.");
        telemetry.addData("Chosen first task", taskManager.getCurrentTask().getTaskName());
        telemetry.update();

        waitForStart();

        telemetry.addLine("Test Point 0");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested() && !autonomousComplete){ // while it is ok to keep running in a loop, do so

            boolean autoTaskComplete = drive.doTasksAsync(); // use the built in drive state machine that decides if/when each task is over, and follows them in order appropriately


            if( autoTaskComplete ){ // if the current task is complete
                taskManager.markCurrentTaskComplete(); // set the current task to complete

                // check the name of the current task against as list of prerequisite tasks
                String currentTaskName = taskManager.getCurrentTask().getTaskName();
                if( currentTaskName.equals("B1") ){  // if this prerec task completed
                    taskManager.markTaskWithNameIncomplete("B2"); // activate the dependant task
                }
                else if( currentTaskName.equals("C1") ){ // if this prerec task completed
                    taskManager.markTaskWithNameIncomplete("C2"); // activate the dependant task
                }
                else if( currentTaskName.equals("D1") ){ // if this prerec task completed
                    taskManager.markTaskWithNameIncomplete("D2"); // activate the dependant task
                }
                else if( currentTaskName.equals("D2") ){ // if this prerec task completed
                    taskManager.markTaskWithNameIncomplete("D3"); // activate the dependant task
                }
                else if( currentTaskName.equals("END")){
                    autonomousComplete = true; // breaks free of the main loop
                }

                Pose2d currentPos = drive.getPoseEstimate(); // get the current position of the robot
                taskManager.updateCurrentTaskToClosest( currentPos ); // have the task manager set the current auto task to the next closest incomplete task
                drive.setTasks(taskManager.generateCurrentTaskDriveTaskList( currentPos, drive.getDriveConstraints() )); // then set the drive to go to that task's position and do any of that auto task's drive tasks
            }

            telemetry.addData("Current task name", taskManager.getCurrentTask().getTaskName() );
            telemetry.addData("Drive task index", drive.getTaskIndex()); // print out what the current drive task index is
            if(drive.currentTaskHasTrajectory()) { // if current task has a trajectory to follow
                telemetry.addLine("Currently following a trajectory. Elapsed time: " + changePrecision( mSecToSec(drive.getTaskElapsedTime()), 2) + " seconds"); // print the projected remaining time on that trajectory
                telemetry.addData("Target position", drive.getCurrentTask().getTraj().end()); // print the end position of the trajectory
            }
            else {
                telemetry.addLine("Currently waiting. Elapsed time: " + changePrecision(mSecToSec(drive.getCurrentTask().getNum() - drive.getRemainingWaitMSecs()), 2) + " seconds");
                telemetry.addLine("Time remaining: " + changePrecision(mSecToSec(drive.getRemainingWaitMSecs()), 2) + " seconds");
            }
            telemetry.addData("Current position", drive.getLocalizer().getPoseEstimate()); // print the estimated current position of the robot

            telemetry.update(); // push all of the telemetry to the phones

        }
    }


    ArrayList<AutoTask> setupAutoTasks(){
        ArrayList<AutoTask> autoTasks = new ArrayList<AutoTask>();
        ArrayList<DriveFollowerTask> atLocationTasks = new ArrayList<DriveFollowerTask>();

        atLocationTasks.clear();
        atLocationTasks.add(new DriveFollowerTask(100)); // once at this location, wait 100 msecs
        autoTasks.add(new AutoTask("Determine Ring Count", 0, ringScanPos, atLocationTasks));


        atLocationTasks.clear();
        atLocationTasks.add(new DriveFollowerTask(700)); // once at this location, wait 700 msecs
        autoTasks.add(new AutoTask("Place Wobble 1", 1, wobbleGoalPosA, atLocationTasks));
        autoTasks.add(new AutoTask("Place Wobble 2", 2, wobbleGoalPosA, atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)

        atLocationTasks.clear();
        atLocationTasks.add(new DriveFollowerTask(800)); // once at this location, wait 700 msecs
        autoTasks.add(new AutoTask("Shoot Rings 1", 1, shootPos, atLocationTasks));
        autoTasks.add(new AutoTask("Shoot Rings 2", 2, shootPos, atLocationTasks).setCompleted(true)); // set completed so that the pathing algorithm won't consider it until it is set true (which will happen once its prerequisite task becomes completed)

        atLocationTasks.clear();
        atLocationTasks.add(new DriveFollowerTask(drive.trajectoryBuilder(ringPickupPos.getPose2d())
                .forward(10)
                .build()
        )); // once at this location, move forwards into the rings
        autoTasks.add(new AutoTask("Collect Rings", 2, ringPickupPos, atLocationTasks));

        atLocationTasks.clear();
        atLocationTasks.add(new DriveFollowerTask(700)); // wait 700 msecs once at location
        autoTasks.add(new AutoTask("Collect Wobble", 2, wobblePickupPos, atLocationTasks));


        return autoTasks;
    }


    void updateManipulators(){




    }

    private int D_PreRecs_Complete = 0;
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
        autoTasks.add(new AutoTask("END", 10, new TargetDrivePosition(startPos.getX(), startPos.getY(), startPos.getHeading()), atLocationTasks));


        return autoTasks;
    }

    public static double mSecToSec(double mSec) { return (mSec/1000); }
    public static double changePrecision(double input, int precision){
        return (double)((int)(input * Math.pow(10, precision))) / Math.pow(10, precision);
    }
}
