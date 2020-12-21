package org.firstinspires.ftc.teamcode.control.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Auto;
import org.firstinspires.ftc.teamcode.hardware.vision.OpenCV.RingStackHeightPipeline;
import org.firstinspires.ftc.teamcode.hardware.vision.OpenCV.Vision_OpenCV_ExternalCam;
import org.firstinspires.ftc.teamcode.util.StateMachine.DriveFollowerTask;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "Test_AutoOp", group = "@@T")

public class Test_AutoOp extends LinearOpMode {
    Drive_Mecanum_Auto drive;
    ArrayList<DriveFollowerTask> driveTasks = new ArrayList<DriveFollowerTask>(); // not just a trajectory array list because we want to be able to pause in the state machine for an amount of time

    Pose2d testStartPos = new Pose2d(0, 0, Math.toRadians(0));


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drive_Mecanum_Auto(hardwareMap);

        setupTestDriveTasks(); // add the list of task objects to the task list

        drive.setPoseEstimate(testStartPos); // set the localizer's start position
        drive.setTasks(driveTasks); // then set the drive to use those tasks when required

        waitForStart();



        while (opModeIsActive() && !isStopRequested()){ // while it is ok to keep running in a loop, do so

            drive.doTasksAsync(); // use the built in drive state machine that decides if/when each task is over, and follows them in order appropriately


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


    void setupTestDriveTasks(){
        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder( testStartPos )
                        .lineTo(new Vector2d(41, 41))
                        .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                400
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(driveTasks.get(0).getTraj().end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                400
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(driveTasks.get(2).getTraj().end())
                        .lineToLinearHeading( new Pose2d(41, 41, Math.toRadians(-90)))
                        .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                400
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(driveTasks.get(4).getTraj().end())
                        .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(180)))
                        .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                400
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(driveTasks.get(6).getTraj().end(), true)
                        .splineTo(new Vector2d(20, 0), Math.toRadians(0))
                        .splineTo(new Vector2d(40, 20), Math.toRadians(90))
                        .splineTo(new Vector2d(20, 40), Math.toRadians(180))
                        .splineTo(new Vector2d(0, 20), Math.toRadians(270))
                        .lineToSplineHeading((new Pose2d(0, 0, Math.toRadians(0))))
                        .build()
        ));
    }


    public static double mSecToSec(double mSec) { return (mSec/1000); }
    public static double changePrecision(double input, int precision){
        return (double)((int)(input * Math.pow(10, precision))) / Math.pow(10, precision);
    }
}