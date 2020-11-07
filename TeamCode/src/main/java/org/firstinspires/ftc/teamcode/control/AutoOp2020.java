package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Auto;
import org.firstinspires.ftc.teamcode.util.DuoHolder;
import org.firstinspires.ftc.teamcode.util.FSM.DriveFollowerTask;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "AutoOp2020", group = "@@@")

public class AutoOp2020 extends LinearOpMode {

    Drive_Mecanum_Auto drive;
    ArrayList<DriveFollowerTask> driveTasks = new ArrayList<DriveFollowerTask>(); // not just a trajectory array list because we want to be able to pause in the state machine for an amount of time

    Pose2d testStartPos = new Pose2d(0, 0, Math.toRadians(0));


    private Pose2d startPose = new Pose2d(-50.0, -50.0, Math.toRadians(180));
    private Pose2d ringPos = new Pose2d(-30.0, -50.0, Math.toRadians(180.0));
    private Pose2d goalPos = new Pose2d(-7.0, -55.0, Math.toRadians(180.0));
    private Pose2d pickupPos = new Pose2d(-44.0, -30.0, Math.toRadians(-60.0));
    private Pose2d shootPos1 = new Pose2d(-7.0, -24.0, Math.toRadians(90.0));
    private Pose2d shootPos2 = new Pose2d(shootPos1.getX(), shootPos1.getY() + 7.5, Math.toRadians(shootPos1.getHeading()));
    private Pose2d shootPos3 = new Pose2d(shootPos2.getX(), shootPos2.getY() + 7.5, Math.toRadians(shootPos1.getHeading()));
    private double goalPosDriveHeading = Math.toRadians(0.0);
    private double pickupPosDriveHeading = Math.toRadians(120.0);


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drive_Mecanum_Auto(hardwareMap, true);

        setupTestDriveTasks(); // add the list of task objects to the task list
        drive.setTasks(driveTasks); // then set the drive to use those tasks when required

        waitForStart();



        while (opModeIsActive() && !isStopRequested()){ // while it is ok to keep running in a loop, do so

            drive.doTasksAsync(); // use the built in drive state machine that decides if/when each task is over, and follows them in order appropriately


            telemetry.addData("Drive task index", drive.getTaskIndex()); // print out what the current drive task index is
            if(drive.currentTaskHasTrajectory()) { // if current task has a trajectory to follow
                telemetry.addLine("Currently following a trajectory. Elapsed time: " + changePrecision( mSecToSec(drive.getCurrentTrajElapsedTime()), 2) + " seconds"); // print the projected remaining time on that trajectory
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
                        .lineTo(new Vector2d(36, 12))
                        .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                2500
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(driveTasks.get(0).getTraj().end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                2500
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(driveTasks.get(2).getTraj().end())
                        .lineToLinearHeading( new Pose2d(36, 12, Math.toRadians(-90)))
                        .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                2500
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(driveTasks.get(4).getTraj().end())
                        .lineToLinearHeading(new Pose2d(0, 12, Math.toRadians(180)))
                        .build()
        ));
        /*
        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder( testStartPos )
                .lineToLinearHeading(new Pose2d(30, 30, Math.toRadians(-90)))
                .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                4000 // wait for 4 seconds
        ));

        driveTasks.add( new DriveFollowerTask (
                drive.trajectoryBuilder( driveTasks.get(0).getTraj().end() )
                .splineTo(new Vector2d(50, 10), Math.toRadians(0))
                .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder( driveTasks.get(2).getTraj().end(), 180 )
                .splineTo(new Vector2d(0, 0), 0)
                .build()
        ));*/
    }



    void setupDriveTasks(){
         driveTasks.add( new DriveFollowerTask(
                 drive.trajectoryBuilder(startPose, true)
                .splineTo(new Vector2d(ringPos.getX(), ringPos.getY()), ringPos.getHeading())
                .build()
         ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(ringPos, true)
                .splineTo(new Vector2d(goalPos.getX(), goalPos.getY()), goalPos.getHeading())
                .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(goalPos)
                .splineToSplineHeading(pickupPos, pickupPosDriveHeading)
                .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(pickupPos)
                .splineToSplineHeading(goalPos, goalPosDriveHeading)
                .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(goalPos)
                .lineToSplineHeading(shootPos1)
                .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(shootPos1)
                .lineTo(new Vector2d(shootPos2.getX(), shootPos2.getY()))
                .build()
        ));

        driveTasks.add( new DriveFollowerTask(
                drive.trajectoryBuilder(shootPos2)
                .lineTo(new Vector2d(shootPos3.getX(), shootPos3.getY()))
                .build()
        ));
    }


    public static double mSecToSec(double mSec) { return (mSec/1000); }
    public static double changePrecision(double input, int precision){
        return (double)((int)(input * Math.pow(10, precision))) / Math.pow(10, precision);
    }
}
