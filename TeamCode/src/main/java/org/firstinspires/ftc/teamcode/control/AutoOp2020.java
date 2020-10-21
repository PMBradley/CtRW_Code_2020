package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Auto;
import org.firstinspires.ftc.teamcode.util.DuoHolder;
import org.firstinspires.ftc.teamcode.util.TrajectoryIntDuoHolder;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "Test_AutoOp", group = "@@@")

public class AutoOp2020 extends LinearOpMode {

    Drive_Mecanum_Auto drive;
    ArrayList<TrajectoryIntDuoHolder> tasks; // not just a trajectory array list because we want to be able to pause in the state machine for an amount of time

    Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));




    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drive_Mecanum_Auto(hardwareMap, true);

        setupTasks(); // add the list of task objects to the task list
        drive.setTasks(tasks); // then set the drive to use those tasks when required

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){ // while it is ok to keep running in a loop, do so

            drive.doTasksAsync(); // use the built in drive state machine that decides if/when each task is over, and follows them in order appropriately
        }
    }


    void setupTasks(){
        tasks.add( new TrajectoryIntDuoHolder(
                drive.trajectoryBuilder( startPos )
                .lineToLinearHeading(new Pose2d(30, 30, Math.toRadians(-90)))
                .build()
        ));

        tasks.add( new TrajectoryIntDuoHolder(
                4000 // wait for 4 seconds
        ));

        tasks.add( new TrajectoryIntDuoHolder (
                drive.trajectoryBuilder( tasks.get(0).getTraj().end() )
                .splineTo(new Vector2d(50, 10), Math.toRadians(0))
                .build()
        ));

        tasks.add( new TrajectoryIntDuoHolder(
                drive.trajectoryBuilder( tasks.get(2).getTraj().end(), 180 )
                .splineTo(new Vector2d(0, 0), 0)
                .build()
        ));
    }

}
