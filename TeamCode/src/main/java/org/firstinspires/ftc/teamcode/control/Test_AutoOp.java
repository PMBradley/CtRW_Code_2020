package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.drive.samples.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class Test_AutoOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d startPos1 = new Pose2d(30, 30,  Math.toRadians(90));
        Pose2d startPos2 = new Pose2d(10, 50,  Math.toRadians(180));
        Pose2d startPos3 = new Pose2d(10, 0, Math.toRadians(180));

        waitForStart();

        if (isStopRequested()) return;


        Trajectory traj = drive.trajectoryBuilder(startPos)
                .splineToLinearHeading(new Pose2d(30, 30, Math.toRadians(0)), Math.toRadians(90))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(startPos2)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                //.splineTo(startPos3)
                .forward(40)
                .build();



        drive.followTrajectory(traj);

        sleep(1000);

        drive.followTrajectory(traj1);

        sleep(1000);

        drive.followTrajectory(traj2);
    }
}
