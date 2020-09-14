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
@Autonomous(name = "Test_AutoOp", group = "@@T")

public class Test_AutoOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d startPos1 = new Pose2d(30, 30,  Math.toRadians(-90));
        Pose2d startPos2 = new Pose2d(50, 10,  Math.toRadians(0));
        Pose2d startPos3 = new Pose2d(0, 0, Math.toRadians(180));

        waitForStart();

        if (isStopRequested()) return;


        Trajectory traj = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Pose2d(30, 30, Math.toRadians(-90)))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .splineTo(new Vector2d(50, 10), Math.toRadians(0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .splineTo(new Vector2d(0, 0), 0)
                //.forward(10)
                .build();

   /*     Trajectory traj = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(new Vector2d(30, 30), Math.toRadians(-90)) // line to startPose1 with linear heading
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .splineTo(startPos2)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(0, 0))
                .build();
*/


        drive.followTrajectory(traj);

        sleep(1000);

        drive.followTrajectory(traj1);

        sleep(1000);

        drive.followTrajectory(traj2);
    }
}
