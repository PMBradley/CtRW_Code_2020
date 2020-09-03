package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

        waitForStart();

        if (isStopRequested()) return;


        Trajectory traj = drive.trajectoryBuilder(startPos)
                .splineToLinearHeading(new Pose2d(30, 30, Math.toRadians(90)), Math.toRadians(0))
                .build();
        
        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
                .build();



        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(traj2);
    }
}
