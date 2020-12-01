package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Auto;
import org.firstinspires.ftc.teamcode.hardware.vision.OpenCV.RingStackHeightPipeline;
import org.firstinspires.ftc.teamcode.hardware.vision.OpenCV.Vision_OpenCV_ExternalCam;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "Test_AutoOp", group = "@@T")

public class Test_AutoOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drive_Mecanum_Auto drive = new Drive_Mecanum_Auto(hardwareMap);

        Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d startPos1 = new Pose2d(30, 30,  Math.toRadians(0));
        Pose2d startPos2 = new Pose2d(0, 0,  Math.toRadians(0));
        Pose2d startPos3 = new Pose2d(30, 30, Math.toRadians(90));
        Pose2d startPos4 = new Pose2d(0, 0, Math.toRadians(180));


        waitForStart();

        if (isStopRequested()) return;


        Trajectory traj = drive.trajectoryBuilder(startPos)
                .lineToLinearHeading(startPos1)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(startPos2.getX(), startPos2.getY()), startPos4.getHeading())
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(startPos3)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .splineToSplineHeading(startPos4, startPos4.getHeading())
                .build();


        //Vision_OpenCV_ExternalCam vision = new Vision_OpenCV_ExternalCam(hardwareMap, new RingStackHeightPipeline());


        drive.followTrajectory(traj);

        sleep(1000);

        drive.followTrajectory(traj1);

        sleep(1000);

        drive.followTrajectory(traj2);

        sleep(1000);

        drive.followTrajectory(traj3);
    }
}
