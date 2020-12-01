package org.firstinspires.ftc.teamcode.hardware.drive.samples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Auto;
import org.firstinspires.ftc.teamcode.hardware.drive.samples.SampleMecanumDrive;

import java.util.Vector;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drive_Mecanum_Auto drive = new Drive_Mecanum_Auto(hardwareMap);

        Pose2d startPos  = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d targetPos = new Pose2d(30, 20, Math.toRadians(0));

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(startPos)
                .splineTo(new Vector2d(targetPos.getX(), targetPos.getY()), targetPos.getHeading())
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(startPos.getX(), startPos.getY()), startPos.getHeading())
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(traj2);


    }
}
