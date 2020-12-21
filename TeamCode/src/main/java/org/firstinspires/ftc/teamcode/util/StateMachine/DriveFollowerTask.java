package org.firstinspires.ftc.teamcode.util.StateMachine;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;

import org.firstinspires.ftc.teamcode.hardware.drive.Drive_Mecanum_Auto;


public class DriveFollowerTask { // this is just a class to hold two objects in one. Good with ArrayLists
    private Trajectory traj; // the RR trajectory class
    private int number; // an integer

    public DriveFollowerTask(){} // default constructor
    public DriveFollowerTask(Trajectory traj){ // trajectory setting constructor
        this.traj = traj;
    }

    public DriveFollowerTask(int number){ // number setting constructor
        this.number = number;
    }
    public DriveFollowerTask(Trajectory traj, int number){ // both setting constructor
        this.traj = traj;
        this.number = number;
    }

    // assorted getters and setters
    public Trajectory getTraj(){return traj;}
    public int getNum(){return number;}
    public void setTraj(Trajectory traj){ this.traj = traj;}
    public void setNum(int number){ this.number = number;}
}