package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.trajectory.Trajectory;


public class TrajectoryIntDuoHolder { // this is just a class to hold two objects in one. Good with ArrayLists
    private Trajectory traj; // the RR trajectory class
    private int number; // an integer

    public TrajectoryIntDuoHolder(){} // default constructor
    public TrajectoryIntDuoHolder(Trajectory traj){ // trajectory setting constructor
        this.traj = traj;
    }
    public TrajectoryIntDuoHolder(int number){ // number setting constructor
        this.number = number;
    }
    public TrajectoryIntDuoHolder(Trajectory traj, int number){ // both setting constructor
        this.traj = traj;
        this.number = number;
    }

    // assorted getters and setters
    public Trajectory getTraj(){return traj;}
    public int getNum(){return number;}
    public void setTraj(Trajectory traj){ this.traj = traj;}
    public void setNum(int number){ this.number = number;}
}
